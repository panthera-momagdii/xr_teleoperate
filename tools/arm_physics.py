"""[RT6d] A simulated G1 arm pair for fake_lowstate.py --physics. TEST FIXTURE.

Pure: pinocchio + numpy, no DDS. The 14-joint arm model is the IK's own reduced model
(teleop/g1_29_model_cache.pkl: legs, waist and fingers locked at 0, fixed base), so the
inertias and the gravity are the ones the launcher's feed-forward is computed from.

Each motor applies what an hg LowCmd asks of it,

    tau = kp * (q_cmd - q) + kd * (dq_cmd - dq) + tau_ff      (clipped to the URDF effort limit)

and the arm moves by forward dynamics (ABA) with a rotor inertia (`armature`, added to
every joint's diagonal; the URDF has none) and viscous friction. Integration is
semi-implicit Euler in fixed substeps. With no command (before the first LowCmd, or
after the last one stops for longer than `cmd_timeout`), the motors apply nothing and
the arm hangs, which is what a G1 arm does in debug mode when nobody commands it.

What it is for: the measured joint speed during the exit lowering (RT6b, RT6c: 0.47 to
0.48 rad/s against a 0.3 rad/s plan) depends on the arm's dynamics, which the kinematic
`--follow-cmd` fake cannot show. Its numbers are a model, not the robot: no gearbox
friction, no motor current limit, an assumed armature.
"""

import pickle
from pathlib import Path

import numpy as np

CACHE = Path(__file__).resolve().parent.parent / "teleop" / "g1_29_model_cache.pkl"


class ArmPhysics:
    def __init__(self, q0, armature=0.02, viscous=0.05, dt=0.0005, cmd_timeout=0.1, cache=CACHE,
                 friction=None, friction_joints=None, stribeck_vel=0.05, stick_vel=0.005, kd_eff=None):
        import pinocchio as pin
        self.pin = pin
        with open(cache, "rb") as fh:
            self.model = pickle.load(fh)["reduced_model"]
        self.model.armature[:] = float(armature)
        self.data = self.model.createData()
        self.q = np.asarray(q0, dtype=float).copy()
        self.dq = np.zeros(14)
        self.tau = np.zeros(14)
        self.viscous = float(viscous)
        self.dt = float(dt)
        self.cmd_timeout = float(cmd_timeout)
        self.lo, self.hi = self.model.lowerPositionLimit.copy(), self.model.upperPositionLimit.copy()
        self.effort = self.model.effortLimit.copy()
        self.cmd = None            # (q, dq, kp, kd, tau_ff) arrays
        self.t_cmd = None
        self.started = False       # the arm is held still until the first command
        self._acc = 0.0            # time not yet integrated (less than one substep)
        self._dq_max = np.zeros(14)  # max |dq| over every substep since dq_max_since_read()
        # [RT6g] Gearbox friction, Stribeck model with stick: static Fs, sliding Fc (Nm) on
        # the joints in friction_joints (indices). Fs > Fc gives stick-slip: a joint held by
        # friction breaks loose, friction drops, and it overshoots -- what the RT6e wrist
        # rolls looked like (0.33 to 0.42 rad/s against a 0.18 rad/s plan).
        # [RT6h] Fs and Fc may be 14-vectors (per joint). Stick is now a constrained solve
        # (see _ddq_friction), not "zero the speed of anything slow".
        self.fs, self.fc = np.zeros(14), np.zeros(14)
        self.fmask = np.zeros(14, dtype=bool)
        if friction is not None:
            self.fs[:] = np.broadcast_to(np.asarray(friction[0], dtype=float), (14,))
            self.fc[:] = np.broadcast_to(np.asarray(friction[1], dtype=float), (14,))
            self.fmask[list(range(14) if friction_joints is None else friction_joints)] = True
            self.fmask &= self.fs > 0.0
        self.vs, self.v_eps = float(stribeck_vel), float(stick_vel)
        # [RT6h] How much of the commanded kd a joint really gets (14-vector or scalar, 1 =
        # all of it). Calibration to RT6g's lowering: with shoulders/elbows Fs 2.7 / Fc 0.81
        # Nm and wrists 1.7 / 0.17 Nm, the RT6g settings read 0.556 rad/s on the right
        # shoulder yaw (robot 0.551) and 0.587 on the left wrist roll (robot 0.598) only if
        # the wrists get 30 % of their kd; at 100 % the wrist roll stays under 0.38, because
        # kd 4 bounds a breakaway from a 2 Nm push at about 0.46 rad/s in any rigid model.
        self.kd_eff = np.ones(14) if kd_eff is None else np.broadcast_to(np.asarray(kd_eff, dtype=float), (14,)).copy()

    def set_command(self, q, dq, kp, kd, tau_ff, t):
        self.cmd = tuple(np.asarray(v, dtype=float).copy() for v in (q, dq, kp, kd, tau_ff))
        self.t_cmd = t
        if np.any(self.cmd[2] > 0):
            self.started = True

    def _torque(self, t):
        if self.cmd is None or (self.t_cmd is not None and t - self.t_cmd > self.cmd_timeout):
            return np.zeros(14)
        q_c, dq_c, kp, kd, ff = self.cmd
        tau = kp * (q_c - self.q) + kd * self.kd_eff * (dq_c - self.dq) + ff
        return np.clip(tau, -self.effort, self.effort)

    def dq_max_since_read(self):
        out, self._dq_max = self._dq_max, np.zeros(14)
        return out

    def _ddq_friction(self, drive):
        """[RT6h] Joint accelerations with Stribeck friction and stick, solved as a
        constraint. Moving joints get -sign(v) (Fc + (Fs - Fc) exp(-(v/vs)^2)). A joint
        too slow to keep moving through this substep is held (ddq = 0) if the torque
        needed to hold it, from M ddq = tau - nle with the held joints' rows as
        unknowns, is within its static friction Fs; otherwise it breaks loose and moves
        against Fs. Released one at a time, the worst first, until consistent."""
        pin = self.pin
        M = pin.crba(self.model, self.data, self.q)
        M = np.triu(M) + np.triu(M, 1).T
        nle = pin.nonLinearEffects(self.model, self.data, self.q, self.dq)
        v = self.dq
        # a joint whose friction alone would stop it within this substep counts as slow
        v_th = np.maximum(self.v_eps, self.fs / np.diag(M) * self.dt)
        moving = self.fmask & (np.abs(v) > v_th)
        fr = np.zeros(14)
        vm = v[moving]
        fr[moving] = -np.sign(vm) * (self.fc[moving] + (self.fs[moving] - self.fc[moving]) * np.exp(-(vm / self.vs) ** 2))
        rhs = drive + fr - nle
        held = list(np.where(self.fmask & ~moving)[0])
        while True:
            free = [i for i in range(14) if i not in held]
            ddq = np.zeros(14)
            ddq[free] = np.linalg.solve(M[np.ix_(free, free)], rhs[free])
            if not held:
                break
            lam = M[np.ix_(held, free)] @ ddq[free] - rhs[held]     # holding torque needed
            ratio = np.abs(lam) / self.fs[held]
            k = int(np.argmax(ratio))
            if ratio[k] <= 1.0:
                break
            i = held.pop(k)
            rhs[i] += np.sign(lam[k]) * self.fs[i]                   # breaks loose against Fs
        if held:
            self.dq[held] = 0.0
        return ddq

    def advance(self, seconds, t_now):
        """Integrate `seconds` of motion in fixed substeps. Returns substeps taken."""
        if not self.started:
            self.tau = self._torque(t_now)
            return 0
        self._acc = min(self._acc + max(0.0, seconds), 200 * self.dt)
        n = int(self._acc / self.dt)
        self._acc -= n * self.dt
        for _ in range(n):
            self.tau = self._torque(t_now)
            drive = self.tau - self.viscous * self.dq
            if self.fmask.any():
                ddq = self._ddq_friction(drive)
            else:
                ddq = self.pin.aba(self.model, self.data, self.q, self.dq, drive)
            self.dq = self.dq + ddq * self.dt
            self.q = self.q + self.dq * self.dt
            self._dq_max = np.maximum(self._dq_max, np.abs(self.dq))
            low, high = self.q < self.lo, self.q > self.hi
            if low.any() or high.any():
                self.q = np.clip(self.q, self.lo, self.hi)
                self.dq[low | high] = 0.0
        return n
