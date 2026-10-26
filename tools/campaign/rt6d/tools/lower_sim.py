#!/usr/bin/env python3
"""RT6d item 4: the exit lowering against a simulated arm, offline. CPU only, no DDS.

Runs the arm controller's own `_lower_step` (teleop/robot_control/robot_arm.py) at its
250 Hz rate against tools/arm_physics.py (the IK's reduced model, ABA dynamics,
armature and viscous friction assumed) at 2 kHz, from a forward pose that holds about
8 Nm on the shoulder pitch joints (what RT6b and RT6c ended in) down to the low pose.
Compares the RT6b linear plan with the RT6d min-jerk plan plus governor, over a sweep
of armature, friction and initial joint speed. Speeds are TRUE simulated joint speeds
and, separately, the reported speeds with gaussian noise like the motors' estimate.

    python lower_sim.py [--out DIR]
"""
import argparse
import csv
import os
import sys
import threading
import types
from pathlib import Path

XRT = Path.home() / "xrt"
sys.path[:0] = [str(XRT), str(XRT / "teleop" / "televuer" / "src")]
os.chdir(XRT / "teleop")
import numpy as np                                                        # noqa: E402
from teleop.robot_control import robot_arm as ra                          # noqa: E402
from teleop.robot_control.robot_arm_ik import G1_29_ArmIK                 # noqa: E402
from teleop.robot_control.start_pose import load_pose                     # noqa: E402
from tools.arm_physics import ArmPhysics                                  # noqa: E402
from televuer.tv_wrapper import (T_ROBOT_OPENXR, T_OPENXR_ROBOT,          # noqa: E402
                                 T_TO_UNITREE_HUMANOID_LEFT_ARM, T_TO_UNITREE_HUMANOID_RIGHT_ARM,
                                 transform_IPunitree_Brobot_world_arm_to_head_then_waist)
import pinocchio as pin                                                   # noqa: E402

ARM_IDS = [j for j in ra.G1_29_JointArmIndex]
WRIST = {ra.G1_29_JointIndex.kLeftWristRoll, ra.G1_29_JointIndex.kLeftWristPitch, ra.G1_29_JointIndex.kLeftWristyaw,
         ra.G1_29_JointIndex.kRightWristRoll, ra.G1_29_JointIndex.kRightWristPitch, ra.G1_29_JointIndex.kRightWristYaw}


def waist(fwd, out, up, side):
    head = np.eye(4); head[1, 3] = 1.5
    m = np.eye(4); m[:3, 3] = [-out * side, 1.5 + up, -fwd]
    arm = T_ROBOT_OPENXR @ m @ T_OPENXR_ROBOT
    arm = arm @ (T_TO_UNITREE_HUMANOID_LEFT_ARM if side > 0 else T_TO_UNITREE_HUMANOID_RIGHT_ARM)
    return transform_IPunitree_Brobot_world_arm_to_head_then_waist(arm, T_ROBOT_OPENXR @ head @ T_OPENXR_ROBOT, "head_yaw")


class FakeCtrl(ra.G1_29_ArmController):
    """Just what _lower_step and lower_and_release's setup touch (class constants inherited)."""
    control_dt = 1.0 / 250.0

    def __init__(self, phys, noise, rng):
        self.phys, self.noise, self.rng = phys, noise, rng
        self.msg = types.SimpleNamespace(motor_cmd=[types.SimpleNamespace(kp=0.0, kd=0.0) for _ in range(35)])
        self._base_kp = {}
        for j in ra.G1_29_JointIndex:
            arm = j in ARM_IDS
            self._base_kp[j] = (40.0 if j in WRIST else 80.0) if arm else 300.0
            self.msg.motor_cmd[j].kp = self._base_kp[j]
            self.msg.motor_cmd[j].kd = (1.5 if j in WRIST else 3.0)
        self._lower_done = threading.Event()

    def get_current_dual_arm_q(self):
        return self.phys.q.copy()

    def get_current_dual_arm_dq(self):
        return self.phys.dq + (self.rng.normal(0.0, self.noise, 14) if self.noise > 0 else 0.0)


def plan(ctrl, q0, q_rest, speed, profile, gfn, hold_t=0.5, release_t=1.5, max_err=0.15):
    # [RT6g] the controller's own plan builder (two groups: shoulders/elbows, wrists)
    return ctrl._make_lower_plan(q0, q_rest, gfn, speed, hold_t, release_t, max_err, profile,
                                 min(0.15, speed), 0.05, 0.0)


def run(q_fwd, q_rest, gfn, profile, armature, viscous, noise, dq0=0.0, speed=0.3, seed=0):
    phys = ArmPhysics(q_fwd, armature=armature, viscous=viscous)
    rng = np.random.default_rng(seed)
    ctrl = FakeCtrl(phys, noise, rng)
    kp = np.array([ctrl._base_kp[j] for j in ARM_IDS]); kd = np.array([ctrl.msg.motor_cmd[j].kd for j in ARM_IDS])
    # 1 s of holding the forward pose (FOLLOW: target = pose, feed-forward = gravity)
    t = 0.0
    phys.dq[:] = dq0
    for _ in range(int(0.3 / ctrl.control_dt)):
        phys.set_command(q_fwd, np.zeros(14), kp, kd, gfn(q_fwd), t)
        phys.advance(ctrl.control_dt, t); t += ctrl.control_dt
    lo = plan(ctrl, phys.q.copy(), q_rest, speed, profile, gfn)
    lo["t0"] = lo["t_last"] = t
    true_max = np.zeros(14); rep_max = np.zeros(14)
    real_mono = ra.time.monotonic
    clock = {"t": t}
    ra.time.monotonic = lambda: clock["t"]
    try:
        t_lim = t + 3.0 * lo["T"] + 10.0 + 2.0 + 5.0      # the controller waits 3 T + 10 s + hold + release + 30 s
        while t < t_lim:
            q_cmd, tau = ra.G1_29_ArmController._lower_step(ctrl, lo)
            kp_now = np.array([ctrl.msg.motor_cmd[j].kp for j in ARM_IDS])
            phys.set_command(q_cmd, np.zeros(14), kp_now, kd, tau, t)
            for _ in range(8):                                  # 4 ms in 0.5 ms substeps
                phys.advance(ctrl.control_dt / 8, t)
                true_max = np.maximum(true_max, np.abs(phys.dq))
            rep_max = np.maximum(rep_max, np.abs(ctrl.get_current_dual_arm_dq()))
            t += ctrl.control_dt
            clock["t"] = t
            if ctrl._lower_done.is_set():
                break
        off_rest = float(np.max(np.abs(phys.q - q_rest)))
        # (no after-release phase: the model has no gearbox friction, so a limp arm here
        # slides to the gravity equilibrium; the real arms stayed put, RT6b/RT6c)
    finally:
        ra.time.monotonic = real_mono
    return dict(profile=profile, armature=armature, viscous=viscous, noise=noise, dq0=dq0,
                T_plan_s=round(lo["T"], 2), done_s=round(t - lo["t0"], 2),
                true_max=round(float(true_max.max()), 3), true_joint=int(np.argmax(true_max)),
                reported_max=round(float(rep_max.max()), 3),
                end_off_rest=round(off_rest, 3), gov_overridden=any(gr["gov_off"] for gr in lo["groups"]),
                gov_s=round(max(gr["gov_steps"] for gr in lo["groups"]) * ctrl.control_dt, 2), done=ctrl._lower_done.is_set())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(Path(__file__).resolve().parent.parent / "out"))
    args = ap.parse_args()
    ik = G1_29_ArmIK()
    model = ik.reduced_robot.model
    data = model.createData()
    z = np.zeros(14)
    gfn = lambda q: pin.rnea(model, data, np.asarray(q, dtype=float), z, z).copy()
    q = np.zeros(14)
    for _ in range(40):                                     # 1:1, wrists 0.40 m ahead, 0.35 m down
        q, _t = ik.solve_ik(waist(0.40, 0.18, -0.35, +1), waist(0.40, 0.18, -0.35, -1), q, np.zeros(14))
        q = np.array(q)
    q_fwd = q.copy()
    q_rest = load_pose(str(XRT / "poses" / "low.yaml"))
    g = gfn(q_fwd)
    print(f"forward pose: shoulder pitch gravity {g[0]:.2f} / {g[7]:.2f} Nm; largest joint gap to the low pose "
          f"{np.max(np.abs(q_rest - q_fwd)):.2f} rad", flush=True)
    rows = []
    cases = [(p, a, v, 0.015, 0.0) for p in ("linear", "minjerk") for a in (0.005, 0.01, 0.02, 0.05, 0.1)
             for v in (0.0, 0.05, 0.2)]
    cases += [("minjerk", 0.02, 0.05, 0.015, 0.25), ("minjerk", 0.1, 0.0, 0.03, 0.25), ("linear", 0.02, 0.05, 0.015, 0.25),
              ("minjerk", 0.02, 0.05, 0.5, 0.0)]       # absurd noise: governor stalls, watchdog must release
    for p, a, v, n, d0 in cases:
        r = run(q_fwd, q_rest, gfn, p, a, v, n, dq0=d0)
        rows.append(r)
        print(f"{p:8s} armature {a:5.3f} viscous {v:4.2f} noise {n:5.3f} dq0 {d0:4.2f}: plan {r['T_plan_s']:5.1f} s, "
              f"done {r['done_s']:5.1f} s, TRUE max {r['true_max']:.3f} rad/s (joint {r['true_joint']}), "
              f"reported max {r['reported_max']:.3f}, off rest at release end {r['end_off_rest']:.3f} rad, "
              f"governor {r['gov_s']:.1f} s{' WATCHDOG (governor overridden)' if r['gov_overridden'] else ''}{'' if r['done'] else ' NOT DONE'}", flush=True)
    out = Path(args.out); out.mkdir(parents=True, exist_ok=True)
    with open(out / "lower_sim.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    print("wrote lower_sim.csv", flush=True)


if __name__ == "__main__":
    main()
    sys.stdout.flush()
    from tools._procs import reap_child_processes
    reap_child_processes()
    os._exit(0)
