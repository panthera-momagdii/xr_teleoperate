#!/usr/bin/env python3
"""RT6g item 3: the exit lowering against the simulated arm WITH gearbox friction. CPU only.

Runs the arm controller's own `_make_lower_plan` / `_lower_step` (teleop/robot_control/
robot_arm.py) at 250 Hz against tools/arm_physics.py with Stribeck stick-slip friction,
from the RT6e [q] pose (shoulder yaws at +2.607 / -2.616 rad, wrists assumed off zero)
down to poses/low.yaml. Compares the RT6d plan (one speed for all joints: wrist peak 0.18,
wrist lead 0.15 rad) with RT6g (wrist peak 0.15, wrist lead 0.05 rad), over a sweep of
wrist friction. Speeds are TRUE simulated speeds (max over every 0.5 ms substep) and
reported speeds with gaussian noise.

    python lower_friction_sim.py [--out DIR]
"""
import argparse, csv, os, sys, threading, types
from pathlib import Path
XRT = Path.home() / "xrt"
sys.path[:0] = [str(XRT)]
os.chdir(XRT / "teleop")
import numpy as np                                                        # noqa: E402
import pinocchio as pin                                                   # noqa: E402
from teleop.robot_control import robot_arm as ra                          # noqa: E402
from teleop.robot_control.start_pose import load_pose                     # noqa: E402
from tools.arm_physics import ArmPhysics                                  # noqa: E402

ARM_IDS = [j for j in ra.G1_29_JointArmIndex]
WR = {ra.G1_29_JointIndex.kLeftWristRoll, ra.G1_29_JointIndex.kLeftWristPitch, ra.G1_29_JointIndex.kLeftWristyaw,
      ra.G1_29_JointIndex.kRightWristRoll, ra.G1_29_JointIndex.kRightWristPitch, ra.G1_29_JointIndex.kRightWristYaw}
NAMES = ["L sh pitch", "L sh roll", "L sh yaw", "L elbow", "L wr roll", "L wr pitch", "L wr yaw",
         "R sh pitch", "R sh roll", "R sh yaw", "R elbow", "R wr roll", "R wr pitch", "R wr yaw"]
# RT6e watcher at [q] (18:21:14): shoulders and elbows; wrists not logged, assumed rolled 1.0 rad
WRIST_KD = float(os.environ.get("XR_LOWER_WRIST_KD", "4.0"))
Q_RT6E = np.array([-0.5444, 0.2970, 2.6071, 0.8375, 1.0, 0.2, 0.0,
                   -0.4650, -0.0373, -2.6159, 0.7305, -1.0, 0.2, 0.0])


class FakeCtrl(ra.G1_29_ArmController):
    control_dt = 1.0 / 250.0

    def __init__(self, phys, noise, rng):
        self.phys, self.noise, self.rng = phys, noise, rng
        self.msg = types.SimpleNamespace(motor_cmd=[types.SimpleNamespace(kp=0.0, kd=0.0) for _ in range(35)])
        self._base_kp = {}
        for j in ra.G1_29_JointIndex:
            self._base_kp[j] = (40.0 if j in WR else 80.0) if j in ARM_IDS else 300.0
            self.msg.motor_cmd[j].kp = self._base_kp[j]
            self.msg.motor_cmd[j].kd = 1.5 if j in WR else 3.0
        self._lower_done = threading.Event()

    def get_current_dual_arm_q(self):
        return self.phys.q.copy()

    def get_current_dual_arm_dq(self):
        return self.phys.dq + (self.rng.normal(0.0, self.noise, 14) if self.noise > 0 else 0.0)


def run(q_start, q_rest, gfn, wrist_speed, wrist_lead, friction, noise=0.015, speed=0.3):
    phys = ArmPhysics(q_start, armature=0.02, viscous=0.05, friction=friction, friction_joints=[4, 5, 6, 11, 12, 13], dt=0.001)
    ctrl = FakeCtrl(phys, noise, np.random.default_rng(0))
    kp = np.array([ctrl._base_kp[j] for j in ARM_IDS]); kd = np.array([ctrl.msg.motor_cmd[j].kd for j in ARM_IDS])
    t = 0.0
    for _ in range(int(0.3 / ctrl.control_dt)):          # hold the start pose 0.3 s
        phys.set_command(q_start, np.zeros(14), kp, kd, gfn(q_start), t)
        phys.advance(ctrl.control_dt, t); t += ctrl.control_dt
    clock = {"t": t}
    real = ra.time.monotonic
    ra.time.monotonic = lambda: clock["t"]
    try:
        lo = ctrl._make_lower_plan(phys.q.copy(), q_rest, gfn, speed, 0.5, 1.5, 0.15, "minjerk",
                                   wrist_speed, wrist_lead, t)
        for j in ARM_IDS:                               # the controller's lowering wrist kd
            if j in WR:
                ctrl.msg.motor_cmd[j].kd = max(ctrl.msg.motor_cmd[j].kd, WRIST_KD)
        true_max = np.zeros(14); rep_max = np.zeros(14)
        t_lim = t + 3.0 * lo["T"] + 20.0
        phys.dq_max_since_read()
        while t < t_lim and not ctrl._lower_done.is_set():
            q_cmd, tau = ctrl._lower_step(lo)
            kp_now = np.array([ctrl.msg.motor_cmd[j].kp for j in ARM_IDS])
            kd_now = np.array([ctrl.msg.motor_cmd[j].kd for j in ARM_IDS])
            phys.set_command(q_cmd, np.zeros(14), kp_now, kd_now, tau, t)
            for _ in range(4):
                phys.advance(ctrl.control_dt / 4, t)
            true_max = np.maximum(true_max, phys.dq_max_since_read())
            rep_max = np.maximum(rep_max, np.abs(ctrl.get_current_dual_arm_dq()))
            t += ctrl.control_dt; clock["t"] = t
    finally:
        ra.time.monotonic = real
    ga, gw = lo["groups"]
    return dict(true_max=true_max, rep_max=rep_max, T_arm=ga["T"], T_wr=gw["T"], done_s=t - lo["t0"],
                done=ctrl._lower_done.is_set(), off_rest=float(np.max(np.abs(phys.q - q_rest))),
                gov=(ga["gov_steps"] * ctrl.control_dt, gw["gov_steps"] * ctrl.control_dt))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--out", default=str(Path(__file__).resolve().parent.parent / "out"))
    a = ap.parse_args()
    import pickle
    model = pickle.load(open(XRT / "teleop" / "g1_29_model_cache.pkl", "rb"))["reduced_model"]
    data = model.createData(); z = np.zeros(14)
    gfn = lambda q: pin.rnea(model, data, np.asarray(q, dtype=float), z, z).copy()
    q_rest = load_pose(str(XRT / "poses" / "low.yaml"))
    rows = []
    for fs in [float(x) for x in os.environ.get("FRICTIONS", "1.0").split(",")]:
        fr = None if fs == 0 else (fs, 0.4 * fs)
        for label, ws, wl in (("RT6d plan (wrist 0.18, lead 0.15)", 0.18, 0.15), ("RT6g plan (wrist 0.15, lead 0.05)", 0.15, 0.05)):
            r = run(Q_RT6E, q_rest, gfn, ws, wl, fr)
            wr = [4, 11]; arm = [0, 1, 2, 3, 7, 8, 9, 10]
            print(f"friction Fs {fs:.1f} Nm  {label:34s}: true max wrist roll {r['true_max'][wr].max():.3f}, "
                  f"all wrists {r['true_max'][[4,5,6,11,12,13]].max():.3f}, shoulders/elbows {r['true_max'][arm].max():.3f} rad/s; "
                  f"reported max {r['rep_max'].max():.3f} ({NAMES[int(np.argmax(r['rep_max']))]}); plan arm {r['T_arm']:.1f} s wrist "
                  f"{r['T_wr']:.1f} s; done {r['done_s']:.1f} s; governor {r['gov'][0]:.1f}/{r['gov'][1]:.1f} s; off rest {r['off_rest']:.3f}",
                  flush=True)
            rows.append(dict(friction_Fs_Nm=fs, plan=label, true_max_wrist_roll=round(float(r['true_max'][wr].max()), 3),
                             true_max_wrists=round(float(r['true_max'][[4,5,6,11,12,13]].max()), 3),
                             true_max_arm=round(float(r['true_max'][arm].max()), 3),
                             reported_max=round(float(r['rep_max'].max()), 3), T_arm_s=round(r['T_arm'], 1),
                             T_wrist_s=round(r['T_wr'], 1), done_s=round(r['done_s'], 1), off_rest=round(r['off_rest'], 3)))
    Path(a.out).mkdir(parents=True, exist_ok=True)
    with open(Path(a.out) / "lower_friction_sim.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)


if __name__ == "__main__":
    main()
    sys.stdout.flush()
    os._exit(0)
