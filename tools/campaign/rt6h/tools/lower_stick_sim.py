#!/usr/bin/env python3
"""RT6h item 2: the exit lowering against the simulated arm with stick-slip. CPU only.

Runs the arm controller's own `_make_lower_plan` / `_lower_step` (teleop/robot_control/
robot_arm.py) against tools/arm_physics.py (RT6h constrained stick-slip friction), from
RT6g's [q] pose (arm_watch 19:55:35; wrists not logged: left wrist roll at +1.95 rad, the
1.98 rad wrist gap the RT6g plan's 24.8 s implies) down to poses/low.yaml.

  calibrate  friction grid, the RT6g controller (lead 0.15 / 0.05 rad, kd 3 / 4, no rate
             limit): which friction reproduces RT6g's peaks, 0.60 rad/s left wrist roll and
             0.55 rad/s right shoulder yaw
  compare    RT6g controller against RT6h variants at the calibrated friction (FS_ARM,
             FC_ARM, FS_WR, FC_WR env) and at neighbouring frictions

Speeds are TRUE simulated speeds (max over every substep), not filtered.

    python lower_stick_sim.py calibrate|compare [--out DIR]
"""
import argparse, csv, itertools, os, sys, threading, types
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
WR_IDX = [4, 5, 6, 11, 12, 13]
ARM_IDX = [0, 1, 2, 3, 7, 8, 9, 10]
NAMES = ["L sh pitch", "L sh roll", "L sh yaw", "L elbow", "L wr roll", "L wr pitch", "L wr yaw",
         "R sh pitch", "R sh roll", "R sh yaw", "R elbow", "R wr roll", "R wr pitch", "R wr yaw"]
Q_RT6G = np.array([-0.964, 0.444, 0.444, 2.090, 1.95, 0.0, 0.0,
                   -0.844, -0.177, -2.466, 1.108, 0.0, 0.0, 0.0])
CTRL_DT = 1.0 / 250.0


class FakeCtrl(ra.G1_29_ArmController):
    control_dt = CTRL_DT

    def __init__(self, phys, noise, rng, delay_steps):
        self.phys, self.noise, self.rng = phys, noise, rng
        self.msg = types.SimpleNamespace(motor_cmd=[types.SimpleNamespace(kp=0.0, kd=0.0) for _ in range(35)])
        self._base_kp = {}
        for k, j in enumerate(ra.G1_29_JointIndex):
            wr = j in ARM_IDS and ARM_IDS.index(j) in WR_IDX
            self._base_kp[j] = (40.0 if wr else 80.0) if j in ARM_IDS else 300.0
            self.msg.motor_cmd[j].kp = self._base_kp[j]
            self.msg.motor_cmd[j].kd = 1.5 if wr else 3.0
        self._lower_done = threading.Event()
        self.hist = [(phys.q.copy(), phys.dq.copy())] * (delay_steps + 1)

    def sample(self):
        self.hist = self.hist[1:] + [(self.phys.q.copy(), self.phys.dq.copy())]

    def get_current_dual_arm_q(self):
        return self.hist[0][0].copy()

    def get_current_dual_arm_dq(self):
        return self.hist[0][1] + (self.rng.normal(0.0, self.noise, 14) if self.noise > 0 else 0.0)


def friction_arrays(fs_arm, fc_arm, fs_wr, fc_wr):
    fs = np.zeros(14); fc = np.zeros(14)
    fs[ARM_IDX], fc[ARM_IDX] = fs_arm, fc_arm
    fs[WR_IDX], fc[WR_IDX] = fs_wr, fc_wr
    return fs, fc


def run(q_rest, gfn, fric, arm_lead=0.15, wrist_lead=0.05, arm_kd=3.0, wrist_kd=4.0, cmd_vel=0.0,
        speed=0.3, wrist_speed=0.15, noise=0.015, delay_steps=None, q_start=Q_RT6G):
    if delay_steps is None:
        delay_steps = int(os.environ.get("DELAY", "1"))
    fs, fc = fric
    arm_a = np.full(14, 0.02)
    arm_a[WR_IDX] = float(os.environ.get("WRIST_ARMATURE", "0.02"))
    phys = ArmPhysics(q_start, armature=0.02, viscous=0.05, friction=(fs, fc), friction_joints=None, dt=0.0005)
    phys.model.armature[:] = arm_a
    ctrl = FakeCtrl(phys, noise, np.random.default_rng(0), delay_steps)
    kp = np.array([ctrl._base_kp[j] for j in ARM_IDS])
    kd = np.array([ctrl.msg.motor_cmd[j].kd for j in ARM_IDS])
    t = 0.0
    for _ in range(int(0.3 / CTRL_DT)):                     # hold the start pose 0.3 s
        phys.set_command(q_start, np.zeros(14), kp, kd, gfn(q_start), t)
        phys.advance(CTRL_DT, t); t += CTRL_DT; ctrl.sample()
    clock = {"t": t}
    real = ra.time.monotonic
    ra.time.monotonic = lambda: clock["t"]
    try:
        lo = ctrl._make_lower_plan(ctrl.get_current_dual_arm_q(), q_rest, gfn, speed, 0.5, 1.5, arm_lead, "minjerk",
                                   wrist_speed, wrist_lead, t, cmd_vel=cmd_vel)
        for k, j in enumerate(ARM_IDS):
            ctrl.msg.motor_cmd[j].kd = max(ctrl.msg.motor_cmd[j].kd, wrist_kd if k in WR_IDX else arm_kd)
        true_max = np.zeros(14); over_s = np.zeros(14)
        t_lim = t + 3.0 * lo["T"] + 20.0
        phys.dq_max_since_read()
        while t < t_lim and not ctrl._lower_done.is_set():
            q_cmd, tau = ctrl._lower_step(lo)
            kp_now = np.array([ctrl.msg.motor_cmd[j].kp for j in ARM_IDS])
            kd_now = np.array([ctrl.msg.motor_cmd[j].kd for j in ARM_IDS])
            kd_now[WR_IDX] *= float(os.environ.get("KD_EFF_WR", "1.0"))   # wrist kd as effective as commanded?
            phys.set_command(q_cmd, np.zeros(14), kp_now, kd_now, tau, t)
            phys.advance(CTRL_DT, t)
            m = phys.dq_max_since_read()
            true_max = np.maximum(true_max, m)
            over_s += (m > 0.3) * CTRL_DT
            t += CTRL_DT; clock["t"] = t; ctrl.sample()
    finally:
        ra.time.monotonic = real
    ga, gw = lo["groups"]
    return dict(true_max=true_max, over_s=over_s, T_arm=ga["T"], T_wr=gw["T"], done_s=t - lo["t0"],
                done=ctrl._lower_done.is_set(), off_rest=float(np.max(np.abs(phys.q - q_rest))),
                gov=(ga["gov_steps"] * CTRL_DT, gw["gov_steps"] * CTRL_DT))


def line(tag, r):
    k = int(np.argmax(r["true_max"]))
    return (f"{tag}: max {r['true_max'].max():.3f} rad/s ({NAMES[k]}); L wr roll {r['true_max'][4]:.3f}, "
            f"R sh yaw {r['true_max'][9]:.3f}; joint-seconds over 0.3 {r['over_s'].sum():.2f}; done {r['done_s']:.1f} s "
            f"(plan {r['T_arm']:.1f}/{r['T_wr']:.1f}); gov {r['gov'][0]:.1f}/{r['gov'][1]:.1f} s; off rest {r['off_rest']:.3f} rad")


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("mode", choices=("calibrate", "compare"))
    ap.add_argument("--out", default=str(Path(__file__).resolve().parent.parent / "out"))
    a = ap.parse_args()
    import pickle
    model = pickle.load(open(XRT / "teleop" / "g1_29_model_cache.pkl", "rb"))["reduced_model"]
    data = model.createData(); z = np.zeros(14)
    gfn = lambda q: pin.rnea(model, data, np.asarray(q, dtype=float), z, z).copy()
    q_rest = load_pose(str(XRT / "poses" / "low.yaml"))
    rows = []
    if a.mode == "calibrate":
        if os.environ.get("PAIRS"):
            # "fs_arm:fc_arm:fs_wrist:fc_wrist;..."
            sets = [tuple(float(v) for v in p.split(":")) for p in os.environ["PAIRS"].split(";")]
        else:
            fs_list = [float(x) for x in os.environ.get("FS", "1.0,2.0,3.0").split(",")]
            ratios = [float(x) for x in os.environ.get("RATIOS", "0.3,0.5").split(",")]
            sets = [(fs, rt * fs, fs, rt * fs) for fs, rt in itertools.product(fs_list, ratios)]
        for fsa, fca, fsw, fcw in sets:
            fric = friction_arrays(fsa, fca, fsw, fcw)
            r = run(q_rest, gfn, fric)
            fs, rt = fsa, fca / fsa
            print(line(f"RT6g ctrl, shoulders/elbows Fs {fsa:.2f} Fc {fca:.2f}, wrists Fs {fsw:.2f} Fc {fcw:.2f} Nm", r), flush=True)
            rows.append(dict(mode="calibrate", fs=f"{fsa:g}/{fsw:g}", fc=f"{fca:g}/{fcw:g}", ctrl="RT6g", max=round(float(r["true_max"].max()), 3),
                             L_wr_roll=round(float(r["true_max"][4]), 3), R_sh_yaw=round(float(r["true_max"][9]), 3),
                             over_s=round(float(r["over_s"].sum()), 2), done_s=round(r["done_s"], 1), off_rest=round(r["off_rest"], 3)))
    else:
        base = (float(os.environ.get("FS_ARM", "2.0")), float(os.environ.get("FC_ARM", "0.8")),
                float(os.environ.get("FS_WR", "1.0")), float(os.environ.get("FC_WR", "0.4")))
        variants = [("RT6g", dict()),
                    ("rate only", dict(cmd_vel=0.3)),
                    ("kd only (arms 6)", dict(arm_kd=6.0)),
                    ("RT6h: rate 0.3 + kd arms 6, wrists 4", dict(cmd_vel=0.3, arm_kd=6.0)),
                    ("RT6h: rate 0.2 + kd arms 6, wrists 4", dict(cmd_vel=0.2, arm_kd=6.0)),
                    ("RT6h: rate 0.2 + kd arms 6 wrists 4, lead 0.05", dict(cmd_vel=0.2, arm_kd=6.0, arm_lead=0.05))]
        extra = os.environ.get("VARIANTS")
        if extra:
            variants = eval(extra)
        scales = [float(x) for x in os.environ.get("SCALES", "1.0").split(",")]
        for sc in scales:
            fric = friction_arrays(sc * base[0], sc * base[1], sc * base[2], sc * base[3])
            for tag, kw in variants:
                r = run(q_rest, gfn, fric, **kw)
                print(line(f"friction x{sc:g} | {tag}", r), flush=True)
                rows.append(dict(mode="compare", fs=round(sc * base[0], 3), fc=round(sc * base[1], 3), ctrl=tag,
                                 max=round(float(r["true_max"].max()), 3), L_wr_roll=round(float(r["true_max"][4]), 3),
                                 R_sh_yaw=round(float(r["true_max"][9]), 3), over_s=round(float(r["over_s"].sum()), 2),
                                 done_s=round(r["done_s"], 1), off_rest=round(r["off_rest"], 3)))
    Path(a.out).mkdir(parents=True, exist_ok=True)
    with open(Path(a.out) / f"lower_stick_{a.mode}.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)


if __name__ == "__main__":
    main()
    sys.stdout.flush()
    os._exit(0)
