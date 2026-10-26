#!/usr/bin/env python3
"""G14 (RT6g): sliding shoulder torque cap, IK joint margin, clip counter. CPU and DOMAIN 1 / LOOPBACK ONLY.

  (A) CPU, the real IK, cap 7 Nm: an operator who reaches past the cap and then sweeps
      sideways. The arm keeps following sideways along the limit (commanded hand moves
      at least 70 % of the IK hand's sideways travel), the commanded shoulder torque
      stays AT the cap, never over; coming back under, the command catches up at
      XR_CAP_CATCHUP_VEL (no jump), then the IK solution passes through untouched.
  (B) CPU, the IK joint margin (XR_IK_JOINT_MARGIN 0.1): today's poses, the RT6e [q]
      configuration (shoulder yaws +2.607 / -2.616 rad) and the RT6e pre-check pose (left
      wrist roll +1.969), and a grid of 1:1 operator poses: every IK solution keeps every
      arm joint at least 0.1 rad inside its limit. With the margin at 0 the RT6e
      configuration stays near the yaw limit (the margin is what changes it).
  (C) Fakes: launcher --sim on lo, follow-cmd fake with the robot's clip (XR_SIM_VEL_CLIP=1),
      cap 7 Nm, the fake headset over the cap and sweeping sideways: commanded shoulder
      torque at the cap, the arm follows sideways, a slow return under the cap has no jump;
      the stats file counts capped frames, clipped control steps per arm, margin clips.

Run:  python tools/overnight/test_g14_slide_margin.py [--cpu-only]
"""
import os, re, signal, subprocess, sys, tempfile, time
from pathlib import Path
import numpy as np

REPO = Path(__file__).resolve().parent.parent.parent
sys.path[:0] = [str(REPO), str(REPO / "tools"), str(REPO / "teleop" / "televuer" / "src")]
PY = sys.executable
CAP = 7.0
results = []


def record(name, ok, detail=""):
    results.append((name, bool(ok), detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  -- {detail}" if detail else ""), flush=True)


def summary_and_exit():
    print("\n=== summary ===")
    n_fail = sum(1 for _, ok, _ in results if not ok)
    for name, ok, d in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"  -- {d}" if d else ""))
    print(f"\n{len(results) - n_fail}/{len(results)} passed", flush=True)
    from tools._procs import reap_child_processes
    reap_child_processes()
    os._exit(1 if n_fail else 0)


cwd0 = os.getcwd()
os.chdir(REPO / "teleop")
from televuer.tv_wrapper import (T_ROBOT_OPENXR, T_OPENXR_ROBOT, T_TO_UNITREE_HUMANOID_LEFT_ARM,  # noqa: E402
                                 T_TO_UNITREE_HUMANOID_RIGHT_ARM,
                                 transform_IPunitree_Brobot_world_arm_to_head_then_waist)
from teleop.robot_control.robot_arm_ik import G1_29_ArmIK                    # noqa: E402
from teleop.robot_control import torque_cap as tc                            # noqa: E402
import pinocchio as pin                                                       # noqa: E402

HEAD = np.eye(4); HEAD[1, 3] = 1.5


def rot(axis, deg):
    a = np.radians(deg); c, s_ = np.cos(a), np.sin(a); R = np.eye(4)
    if axis == "z": R[:2, :2] = [[c, -s_], [s_, c]]
    if axis == "x": R[1:3, 1:3] = [[c, -s_], [s_, c]]
    if axis == "y": R[0, 0] = c; R[0, 2] = s_; R[2, 0] = -s_; R[2, 2] = c
    return R


def hand_rot(side, zr=0.0, xr_=0.0, yr=0.0):
    """OpenXR wrist rotation, mirrored for the right hand."""
    return rot("z", zr * side) @ rot("x", xr_) @ rot("y", yr * side)


def waist(fwd, out, up, side, roll=0.0, R=None):
    m = hand_rot(side, zr=roll) if R is None else R(side)
    m = m.copy()
    m[:3, 3] = [-out * side, 1.5 + up, -fwd]
    arm = T_ROBOT_OPENXR @ m @ T_OPENXR_ROBOT
    arm = arm @ (T_TO_UNITREE_HUMANOID_LEFT_ARM if side > 0 else T_TO_UNITREE_HUMANOID_RIGHT_ARM)
    return transform_IPunitree_Brobot_world_arm_to_head_then_waist(arm, T_ROBOT_OPENXR @ HEAD @ T_OPENXR_ROBOT, "head_yaw")


# ====================================================================== (A)
print("=== (A) CPU: sliding cap at 7 Nm on the real IK ===", flush=True)
os.environ.pop("XR_IK_JOINT_MARGIN", None)
ik = G1_29_ArmIK()
model = ik.reduced_robot.model; data = model.createData()
g = tc.gravity_fn_for(model)
cap = tc.ShoulderTorqueCap(CAP, g, frequency=30.0)
qm = np.zeros(14); cap.reset(qm)
seq = [("reach", (f, 0.18, -0.35)) for f in np.arange(0.05, 0.42, 0.01)]
seq += [("sweep", (0.42, y, -0.35)) for y in np.arange(0.18, 0.40, 0.004)]
seq += [("sweep", (0.42, y, -0.35)) for y in np.arange(0.40, 0.10, -0.004)]
seq += [("back", (f, 0.18, -0.35)) for f in np.arange(0.42, 0.05, -0.005)]
seq += [("settle", (0.05, 0.18, -0.35))] * 90
fr, times = [], []
for ph, (f, y, u) in seq:
    sq, st = ik.solve_ik(waist(f, y, u, +1), waist(f, y, u, -1), qm.copy(), np.zeros(14))
    rec = list(cap.recovering)
    t0 = time.perf_counter(); cq, ct = cap.step(sq, st, qm); times.append(time.perf_counter() - t0)
    pin.framesForwardKinematics(model, data, np.asarray(cq)); hc = data.oMf[ik.L_hand_id].translation.copy()
    pin.framesForwardKinematics(model, data, np.asarray(sq)); hi = data.oMf[ik.L_hand_id].translation.copy()
    fr.append(dict(ph=ph, y=y, cq=np.array(cq), sq=np.array(sq), ct=np.array(ct), st=np.array(st), hc=hc, hi=hi,
                   capped=list(cap.capped), rec=rec, same=(cq is sq and ct is st)))
    qm = np.array(cq)
ctm = max(max(abs(x["ct"][0]), abs(x["ct"][7])) for x in fr)
record("A: commanded shoulder pitch torque never over 7 Nm", ctm <= CAP + 1e-3, f"max {ctm:.3f} Nm")
sw = [x for x in fr if x["ph"] == "sweep"]
record("A: the operator's reach is over the cap during the sweep", min(abs(x["st"][0]) for x in sw) > CAP,
       f"IK asks {min(abs(x['st'][0]) for x in sw):.2f} to {max(abs(x['st'][0]) for x in sw):.2f} Nm")
ys_c = [x["hc"][1] for x in sw]; ys_i = [x["hi"][1] for x in sw]
ratio = (max(ys_c) - min(ys_c)) / max(1e-6, (max(ys_i) - min(ys_i)))
record("A: the arm follows sideways along the limit (commanded hand y travel >= 70 % of the IK's)", ratio >= 0.7,
       f"commanded {min(ys_c):+.3f} to {max(ys_c):+.3f} m, IK {min(ys_i):+.3f} to {max(ys_i):+.3f} m ({100 * ratio:.0f} %)")
tsw = [abs(x["ct"][0]) for x in sw]
record("A: ...with the commanded torque at the cap (>= 6.9 Nm) the whole sweep", min(tsw) >= 6.9,
       f"{min(tsw):.2f} to {max(tsw):.2f} Nm")
over = [float(np.max(np.abs(fr[k]["cq"] - fr[k - 1]["cq"]))) - float(np.max(np.abs(fr[k]["sq"] - fr[k - 1]["sq"])))
        for k in range(1, len(fr)) if fr[k]["rec"][0] or fr[k]["capped"][0]]
record("A: while capped or catching up, no frame steps further than the IK's own step plus the catch-up step",
       max(over) <= cap.catchup_step + 1e-6, f"largest excess over the IK's own step {max(over):+.4f} rad (catch-up {cap.catchup_step:.4f})")
bk = [x for x in fr if x["ph"] in ("back", "settle")]
rec_steps = [float(np.max(np.abs(bk[k]["cq"] - bk[k - 1]["cq"]))) for k in range(1, len(bk)) if bk[k]["rec"][0] and not bk[k]["capped"][0]]
record("A: coming back under, the catch-up is limited to 0.5 rad/s (no jump)",
       bool(rec_steps) and max(rec_steps) <= cap.catchup_step + 1e-6,
       f"{len(rec_steps)} catch-up frames, max {max(rec_steps) if rec_steps else 0:.4f} rad per frame")
record("A: caught up: the IK solution passes through untouched again", all(x["same"] for x in fr[-5:]))
tp = np.array(times) * 1e3
record("A: cap time per loop", np.percentile(tp, 99) < 3.0, f"p50 {np.percentile(tp, 50):.2f} ms, p99 {np.percentile(tp, 99):.2f} ms")

# ====================================================================== (B)
print("\n=== (B) CPU: IK joint margin 0.1 rad ===", flush=True)
lo, hi = model.lowerPositionLimit, model.upperPositionLimit
record("B: the IK's bounds are 0.1 rad inside the URDF limits", np.allclose(ik.ik_lower, lo + 0.1) and np.allclose(ik.ik_upper, hi - 0.1))


def inside(q):
    return float(min(np.min(q - lo), np.min(hi - q)))


Q_RT6E = np.array([-0.5444, 0.2970, 2.6071, 0.8375, 0.0, 0.0, 0.0,
                   -0.4650, -0.0373, -2.6159, 0.7305, 0.0, 0.0, 0.0])
pin.framesForwardKinematics(model, data, Q_RT6E)
tl, tr = data.oMf[ik.L_hand_id].homogeneous.copy(), data.oMf[ik.R_hand_id].homogeneous.copy()
q = Q_RT6E.copy()
for _ in range(40):
    q, _t = ik.solve_ik(tl, tr, q, np.zeros(14)); q = np.array(q)
record("B: RT6e [q] configuration (yaws +2.607 / -2.616): IK solution >= 0.1 rad inside every limit",
       inside(q) >= 0.1 - 1e-4, f"closest {inside(q):.3f} rad; shoulder yaws {q[2]:+.3f} / {q[9]:+.3f}")
Q_PRE = np.array([0.088, -0.052, 1.64, 1.375, 1.969, -0.027, -0.182, 0.034, 0.003, -1.843, 1.39, -1.76, -0.135, 0.161])
q = Q_PRE.copy()
for _ in range(40):
    q, _t = ik.solve_ik(waist(0.10, 0.18, -0.45, +1), waist(0.10, 0.18, -0.45, -1), q, np.zeros(14)); q = np.array(q)
record("B: RT6e pre-check pose as the warm start (left wrist roll +1.969): solution >= 0.1 rad inside",
       inside(q) >= 0.1 - 1e-4, f"closest {inside(q):.3f} rad; wrist rolls {q[4]:+.3f} / {q[11]:+.3f}")
def grid(the_ik):
    """162 1:1 operator poses: 54 hand orientations x 3 positions, solved in sequence as an
    operator would move (the IK's smoothing filter carries over, as on the robot).
    Returns (closest distance to any limit, poses closer than 0.1 rad, worst yaws)."""
    worst, n_near, wy = 9.9, 0, (0.0, 0.0)
    for zr in (-150, -90, 0, 90, 150, 180):
        for xr_ in (-60, 0, 60):
            for yr in (-60, 0, 60):
                R = (lambda zr, xr_, yr: (lambda side: hand_rot(side, zr, xr_, yr)))(zr, xr_, yr)
                q = np.zeros(14)
                for fwd, up in ((0.30, -0.35), (0.45, -0.30), (0.20, -0.45)):
                    for _ in range(15):
                        q, _t = the_ik.solve_ik(waist(fwd, 0.18, up, +1, R=R), waist(fwd, 0.18, up, -1, R=R), q, np.zeros(14))
                        q = np.array(q)
                    d = inside(q)
                    n_near += int(d < 0.1)
                    if d < worst:
                        worst, wy = d, (q[2], q[9])
    return worst, n_near, wy


w1, n1, y1 = grid(ik)
record("B: 162 operator poses (54 hand orientations x 3 positions): every solution >= 0.1 rad inside",
       w1 >= 0.1 - 1e-4, f"closest {w1:.3f} rad, {n1} closer than 0.1")
os.environ["XR_IK_JOINT_MARGIN"] = "0"
ik0 = G1_29_ArmIK()
os.environ.pop("XR_IK_JOINT_MARGIN", None)
w0, n0, y0 = grid(ik0)
record("B: control, margin 0, the same 162 poses: some solutions reach a limit (as the yaws did in RT6e)",
       n0 > 0 and w0 < 0.1, f"{n0} of 162 closer than 0.1 rad, closest {w0:.3f} rad (shoulder yaws {y0[0]:+.3f} / {y0[1]:+.3f})")
os.chdir(cwd0)

if "--cpu-only" in sys.argv:
    summary_and_exit()

# ====================================================================== (C)
print("\n=== (C) fakes: sliding cap, clip counter (domain 1, lo) ===", flush=True)
from fake_xr_client import FakeXR                                     # noqa: E402
from tools import port_guard                                          # noqa: E402
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber  # noqa: E402
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_            # noqa: E402
ChannelFactoryInitialize(1, "lo")
cmd = []
def on_cmd(m):
    cmd.append((time.monotonic(), np.array([m.motor_cmd[i].q for i in range(15, 29)]),
                float(m.motor_cmd[15].tau), float(m.motor_cmd[22].tau)))
_s = ChannelSubscriber("rt/lowcmd", LowCmd_); _s.Init(on_cmd, 10)
tmp = Path(tempfile.mkdtemp(prefix="g14_"))
ENV = {"PYTHONUNBUFFERED": "1", "PYTHONNOUSERSITE": "1", "PYTHONDONTWRITEBYTECODE": "1", "XR_ARM_VEL_LIMIT": "1",
       "XR_SHOULDER_TAU_CAP": str(CAP), "XR_SIM_VEL_CLIP": "1"}


def wait_file(path, pattern, timeout):
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        try:
            m = re.search(pattern, Path(path).read_text(errors="replace"))
            if m:
                return m
        except FileNotFoundError:
            pass
        time.sleep(0.2)
    return None


def stop(p):
    try:
        os.killpg(p.pid, signal.SIGTERM); p.wait(timeout=5)
    except Exception:
        try: os.killpg(p.pid, signal.SIGKILL)
        except Exception: pass


fx = subprocess.Popen([PY, str(REPO / "tools/fake_lowstate.py"), "--domain", "1", "--iface", "lo", "--rate", "500",
                       "--follow-cmd", "--pose", str(REPO / "poses/low.yaml")], env=dict(os.environ, **ENV),
                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
time.sleep(2)
pane = tmp / "pane.log"
lcmd = (f"cd {REPO / 'teleop'} && {PY} teleop_hand_and_arm.py --sim --network-interface lo; "
        f"echo \"[g14] launcher exited with $?\"; sleep 15")
env_str = " ".join(f"{k}={v}" for k, v in ENV.items())
t_start = time.time()
try:
    subprocess.run(["tmux", "new", "-d", "-s", "g14", "-x", "220", "-y", "50", f"env {env_str} bash -c '{lcmd}'"], check=True)
    subprocess.run(["tmux", "pipe-pane", "-o", "-t", "g14", f"cat >> {pane}"], check=True)
    xr = FakeXR(radius=0.01).start()
    up = wait_file(pane, r"attach check", 120)
    text = pane.read_text(errors="replace")
    record("C: launcher up; cap line says sliding at 7 Nm", bool(up) and "capped at 7 Nm" in text and "sliding" in text)
    record("C: joint margin line", "[ik] every arm joint kept 0.1 rad inside its limit" in text)
    time.sleep(2)
    subprocess.run(["tmux", "send-keys", "-t", "g14", "r"])
    record("C: [r] -> tracking", bool(wait_file(pane, r"\[link\] tracking", 15)))
    time.sleep(10)                                    # fake default pose: about 8 Nm, over the cap
    t_sw = time.monotonic()
    for k in range(60):                               # sweep sideways 0.15 m out and back, 6 s
        x = 0.15 * np.sin(np.pi * k / 30.0)
        xr.set_offset([-x, 0.0, 0.0]); time.sleep(0.1)
    t_sw_end = time.monotonic()
    seg = [c for c in cmd if t_sw <= c[0] <= t_sw_end]
    taus = np.array([max(abs(c[2]), abs(c[3])) for c in seg])
    roll = np.array([c[1][1] for c in seg]); yaw = np.array([c[1][2] for c in seg])
    record("C: during the sweep the commanded shoulder torque stays at the cap (6.9 to 7.0 Nm)",
           len(taus) and taus.max() <= CAP + 0.01 and np.percentile(taus, 10) >= 6.9,
           f"{np.percentile(taus, 10):.2f} to {taus.max():.3f} Nm" if len(taus) else "none")
    record("C: ...and the left arm follows sideways (shoulder roll or yaw command travels > 0.1 rad)",
           len(seg) and max(np.ptp(roll), np.ptp(yaw)) > 0.1, f"roll {np.ptp(roll):.3f} rad, yaw {np.ptp(yaw):.3f} rad")
    t_bk = time.monotonic()
    for k in range(40):                               # slow return under the cap, 4 s
        s = (k + 1) / 40.0
        xr.set_offset([0.0, -0.10 * s, 0.25 * s]); time.sleep(0.1)
    time.sleep(4)
    t_bk_end = time.monotonic()
    segb = [c for c in cmd if t_bk <= c[0] <= t_bk_end]
    qb = np.array([c[1] for c in segb])
    jumps = np.max(np.abs(np.diff(qb, axis=0)), axis=1) if len(qb) > 1 else np.zeros(1)
    tb = np.array([max(abs(c[2]), abs(c[3])) for c in segb])
    record("C: back under the cap: no jump (largest step between commands <= 0.02 rad)", jumps.max() <= 0.02,
           f"largest step {jumps.max():.4f} rad; torque at the end {tb[-1]:.2f} Nm")
    subprocess.run(["tmux", "send-keys", "-t", "g14", "q"])
    done = wait_file(pane, r"\[g14\] launcher exited with (-?\d+)", 90)
    xr.stop()
    record("C: exit 0", bool(done) and done.group(1) == "0", done.group(0) if done else "no exit line")
    files = sorted((REPO / "logs").glob("launcher_stats_*.csv"), key=lambda p: p.stat().st_mtime)
    files = [f for f in files if f.stat().st_mtime >= t_start]
    lines = files[-1].read_text().strip().splitlines() if files else []
    hdr = lines[0].split(",") if lines else []
    rows = [dict(zip(hdr, l.split(","))) for l in lines[1:]]
    need = ["ctrl_steps", "clip_steps", "clip_steps_L", "clip_steps_R", "margin_clips", "cap_frames_L"]
    record("C: stats file has the clip and margin columns", all(c in hdr for c in need), ", ".join(c for c in need if c not in hdr))
    cf = sum(int(r.get("cap_frames_L") or 0) for r in rows); cs = sum(int(r.get("ctrl_steps") or 0) for r in rows)
    cl = sum(int(r.get("clip_steps_L") or 0) for r in rows); cr = sum(int(r.get("clip_steps_R") or 0) for r in rows)
    record("C: capped frames and clipped control steps counted", cf > 0 and cs > 0,
           f"cap frames L {cf}; control steps {cs}, clipped L {cl} R {cr}")
    record("8012 free at the end", port_guard.port_is_free(8012))
finally:
    subprocess.run(["tmux", "kill-session", "-t", "g14"], capture_output=True)
    stop(fx)

summary_and_exit()
