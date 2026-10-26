#!/usr/bin/env python3
"""G13 (RT6e): wrist at its stop at start, lowering to low.yaml. CPU and DOMAIN 1 / LOOPBACK ONLY.

Owner's rule (22 Oct): a wrist joint (roll, pitch, yaw) at its limit passes the start gate
when the start sequence moves it away from the limit, toward low.yaml; shoulders and
elbows keep the 0.05 rad gate. This run lowers to low.yaml (XR_REST_POSE), not to the
pose measured at start, so the arms end low and released, never back at a stop.

  (A) CPU: start_gate.check with today's pre-check pose (17:57: left wrist roll +1.969,
      0.003 rad from +1.972; right wrist roll -1.760) and the variants that must still
      refuse.
  (B) Fakes: fake_lowstate --follow-cmd starting at today's pose; the launcher in tmux,
      --sim on lo, with the RT6e settings (XR_ARM_VEL_LIMIT=1, XR_START_POSE and
      XR_REST_POSE = low.yaml, XR_SHOULDER_TAU_CAP=6, 1:1 mapping); the fake headset.
      Start passes; every commanded wrist moves away from its stop toward 0 at 1 rad/s or
      less and stays inside its limits; [r]; FOLLOW; [q] lowers to low.yaml at 0.3 rad/s
      or less and releases; exit 0; the released pose is saved.
      Note: the follow-cmd fake has no gravity, so before [r] the controller's start-up
      target (q = 0) moves the whole arm; on the robot a hanging arm barely moves then.

Run:  python tools/overnight/test_g13_wrist_start.py [--cpu-only]
"""
import os, re, signal, subprocess, sys, tempfile, time
from pathlib import Path
import numpy as np

REPO = Path(__file__).resolve().parent.parent.parent
sys.path[:0] = [str(REPO), str(REPO / "tools"), str(REPO / "teleop")]
PY = sys.executable
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


from teleop.robot_control import start_gate as g                        # noqa: E402
from teleop.robot_control.start_pose import ARM_JOINT_NAMES, load_pose  # noqa: E402

# rt6/out/rt6e3_precheck/arm_pose.json (22 Oct 17:57, after the hand adjustment)
Q_TODAY = np.array([0.088, -0.052, 1.64, 1.375, 1.969, -0.027, -0.182,
                    0.034, 0.003, -1.843, 1.39, -1.76, -0.135, 0.161])
Q_LOW = load_pose(str(REPO / "poses" / "low.yaml"))
LO, HI = g.urdf_arm_limits()
K = {n: ARM_JOINT_NAMES.index(n) for n in ARM_JOINT_NAMES}
LWR, RWR = K["left_wrist_roll_joint"], K["right_wrist_roll_joint"]
WRISTS = [k for k in range(14) if "_wrist_" in ARM_JOINT_NAMES[k]]

# ====================================================================== (A) CPU
print("=== (A) CPU: the wrist exception ===", flush=True)
r, n = g.check(Q_TODAY, LO, HI, 0.05, 0.2, None, q_start=Q_LOW, rest_pose_set=True)
record("A: today's pose passes with the start pose and XR_REST_POSE", r is None)
record("A: ...and says why: left wrist roll ALLOWED, moved to +0.000",
       any("left_wrist_roll_joint at +1.969 rad is 0.003 rad from its upper limit: ALLOWED (wrist): "
           "the start pose moves it to +0.000 rad" in x for x in n))
r, n = g.check(Q_TODAY, LO, HI, 0.05, 0.2, None, q_start=Q_LOW, rest_pose_set=False)
record("A: without XR_REST_POSE it refuses, and says to set it",
       r is not None and any("only with XR_REST_POSE" in x for x in n))
r, n = g.check(Q_TODAY, LO, HI, 0.05, 0.2, None)
record("A: without a start pose it refuses", r is not None)
q = Q_TODAY.copy(); q[K["left_shoulder_yaw_joint"]] = 2.604
r, n = g.check(q, LO, HI, 0.05, 0.2, None, q_start=Q_LOW, rest_pose_set=True)
record("A: a shoulder at its limit still refuses (no exception)", r is not None and "left_shoulder_yaw_joint" in r
       and "left_wrist_roll_joint" not in r)
q = Q_TODAY.copy(); q[K["left_elbow_joint"]] = HI[K["left_elbow_joint"]] - 0.01
r, n = g.check(q, LO, HI, 0.05, 0.2, None, q_start=Q_LOW, rest_pose_set=True)
record("A: an elbow at its limit still refuses", r is not None and "left_elbow_joint" in r)
st = Q_LOW.copy(); st[LWR] = 1.95
r, n = g.check(Q_TODAY, LO, HI, 0.05, 0.2, None, q_start=st, rest_pose_set=True)
record("A: a start pose that is itself inside the margin: refuses", r is not None)
st = Q_LOW.copy(); st[LWR] = 1.97
q = Q_TODAY.copy(); q[LWR] = 1.90
r, n = g.check(Q_TODAY, LO, HI, 0.05, 0.2, None, q_start=st, rest_pose_set=True)
record("A: a start pose toward the stop: refuses", r is not None)
q = Q_TODAY.copy(); q[LWR] = -1.981
r, n = g.check(q, LO, HI, 0.05, 0.2, None, q_start=Q_LOW, rest_pose_set=True)
record("A: a wrist PAST its stop (-1.981, the 17:30 reading) moving to 0: allowed", r is None)

if "--cpu-only" in sys.argv:
    summary_and_exit()

# ====================================================================== (B) fakes
print("\n=== (B) fakes: RT6e settings, today's pose ===", flush=True)
from fake_xr_client import FakeXR                                     # noqa: E402
from tools import port_guard                                          # noqa: E402
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber  # noqa: E402
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_  # noqa: E402
ChannelFactoryInitialize(1, "lo")
cmd = []
st = []        # the follow-cmd fake reports each command as the arm q: continuous from before launch
def on_state(m):
    st.append((time.monotonic(), np.array([m.motor_state[i].q for i in range(15, 29)])))
_s2 = ChannelSubscriber("rt/lowstate", LowState_); _s2.Init(on_state, 10)
def on_cmd(m):
    cmd.append((time.monotonic(), np.array([m.motor_cmd[i].q for i in range(15, 29)]),
                float(m.motor_cmd[15].kp), float(m.motor_cmd[19].kp)))
_s = ChannelSubscriber("rt/lowcmd", LowCmd_); _s.Init(on_cmd, 10)
tmp = Path(tempfile.mkdtemp(prefix="g13_"))
import yaml                                                           # noqa: E402
pose_today = tmp / "today.yaml"
yaml.safe_dump({"name": "today", "robot": "G1_29", "units": "radians",
                "joints": {n: float(v) for n, v in zip(ARM_JOINT_NAMES, Q_TODAY)}}, open(pose_today, "w"))
last = tmp / "last_release.yaml"
ENV = {"PYTHONUNBUFFERED": "1", "PYTHONNOUSERSITE": "1", "PYTHONDONTWRITEBYTECODE": "1",
       "XR_ARM_VEL_LIMIT": "1", "XR_START_POSE": str(REPO / "poses/low.yaml"),
       "XR_REST_POSE": str(REPO / "poses/low.yaml"), "XR_SHOULDER_TAU_CAP": "6",
       "XR_LAST_POSE_FILE": str(last), "XR_SIM_VEL_CLIP": "1"}   # the robot's velocity clip, in --sim


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


def speeds(seg, joints, win=0.25):
    """Max speed per joint over 0.25 s windows. Samples are timed on arrival, and DDS
    delivers them in bursts: short windows read a burst as speed (G11 saw the same)."""
    t = np.array([c[0] for c in seg]); q = np.array([c[1] for c in seg])
    out = np.zeros(14); j = 0
    for i in range(len(t)):
        while t[i] - t[j] > win:
            j += 1
        if i > j and t[i] - t[j] > 0.8 * win:
            out = np.maximum(out, np.abs(q[i] - q[j]) / (t[i] - t[j]))
    return out[joints]


fx = subprocess.Popen([PY, str(REPO / "tools/fake_lowstate.py"), "--domain", "1", "--iface", "lo", "--rate", "500",
                       "--follow-cmd", "--pose", str(pose_today)], env=dict(os.environ, **ENV),
                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
time.sleep(2)
pane = tmp / "pane.log"
lcmd = (f"cd {REPO / 'teleop'} && {PY} teleop_hand_and_arm.py --sim --network-interface lo; "
        f"echo \"[g13] launcher exited with $?\"; sleep 15")
env_str = " ".join(f"{k}={v}" for k, v in ENV.items())
cmd.clear(); st.clear(); t_launch = time.monotonic()
try:
    subprocess.run(["tmux", "new", "-d", "-s", "g13", "-x", "220", "-y", "50", f"env {env_str} bash -c '{lcmd}'"], check=True)
    subprocess.run(["tmux", "pipe-pane", "-o", "-t", "g13", f"cat >> {pane}"], check=True)
    xr = FakeXR().start()
    up = wait_file(pane, r"attach check", 120)
    text = pane.read_text(errors="replace")
    record("B: launcher up with today's pose", bool(up))
    record("B: start gate: left wrist roll ALLOWED, the rest clear",
           "left_wrist_roll_joint at +1.969 rad is 0.003 rad from its upper limit: ALLOWED (wrist)" in text
           and "arm joints clear of their limits" in text and "REFUSING" not in text)
    record("B: cap at 6 Nm", "[cap] shoulder pitch gravity torque capped at 6 Nm" in text)
    record("B: lowering goes to XR_REST_POSE", "[lower] at exit: arms lowered to the XR_REST_POSE yaml" in text)
    time.sleep(3)
    t_r = time.monotonic()
    subprocess.run(["tmux", "send-keys", "-t", "g13", "r"])
    tr = wait_file(pane, r"\[link\] tracking", 15)
    record("B: [r] -> tracking", bool(tr))
    time.sleep(12)                                  # approach 3 s, blend 2 s, then FOLLOW
    t_q = time.monotonic()
    subprocess.run(["tmux", "send-keys", "-t", "g13", "q"])
    done = wait_file(pane, r"\[g13\] launcher exited with (-?\d+)", 90)
    xr.stop()
    text = pane.read_text(errors="replace")
    record("B: exit 0", bool(done) and done.group(1) == "0", done.group(0) if done else "no exit line")
    pre = [c for c in st if c[0] < t_q]           # arm q as the fake reports it (= the commands)
    lw = np.array([c[1][LWR] for c in pre]); rw = np.array([c[1][RWR] for c in pre])
    record("B: the fake's arm started at today's pose", abs(pre[0][1][LWR] - Q_TODAY[LWR]) < 1e-3
           and abs(pre[0][1][RWR] - Q_TODAY[RWR]) < 1e-3, f"{pre[0][1][LWR]:+.3f} / {pre[0][1][RWR]:+.3f}")
    record("B: left wrist roll never closer to its stop than at start", lw.max() <= Q_TODAY[LWR] + 1e-3,
           f"start +1.969, max commanded {lw.max():+.3f}")
    record("B: right wrist roll never closer to its stop than at start", rw.min() >= Q_TODAY[RWR] - 1e-3,
           f"start -1.760, min commanded {rw.min():+.3f}")
    end_appr = [c for c in pre if c[0] >= t_r + 2.9]
    record("B: both wrist rolls at the start pose's 0 (|q| < 0.05) by the end of the approach",
           bool(end_appr) and abs(end_appr[0][1][LWR]) < 0.05 and abs(end_appr[0][1][RWR]) < 0.05,
           f"at [r] + 2.9 s {end_appr[0][1][LWR]:+.3f} / {end_appr[0][1][RWR]:+.3f}" if end_appr else "none")
    lw_moves = np.diff(lw)
    _lw_pre = np.array([c[1][LWR] for c in pre if c[0] < t_r])
    np.savetxt(tmp / "lwr_before_r.csv", np.array([[c[0] - t_launch, c[1][LWR], c[1][RWR]] for c in pre]), delimiter=",")
    _back = float(np.max(np.maximum.accumulate(-_lw_pre) + _lw_pre)) if len(_lw_pre) else 9.9
    record("B: left wrist roll moves down, away from +1.972, until [r] (no step back over 0.01 rad)",
           _back < 0.01, f"largest step back {_back:.4f} rad; trace {tmp / 'lwr_before_r.csv'}")
    v = speeds(pre, WRISTS)
    record("B: every wrist speed <= 1 rad/s, launch to [q]", v.max() <= 1.05,
           f"max {v.max():.3f} rad/s (0.25 s windows, as reported by the fake)")
    allq = np.array([c[1] for c in cmd])
    inside = np.all(allq >= LO - 1e-3) and np.all(allq <= HI + 1e-3)
    record("B: every commanded joint inside its limits", bool(inside))
    qa = np.array([c[1] for c in pre])
    step = float(np.max(np.abs(np.diff(qa, axis=0)))) if len(qa) > 1 else 0.0
    record("B: no single step over 0.02 rad between the fake's samples (2 ms apart; 1 rad/s is 0.004 rad per "
           "4 ms command), launch to [q]: the controller is at 1 rad/s from its first command",
           step <= 0.02, f"largest step {step:.4f} rad")
    low_seg = [c for c in cmd if c[0] >= t_q]
    final = low_seg[-1][1] if low_seg else np.zeros(14)
    record("B: [q] lowers to low.yaml (all 14 joints)", float(np.max(np.abs(final - Q_LOW))) < 2e-3,
           f"max |q - low| {float(np.max(np.abs(final - Q_LOW))):.4f} rad")
    vl = speeds(low_seg, list(range(14)))
    record("B: lowering command speed <= 0.3 rad/s", vl.max() <= 0.3,
           f"max {vl.max():.3f} rad/s")
    record("B: arm and wrist kp released to 0", bool(low_seg) and low_seg[-1][2] == 0.0 and low_seg[-1][3] == 0.0)
    back = g.load_last(str(last))
    record("B: released pose saved, equal to low.yaml", back is not None and np.allclose(back[0], Q_LOW, atol=2e-3)
           and "[gate] released pose saved" in text, f"released_at {back[1] if back else '-'}")
    record("8012 free at the end", port_guard.port_is_free(8012))
finally:
    subprocess.run(["tmux", "kill-session", "-t", "g13"], capture_output=True)
    stop(fx)

summary_and_exit()
