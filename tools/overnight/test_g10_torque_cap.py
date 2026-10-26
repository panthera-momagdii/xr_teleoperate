#!/usr/bin/env python3
"""G10 (RT6d): shoulder pitch torque cap and reach mapping. CPU and DOMAIN 1 / LOOPBACK ONLY.

  (A) CPU, no DDS. The real IK (G1_29_ArmIK) on synthetic operator wrists, through
      televuer's own transform. An operator who walks the hands from the chest out to
      arms-straight-forward: every commanded shoulder pitch feed-forward stays at or under
      the limit, the cap reaches the limit (does not stop short), the commanded path is
      continuous, it never pulls the arm back past where it was, it lets go exactly when
      the operator comes back, it caps one arm without touching the other, and it costs
      little time.
  (B) End to end on fakes: launcher --sim on lo, fake_lowstate --follow-cmd, the fake
      headset. Cap on at the default (5 Nm since RT6h) with the hands far forward: rt/lowcmd tau on motors 15 and
      22 stays at or under the default limit and the [cap] notices appear; then the hands come back and
      it follows again. Cap off (XR_SHOULDER_TAU_CAP=0): the same pose commands over 6 Nm.
      Mapping XY 0.7 / X -0.05 with the hands at the chest: the live launcher commands
      the torque the offline table predicts.

Run:  python tools/overnight/test_g10_torque_cap.py [--cpu-only]
"""
import os, re, signal, subprocess, sys, tempfile, time
from pathlib import Path
import numpy as np

REPO = Path(__file__).resolve().parent.parent.parent
sys.path[:0] = [str(REPO), str(REPO / "tools"), str(REPO / "teleop" / "televuer" / "src")]
PY = sys.executable
LIMIT = 4.0                 # the CPU part (A) runs at 4 Nm
DEF = 5.0                   # [RT6h] the default limit; the fakes part (B) runs at it
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


# ====================================================================== (A) CPU
print("=== (A) CPU: the cap on the real IK ===", flush=True)
cwd0 = os.getcwd()
os.chdir(REPO / "teleop")                      # the IK loads its URDF/cache relative to here
from televuer.tv_wrapper import (T_ROBOT_OPENXR, T_OPENXR_ROBOT, T_TO_UNITREE_HUMANOID_LEFT_ARM,  # noqa: E402
                                 T_TO_UNITREE_HUMANOID_RIGHT_ARM,
                                 transform_IPunitree_Brobot_world_arm_to_head_then_waist)
from teleop.robot_control.robot_arm_ik import G1_29_ArmIK                    # noqa: E402
from teleop.robot_control import torque_cap as tc                            # noqa: E402
from teleop.robot_control.wrist_offset import WristMapping                   # noqa: E402

HEAD = np.eye(4); HEAD[1, 3] = 1.5


def waist(fwd, out, up, side):
    m = np.eye(4); m[:3, 3] = [-out * side, 1.5 + up, -fwd]
    arm = T_ROBOT_OPENXR @ m @ T_OPENXR_ROBOT
    arm = arm @ (T_TO_UNITREE_HUMANOID_LEFT_ARM if side > 0 else T_TO_UNITREE_HUMANOID_RIGHT_ARM)
    return transform_IPunitree_Brobot_world_arm_to_head_then_waist(arm, T_ROBOT_OPENXR @ HEAD @ T_OPENXR_ROBOT, "head_yaw")


# env parsing
os.environ.pop("XR_SHOULDER_TAU_CAP", None)
record("A: default limit 5 Nm (RT6h)", tc.limit_from_env() == DEF)
os.environ["XR_SHOULDER_TAU_CAP"] = "0"
record("A: XR_SHOULDER_TAU_CAP=0 parses as off", tc.limit_from_env() == 0.0)
bad = []
for v in ("-1", "abc", "nan"):
    os.environ["XR_SHOULDER_TAU_CAP"] = v
    try:
        tc.limit_from_env(); bad.append(v)
    except tc.TorqueCapError:
        pass
os.environ.pop("XR_SHOULDER_TAU_CAP", None)
record("A: negative, non-number and nan refused", not bad, f"accepted: {bad}" if bad else "")

ik = G1_29_ArmIK()
g = tc.gravity_fn_for(ik.reduced_robot.model)
off = tc.ShoulderTorqueCap(0.0, g)
qa, ta = np.zeros(14), np.ones(14)
qo, to = off.step(qa, ta)
record("A: cap off returns the same objects", qo is qa and to is ta)

cap = tc.ShoulderTorqueCap(LIMIT, g)
q_meas = np.zeros(14)
cap.reset(q_meas)
frames, times = [], []
path = [(f, -0.40) for f in np.arange(0.05, 0.50, 0.01)]          # chest height, out to the front
path += [(0.48, u) for u in np.arange(-0.40, -0.20, 0.01)]         # raise the arms a little
path += [(f, -0.40) for f in np.arange(0.48, 0.04, -0.01)]         # and back to the chest
path += [(0.05, -0.40)] * 40                                       # [RT6g] stay: the catch-up finishes
for f, u in path:
    lw, rw = waist(f, 0.18, u, +1), waist(f, 0.18, u, -1)
    for _ in range(3):
        sq, st = ik.solve_ik(lw, rw, q_meas.copy(), np.zeros(14))
        t0 = time.perf_counter()
        cq, ct = cap.step(sq, st, q_meas)
        times.append(time.perf_counter() - t0)
        frames.append((f, u, np.array(sq), np.array(st), np.array(cq), np.array(ct), cq is sq and ct is st,
                       list(cap.capped)))
        q_meas = np.array(cq)                                       # the arm follows the command
pred = np.array([[abs(fr[3][0]), abs(fr[3][7])] for fr in frames])
cmd = np.array([[abs(fr[5][0]), abs(fr[5][7])] for fr in frames])
capped_any = np.array([fr[7][0] or fr[7][1] for fr in frames])
record("A: the operator path goes well over the limit (uncapped IK torque)", pred.max() > 7.0,
       f"max predicted {pred.max():.2f} Nm")
record("A: every commanded shoulder pitch torque <= limit", cmd.max() <= LIMIT + 1e-3,
       f"max commanded {cmd.max():.3f} Nm (limit {LIMIT})")
at_lim = cmd[capped_any]
record("A: when capped, the reach stops AT the limit (not short)", len(at_lim) and at_lim.max() >= 0.97 * LIMIT,
       f"capped frames {int(capped_any.sum())}, commanded {at_lim.min():.2f} to {at_lim.max():.2f} Nm")
gt = np.array([abs(g(fr[4])[0] - fr[5][0]) + abs(g(fr[4])[7] - fr[5][7]) for fr in frames])
record("A: the feed-forward sent is the gravity torque of the pose sent", gt.max() < 1e-9, f"max diff {gt.max():.2e} Nm")
steps_cmd = np.array([np.max(np.abs(frames[k][4] - frames[k - 1][4])) for k in range(1, len(frames))])
steps_ik = np.array([np.max(np.abs(frames[k][2] - frames[k - 1][2])) for k in range(1, len(frames))])
record("A: commanded path continuous (no step larger than the IK's own)", steps_cmd.max() <= steps_ik.max() + 1e-6,
       f"max step cmd {steps_cmd.max():.4f} rad, IK {steps_ik.max():.4f} rad")
back = [fr for fr in frames if fr[0] <= 0.10 and fr[1] == -0.40 and fr is not frames[0]]
tail = back[-6:]
record("A: back at the chest the IK solution passes untouched (same objects)", all(fr[6] for fr in tail),
       f"{sum(fr[6] for fr in tail)}/{len(tail)} frames")
tp = np.array(times) * 1e3
record("A: cap time per loop small", np.percentile(tp, 99) < 3.0,
       f"p50 {np.percentile(tp, 50):.3f} ms, p99 {np.percentile(tp, 99):.3f} ms, max {tp.max():.3f} ms")
# one arm only: left at the chest, right reaching
cap1 = tc.ShoulderTorqueCap(LIMIT, g); qm = np.zeros(14); cap1.reset(qm)
ok_left, capped_right = True, 0
for f in np.arange(0.05, 0.50, 0.01):
    lw, rw = waist(0.08, 0.18, -0.40, +1), waist(f, 0.18, -0.40, -1)
    for _ in range(3):
        sq, st = ik.solve_ik(lw, rw, qm.copy(), np.zeros(14))
        cq, ct = cap1.step(sq, st, qm)
        if not np.array_equal(np.asarray(cq)[:7], np.asarray(sq)[:7]):
            ok_left = False
        capped_right += int(cap1.capped[1] and not cap1.capped[0])
        qm = np.array(cq)
record("A: one arm capped, the other untouched", ok_left and capped_right > 0,
       f"right-only capped frames {capped_right}")
# first accepted pose already over the limit: only moves that lower the torque
cap2 = tc.ShoulderTorqueCap(LIMIT, g)
fwd = None
for _ in range(10):
    fwd_q, fwd_t = ik.solve_ik(waist(0.45, 0.18, -0.30, +1), waist(0.45, 0.18, -0.30, -1), fwd, np.zeros(14))
    fwd = np.array(fwd_q)
cap2.reset(fwd)
higher_q, higher_t = None, None
for _ in range(10):
    hq, ht = ik.solve_ik(waist(0.50, 0.18, -0.25, +1), waist(0.50, 0.18, -0.25, -1), fwd, np.zeros(14))
cq, ct = cap2.step(np.array(hq), np.array(ht), fwd)
held = np.array_equal(np.asarray(cq)[:7], fwd[:7])
record("A: starting over the limit, a move that raises the torque is refused", held and abs(g(fwd)[0]) > LIMIT,
       f"start {abs(g(fwd)[0]):.2f} Nm, asked {abs(ht[0]):.2f} Nm, sent {abs(ct[0]):.2f} Nm")
os.chdir(cwd0)

if "--cpu-only" in sys.argv:
    summary_and_exit()

# ====================================================================== (B) fakes
print("\n=== (B) end to end on fakes (domain 1, lo) ===", flush=True)
from fake_xr_client import FakeXR                                     # noqa: E402
from tools import port_guard                                          # noqa: E402
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber  # noqa: E402
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_            # noqa: E402
ChannelFactoryInitialize(1, "lo")
cmdlog = []
def on_cmd(m):
    cmdlog.append((time.monotonic(), float(m.motor_cmd[15].tau), float(m.motor_cmd[22].tau)))
_s = ChannelSubscriber("rt/lowcmd", LowCmd_); _s.Init(on_cmd, 10)
BASE = dict(os.environ, PYTHONUNBUFFERED="1", PYTHONNOUSERSITE="1", PYTHONDONTWRITEBYTECODE="1",
            XR_ARM_VEL_LIMIT="1")
BASE.pop("XR_SHOULDER_TAU_CAP", None)
tmp = Path(tempfile.mkdtemp(prefix="g10_"))


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


def session(tag, env_extra, radius=0.04):
    """Bring up fake lowstate + launcher in tmux g10 + fake headset; press r. Returns
    (pane path, FakeXR, fixture, pid_of_launcher)."""
    fx = subprocess.Popen([PY, str(REPO / "tools/fake_lowstate.py"), "--domain", "1", "--iface", "lo",
                           "--rate", "500", "--follow-cmd", "--q-arms", "0.0"], env=BASE,
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    time.sleep(2)
    env = dict(BASE, **env_extra)
    keys = ["PYTHONUNBUFFERED", "PYTHONNOUSERSITE", "PYTHONDONTWRITEBYTECODE", "XR_ARM_VEL_LIMIT"] + list(env_extra)
    env_str = " ".join(f"{k}={env[k]}" for k in keys)
    pane = tmp / f"pane_{tag}.log"
    lcmd = (f"cd {REPO / 'teleop'} && {PY} teleop_hand_and_arm.py --sim --network-interface lo; "
            f"echo \"[g10] launcher exited with $?\"; sleep 15")
    subprocess.run(["tmux", "new", "-d", "-s", "g10", "-x", "200", "-y", "50", f"env {env_str} bash -c '{lcmd}'"], check=True)
    subprocess.run(["tmux", "pipe-pane", "-o", "-t", "g10", f"cat >> {pane}"], check=True)
    xr = FakeXR(radius=radius).start()
    up = wait_file(pane, r"attach check", 120)
    time.sleep(2)
    subprocess.run(["tmux", "send-keys", "-t", "g10", "r"])
    tr = wait_file(pane, r"\[link\] tracking", 15)
    return pane, xr, fx, bool(up), bool(tr)


def end(pane, xr, fx):
    subprocess.run(["tmux", "send-keys", "-t", "g10", "q"])
    m = wait_file(pane, r"\[g10\] launcher exited with (-?\d+)", 60)
    xr.stop()
    subprocess.run(["tmux", "kill-session", "-t", "g10"], capture_output=True)
    stop(fx)
    time.sleep(2)
    return m.group(1) if m else None


def window(t0, t1):
    seg = [c for c in cmdlog if t0 <= c[0] <= t1]
    return np.array([[abs(c[1]), abs(c[2])] for c in seg]) if seg else np.zeros((0, 2))


def stats_rows(since):
    files = sorted((REPO / "logs").glob("launcher_stats_*.csv"), key=lambda p: p.stat().st_mtime)
    files = [f for f in files if f.stat().st_mtime >= since]
    if not files:
        return []
    lines = files[-1].read_text().strip().splitlines()
    hdr = lines[0].split(",")
    return [dict(zip(hdr, l.split(","))) for l in lines[1:] if not l.startswith("t_mono")]


try:
    # ---- B1: cap on, hands far forward (fake headset default: 0.32 m ahead, 0.35 m down)
    t_start = time.time()
    pane, xr, fx, up, tr = session("on", {})
    record("B1: lowering line says no joint above 0.3 rad/s", "with no joint above 0.3 rad/s" in pane.read_text(errors="replace"))
    record("B1: launcher up, cap line printed", up and f"[cap] shoulder pitch gravity torque capped at {DEF:g} Nm" in pane.read_text(errors="replace"))
    record("B1: [r] -> tracking", tr)
    time.sleep(10)                              # start pose is off: FOLLOW at once, at 1 rad/s
    t_a = time.monotonic()
    time.sleep(8)
    w = window(t_a, time.monotonic())
    text = pane.read_text(errors="replace")
    record(f"B1: commanded shoulder pitch tau <= {DEF:g} Nm with the hands far forward",
           len(w) and w.max() <= DEF + 0.01, f"{len(w)} commands, max |tau| L {w[:, 0].max():.3f} R {w[:, 1].max():.3f} Nm" if len(w) else "none")
    record("B1: ...and it sits at the limit", len(w) and w.max() >= 0.95 * DEF, f"max {w.max():.3f} Nm" if len(w) else "")
    record("B1: [cap] notices for both arms", f"[cap] left shoulder pitch at the {DEF:g} Nm limit" in text
           and f"[cap] right shoulder pitch at the {DEF:g} Nm limit" in text)
    xr.set_offset([0.10, 0.0, 0.25])            # wrists back to about 0.07 m ahead, at the chest
    time.sleep(4)
    t_b = time.monotonic()
    time.sleep(4)
    w2 = window(t_b, time.monotonic())
    text = pane.read_text(errors="replace")
    record("B1: hands back at the chest: under the limit and following",
           len(w2) and w2.max() < DEF and "back under the limit: following" in text,
           f"max |tau| {w2.max():.2f} Nm" if len(w2) else "none")
    time.sleep(5.5)
    rc = end(pane, xr, fx)
    record("B1: exit 0", rc == "0", f"exit {rc}")
    rows = stats_rows(t_start)
    pm = max((float(r["sp_ff_pred_max_L_nm"]) for r in rows if r.get("sp_ff_pred_max_L_nm")), default=0)
    cm = max((float(r["sp_ff_cmd_max_L_nm"]) for r in rows if r.get("sp_ff_cmd_max_L_nm")), default=99)
    cf = sum(int(r["cap_frames_L"]) for r in rows if r.get("cap_frames_L", "").isdigit())
    record("B1: stats file logs the predicted and commanded shoulder torque",
           pm > DEF and cm <= DEF + 0.01 and cf > 0, f"{len(rows)} rows: predicted max {pm:.2f}, commanded max {cm:.2f} Nm, capped frames {cf}")

    # ---- B2: cap off, same pose
    pane, xr, fx, up, tr = session("off", {"XR_SHOULDER_TAU_CAP": "0"})
    record("B2: cap OFF line printed", up and "torque cap OFF" in pane.read_text(errors="replace"))
    time.sleep(10)
    t_a = time.monotonic()
    time.sleep(6)
    w = window(t_a, time.monotonic())
    record("B2: without the cap the same pose commands over 6 Nm", len(w) and w.max() > 6.0,
           f"max |tau| {w.max():.2f} Nm" if len(w) else "none")
    rc = end(pane, xr, fx)
    record("B2: exit 0", rc == "0", f"exit {rc}")

    # ---- B3: mapping XY 0.7 / X -0.05, hands at the chest, a 5 mm circle (a pose that never
    # changes reads as a dead stream to the link guard, so the fake always moves a little)
    pane, xr, fx, up, tr = session("map", {"XR_WRIST_XY_SCALE": "0.7", "XR_WRIST_X_OFFSET": "-0.05"}, radius=0.005)
    text = pane.read_text(errors="replace")
    record("B3: mapping line printed", "XY_SCALE=0.7" in text and "X_OFFSET=-0.05m" in text)
    # fake wrists: (-+0.22, 1.15, -0.32) OpenXR; chest pose (fwd 0.10, out 0.12, up -0.40) needs
    # d = (+0.10, -0.05, +0.22) for the left, mirrored in x for the right by the fake itself
    xr.set_offset([0.10, -0.05, 0.22])
    time.sleep(12)
    t_a = time.monotonic()
    time.sleep(8)                               # two periods of the circle
    seg = [c for c in cmdlog if c[0] >= t_a]
    tl = float(np.mean([c[1] for c in seg])) if seg else float("nan")
    record("B3: hands at the chest under XY 0.7 / X -0.05: commanded tau matches the offline table (-2.08 Nm)",
           abs(tl - (-2.08)) < 0.3, f"live {tl:.2f} Nm")
    rc = end(pane, xr, fx)
    record("B3: exit 0", rc == "0", f"exit {rc}")
    record("8012 free at the end", port_guard.port_is_free(8012))
finally:
    subprocess.run(["tmux", "kill-session", "-t", "g10"], capture_output=True)

summary_and_exit()
