#!/usr/bin/env python3
"""G15 (RT6h): cap kept inside the joint margin, arm controller rate with both arms capped,
IK worker process. CPU and DOMAIN 1 / LOOPBACK ONLY.

  (A) CPU, cap only: RT6g's left arm pose 2 s before its elbow reached the 2.094 rad stop
      (arm_watch 19:55:05, elbow 1.960), with an operator who keeps raising the arms
      (IK target: shoulder pitch -1.4 / -1.6 rad, elbow 1.9, all inside the margin).
      Control, the RT6g cap (no bounds): the slide folds the elbow to the stop. The RT6h
      cap (the IK's bounds): every commanded arm joint stays at least 0.1 rad inside its
      URDF limit, the torque stays at the cap and the arm still follows (pitch moves).
      At 7 Nm (RT6g) and 5 Nm (RT6h, from a pose under 5 Nm). A joint already outside
      (the elbow at 2.09 rad at [r]) never moves further out.
  (B) CPU, the IK: warm start from its own last solution with both arms capped, solve
      time p50 against the measured-pose start.
  (C) Fakes: launcher --sim on lo, the robot's clip (XR_SIM_VEL_CLIP=1), cap 3 Nm so the
      fake headset (about 8 Nm) caps both arms (4.5 Nm, over the 3.9 Nm of the kinematic fake's q = 0 at [r]): the IK runs in a worker process and the
      controller holds 250 Hz (every full 5 s stats row at 245 Hz or more) with both arms
      capped. Then the worker is killed: the launcher says so, solves in-process, keeps
      commanding, and exits 0 on [q]; no worker left.
  (D) Fakes, control: the same with XR_IK_WORKER=0 (the RT6g in-process IK): the rate
      is lower.

Run:  python tools/overnight/test_g15_cap_bounds_rate.py [--cpu-only]
"""
import os, re, signal, subprocess, sys, tempfile, time
from pathlib import Path
import numpy as np

REPO = Path(__file__).resolve().parent.parent.parent
sys.path[:0] = [str(REPO), str(REPO / "tools"), str(REPO / "teleop" / "televuer" / "src")]
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


cwd0 = os.getcwd()
os.chdir(REPO / "teleop")
from televuer.tv_wrapper import (T_ROBOT_OPENXR, T_OPENXR_ROBOT, T_TO_UNITREE_HUMANOID_LEFT_ARM,  # noqa: E402
                                 T_TO_UNITREE_HUMANOID_RIGHT_ARM,
                                 transform_IPunitree_Brobot_world_arm_to_head_then_waist)
os.environ.pop("XR_IK_JOINT_MARGIN", None)
os.environ.pop("XR_IK_WARM_LAST", None)
from teleop.robot_control.robot_arm_ik import G1_29_ArmIK                    # noqa: E402
from teleop.robot_control import torque_cap as tc                            # noqa: E402

# ====================================================================== (A)
print("=== (A) CPU: the cap's slide and catch-up inside the joint margin ===", flush=True)
ik = G1_29_ArmIK()
model = ik.reduced_robot.model
lo, hi = ik.ik_lower, ik.ik_upper
ulo, uhi = model.lowerPositionLimit, model.upperPositionLimit
g = tc.gravity_fn_for(model)
EL = 3                                                    # left elbow in the 14-joint vector
# RT6g arm_watch 19:55:05: left, right shoulder pitch, roll, yaw, elbow (wrists not logged)
Q_RT6G = np.array([-0.759, 0.221, 0.482, 1.960, 0, 0, 0, -0.451, -0.136, -1.192, 1.185, 0, 0, 0])


def inside(q):
    return float(np.min(np.minimum(q - ulo, uhi - q)))


def run(q0, q1, limit, bounded, n=150, hold=90):
    """The operator moves the IK target from q0 to q1 in n frames (30 Hz), then holds."""
    cap = tc.ShoulderTorqueCap(limit, g, frequency=30.0, lower=lo if bounded else None,
                               upper=hi if bounded else None)
    qm = q0.copy(); cap.reset(qm)
    worst, el, tmax, el_steps = 9.0, 0.0, 0.0, []
    for k in range(n + hold):
        sq = q0 + min(1.0, (k + 1) / n) * (q1 - q0)
        cq, ct = cap.step(sq, g(sq), qm)
        cq = np.asarray(cq)
        worst = min(worst, inside(cq)); el = max(el, cq[EL]); tmax = max(tmax, abs(ct[0]), abs(ct[7]))
        el_steps.append(cq[EL] - qm[EL])
        qm = cq.copy()
    return dict(worst=worst, el=el, tmax=tmax, end=qm, cuts=list(cap.bound_clips), el_steps=el_steps)


record("A: the IK bounds are 0.1 rad inside the URDF limits (left elbow URDF 2.094, bound 1.994)",
       np.allclose(lo, ulo + 0.1) and np.allclose(hi, uhi - 0.1), f"elbow bound {hi[EL]:.4f}")
for p1 in (-1.4, -1.6):
    q1 = Q_RT6G.copy(); q1[0] = p1; q1[2] = 0.4; q1[EL] = 1.9; q1[7] = p1
    old = run(Q_RT6G, q1, 7.0, False)
    new = run(Q_RT6G, q1, 7.0, True)
    record(f"A: control, RT6g cap (no bounds), 7 Nm, target pitch {p1}: the slide folds the left elbow into the margin",
           old["worst"] < 0.1 - 1e-3, f"left elbow up to {old['el']:.3f} rad, {old['worst']:+.3f} rad from the URDF limit")
    record(f"A: RT6h cap, 7 Nm, target pitch {p1}: every joint >= 0.1 rad inside, torque <= 7 Nm, the arm still raises",
           new["worst"] >= 0.1 - 1e-6 and new["tmax"] <= 7.0 + 1e-3 and new["end"][0] < Q_RT6G[0] - 0.2,
           f"closest {new['worst']:.4f} rad, elbow max {new['el']:.4f}, torque max {new['tmax']:.2f} Nm, "
           f"pitch {Q_RT6G[0]:+.3f} -> {new['end'][0]:+.3f}, bound cuts {new['cuts']}")
# 5 Nm: the RT6g pose is at 5.3 Nm (over: the cap would hold it), so start from it with the
# shoulder pitch lowered until under 4.8 Nm.
q0 = Q_RT6G.copy()
while abs(g(q0)[0]) > 4.8 or abs(g(q0)[7]) > 4.8:
    q0[0] += 0.01; q0[7] += 0.01
for p1 in (-1.4, -1.6):
    q1 = q0.copy(); q1[0] = p1; q1[2] = 0.4; q1[EL] = 1.9; q1[7] = p1
    old = run(q0, q1, 5.0, False)
    new = run(q0, q1, 5.0, True)
    record(f"A: 5 Nm, start pitch {q0[0]:+.2f}, target pitch {p1}: control without bounds reaches the margin; RT6h stays >= 0.1 rad inside",
           old["worst"] < 0.1 - 1e-3 and new["worst"] >= 0.1 - 1e-6 and new["tmax"] <= 5.0 + 1e-3,
           f"control {old['worst']:+.3f} rad (elbow {old['el']:.3f}); RT6h {new['worst']:.4f} rad (elbow {new['el']:.4f}), "
           f"torque max {new['tmax']:.2f} Nm, bound cuts {new['cuts']}")
# Already outside: the elbow on its stop at [r] (2.090), target inside the margin, over the cap.
q0 = Q_RT6G.copy(); q0[EL] = 2.090
q1 = q0.copy(); q1[0] = -1.6; q1[EL] = 1.9; q1[7] = -1.6
new = run(q0, q1, 7.0, True)
record("A: left elbow starting outside the margin (2.090 rad), capped: never commanded further out, moves back in",
       new["el"] <= 2.090 + 1e-9 and new["end"][EL] < 2.090 - 0.02,
       f"elbow max {new['el']:.4f}, end {new['end'][EL]:.4f} rad")
# Catch-up after capping, back under: inside the bounds and at most the catch-up step.
cap = tc.ShoulderTorqueCap(7.0, g, frequency=30.0, lower=lo, upper=hi)
q1 = Q_RT6G.copy(); q1[0] = -1.6; q1[EL] = 1.9; q1[7] = -1.6
qm = Q_RT6G.copy(); cap.reset(qm); worst, big = 9.0, 0.0
for k in range(300):
    s = min(1.0, (k + 1) / 100) if k < 150 else max(0.0, 1.0 - (k - 149) / 60)
    sq = Q_RT6G + s * (q1 - Q_RT6G)
    cq, _ = cap.step(sq, g(sq), qm); cq = np.asarray(cq)
    worst = min(worst, inside(cq))
    if k >= 150 and not cap.capped[0]:
        big = max(big, float(np.max(np.abs(cq[:7] - qm[:7]))))
    qm = cq.copy()
record("A: up over the cap and back under: inside the bounds, no left arm step over the catch-up step",
       worst >= 0.1 - 1e-6 and big <= cap.catchup_step + 1e-6,
       f"closest {worst:.4f} rad, largest uncapped step {big:.4f} rad (catch-up {cap.catchup_step:.4f})")

# ====================================================================== (B)
print("\n=== (B) CPU: IK warm start from its last solution, both arms capped ===", flush=True)
HEAD = np.eye(4); HEAD[1, 3] = 1.5


def waist(fwd, out, up, side):
    m = np.eye(4); m[:3, 3] = [-out * side, 1.5 + up, -fwd]
    arm = T_ROBOT_OPENXR @ m @ T_OPENXR_ROBOT
    arm = arm @ (T_TO_UNITREE_HUMANOID_LEFT_ARM if side > 0 else T_TO_UNITREE_HUMANOID_RIGHT_ARM)
    return transform_IPunitree_Brobot_world_arm_to_head_then_waist(arm, T_ROBOT_OPENXR @ HEAD @ T_OPENXR_ROBOT, "head_yaw")


def capped_session(the_ik):
    cap = tc.ShoulderTorqueCap(3.0, g, frequency=30.0, lower=lo, upper=hi)
    qm = np.zeros(14); cap.reset(qm); ts, nc = [], 0
    for k in range(240):
        f = 0.32 + 0.03 * np.sin(2 * np.pi * k / 60)
        a = time.perf_counter()
        sq, st = the_ik.solve_ik(waist(f, 0.18, -0.35, +1), waist(f, 0.18, -0.35, -1), qm.copy(), np.zeros(14))
        ts.append(time.perf_counter() - a)
        cq, _ = cap.step(np.clip(sq, lo, hi), st, qm); qm = np.array(cq)
        nc += int(cap.capped[0] and cap.capped[1])
    return 1e3 * np.median(ts[30:]), 1e3 * np.percentile(ts[30:], 99), nc


os.environ["XR_IK_WARM_LAST"] = "0"
ik_cold = G1_29_ArmIK()
os.environ.pop("XR_IK_WARM_LAST", None)
ik_warm = G1_29_ArmIK()
c50, c99, cn = capped_session(ik_cold)
w50, w99, wn = capped_session(ik_warm)
record("B: both arms capped in most frames", min(cn, wn) > 150, f"{cn} and {wn} of 240 frames")
record("B: warm start from the last solution is faster than from the measured pose", w50 < c50,
       f"p50 {w50:.2f} ms (p99 {w99:.2f}) against {c50:.2f} ms (p99 {c99:.2f})")
os.chdir(cwd0)

if "--cpu-only" in sys.argv:
    summary_and_exit()

# ====================================================================== (C), (D)
from fake_xr_client import FakeXR                                     # noqa: E402
from tools import port_guard                                          # noqa: E402
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber  # noqa: E402
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_            # noqa: E402
ChannelFactoryInitialize(1, "lo")
cmd = []
def on_cmd(m):
    cmd.append((time.monotonic(), float(m.motor_cmd[15].tau), float(m.motor_cmd[22].tau)))
_s = ChannelSubscriber("rt/lowcmd", LowCmd_); _s.Init(on_cmd, 10)
tmp = Path(tempfile.mkdtemp(prefix="g15_"))
BASE_ENV = {"PYTHONUNBUFFERED": "1", "PYTHONNOUSERSITE": "1", "PYTHONDONTWRITEBYTECODE": "1", "XR_ARM_VEL_LIMIT": "1",
            "XR_SHOULDER_TAU_CAP": "4.5", "XR_SIM_VEL_CLIP": "1"}
CAPL = 4.5


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


def stats_rows(since):
    files = sorted((REPO / "logs").glob("launcher_stats_*.csv"), key=lambda p: p.stat().st_mtime)
    files = [f for f in files if f.stat().st_mtime >= since]
    if not files:
        return []
    lines = files[-1].read_text().strip().splitlines()
    hdr = lines[0].split(",")
    return [dict(zip(hdr, l.split(","))) for l in lines[1:]]


def rates(rows):
    """(control Hz, capped frames L, R, link state) per full row, from consecutive t_mono."""
    out = []
    for a, b in zip(rows, rows[1:]):
        try:
            dt = float(b["t_mono"]) - float(a["t_mono"])
            out.append((int(b["ctrl_steps"]) / dt, int(b["cap_frames_L"]), int(b["cap_frames_R"]), b.get("guard", "")))
        except (ValueError, KeyError, ZeroDivisionError):
            pass
    return out


def session(tag, env_extra, follow_s, kill_worker=False):
    env = dict(BASE_ENV, **env_extra)
    fx = subprocess.Popen([PY, str(REPO / "tools/fake_lowstate.py"), "--domain", "1", "--iface", "lo", "--rate", "500",
                           "--follow-cmd", "--pose", str(REPO / "poses/low.yaml")], env=dict(os.environ, **env),
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    time.sleep(2)
    pane = tmp / f"pane_{tag}.log"
    lcmd = (f"cd {REPO / 'teleop'} && {PY} teleop_hand_and_arm.py --sim --network-interface lo; "
            f"echo \"[g15] launcher exited with $?\"; sleep 15")
    env_str = " ".join(f"{k}={v}" for k, v in env.items())
    t_start = time.time()
    out = {}
    try:
        subprocess.run(["tmux", "new", "-d", "-s", "g15", "-x", "220", "-y", "50", f"env {env_str} bash -c '{lcmd}'"], check=True)
        subprocess.run(["tmux", "pipe-pane", "-o", "-t", "g15", f"cat >> {pane}"], check=True)
        xr = FakeXR(radius=0.01).start()
        out["up"] = bool(wait_file(pane, r"attach check", 120))
        m = re.search(r"\[ik\] solving in a worker process \(pid (\d+)\)", pane.read_text(errors="replace"))
        out["worker_pid"] = int(m.group(1)) if m else None
        out["worker_off_line"] = "[ik] worker process off" in pane.read_text(errors="replace") or "XR_IK_WORKER" in env_extra
        time.sleep(2)
        subprocess.run(["tmux", "send-keys", "-t", "g15", "r"])
        out["tracking"] = bool(wait_file(pane, r"\[link\] tracking", 15))
        t_f = time.monotonic()
        time.sleep(follow_s)
        out["rows_follow"] = [r for r in stats_rows(t_start)]
        ts = np.array([c[0] for c in cmd if t_f + 5.0 <= c[0] <= t_f + follow_s])
        gaps = np.diff(ts) * 1e3 if len(ts) > 2 else np.zeros(1)
        out["gaps"] = (float(np.percentile(gaps, 50)), float(np.percentile(gaps, 99)), float(gaps.max()), len(ts))
        out["tau_follow"] = max((max(abs(c[1]), abs(c[2])) for c in cmd if t_f + 5.0 <= c[0] <= t_f + follow_s), default=0.0)
        if kill_worker and out["worker_pid"]:
            os.kill(out["worker_pid"], signal.SIGKILL)
            t_k = time.monotonic()
            out["fail_line"] = bool(wait_file(pane, r"\[ik\] worker process failed", 10))
            time.sleep(6)
            out["cmds_after_kill"] = sum(1 for c in cmd if c[0] > t_k + 1.0)
            out["tau_after_kill"] = max((max(abs(c[1]), abs(c[2])) for c in cmd if c[0] > t_k + 1.0), default=0.0)
        subprocess.run(["tmux", "send-keys", "-t", "g15", "q"])
        done = wait_file(pane, r"\[g15\] launcher exited with (-?\d+)", 90)
        xr.stop()
        out["exit"] = done.group(1) if done else None
        out["rows"] = stats_rows(t_start)
        out["pane"] = pane.read_text(errors="replace")
        if out["worker_pid"]:
            out["worker_gone"] = not Path(f"/proc/{out['worker_pid']}").exists()
    finally:
        subprocess.run(["tmux", "kill-session", "-t", "g15"], capture_output=True)
        stop(fx)
    return out


print("\n=== (C) fakes: IK worker, both arms capped at 4.5 Nm, 250 Hz ===", flush=True)
c = session("worker", {}, follow_s=27, kill_worker=True)
record("C: launcher up, IK worker started", c["up"] and c["worker_pid"] is not None, f"worker pid {c['worker_pid']}")
record("C: [r] -> tracking", c["tracking"])
rc = [r for r in rates(c["rows_follow"]) if r[3] == "FOLLOW" and r[1] > 0 and r[2] > 0]
hz = [r[0] for r in rc]
record("C: both arms capped in FOLLOW rows (at least 3 full rows)", len(rc) >= 3,
       f"{len(rc)} rows; capped frames L {[r[1] for r in rc]} R {[r[2] for r in rc]}")
record("C: the controller holds 250 Hz with both arms capped (every such row >= 245 Hz)", len(hz) >= 3 and min(hz) >= 245.0,
       f"{', '.join(f'{h:.1f}' for h in hz)} Hz")
record("C: commands evenly spaced while capped (rt/lowcmd gap p99 <= 8 ms, two control steps)", c["gaps"][1] <= 8.0,
       f"gap p50 {c['gaps'][0]:.2f} ms, p99 {c['gaps'][1]:.2f}, max {c['gaps'][2]:.2f} ms over {c['gaps'][3]} commands")
record("C: commanded shoulder torque at or under 4.5 Nm while capped", c["tau_follow"] <= CAPL + 0.01, f"max {c['tau_follow']:.3f} Nm")
record("C: worker killed -> the launcher says so and solves in-process", c.get("fail_line"))
record("C: ...and keeps commanding at the cap (commands flowing, torque <= 4.5 Nm)",
       c.get("cmds_after_kill", 0) > 500 and c.get("tau_after_kill", 9) <= CAPL + 0.01,
       f"{c.get('cmds_after_kill')} commands in about 5 s, max |tau| {c.get('tau_after_kill', 0):.2f} Nm")
record("C: exit 0 on [q]", c["exit"] == "0", f"exit {c['exit']}")
record("C: no worker left", c.get("worker_gone", False))
time.sleep(3)

print("\n=== (D) fakes, control: XR_IK_WORKER=0 (in-process IK, as in RT6g) ===", flush=True)
d = session("inproc", {"XR_IK_WORKER": "0"}, follow_s=22)
rd = [r for r in rates(d["rows_follow"]) if r[3] == "FOLLOW" and r[1] > 0 and r[2] > 0]
hd = [r[0] for r in rd]
record("D: in-process IK, both arms capped: commands stall during the solves (gap p99 at least 2 ms over the worker's)",
       d["gaps"][1] >= c["gaps"][1] + 2.0,
       f"gap p50 {d['gaps'][0]:.2f} ms, p99 {d['gaps'][1]:.2f}, max {d['gaps'][2]:.2f} ms; rate {', '.join(f'{h:.1f}' for h in hd)} Hz "
       f"(worker: p99 {c['gaps'][1]:.2f}, max {c['gaps'][2]:.2f} ms, rate median {np.median(hz) if hz else 0:.1f} Hz)")
record("D: exit 0", d["exit"] == "0", f"exit {d['exit']}")
record("8012 free at the end", port_guard.port_is_free(8012))
summary_and_exit()
