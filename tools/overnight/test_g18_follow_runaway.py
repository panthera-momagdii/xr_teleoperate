#!/usr/bin/env python3
"""G18 (G0): the 23 Oct runaway (RT7, GAPS 17), fakes first. CPU and DOMAIN 1 / LOOPBACK ONLY.

RT7 (24 Oct 00:21): both shoulder pitches ran to -3.09 rad (arms straight up, at the stop)
for about 60 s while the IK targets stayed far inside. The controller's speed limiter kept
each command 0.004 rad ahead of the MEASURED pose (0.3 Nm of PD at most) while the gravity
feed-forward was computed for the TARGET (7 Nm, the cap). The CPU parts drive the arm
controller's own per-step code (G1_29_ArmController without DDS: its follow step if it has
one, else its old clip_arm_q_target plus the launcher's target feed-forward) against the
simulated arm (tools/arm_physics.py: the IK's model, ABA), at 250 Hz.

  (A) CPU replay: RT7 arm3's own IK targets (fixtures/rt7_arm3_target_log.csv.gz, 79 s at
      30 Hz, cap 7 Nm, starting at the measured pose), frictionless and with the RT6g
      friction calibration (arms Fs 2.7 / Fc 0.81 Nm, wrists 1.7 / 0.17, wrist kd 30 %).
      Pass: no arm joint within 0.02 rad of a URDF stop, no follow stop, tracking error
      (against the IK target) over 0.35 rad never for 1 s or more.
  (B) CPU still hold, friction and 0.002 rad of q noise: low.yaml held 60 s; and the arm
      raised in 3 s to a 7 Nm pose and held there 57 s. No false stop, the arm on target.
  (C) CPU stops: (C1) an obstacle at -0.2 rad holds the left shoulder pitch short of its
      -1.0 rad target: tracking stop after the 1 s grace, then the command holds the
      measured pose (latched). (C2) 25 Nm pushes the left elbow onto its stop while its target is 0.25
      rad inside: joint-at-stop stop at once.
  (D) Fakes: launcher --sim on lo with the robot's clip (XR_SIM_VEL_CLIP=1), fake_lowstate
      --physics with the friction calibration, cap 7 Nm, fake headset: (D1) [r], 60 s of
      FOLLOW: the follow line is printed, no stop, no joint at a stop, [q] exits 0.
      (D2) XR_TRACK_ERR_MAX=0.001, XR_TRACK_GRACE_S=0.2: the launcher reports the stop,
      lowers and releases, exits 8.

Run:  python tools/overnight/test_g18_follow_runaway.py [--cpu-only]
"""
import csv
import gzip
import os
import re
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent.parent
sys.path[:0] = [str(REPO), str(REPO / "tools")]
PY = sys.executable
FIXTURE = REPO / "tools/overnight/fixtures/rt7_arm3_target_log.csv.gz"
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


from arm_physics import ArmPhysics                                    # noqa: E402
from teleop.robot_control import robot_arm as ra                      # noqa: E402
from teleop.robot_control import torque_cap as tc                     # noqa: E402

DT = 1.0 / 250.0
# G1_29_ArmController's arm gains: kp_low / kd_low (shoulders, elbows), kp_wrist / kd_wrist
KP = np.array([80.0] * 4 + [40.0] * 3 + [80.0] * 4 + [40.0] * 3)
KD = np.array([3.0] * 4 + [1.5] * 3 + [3.0] * 4 + [1.5] * 3)
FRICTION = (np.array([2.7] * 4 + [1.7] * 3 + [2.7] * 4 + [1.7] * 3),
            np.array([0.81] * 4 + [0.17] * 3 + [0.81] * 4 + [0.17] * 3))
KD_EFF = np.array([1.0] * 4 + [0.3] * 3 + [1.0] * 4 + [0.3] * 3)
MODEL = ArmPhysics(np.zeros(14)).model
G = tc.gravity_fn_for(MODEL)
LO, HI = MODEL.lowerPositionLimit.copy(), MODEL.upperPositionLimit.copy()
LOW = np.array([0.15, 0.10, 0, 1.2, 0, 0, 0, 0.15, -0.10, 0, 1.2, 0, 0, 0])
MAX_ERR, GRACE = 0.35, 1.0


def feed(c, q):
    ls = ra.G1_29_LowState()
    for k, idx in enumerate(ra.G1_29_JointArmIndex):
        ls.motor_state[idx].q = float(q[k])
    c.lowstate_buffer.SetData(ls)


def make_ctrl(q0, vel=1.0):
    """The arm controller without DDS or threads: only what its per-step code reads."""
    c = ra.G1_29_ArmController.__new__(ra.G1_29_ArmController)
    c.control_dt, c.arm_velocity_limit, c._clip_stats = DT, float(vel), [0, 0, 0, 0]
    c.lowstate_buffer = ra.DataBuffer()
    feed(c, q0)
    if hasattr(c, "_init_follow"):
        c._init_follow(tc.gravity_fn_for(MODEL))
    c._following = True              # the launcher has sent a target
    return c


def law(c, t, q_t, tau_t):
    """The controller's command for one step: (q, tau, stop reason or None)."""
    if hasattr(c, "_follow_step"):
        q, tau = c._follow_step(q_t, tau_t, t=t)
        return q, tau, c.follow_fault
    # the controller before G0: clip relative to the measured pose, the launcher's feed-forward
    return c.clip_arm_q_target(q_t, c.arm_velocity_limit), tau_t, None


def run(target_at, q0, t_end, friction=False, noise=0.0, ext=None, seed=1):
    phys = ArmPhysics(q0, friction=FRICTION if friction else None, kd_eff=KD_EFF if friction else None)
    c = make_ctrl(q0)
    rng = np.random.default_rng(seed)
    t, out = 0.0, dict(stop_hit=None, fault=None, t_fault=None, errs=[], cmd_err_max=0.0, q_cmds=[])
    while t < t_end:
        q_t = target_at(t)
        q_m = phys.q + (rng.normal(0.0, noise, 14) if noise else 0.0)
        feed(c, q_m)
        q_c, tau, fault = law(c, t, q_t, G(q_t))          # the launcher sends g(target)
        if fault and out["fault"] is None:
            out["fault"], out["t_fault"], out["q_fault"] = fault, t, phys.q.copy()
        if out["fault"]:
            out["q_cmds"].append(np.asarray(q_c).copy())
        phys.set_command(q_c, np.zeros(14), KP, KD, np.asarray(tau) + (ext(t, phys.q) if ext else 0.0), t)
        phys.advance(DT, t)
        t += DT
        near = np.minimum(phys.q - LO, HI - phys.q)
        if out["stop_hit"] is None and float(np.min(near)) < 0.02:
            j = int(np.argmin(near))
            out["stop_hit"] = (round(t, 2), j, round(float(phys.q[j]), 3))
        out["errs"].append(float(np.max(np.abs(phys.q - q_t))))
        out["cmd_err_max"] = max(out["cmd_err_max"], float(np.max(np.abs(np.asarray(q_c) - q_m))))
    out["q_end"] = phys.q.copy()
    e = np.array(out["errs"])
    over = e > MAX_ERR
    runs, cur = [], 0
    for o in over:
        cur = cur + 1 if o else 0
        runs.append(cur)
    out["longest_over_s"] = (max(runs) if runs else 0) * DT
    out["p50"], out["p95"], out["max"] = (float(np.percentile(e, 50)), float(np.percentile(e, 95)), float(e.max()))
    return out


def fmt(o):
    return (f"stop {o['stop_hit']}; follow stop {o['fault'] and o['fault'][:60]}; err p50 {o['p50']:.3f} "
            f"p95 {o['p95']:.3f} max {o['max']:.3f} rad, longest over {MAX_ERR} {o['longest_over_s']:.2f} s; "
            f"|cmd - measured| max {o['cmd_err_max']:.3f}")


print(f"=== controller under test: {'follow step (G0)' if hasattr(ra.G1_29_ArmController, '_follow_step') else 'clip_arm_q_target (before G0)'} ===", flush=True)

# ====================================================================== (A)
print("=== (A) CPU: RT7 arm3 targets replayed into the simulated arm ===", flush=True)
with gzip.open(FIXTURE, "rt") as fh:
    rows = list(csv.reader(fh))[1:]
T = np.array([float(r[0]) for r in rows]); T -= T[0]
TG = np.array([[float(v) for v in r[2:16]] for r in rows])
ME0 = np.array([float(v) for v in rows[0][16:30]])


def replay(t):
    return TG[min(int(np.searchsorted(T, t, side="right")) - 1, len(T) - 1)]


for fr in (False, True):
    tag = "calibrated friction" if fr else "frictionless"
    t0 = time.time()
    o = run(replay, ME0, float(T[-1]), friction=fr)
    record(f"A ({tag}): no arm joint within 0.02 rad of a stop over {T[-1]:.0f} s", o["stop_hit"] is None, fmt(o))
    record(f"A ({tag}): no follow stop, error over {MAX_ERR} rad never {GRACE:g} s long",
           o["fault"] is None and o["longest_over_s"] < GRACE, f"wall {time.time() - t0:.0f} s")

# ====================================================================== (B)
print("\n=== (B) CPU: still hold, friction, 0.002 rad q noise ===", flush=True)
o = run(lambda t: LOW, LOW, 60.0, friction=True, noise=0.002)
record("B1: low.yaml held 60 s: no stop, error under 0.05 rad, no drift",
       o["fault"] is None and o["stop_hit"] is None and o["max"] < 0.05
       and float(np.max(np.abs(o["q_end"] - LOW))) < 0.05,
       fmt(o) + f"; end off target {float(np.max(np.abs(o['q_end'] - LOW))):.4f}")
cap = tc.ShoulderTorqueCap(7.0, G)
FWD = np.array([-1.3, 0.25, 0, 0.2, 0, 0, 0, -1.3, -0.25, 0, 0.2, 0, 0, 0])
Q7 = LOW.copy()
for a, j in enumerate(tc.SHOULDER_PITCH):
    Q7[tc.ARM_SLICES[a]] = cap._search(LOW, FWD, a, j)[1][tc.ARM_SLICES[a]]
o = run(lambda t: LOW + min(1.0, t / 3.0) * (Q7 - LOW), LOW, 60.0, friction=True, noise=0.002)
tail = np.array(o["errs"][int(5.0 / DT):])
record(f"B2: raised in 3 s to a 7 Nm pose (shoulder pitch gravity {G(Q7)[0]:+.1f} / {G(Q7)[7]:+.1f} Nm), held 57 s: "
       f"no stop, on target after 5 s (error under 0.05 rad)",
       o["fault"] is None and o["stop_hit"] is None and float(tail.max()) < 0.05,
       fmt(o) + f"; after 5 s max {float(tail.max()):.4f}")

# ====================================================================== (C)
print("\n=== (C) CPU: the stops ===", flush=True)
Q_C1 = LOW.copy(); Q_C1[0] = -1.0
WALL = -0.2


def wall(t, q):                       # a stiff obstacle under the left shoulder pitch at -0.2 rad
    f = np.zeros(14)
    if q[0] < WALL:
        f[0] = 400.0 * (WALL - q[0])
    return f


o = run(lambda t: LOW + min(1.0, t / 1.5) * (Q_C1 - LOW), LOW, 6.0, ext=wall)
held = o["q_cmds"]
same = bool(held) and all(np.array_equal(h, held[0]) for h in held)
record("C1: an obstacle at -0.2 rad holds the left shoulder pitch short of its -1.0 rad target: tracking stop after the 1 s grace",
       o["fault"] is not None and "tracking error" in o["fault"] and o["t_fault"] is not None
       and 1.0 + 0.5 <= o["t_fault"] <= 1.0 + 0.5 + GRACE + 1.0,
       f"t {o['t_fault'] if o['t_fault'] is None else round(o['t_fault'], 2)} s: {o['fault']}")
record("C1: then the command holds the measured pose at the stop (latched, never changes)",
       same and o["fault"] is not None and float(np.max(np.abs(held[0] - o["q_fault"]))) < 0.02,
       f"{len(held)} held steps" + (f", off the stop pose {float(np.max(np.abs(held[0] - o['q_fault']))):.4f} rad" if held else ""))
Q_C2 = LOW.copy(); Q_C2[3] = HI[3] - 0.25


def push(t, q):                       # 25 Nm on the left elbow from t = 1 s
    f = np.zeros(14)
    if t >= 1.0:
        f[3] = 25.0
    return f


o = run(lambda t: Q_C2, Q_C2, 3.0, ext=push)
record("C2: left elbow pushed onto its stop, target 0.25 rad inside: joint-at-stop stop",
       o["fault"] is not None and "stop" in o["fault"] and "tracking" not in o["fault"]
       and o["t_fault"] is not None and o["t_fault"] < 2.0,
       f"t {o['t_fault'] if o['t_fault'] is None else round(o['t_fault'], 2)} s: {o['fault']}")

if "--cpu-only" in sys.argv:
    summary_and_exit()

# ====================================================================== (D)
print("\n=== (D) fakes: launcher --sim, simulated arm, cap 7 Nm ===", flush=True)
from fake_xr_client import FakeXR                                     # noqa: E402
from tools import port_guard                                          # noqa: E402
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber  # noqa: E402
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_          # noqa: E402

ChannelFactoryInitialize(1, "lo")
state = []


def on_state(m):
    state.append((time.monotonic(), np.array([m.motor_state[i].q for i in range(15, 29)])))


_s = ChannelSubscriber("rt/lowstate", LowState_); _s.Init(on_state, 10)
tmp = Path(tempfile.mkdtemp(prefix="g18_"))
BASE = dict(os.environ, PYTHONNOUSERSITE="1", PYTHONDONTWRITEBYTECODE="1", PYTHONUNBUFFERED="1")


def stop(p):
    try:
        os.killpg(p.pid, signal.SIGTERM); p.wait(timeout=5)
    except Exception:
        try:
            os.killpg(p.pid, signal.SIGKILL)
        except Exception:
            pass


def wait_file(path, pattern, timeout):
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        try:
            m = re.search(pattern, Path(path).read_text(errors="replace"))
            if m:
                return m
        except FileNotFoundError:
            pass
        time.sleep(0.1)
    return None


def launch(tag, extra_env):
    fx = subprocess.Popen([PY, str(REPO / "tools/fake_lowstate.py"), "--domain", "1", "--iface", "lo", "--rate", "500",
                           "--physics", "--pose", str(REPO / "poses/low.yaml"), "--dq-noise", "0.015",
                           "--friction-arm", "2.7", "0.81", "--friction", "1.7", "0.17", "--wrist-kd-eff", "0.3"],
                          env=BASE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    time.sleep(2)
    pane = tmp / f"pane_{tag}.log"
    envl = {"XR_ARM_VEL_LIMIT": "1", "XR_SIM_VEL_CLIP": "1", "XR_SHOULDER_TAU_CAP": "7", **extra_env}
    env_str = " ".join(f"{k}={v}" for k, v in dict(PYTHONUNBUFFERED="1", PYTHONNOUSERSITE="1",
                                                       PYTHONDONTWRITEBYTECODE="1", **envl).items())
    lcmd = (f"cd {REPO / 'teleop'} && {PY} teleop_hand_and_arm.py --sim --network-interface lo; "
            f"echo \"[g18] launcher exited with $?\"; sleep 15")
    subprocess.run(["tmux", "kill-session", "-t", "g18"], capture_output=True)
    subprocess.run(["tmux", "new", "-d", "-s", "g18", "-x", "200", "-y", "50", f"env {env_str} bash -c '{lcmd}'"], check=True)
    subprocess.run(["tmux", "pipe-pane", "-o", "-t", "g18", f"cat >> {pane}"], check=True)
    xr = FakeXR(radius=0.01).start()
    up = wait_file(pane, r"attach check", 120)
    time.sleep(2)
    subprocess.run(["tmux", "send-keys", "-t", "g18", "r"])
    tr = wait_file(pane, r"\[link\] tracking", 15)
    return dict(fx=fx, pane=pane, xr=xr, up=bool(up), tr=bool(tr), t_r=time.monotonic())


def finish(L, send_q=True, timeout=90):
    if send_q:
        subprocess.run(["tmux", "send-keys", "-t", "g18", "q"])
    m = wait_file(L["pane"], r"\[g18\] launcher exited with (-?\d+)", timeout)
    L["xr"].stop()
    subprocess.run(["tmux", "kill-session", "-t", "g18"], capture_output=True)
    stop(L["fx"])
    time.sleep(2)
    return m.group(1) if m else None


L = launch("d1", {})
text = L["pane"].read_text(errors="replace")
record("D1: launcher up, tracking, the follow line printed", L["up"] and L["tr"] and "[follow] command rate-limited from the last command" in text,
       (re.search(r"\[follow\][^\n]*", text).group(0)[:150] if "[follow]" in text else "no [follow] line"))
time.sleep(60)
seg = [q for (ts, q) in state if ts >= L["t_r"]]
near = min((float(np.min(np.minimum(q - LO, HI - q))) for q in seg), default=float("nan"))
text = L["pane"].read_text(errors="replace")
record("D1: 60 s of FOLLOW at the 7 Nm cap: no follow stop, no arm joint within 0.02 rad of a stop",
       "[follow] STOP" not in text and seg and near >= 0.02, f"{len(seg)} states, closest to a stop {near:.3f} rad")
rc = finish(L)
record("D1: [q] exits 0", rc == "0", f"exit {rc}")

L = launch("d2", {"XR_TRACK_ERR_MAX": "0.001", "XR_TRACK_GRACE_S": "0.2"})
st = wait_file(L["pane"], r"\[follow\] STOP[^\n]*", 20)
rc = finish(L, send_q=False, timeout=120)
text = L["pane"].read_text(errors="replace")
record("D2: tracking stop reported, then the lowering, exit 8",
       bool(st) and "[lower] lowering the arms" in text and rc == "8", f"exit {rc}; {st.group(0)[:150] if st else 'no stop line'}")
record("8012 free at the end", port_guard.port_is_free(8012))
summary_and_exit()
