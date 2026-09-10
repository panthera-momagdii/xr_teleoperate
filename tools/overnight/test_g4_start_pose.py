#!/usr/bin/env python3
"""G4: the fixed start pose. Mostly pure arithmetic; the launcher case uses domain 1.

The three properties that matter, and why:

  * MONOTONE toward the pose. Every approach step must reduce the distance to the pose
    on every joint -- an approach that overshoots and comes back is a wobble on a real
    arm.
  * NEVER exceeds the velocity limit. Not on the approach, and not on the blend either:
    a blend is not a licence to move faster than the arm may move.
  * The blend weight goes 0 -> 1 over exactly XR_BLEND_T, and only AFTER the pose has
    actually been reached.

Run:  python tools/overnight/test_g4_start_pose.py
"""

import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO))

from teleop.robot_control import start_pose as sp  # noqa: E402

PY = sys.executable
results = []


def record(name, ok, detail=""):
    results.append((name, ok, detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  -- {detail}" if detail else ""))


def write_pose(q, path, drop=None, extra=None, mangle=None):
    lines = ["name: t", "robot: G1_29", "units: radians", "joints:"]
    for i, n in enumerate(sp.ARM_JOINT_NAMES):
        if drop and n in drop:
            continue
        lines.append(f"  {n}: {mangle if (mangle is not None and i == 0) else q[i]}")
    for n in (extra or []):
        lines.append(f"  {n}: 0.0")
    Path(path).write_text("\n".join(lines) + "\n")
    return path


# ======================================================================== 1
print("\n=== (1) poses/ready.yaml loads, and is inside the URDF limits ===")
ready = REPO / "poses/ready.yaml"
record("poses/ready.yaml exists", ready.exists(), str(ready))
if ready.exists():
    q_ready = sp.load_pose(ready)
    record("it has exactly 14 joints, all finite",
           q_ready.shape == (14,) and np.all(np.isfinite(q_ready)))
    # limits straight from the URDF, via the same reduced model the IK uses
    try:
        from tools.reach_map import build_model
        model = build_model(str(REPO / "assets/g1/g1_body29_hand14.urdf"),
                            str(REPO / "assets/g1/"))
        lower, upper = model.lowerPositionLimit, model.upperPositionLimit
        margin = np.minimum(q_ready - lower, upper - q_ready)
        record("every joint is inside its URDF limit", bool(np.all(margin >= 0)),
               f"smallest margin {margin.min():.4f} rad "
               f"({sp.ARM_JOINT_NAMES[int(np.argmin(margin))]})")
        # and load_pose enforces them when asked
        bad = q_ready.copy(); bad[3] = upper[3] + 0.5
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as fh:
            bad_path = fh.name
        write_pose(bad, bad_path)
        try:
            sp.load_pose(bad_path, joint_limits=(lower, upper))
            record("load_pose refuses an out-of-limit pose", False, "it was ACCEPTED")
        except sp.StartPoseError as exc:
            record("load_pose refuses an out-of-limit pose", True, str(exc)[-70:])
        os.unlink(bad_path)
    except Exception as exc:
        record("URDF limit check", False, f"could not build the model: {exc}")

# ======================================================================== 2
print("\n=== (2) malformed pose files are refused, never guessed at ===")
tmpdir = tempfile.mkdtemp()
q = np.linspace(-0.3, 0.3, 14)
cases = [
    ("a missing joint", dict(drop={"left_elbow_joint"})),
    ("an unexpected joint", dict(extra=["waist_yaw_joint"])),
    ("a non-numeric value", dict(mangle="'quite bent'")),
    ("a nan", dict(mangle=".nan")),
]
for label, kwargs in cases:
    path = write_pose(q, os.path.join(tmpdir, "t.yaml"), **kwargs)
    try:
        sp.load_pose(path)
        record(f"refuses {label}", False, "it was ACCEPTED")
    except sp.StartPoseError as exc:
        record(f"refuses {label}", True, str(exc).split(":")[-1].strip()[:60])
Path(os.path.join(tmpdir, "empty.yaml")).write_text("name: t\n")
try:
    sp.load_pose(os.path.join(tmpdir, "empty.yaml"))
    record("refuses a file with no 'joints' key", False, "it was ACCEPTED")
except sp.StartPoseError as exc:
    record("refuses a file with no 'joints' key", True, str(exc)[-50:])
try:
    sp.load_pose(os.path.join(tmpdir, "does_not_exist.yaml"))
    record("refuses a missing file", False, "it was ACCEPTED")
except sp.StartPoseError:
    record("refuses a missing file", True)

# ======================================================================== 3
print("\n=== (3) disabled == today's behaviour, exactly ===")
for k in ("XR_START_POSE", "XR_START_T", "XR_BLEND_T"):
    os.environ.pop(k, None)
seq = sp.from_env(velocity_limit=5.0)
record("no XR_START_POSE -> disabled", not seq.enabled, seq.describe())
ik = np.arange(14, dtype=float)
out = seq.step(0.0, np.zeros(14), ik, dt=1 / 30)
record("disabled step() returns the IK target ITSELF (no copy, no change)",
       out is ik)
seq.start(0.0, np.zeros(14))
out = seq.step(1.0, np.zeros(14), ik, dt=1 / 30)
record("disabled stays pass-through even after start()", out is ik)

# ======================================================================== 4
print("\n=== (4) approach: monotone, rate-limited, then blend 0 -> 1 ===")
VEL, FREQ = 5.0, 30.0
dt = 1.0 / FREQ
max_step = VEL * dt
pose = np.full(14, 0.6)
q_now = np.zeros(14)
ik = np.full(14, -1.2)                      # operator far away, the hard case
seq = sp.StartPoseSequencer(pose=pose, start_t=3.0, blend_t=2.0, velocity_limit=VEL)
seq.start(0.0, q_now)

cmds, weights, phases = [], [], []
prev = q_now.copy()
worst_step = 0.0
monotone = True
t = 0.0
for i in range(int(12.0 * FREQ)):
    t = i * dt
    cmd = seq.step(t, prev, ik, dt=dt)
    step = float(np.max(np.abs(cmd - prev)))
    worst_step = max(worst_step, step)
    if seq.phase == sp.APPROACH:
        # distance to the pose must not increase on any joint
        if np.any(np.abs(cmd - pose) > np.abs(prev - pose) + 1e-12):
            monotone = False
    cmds.append(cmd.copy()); weights.append(seq.blend_weight); phases.append(seq.phase)
    prev = cmd
cmds = np.array(cmds); weights = np.array(weights)

record("approach is monotone toward the pose on every joint", monotone)
record("no commanded step ever exceeds velocity_limit * dt",
       worst_step <= max_step + 1e-12,
       f"worst {worst_step:.6f} rad/cycle, limit {max_step:.6f} "
       f"(= {VEL} rad/s at {FREQ} Hz)")
reached_at = next((i for i, p in enumerate(phases) if p != sp.APPROACH), None)
record("the pose is actually reached before blending starts",
       reached_at is not None and np.allclose(cmds[reached_at - 1], pose, atol=1e-3),
       f"approach ended at t={reached_at*dt:.2f}s, "
       f"max |cmd-pose| = {np.max(np.abs(cmds[reached_at-1]-pose)):.2e}")
record("approach is not cut short by the clock (0.6 rad at 5 rad/s needs >= 0.12s, "
       "but XR_START_T=3.0 governs)",
       reached_at is not None and reached_at * dt >= 3.0 - dt,
       f"t={reached_at*dt:.2f}s >= 3.0s")

blend_idx = [i for i, p in enumerate(phases) if p == sp.BLEND]
if blend_idx:
    w = weights[blend_idx]
    record("blend weight starts at 0 and is non-decreasing",
           w[0] <= 1e-9 and bool(np.all(np.diff(w) >= -1e-12)),
           f"w: {w[0]:.3f} -> {w[-1]:.3f} over {len(w)} cycles")
    record("blend weight reaches 1.0", float(weights.max()) >= 1.0 - 1e-9)
    span = (blend_idx[-1] - blend_idx[0] + 1) * dt
    record("blend takes XR_BLEND_T (2.0 s) +/- one cycle",
           abs(span - 2.0) <= dt + 1e-9, f"{span:.3f}s")
    # weight should track (t - t_blend_start)/blend_t
    t_start = blend_idx[0] * dt
    want = np.clip((np.array(blend_idx) * dt - t_start) / 2.0, 0, 1)
    record("blend weight is linear in time", float(np.max(np.abs(w - want))) < 1e-9,
           f"max deviation {np.max(np.abs(w - want)):.2e}")
else:
    record("blend phase happens at all", False)
record("ends in FOLLOW", phases[-1] == sp.FOLLOW)
record("the final command is the IK target", np.allclose(cmds[-1], ik))

# ======================================================================== 5
print("\n=== (5) a pose too far for XR_START_T takes LONGER, it does not speed up ===")
far = np.full(14, 3.0)                      # 3.0 rad away
seq = sp.StartPoseSequencer(pose=far, start_t=0.5, blend_t=1.0, velocity_limit=VEL)
seq.start(0.0, np.zeros(14))
prev = np.zeros(14); worst = 0.0
cmd_at_transition = None
t_transition = None
for i in range(int(10.0 * FREQ)):
    cmd = seq.step(i * dt, prev, ik, dt=dt)
    worst = max(worst, float(np.max(np.abs(cmd - prev))))
    # Only the FIRST cycle that is no longer APPROACH is interesting. On that cycle
    # step() flips the phase and still returns the approach command, so the command it
    # returns must already BE the pose. Later blend cycles legitimately move away from
    # it, so checking them all would flag every normal blend.
    if seq.phase != sp.APPROACH and cmd_at_transition is None:
        cmd_at_transition = cmd.copy()
        t_transition = i * dt
    prev = cmd
blended_early = (cmd_at_transition is None
                 or not np.allclose(cmd_at_transition, far, atol=1e-3))
record("still never exceeds the rate limit", worst <= max_step + 1e-12,
       f"worst {worst:.6f} vs {max_step:.6f}")
record("blend does NOT start before the pose is reached", not blended_early,
       f"3.0 rad at 5 rad/s needs 0.60 s; XR_START_T was only 0.5 s, and the approach "
       f"ran to t={t_transition:.2f}s" if t_transition is not None else "never left APPROACH")
record("...so the approach took LONGER than XR_START_T rather than moving faster",
       t_transition is not None and t_transition >= 0.6 - dt - 1e-9,
       f"t={t_transition:.2f}s vs XR_START_T=0.5s")

# ======================================================================== 6
print("\n=== (6) [r] twice re-arms from wherever the arms are now ===")
seq = sp.StartPoseSequencer(pose=pose, start_t=0.2, blend_t=0.2, velocity_limit=VEL)
seq.start(0.0, np.zeros(14))
prev = np.zeros(14)
for i in range(int(5.0 * FREQ)):
    prev = seq.step(i * dt, prev, ik, dt=dt)
record("first run reaches FOLLOW", seq.phase == sp.FOLLOW)
restart_from = np.full(14, -0.4)
seq.start(100.0, restart_from)
record("start() again returns to APPROACH", seq.phase == sp.APPROACH)
record("start() again resets the blend weight to 0", seq.blend_weight == 0.0)
cmd = seq.step(100.0 + dt, restart_from, ik, dt=dt)
record("and it approaches from the NEW current q, not the old one",
       float(np.max(np.abs(cmd - restart_from))) <= max_step + 1e-12,
       f"first step {float(np.max(np.abs(cmd - restart_from))):.6f} rad")

# ======================================================================== 7
print("\n=== (7) env plumbing and refusals ===")
os.environ["XR_START_POSE"] = str(ready)
os.environ["XR_START_T"] = "1.5"
os.environ["XR_BLEND_T"] = "0.5"
seq = sp.from_env(velocity_limit=5.0)
record("XR_START_POSE / XR_START_T / XR_BLEND_T are read",
       seq.enabled and seq.start_t == 1.5 and seq.blend_t == 0.5, seq.describe())
for name, value in (("XR_START_T", "-1"), ("XR_BLEND_T", "nope"), ("XR_START_T", "inf")):
    os.environ["XR_START_T"] = "1.5"; os.environ["XR_BLEND_T"] = "0.5"
    os.environ[name] = value
    try:
        sp.from_env(velocity_limit=5.0)
        record(f"{name}={value!r} refused", False, "it was ACCEPTED")
    except sp.StartPoseError as exc:
        record(f"{name}={value!r} refused", True, str(exc)[:55])
for k in ("XR_START_POSE", "XR_START_T", "XR_BLEND_T"):
    os.environ.pop(k, None)

# ======================================================================== 8
print("\n=== (8) the launcher accepts it, and refuses a bad one (domain 1 / lo) ===")


def run_launcher(env_extra, timeout, with_fake):
    env = dict(os.environ); env.update(env_extra)
    env.setdefault("PYTHONUNBUFFERED", "1")
    fake = None
    if with_fake:
        fake = subprocess.Popen(
            [PY, str(REPO / "tools/fake_lowstate.py"), "--domain", "1",
             "--iface", "lo", "--seconds", "70", "--rate", "500"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        time.sleep(3.0)
    with tempfile.NamedTemporaryFile("w+", suffix=".log", delete=False) as fh:
        log_path = fh.name
    try:
        with open(log_path, "wb") as sink:
            proc = subprocess.Popen(
                [PY, "teleop_hand_and_arm.py", "--sim", "--network-interface", "lo"],
                cwd=str(REPO / "teleop"), env=env, stdout=sink,
                stderr=subprocess.STDOUT, start_new_session=True)
            try:
                rc = proc.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                import signal
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except OSError:
                    pass
                proc.wait(timeout=10)
                rc = None
        return rc, Path(log_path).read_text(errors="replace")
    finally:
        os.unlink(log_path)
        if fake is not None:
            fake.terminate()
            try:
                fake.wait(timeout=5)
            except subprocess.TimeoutExpired:
                fake.kill()


rc, out = run_launcher({"XR_START_POSE": str(ready), "XR_ARM_VEL_LIMIT": "5"},
                       timeout=40, with_fake=True)
note = [l.strip() for l in out.splitlines() if l.startswith("[start-pose]")]
record("launcher loads XR_START_POSE and says so", any("14 joints loaded" in l for l in note),
       note[0] if note else "no [start-pose] line")
record("launcher still reaches 'Press [r]' with a start pose set",
       any(l.startswith("🟢  Press [r]") for l in out.splitlines()), f"exit={rc}")

bad_yaml = os.path.join(tmpdir, "bad.yaml")
write_pose(q, bad_yaml, drop={"left_elbow_joint"})
# No fake lowstate on purpose: a malformed pose file must be refused BEFORE any DDS
# exists, so this must fail fast rather than after the arm controller times out.
t0 = time.monotonic()
rc, out = run_launcher({"XR_START_POSE": bad_yaml}, timeout=60, with_fake=False)
refuse_secs = time.monotonic() - t0
record("launcher refuses a malformed start pose with exit 2", rc == 2, f"exit={rc}")
record("...and does so BEFORE any DDS init (fast, no arm-controller timeout)",
       rc == 2 and refuse_secs < 15 and "G1_29_ArmController" not in out,
       f"{refuse_secs:.1f}s")
record("...and names the missing joint",
       "left_elbow_joint" in out, "looked for 'left_elbow_joint' in the refusal")

# ======================================================================== summary
print("\n=== summary ===")
n_fail = sum(1 for _, ok, _ in results if not ok)
for name, ok, detail in results:
    print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"  -- {detail}" if detail else ""))
print(f"\n{len(results) - n_fail}/{len(results)} passed")
sys.exit(1 if n_fail else 0)
