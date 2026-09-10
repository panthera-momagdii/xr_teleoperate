#!/usr/bin/env python3
"""G5: Inspire clamp, slew and first-command ramp. DOMAIN 1, LOOPBACK ONLY.

Two layers:
  * the limiter as pure arithmetic (fast, exhaustive)
  * the REAL Inspire_Controller_FTP driven against tools/fake_inspire_state.py on
    domain 1, with the commands it publishes captured off the wire -- because a slew
    that is right in a unit test and not wired into the control loop is worth nothing.

Run:  python tools/overnight/test_g5_inspire_slew.py
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

PY = sys.executable
results = []


def record(name, ok, detail=""):
    results.append((name, ok, detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  -- {detail}" if detail else ""))


# ======================================================================== 1
print("\n=== (1) the limiter, as arithmetic ===")
from teleop.robot_control import hand_config as hc  # noqa: E402

record("default XR_HAND_SLEW is 1500 units/s", hc.INSPIRE_SLEW_UNITS_PER_S == 1500.0,
       f"{hc.INSPIRE_SLEW_UNITS_PER_S} units/s")
record("at 100 Hz that is 15 units/cycle", hc.inspire_max_step(100.0) == 15.0,
       f"{hc.inspire_max_step(100.0)} units/cycle, full travel in "
       f"{1000/hc.INSPIRE_SLEW_UNITS_PER_S:.2f}s")

FPS = 100.0
step = hc.inspire_max_step(FPS)

# a step input from 0 to 1000 must ramp, not jump
last = np.zeros(6)
traj = []
for _ in range(200):
    last = hc.limit_inspire_command(np.full(6, 1000.0), last, FPS)
    traj.append(last.copy())
traj = np.array(traj)
deltas = np.diff(np.vstack([np.zeros(6), traj]), axis=0)
record("step 0 -> 1000 never moves more than the slew per cycle",
       float(np.max(np.abs(deltas))) <= step + 1e-9,
       f"worst {float(np.max(np.abs(deltas))):.4f} vs {step}")
record("step 0 -> 1000 is monotone", bool(np.all(deltas >= -1e-12)))
n_cycles = int(np.argmax(np.all(traj >= 1000 - 1e-9, axis=1))) + 1
record("it takes the predicted number of cycles to arrive",
       n_cycles == int(np.ceil(1000.0 / step)),
       f"{n_cycles} cycles = {n_cycles/FPS:.2f}s (predicted {int(np.ceil(1000.0/step))})")
record("and it does arrive exactly at 1000", np.allclose(traj[-1], 1000.0))

# and the reverse
last = np.full(6, 1000.0)
back = []
for _ in range(200):
    last = hc.limit_inspire_command(np.zeros(6), last, FPS)
    back.append(last.copy())
back = np.array(back)
d = np.diff(np.vstack([np.full(6, 1000.0), back]), axis=0)
record("step 1000 -> 0 ramps down at the same rate and reaches 0",
       float(np.max(np.abs(d))) <= step + 1e-9 and np.allclose(back[-1], 0.0))

# clamping
record("a target above 1000 clamps to 1000",
       float(hc.limit_inspire_command(np.full(6, 5000.0), np.full(6, 995.0), FPS)[0]) == 1000.0)
record("a target below 0 clamps to 0",
       float(hc.limit_inspire_command(np.full(6, -5000.0), np.full(6, 5.0), FPS)[0]) == 0.0)
out = hc.limit_inspire_command(np.full(6, 5000.0), np.full(6, 0.0), FPS)
record("clamping never lets a step exceed the slew", float(out[0]) == step, f"{out[0]}")
record("a last_cmd already outside the range is pulled back in, not amplified",
       float(hc.limit_inspire_command(np.full(6, 500.0), np.full(6, 1500.0), FPS)[0])
       == 1000.0)

# per-finger, not ganged
tgt = np.array([1000.0, 0.0, 500.0, 500.0, 1000.0, 0.0])
last = np.full(6, 500.0)
out = hc.limit_inspire_command(tgt, last, FPS)
record("the limit is applied PER FINGER",
       np.allclose(out, [515.0, 485.0, 500.0, 500.0, 515.0, 485.0]),
       np.round(out, 1).tolist())

# the knob is honoured
os.environ["XR_HAND_SLEW"] = "300"
proc = subprocess.run(
    [PY, "-c",
     "import sys; sys.path.insert(0, %r);"
     "from teleop.robot_control import hand_config as h;"
     "print(h.INSPIRE_SLEW_UNITS_PER_S, h.inspire_max_step(100.0))" % str(REPO)],
    capture_output=True, text=True, timeout=120)
record("XR_HAND_SLEW is read from the environment",
       proc.stdout.strip() == "300.0 3.0", proc.stdout.strip() or proc.stderr[-120:])
os.environ["XR_HAND_SLEW"] = "0"
proc = subprocess.run(
    [PY, "-c", "import sys; sys.path.insert(0, %r);"
               "from teleop.robot_control import hand_config" % str(REPO)],
    capture_output=True, text=True, timeout=120)
record("XR_HAND_SLEW=0 is refused at import", proc.returncode != 0,
       [l for l in proc.stderr.splitlines() if "XR_HAND_SLEW" in l][:1])
os.environ.pop("XR_HAND_SLEW", None)

# ======================================================================== 2
print("\n=== (2) the REAL controller against a fake hand, on domain 1 ===")
# The fake publishes angle_act = --angle on every DOF. The controller must therefore
# start its slew from there, not from 1000 (fully open), which is what it used to send.
START_ANGLE = 200

# The listener runs in its OWN process. Capturing in the same process that constructs
# the controller does not work: the controller forks its control loop
# (Process(target=self.control_process)) and the forked child inherits the publishers,
# and a subscriber created in the parent before the fork never sees the child's writes.
# A separate process has its own participant and discovers it normally.
listener_src = r'''
import os, sys, time
sys.path.insert(0, %r)
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
ChannelFactoryInitialize(1, "lo")
from inspire_sdkpy import inspire_dds
def on_cmd(msg):
    print("CMD", round(time.monotonic(), 4), *list(msg.angle_set), flush=True)
sub = ChannelSubscriber("rt/inspire_hand/ctrl/l", inspire_dds.inspire_hand_ctrl)
sub.Init(on_cmd)
time.sleep(float(sys.argv[1]))
os._exit(0)
''' % str(REPO)

harness = r'''
import os, sys, time
import numpy as np
sys.path.insert(0, %r)
import logging_mp
logging_mp.basicConfig(level=logging_mp.INFO)
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
ChannelFactoryInitialize(1, "lo")
from multiprocessing import Array, Lock
from teleop.robot_control.robot_hand_inspire import Inspire_Controller_FTP
left_in  = Array('d', 75, lock=True)
right_in = Array('d', 75, lock=True)
lock = Lock()
state_out  = Array('d', 12, lock=False)
action_out = Array('d', 12, lock=False)
ctrl = Inspire_Controller_FTP(left_in, right_in, lock, state_out, action_out)
print("CONSTRUCTED", flush=True)
time.sleep(4.0)
os._exit(0)
''' % str(REPO)


def spawn(argv, cwd=None):
    fh = tempfile.NamedTemporaryFile("w+", suffix=".log", delete=False)
    fh.close()
    sink = open(fh.name, "wb")
    proc = subprocess.Popen(argv, cwd=cwd, stdout=sink, stderr=subprocess.STDOUT,
                            start_new_session=True)
    return proc, sink, fh.name


def reap(proc, sink, path, timeout=20):
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        import signal
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except OSError:
            pass
        proc.wait(timeout=10)
    sink.close()
    text = Path(path).read_text(errors="replace")
    os.unlink(path)
    return text


with tempfile.NamedTemporaryFile("w", suffix=".py", delete=False) as fh:
    listener_path = fh.name
    fh.write(listener_src)
with tempfile.NamedTemporaryFile("w", suffix=".py", delete=False) as fh:
    harness_path = fh.name
    fh.write(harness)

fake = subprocess.Popen(
    [PY, str(REPO / "tools/fake_inspire_state.py"), "--domain", "1", "--iface", "lo",
     "--angle", str(START_ANGLE), "--seconds", "40"],
    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
lp = hp = None
try:
    time.sleep(3.0)
    lp = spawn([PY, listener_path, "14"], cwd=str(REPO / "teleop"))
    time.sleep(2.0)                               # let the listener discover first
    hp = spawn([PY, harness_path], cwd=str(REPO / "teleop"))
    out = reap(*hp, timeout=60)
    listen_out = reap(*lp, timeout=40)
finally:
    fake.terminate()
    try:
        fake.wait(timeout=5)
    except subprocess.TimeoutExpired:
        fake.kill()
os.unlink(listener_path); os.unlink(harness_path)

harness_out = out
record("Inspire_Controller_FTP constructs at all", "CONSTRUCTED" in harness_out,
       "AttributeError: _on_state" if "_on_state" in harness_out
       else harness_out[-160:].replace("\n", " "))

out = listen_out
cmds = []
for line in out.splitlines():
    if line.startswith("CMD "):
        parts = line.split()
        cmds.append((float(parts[1]), [int(float(v)) for v in parts[2:]]))

record("the controller started and published commands", len(cmds) > 10,
       f"{len(cmds)} commands captured")

if cmds:
    arr = np.array([c[1] for c in cmds], dtype=float)
    first = arr[0]
    record("the FIRST command starts from the measured state, not from fully open",
           bool(np.all(np.abs(first - START_ANGLE) <= 20)),
           f"first={first.tolist()} measured={START_ANGLE} (was 1000 before this patch)")
    d = np.abs(np.diff(arr, axis=0))
    record("no published command ever jumps more than the slew per cycle",
           float(d.max()) <= 15.0 + 1.0,
           f"worst {float(d.max()):.0f} units/cycle vs 15 allowed")
    record("every published value is inside 0..1000",
           bool(np.all(arr >= 0) and np.all(arr <= 1000)),
           f"range {arr.min():.0f}..{arr.max():.0f}")
    # Printed by the CONTROLLER, so it is in the harness's output -- and logging_mp
    # renders through rich, which wraps the message into a narrow column and injects a
    # "robot_hand_inspire.py:NNN" gutter into the middle of the first line. Match the
    # pieces, not the sentence. (Same trap as G1; see notices() there.)
    # rich injects its "robot_hand_inspire.py:NNN" gutter BETWEEN words, so even after
    # flattening "slew starts" is not contiguous -- it renders as
    #     "[Inspire_Controller_FTP] slew robot_hand_inspire.py:358 starts from the
    #      measured state: left=[200.0, ...]"
    # Assert the claim numerically instead: the line must report the measured angle.
    import re as _re
    flat_h = _re.sub(r"\s+", " ", harness_out)
    record("the slew line reports the MEASURED start, numerically",
           bool(_re.search(r"left=\[%d\.0" % START_ANGLE, flat_h))
           and "measured" in flat_h,
           f"expected left=[{START_ANGLE}.0 in the controller's own log line")
else:
    record("controller command capture", False, out[-400:])

# ======================================================================== 3
print("\n=== (3) the stale fail-closed assertions are gone ===")
src = (REPO / "tools/test_dex5_failclosed.py").read_text()
record("no 'no HandState_' assertion remains",
       'and "no HandState_" in str(exc)' not in src)
record("it now asserts the real wording and both topic names",
       src.count('"no hand state on" in str(exc)') == 2
       and src.count("hand_config.TOPIC_LEFT_STATE in str(exc)") == 2)

proc = subprocess.run([str(REPO / "tools/overnight/run_failclosed.sh"), "4"],
                      capture_output=True, text=True, timeout=600)
tail = proc.stdout.strip().splitlines()[-1] if proc.stdout.strip() else ""
record("the whole fail-closed suite is green (was 4/6)",
       proc.returncode == 0 and "0 mismatch" in tail, tail)

# ======================================================================== 4
print("\n=== (4) params_proto is pinned ===")
req = (REPO / "requirements.txt").read_text()
record("requirements.txt pins params_proto==2.13.2", "params_proto==2.13.2" in req)
from importlib.metadata import version  # noqa: E402
record("and that is what is installed", version("params_proto") == "2.13.2",
       version("params_proto"))

# ======================================================================== summary
print("\n=== summary ===")
n_fail = sum(1 for _, ok, _ in results if not ok)
for name, ok, detail in results:
    print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"  -- {detail}" if detail else ""))
print(f"\n{len(results) - n_fail}/{len(results)} passed")
sys.exit(1 if n_fail else 0)
