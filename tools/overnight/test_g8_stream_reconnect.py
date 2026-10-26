#!/usr/bin/env python3
"""G8 (RT5b): the headset image task survives websocket drops quietly. No DDS, no robot.

televuer + teleop/utils/televuer_stream.py serve a synthetic head image on :8012 to
tools/fake_xr_client.py, which drops its websocket three times and reconnects. Checked:
no traceback and no "Websocket session is missing" in the output, and video bytes flow
again after every reconnect. The same run with upstream's task (the guard NOT installed)
is the control: it must show the traceback, or this test proves nothing.

Run:  python tools/overnight/test_g8_stream_reconnect.py
"""
import os, re, subprocess, sys, time
from pathlib import Path
REPO = Path(__file__).resolve().parent.parent.parent
PY = sys.executable
results = []
def record(name, ok, detail=""):
    results.append((name, bool(ok), detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  -- {detail}" if detail else ""), flush=True)

CHILD = r'''
import os, re, subprocess, sys, time
import numpy as np
sys.path[:0] = [%r, %r]
if os.environ.get("G8_GUARD") == "1":
    from teleop.utils import televuer_stream
    print(televuer_stream.install(), flush=True)
from televuer import TeleVuerWrapper
from fake_xr_client import FakeXR
def acked():
    out = subprocess.run(["ss", "-tinH", "state", "established", "( sport = :8012 )"], capture_output=True, text=True).stdout
    return sum(int(x) for x in re.findall(r"bytes_acked:(\d+)", out))
w = TeleVuerWrapper(use_hand_tracking=True, binocular=False, img_shape=(480, 640), display_mode="immersive",
                    zmq=True, webrtc=False, arm_reference_mode="head_yaw")
xr = FakeXR().start()
rng = np.random.default_rng(0)
def feed(sec):
    t = time.monotonic()
    while time.monotonic() - t < sec:
        w.render_to_xr(rng.integers(0, 255, (480, 640, 3), dtype=np.uint8)); time.sleep(1 / 30)
t0 = time.monotonic()
while not xr.connected and time.monotonic() - t0 < 30: time.sleep(0.1)
feed(2.0)
for i in range(3):
    xr.drop(); feed(0.3)
    t = time.monotonic()
    while xr.connects < i + 2 and time.monotonic() - t < 10: time.sleep(0.1)
    feed(1.0); a = acked(); feed(2.0); b = acked()
    print(f"CYCLE {i} connects={xr.connects} bytes_after_reconnect={b - a}", flush=True)
xr.stop(); w.close()
sys.path.insert(0, %r)
from tools._procs import reap_child_processes; reap_child_processes(); os._exit(0)
''' % (str(REPO), str(REPO / "tools"), str(REPO))

def run(guard):
    env = dict(os.environ, G8_GUARD="1" if guard else "0", PYTHONUNBUFFERED="1")
    p = subprocess.run([PY, "-c", CHILD], env=env, capture_output=True, text=True, timeout=120,
                       cwd=str(REPO))
    return p.returncode, p.stdout + p.stderr

sys.path.insert(0, str(REPO))
from tools import port_guard  # noqa: E402
record("8012 free before the test", port_guard.port_is_free(8012))

print("=== control: upstream task, no guard ===", flush=True)
rc, out = run(False)
control_hits = len(re.findall(r"Websocket session is missing", out))
record("control shows the upstream traceback (the test can see it)", control_hits > 0,
       f"{control_hits} 'Websocket session is missing' lines")

print("=== guarded task (teleop/utils/televuer_stream.py) ===", flush=True)
time.sleep(2)
rc, out = run(True)
cycles = re.findall(r"CYCLE (\d) connects=(\d+) bytes_after_reconnect=(\d+)", out)
record("three drop/reconnect cycles ran", len(cycles) == 3, str(cycles))
record("no 'Websocket session is missing'", "Websocket session is missing" not in out)
record("no traceback", "Traceback" not in out and "Task exception was never retrieved" not in out)
record("video bytes flow again after every reconnect",
       len(cycles) == 3 and all(int(c[2]) > 100000 for c in cycles),
       ", ".join(f"cycle {c[0]}: {int(c[2]) / 1e6:.2f} MB in 2 s" for c in cycles))
record("the guard announced itself", "[video] headset stream:" in out)
time.sleep(1)
record("8012 free after", port_guard.port_is_free(8012))

print("\n=== summary ===")
n_fail = sum(1 for _, ok, _ in results if not ok)
for name, ok, d in results:
    print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"  -- {d}" if d else ""))
print(f"\n{len(results) - n_fail}/{len(results)} passed")
sys.exit(1 if n_fail else 0)
