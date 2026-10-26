#!/usr/bin/env python3
"""Offline test (simulated hands, no robot): sweep.py from the open pose (fingers held at 948): the range 650..950 is allowed below the
hold value; the finger goes 948 -> 650 -> 950 -> 650 -> 948; speed back to 1000."""
import os
import struct
import subprocess
import sys

sys.path.insert(0, "/home/unitree/calibration_ws/tests")
sys.path.insert(0, "/home/unitree/h2_20261007/tools")
import test_h2_offline as T  # noqa: E402
from mock_p3_parts import ForceHand  # noqa: E402

right = ForceHand([547, 549, 552, 548, 460, 961])
left = ForceHand([948, 948, 948, 948, 455, 972])
pr, pl = T.serve(right), T.serve(left)
env = dict(os.environ, CW_MOCK_PORTS="right:%d,left:%d" % (pr, pl), G1RS_PRE_S="1", CW_HOLD_POST_S="2", G1RS_DW="0.05", CW_MOCK_STOP_FILE="/nonexistent/STOP")
p = subprocess.run([sys.executable, "-B", "/home/unitree/calibration_ws/scripts/sweep.py", "--hand", "left", "--axis", "2", "--range", "650,950", "--live"],
                   env=env, capture_output=True, text=True)
six = [list(struct.unpack(">6H", f[6:18]))[2] for f in T.FRAMES if f[0] == 0x10]
res = [l for l in (p.stdout + p.stderr).splitlines() if l.startswith("RESULT")][-1:]
print("sweep from 948:", res, "middle targets", six, "speed", left.speed)
ok = res == ["RESULT DONE"] and six == [948, 958, 948, 650, 950, 650, 948] and left.speed == [1000] * 6
print("MOCK SWEEP OPEN TEST", "OK" if ok else "FAILED")
