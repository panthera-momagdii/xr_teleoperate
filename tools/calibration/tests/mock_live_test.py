#!/usr/bin/env python3
"""Offline test of the calibration_ws live paths against simulated hands on localhost (h2_20261007 test fakes; no real hand, no
DDS): sweep, steps (100, 500, 1000) and wake_test on the right hand from the power-up state. Checks that every write is a
six-value ANGLE_SET at 1486, a single-register SPEED_SET of the finger, or a Part B single-register ANGLE_SET, never 0xFFFF, and
that the hand ends at its start pose (ANGLE_SET = hold, start speeds)."""
import glob
import json
import os
import struct
import subprocess
import sys

sys.path.insert(0, "/home/unitree/h2_20261007/tools")
import test_h2_offline as T  # noqa: E402
import test_h2_v2_offline as T2  # noqa: E402

PY = sys.executable
S = "/home/unitree/calibration_ws/scripts"
right = T2.Hand2([547, 549, 552, 548, 460, 961], [0, 0, 0, 0, 0, 1000], [1000] * 6)
left = T2.Hand2([543, 547, 547, 547, 479, 977], [0, 0, 0, 0, 0, 1000], [1000] * 6)
pr, pl = T.serve(right), T.serve(left)
env = dict(os.environ, CW_MOCK_PORTS="right:%d,left:%d" % (pr, pl), G1RS_PRE_S="1", CW_HOLD_POST_S="2", G1RS_DW="0.05", G1RS_OBS_S="0.5", G1RS_WAIT_S="1")
cases = [("sweep", [S + "/sweep.py", "--hand", "right", "--axis", "3", "--live", "--note", "MOCK TEST"]),
         ("steps", [S + "/steps.py", "--hand", "right", "--axis", "2", "--speeds", "100,500,1000", "--reps", "2", "--live", "--note", "MOCK TEST"]),
         ("wake_test", [S + "/wake_test.py", "--hand", "right", "--live", "--note", "MOCK TEST"])]
ok = True
for name, argv in cases:
    T.FRAMES.clear()
    r = subprocess.run([PY, "-B"] + argv, env=env, capture_output=True, text=True)
    res = [l for l in r.stdout.splitlines() if l.startswith("RESULT")][-1:]
    w = [f for f in T.FRAMES if f[0] != 3]
    bad = []
    for f in w:
        if f[0] == 0x10:
            a, n = struct.unpack(">HH", f[1:5])
            vals = struct.unpack(">6H", f[6:18])
            if a != 1486 or n != 6 or max(vals) > 1000:
                bad.append(f.hex())
        elif f[0] == 0x06:
            a, v = struct.unpack(">HH", f[1:5])
            if v == 0xFFFF or a % 2 or not (1486 <= a <= 1496 or 1522 <= a <= 1528):
                bad.append(f.hex())
        else:
            bad.append(f.hex())
    n16 = sum(1 for f in w if f[0] == 0x10)
    n06 = len(w) - n16
    print("%-9s %s  writes: %d six-value, %d single-register; bad frames %s; end ANGLE_SET %s SPEED %s" % (name, res, n16, n06, bad, right.ang_set, right.speed))
    ok = ok and res == ["RESULT DONE"] and not bad
    if res != ["RESULT DONE"]:
        print(r.stdout[-2500:], r.stderr[-1500:])
print("MOCK LIVE TEST", "OK" if ok else "FAILED")
