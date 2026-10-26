#!/usr/bin/env python3
"""Offline test (simulated hands, no robot) of scale_test.py --soft-stop (18 Oct): leftovers from the stopped run (middle limit 755,
index 1766, speeds 100) are overwritten at the start; contact search, presses at contact - 20, 60, 100 with the register at 2000;
ends at 2000 / 1000, open at 950 and still, packed; and the software stop (rest + 300 for the test) opens that finger, skips its
remaining presses and goes on to the next finger."""
import os
import shutil
import struct
import subprocess
import sys
import time
import json

sys.path.insert(0, "/home/unitree/calibration_ws/tests")
sys.path.insert(0, "/home/unitree/h2_20261007/tools")
import test_h2_offline as T  # noqa: E402
from mock_p3_parts import ForceHand  # noqa: E402

S = "/home/unitree/calibration_ws/scripts"
SP = "/tmp/claude-1000/-home-unitree/f20c71cb-b449-4e36-bc97-51c33ddcb58b/scratchpad/mock_scale_soft"
shutil.rmtree(SP, ignore_errors=True)
os.makedirs(SP)


class CoolHand(ForceHand):
    def read(self, a, n):
        if a == 1618:
            return [0x2020] * 3
        return super().read(a, n)


def fresh():
    h = CoolHand([934, 930, 951, 948, 453, 972], contact=[None, None, 898, 883, None, None], base=[6, 64, 55, 1066, 250, -214])
    h.force_set = [2000, 2000, 755, 1766, 2000, 2000]          # leftovers from the stopped run
    h.speed = [1000, 1000, 100, 100, 1000, 1000]
    return h


def run(name, extra, hand):
    T.FRAMES.clear()
    env = dict(os.environ, CW_MOCK_PORTS="right:%d,left:%d" % (pr, pl), CW_MOCK_P3="1", G1RS_PRE_S="1", CW_MOCK_STOP_FILE=os.path.join(SP, "STOP"),
               CW_MOCK_PACK_DIR=SP)
    t0 = time.time()
    p = subprocess.run([sys.executable, "-B", S + "/scale_test.py", "--soft-stop"] + extra + ["--live"], env=env, capture_output=True, text=True)
    dt = time.time() - t0
    o = p.stdout + p.stderr
    ws = [("six", list(struct.unpack(">6H", f[6:18]))) if f[0] == 0x10 else ("one", struct.unpack(">HH", f[1:5])) for f in T.FRAMES if f[0] in (0x10, 0x06)]
    d = [l.split("dir ")[1].strip() for l in o.splitlines() if l.startswith("RUN ")][0]
    sm = json.load(open(os.path.join(d, "block", "summary.json")))
    res = [l for l in o.splitlines() if l.startswith("RESULT")][-1:]
    print("%-34s %s in %.0f s | presses %d | max gap %s s | FORCE_SET writes %s | end FORCE_SET %s SPEED %s ANGLE_SET %s" % (name, res, dt, len(sm["presses"]),
          sm["max_gap_s"], [w[1] for w in ws if w[0] == "one" and 1498 <= w[1][0] <= 1508], hand.force_set, hand.speed, hand.ang_set))
    for l in o.splitlines():
        if l.startswith(("CONTACT", "PRESS ", "SOFT STOP", "STOP", "END", "PACKED")):
            print("     " + l[:170])
    return res, o, ws, sm, dt


right = CoolHand([1000, 549, 552, 550, 450, 962])
left = fresh()
pr, pl = T.serve(right), T.serve(left)
ok = True
res, o, ws, sm, dt = run("soft run, 8 s holds", [], left)
fsw = [w[1] for w in ws if w[0] == "one" and 1498 <= w[1][0] <= 1508]
ok &= res == ["RESULT DONE"] and len(sm["presses"]) == 6 and sorted(fsw[:2]) == [(1502, 2000), (1504, 2000)] and (1504, 2000) in fsw[2:]
ok &= left.force_set == [2000] * 6 and left.speed == [1000] * 6 and left.ang_set[2:4] == [950, 950] and sm["max_gap_s"] <= 15 and "verified" in o
print("     total %.0f s" % dt)
left2 = fresh(); pl = T.serve(left2)
res, o, ws, sm, dt = run("software stop at rest + 300", ["--hold-s", "1", "--safety-g", "300"], left2)
ok &= res == ["RESULT DONE"] and "SOFT STOP index" in o and any(p.get("finger") == "middle" and "reached" in p for p in sm["presses"]) and left2.ang_set[2:4] == [950, 950]
print("MOCK SCALE SOFT TEST", "OK" if ok else "FAILED")
