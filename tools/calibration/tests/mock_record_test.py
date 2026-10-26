#!/usr/bin/env python3
"""Offline test (simulated hands, no robot) of sweep.py --record (block stage R): left index with its measured floor 35 (floor pass
to 50), left little with no measured floor and a contact (floor-finding pass stops at STATUS 3), left ring with no measured floor
and nothing in the way (finding reaches 0, floor pass to 15), and a touch in the clip's floor pass (stops the run)."""
import json
import os
import struct
import subprocess
import sys

sys.path.insert(0, "/home/unitree/calibration_ws/tests")
sys.path.insert(0, "/home/unitree/h2_20261007/tools")
import test_h2_offline as T  # noqa: E402
from mock_p3_parts import ForceHand  # noqa: E402

S = "/home/unitree/calibration_ws/scripts"


def run(name, argv, hand):
    T.FRAMES.clear()
    env = dict(os.environ, CW_MOCK_PORTS="right:%d,left:%d" % (pr, pl), CW_MOCK_P3="1", G1RS_PRE_S="1", CW_HOLD_POST_S="2",
               CW_MOCK_STOP_FILE="/nonexistent/STOP")
    p = subprocess.run([sys.executable, "-B"] + argv + ["--dwell", "0.3", "--blip-dwell", "0.3", "--live"], env=env, capture_output=True, text=True)
    o = p.stdout + p.stderr
    ws = [("six", list(struct.unpack(">6H", f[6:18]))) if f[0] == 0x10 else ("one", struct.unpack(">HH", f[1:5])) for f in T.FRAMES if f[0] in (0x10, 0x06)]
    d = [l.split("dir ")[1].strip() for l in o.splitlines() if l.startswith("RUN ")][0]
    sm = json.load(open(os.path.join(d, "block", "summary.json")))
    res = [l for l in o.splitlines() if l.startswith("RESULT")][-1:]
    print("%-36s %s found %s floor target %s | writes six %d single %d | end SPEED %s FORCE_SET %s" % (name, res, sm.get("floor_found"), sm.get("record_floor_target"),
          sum(1 for w in ws if w[0] == "six"), sum(1 for w in ws if w[0] == "one"), hand.speed, hand.force_set))
    return res, o, ws, sm


right = ForceHand([547, 549, 552, 548, 460, 961])
left = ForceHand([542, 545, 543, 558, 470, 969], base=[50, 60, 20, 1050, 230, -130])
pr, pl = T.serve(right), T.serve(left)
ok = True
res, o, ws, sm = run("index, measured floor 35", [S + "/sweep.py", "--record", "--hand", "left", "--axis", "3"], left)
six = [w[1] for w in ws if w[0] == "six"]
h = six[0]
ok &= res == ["RESULT DONE"] and six == [h, h[:3] + [h[3] + 10] + h[4:], h, h[:3] + [950] + h[4:], h, h[:3] + [50] + h[4:], h] and sm["floor_found"] is None
ok &= ("one", (1504, 1450)) in ws and ("one", (1504, 2000)) in ws and left.speed == [1000] * 6 and left.force_set == [2000] * 6
left.contact = [300, None, None, None, None, None]
res, o, ws, sm = run("little, no floor, contact at 300", [S + "/sweep.py", "--record", "--hand", "left", "--axis", "0"], left)
ff = sm["floor_found"] or {}
ok &= res == ["RESULT DONE"] and ff.get("by", "").startswith("STATUS 3") and abs(ff["floor"] - 200) <= 3 and sm["record_floor_target"] == ff["floor"] + 15
ok &= left.speed == [1000] * 6 and left.force_set == [2000] * 6
left.contact = [None] * 6
res, o, ws, sm = run("ring, no floor, nothing in the way", [S + "/sweep.py", "--record", "--hand", "left", "--axis", "1"], left)
ff = sm["floor_found"] or {}
ok &= res == ["RESULT DONE"] and ff.get("floor") == 0 and sm["record_floor_target"] == 15 and left.force_set == [2000] * 6
left.contact = [None, None, None, 300, None, None]
res, o, ws, sm = run("index, touch in the clip floor pass", [S + "/sweep.py", "--record", "--hand", "left", "--axis", "3"], left)
ok &= res == ["RESULT ABORTED"] and "STATUS_NOT_0_1_2" in o
print("MOCK RECORD TEST", "OK" if ok else "FAILED")
