#!/usr/bin/env python3
"""Offline test (simulated hands, no robot) of sweep.py --record on the thumbs (block stage TR): bend with nothing in the way
(finding reaches 0), rotation with a contact at 300 (finding stops at STATUS 3); fingers to 950 first and back, thumb FORCE_SET
rest + 400 and back, speeds back, write forms (six-value 1486, single 1498..1508 / 1522..1532)."""
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


def run(name, axis, hand):
    T.FRAMES.clear()
    env = dict(os.environ, CW_MOCK_PORTS="right:%d,left:%d" % (pr, pl), CW_MOCK_P3="1", G1RS_PRE_S="1", CW_HOLD_POST_S="2",
               CW_MOCK_STOP_FILE="/nonexistent/STOP")
    p = subprocess.run([sys.executable, "-B", S + "/sweep.py", "--record", "--hand", "left", "--axis", str(axis), "--dwell", "0.3", "--blip-dwell", "0.3",
                        "--live"], env=env, capture_output=True, text=True)
    o = p.stdout + p.stderr
    ws = [("six", list(struct.unpack(">6H", f[6:18]))) if f[0] == 0x10 else ("one", struct.unpack(">HH", f[1:5])) for f in T.FRAMES if f[0] in (0x10, 0x06)]
    bad = [w for w in ws if (w[0] == "six" and max(w[1]) > 1000) or (w[0] == "one" and not (1498 <= w[1][0] <= 1508 or 1522 <= w[1][0] <= 1532))]
    d = [l.split("dir ")[1].strip() for l in o.splitlines() if l.startswith("RUN ")][0]
    sm = json.load(open(os.path.join(d, "block", "summary.json")))
    res = [l for l in o.splitlines() if l.startswith("RESULT")][-1:]
    six = [w[1] for w in ws if w[0] == "six"]
    print("%-28s %s found %s | six %d single %d bad %s | end ANGLE_SET %s SPEED %s FORCE_SET %s" % (name, res, sm.get("floor_found"), len(six),
          sum(1 for w in ws if w[0] == "one"), bad, hand.ang_set, hand.speed, hand.force_set))
    print("    first six-value writes:", six[:5], "... last:", six[-2:])
    return res, sm, six, bad


right = ForceHand([547, 549, 552, 548, 460, 961])
left = ForceHand([540, 541, 550, 558, 470, 971], base=[50, 60, 20, 1050, 244, -180])
pr, pl = T.serve(right), T.serve(left)
ok = True
res, sm, six, bad = run("thumb bend, nothing in the way", 4, left)
h = six[0]
ok &= res == ["RESULT DONE"] and sm["floor_found"]["floor"] == 0 and six[1] == [950] * 4 + h[4:] and six[-1] == h and not bad
ok &= left.speed == [1000] * 6 and left.force_set == [2000] * 6 and left.ang_set == h
left.contact = [None, None, None, None, None, 300]
res, sm, six, bad = run("thumb rotation, contact at 300", 5, left)
h = six[0]
ok &= res == ["RESULT DONE"] and sm["floor_found"]["by"].startswith("STATUS 3") and abs(sm["floor_found"]["floor"] - 200) <= 3
ok &= [950] * 4 + [h[4], 1000] in six and six[-1] == h and not bad and left.speed == [1000] * 6 and left.force_set == [2000] * 6
print("MOCK THUMB RECORD TEST", "OK" if ok else "FAILED")
