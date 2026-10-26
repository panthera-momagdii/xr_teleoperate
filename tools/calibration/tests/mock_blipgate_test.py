#!/usr/bin/env python3
"""Offline test (simulated hands, no robot) of press.py --blip-gate --gate all: blip held, operator gate after the blip, then one
gate per level; STOP at the blip gate runs no level (no FORCE_SET write); contact on the scale gives STATUS 3 at the limit."""
import os
import struct
import subprocess
import sys

sys.path.insert(0, "/home/unitree/calibration_ws/tests")
sys.path.insert(0, "/home/unitree/h2_20261007/tools")
import test_h2_offline as T  # noqa: E402
from mock_p3_parts import ForceHand  # noqa: E402

S = "/home/unitree/calibration_ws/scripts"


def run(name, argv, answers):
    T.FRAMES.clear()
    env = dict(os.environ, CW_MOCK_PORTS="right:%d,left:%d" % (pr, pl), CW_MOCK_P3="1", G1RS_PRE_S="1", CW_HOLD_POST_S="2", G1RS_DW="0.05",
               CW_MOCK_STOP_FILE="/nonexistent/STOP")
    p = subprocess.Popen([sys.executable, "-B"] + argv, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    out, n = [], 0
    for line in p.stdout:
        out.append(line)
        if line.startswith("WAITING_FOR_OPERATOR"):
            d = line.split("create ")[1].split("/CONTINUE")[0]
            open(os.path.join(d, answers[n]), "w").write("mock answer %d" % n)
            n += 1
    p.wait()
    o = "".join(out)
    ws = [("six", list(struct.unpack(">6H", f[6:18]))) if f[0] == 0x10 else ("one", struct.unpack(">HH", f[1:5])) for f in T.FRAMES if f[0] in (0x10, 0x06)]
    res = [l.strip() for l in out if l.startswith("RESULT")][-1:]
    print("%-34s %s waits %d | writes %s" % (name, res, o.count("WAITING_FOR_OPERATOR"), [w[1] for w in ws if w[0] == "one"]))
    for l in out:
        if l.startswith(("PRESS ", "GATE", "STOPPED")):
            print("     " + l.rstrip())
    return res, o, ws


right = ForceHand([547, 549, 552, 548, 460, 961])
left = ForceHand([539, 542, 555, 561, 465, 972], contact=[None, None, 450, None, None, None], base=[44, 31, -37, 997, 245, -227])
pr, pl = T.serve(right), T.serve(left)
ok = True
res, o, ws = run("middle, blip gate + 2 levels", [S + "/press.py", "--hand", "left", "--axis", "2", "--levels", "200,300", "--gate", "all", "--blip-gate",
                 "--blip-dwell", "0.3", "--pause", "0.5", "--close-to", "200", "--live"], ["CONTINUE", "CONTINUE"])
ok &= res == ["RESULT DONE"] and o.count("WAITING_FOR_OPERATOR") == 2 and ("one", (1502, 163)) in ws and ("one", (1502, 2000)) in ws and "status 3" in o
res, o, ws = run("middle, STOP at the blip gate", [S + "/press.py", "--hand", "left", "--axis", "2", "--levels", "200,300", "--gate", "all", "--blip-gate",
                 "--blip-dwell", "0.3", "--pause", "0.5", "--close-to", "200", "--live"], ["STOP"])
ok &= res == ["RESULT DONE"] and not any(w[0] == "one" and 1498 <= w[1][0] <= 1504 for w in ws) and "STOPPED after the blip" in o and left.speed == [1000] * 6
print("MOCK BLIP GATE TEST", "OK" if ok else "FAILED")
