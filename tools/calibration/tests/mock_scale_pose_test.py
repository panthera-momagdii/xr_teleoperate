#!/usr/bin/env python3
"""Offline test (simulated hands, no robot) of the 16 Oct scale-test sequence on the left hand: press.py --pose open (fingers to 950,
rest values saved), press middle from the open pose with --blip-gate --gate all (plate contact at 700), press index, --pose open again
refused, --pose rest (fingers back to the saved rest values). Checks that every press starts, opens back and ends at the open pose."""
import os
import shutil
import struct
import subprocess
import sys

sys.path.insert(0, "/home/unitree/calibration_ws/tests")
sys.path.insert(0, "/home/unitree/h2_20261007/tools")
import test_h2_offline as T  # noqa: E402
from mock_p3_parts import ForceHand  # noqa: E402

S = "/home/unitree/calibration_ws/scripts"
SD = "/tmp/claude-1000/-home-unitree/f20c71cb-b449-4e36-bc97-51c33ddcb58b/scratchpad/mock_state"
shutil.rmtree(SD, ignore_errors=True)
os.makedirs(SD)


def run(name, argv, answers=()):
    T.FRAMES.clear()
    env = dict(os.environ, CW_MOCK_PORTS="right:%d,left:%d" % (pr, pl), CW_MOCK_P3="1", G1RS_PRE_S="1", CW_HOLD_POST_S="2", G1RS_DW="0.05",
               CW_MOCK_STOP_FILE="/nonexistent/STOP", CW_MOCK_STATE_DIR=SD)
    p = subprocess.Popen([sys.executable, "-B", S + "/press.py"] + argv, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    out, n = [], 0
    for line in p.stdout:
        out.append(line)
        if line.startswith("WAITING_FOR_OPERATOR"):
            d = line.split("create ")[1].split("/CONTINUE")[0]
            open(os.path.join(d, answers[n]), "w").write("mock %d" % n)
            n += 1
    p.wait()
    o = "".join(out)
    ws = [("six", list(struct.unpack(">6H", f[6:18]))) if f[0] == 0x10 else ("one", struct.unpack(">HH", f[1:5])) for f in T.FRAMES if f[0] in (0x10, 0x06)]
    res = [l.strip() for l in out if l.startswith(("RESULT", "REFUSED"))][-1:]
    print("%-30s %s | six-value %s | end ANGLE_SET %s SPEED %s FORCE_SET %s" % (name, res, [w[1] for w in ws if w[0] == "six"][:6], left.ang_set, left.speed, left.force_set))
    for l in out:
        if l.startswith(("PRESS ", "GATE", "POSE", "SAVED", "STOPPED")):
            print("     " + l.rstrip())
    return res, o, ws


right = ForceHand([547, 549, 552, 548, 460, 961])
rest = [539, 542, 555, 561, 465, 972]
left = ForceHand(list(rest), contact=[None, None, 700, 700, None, None], base=[44, 31, -37, 997, 245, -227])
pr, pl = T.serve(right), T.serve(left)
ok = True
res, o, ws = run("pose open", ["--hand", "left", "--pose", "open", "--live"])
ok &= res == ["RESULT DONE"] and left.ang_set[:4] == [950] * 4 and os.path.exists(os.path.join(SD, "scale_pose_left.json")) and left.speed == [1000] * 6
res, o, ws = run("press middle from 950", ["--hand", "left", "--axis", "2", "--levels", "200,300", "--gate", "all", "--blip-gate", "--blip-dwell", "0.3",
                 "--pause", "0.5", "--close-to", "200", "--live"], ["CONTINUE", "CONTINUE"])
six = [w[1] for w in ws if w[0] == "six"]
ok &= res == ["RESULT DONE"] and six[0][:4] == [950] * 4 and six[1][2] == 960 and six[-1][:4] == [950] * 4 and left.ang_set[:4] == [950] * 4
ok &= "status 3" in o and left.force_set == [2000] * 6
res, o, ws = run("press index from 950", ["--hand", "left", "--axis", "3", "--levels", "200", "--gate", "all", "--blip-gate", "--blip-dwell", "0.3",
                 "--pause", "0.5", "--close-to", "200", "--live"], ["CONTINUE"])
ok &= res == ["RESULT DONE"] and left.ang_set[:4] == [950] * 4 and "status 3" in o
res, o, ws = run("pose open again (refused)", ["--hand", "left", "--pose", "open", "--live"])
ok &= not ws and "REFUSED" in o
res, o, ws = run("pose rest", ["--hand", "left", "--pose", "rest", "--live"])
ok &= res == ["RESULT DONE"] and left.ang_set[:4] == rest[:4] and not os.path.exists(os.path.join(SD, "scale_pose_left.json")) and left.speed == [1000] * 6
print("MOCK SCALE POSE TEST", "OK" if ok else "FAILED")
