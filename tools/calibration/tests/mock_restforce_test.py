#!/usr/bin/env python3
"""Offline test (simulated hands, no robot) of press.py --rest-force from the open pose (16 Oct plan b): the limits are rest-force +
level (not the reading at the hold), blip gate and one gate per level, the press opens back to the open pose; --pose rest puts all
finger speeds back to 1000 even if one started at 100; a rest force + level outside 0..3000 is refused."""
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
SD = "/tmp/claude-1000/-home-unitree/f20c71cb-b449-4e36-bc97-51c33ddcb58b/scratchpad/mock_state_rf"
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
    print("%-34s %s | single %s | end ANGLE_SET %s SPEED %s" % (name, res, [w[1] for w in ws if w[0] == "one"], left.ang_set, left.speed))
    for l in out:
        if l.startswith(("PRESS ", "REST FORCE", "GATE")):
            print("     " + l.rstrip())
    return res, o, ws


right = ForceHand([547, 549, 552, 548, 460, 961])
left = ForceHand([540, 543, 556, 561, 459, 972], contact=[None, None, 700, 700, None, None], base=[0, -52, -383, 905, 251, -192])
pr, pl = T.serve(right), T.serve(left)
ok = True
res, o, ws = run("pose open", ["--hand", "left", "--pose", "open", "--live"])
ok &= res == ["RESULT DONE"]
left.speed[2] = 100                                   # as on the robot: the middle speed left at 100
res, o, ws = run("press middle --rest-force 55", ["--hand", "left", "--axis", "2", "--levels", "200,300", "--rest-force", "55", "--gate", "all",
                 "--blip-gate", "--blip-dwell", "0.3", "--pause", "0.5", "--close-to", "200", "--live"], ["CONTINUE", "CONTINUE"])
ok &= res == ["RESULT DONE"] and ("one", (1502, 255)) in ws and ("one", (1502, 355)) in ws and ("one", (1502, 2000)) in ws and left.ang_set[:4] == [950] * 4
res, o, ws = run("press index --rest-force 1066", ["--hand", "left", "--axis", "3", "--levels", "200", "--rest-force", "1066", "--gate", "all",
                 "--blip-gate", "--blip-dwell", "0.3", "--pause", "0.5", "--close-to", "200", "--live"], ["CONTINUE"])
ok &= res == ["RESULT DONE"] and ("one", (1504, 1266)) in ws and left.ang_set[:4] == [950] * 4
res, o, ws = run("rest force + level < 0 refused", ["--hand", "left", "--axis", "2", "--levels", "200", "--rest-force", "-300", "--live"])
ok &= not ws and "REFUSED" in o
res, o, ws = run("pose rest", ["--hand", "left", "--pose", "rest", "--live"])
ok &= res == ["RESULT DONE"] and left.ang_set[:4] == [540, 543, 556, 561] and left.speed == [1000] * 6
print("MOCK REST FORCE TEST", "OK" if ok else "FAILED")
