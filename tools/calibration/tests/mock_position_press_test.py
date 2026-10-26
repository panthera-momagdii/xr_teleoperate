#!/usr/bin/env python3
"""Offline test (simulated hands, no robot) of press.py --targets (position press, 16 Oct): from the open pose, index contact at 859,
targets 849, 839, 829, 819 with one gate per level, safety limit rest force + 700 not reached, opens back to 950 each time; and a
deep target that drives the reading past the safety limit stops the run."""
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
SD = "/tmp/claude-1000/-home-unitree/f20c71cb-b449-4e36-bc97-51c33ddcb58b/scratchpad/mock_state_pos"
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
            open(os.path.join(d, answers[n] if n < len(answers) else "STOP"), "w").write("mock %d" % n)
            n += 1
    p.wait()
    o = "".join(out)
    ws = [("six", list(struct.unpack(">6H", f[6:18]))) if f[0] == 0x10 else ("one", struct.unpack(">HH", f[1:5])) for f in T.FRAMES if f[0] in (0x10, 0x06)]
    res = [l.strip() for l in out if l.startswith(("RESULT", "REFUSED"))][-1:]
    print("%-32s %s | index targets %s | single %s | end ANGLE_SET %s" % (name, res, [w[1][3] for w in ws if w[0] == "six"], [w[1] for w in ws if w[0] == "one"], left.ang_set))
    for l in out:
        if l.startswith(("POSITION", "REST FORCE", "GATE", "STOPPED")) or "SAFETY" in l:
            print("     " + l.rstrip()[:200])
    return res, o, ws


right = ForceHand([547, 549, 552, 548, 460, 961])
left = ForceHand([540, 543, 556, 561, 459, 972], contact=[None, None, None, 859, None, None], base=[0, -52, -383, 905, 251, -192])
pr, pl = T.serve(right), T.serve(left)
ok = True
res, o, ws = run("pose open", ["--hand", "left", "--pose", "open", "--live"])
ok &= res == ["RESULT DONE"]
res, o, ws = run("index targets 849..819", ["--hand", "left", "--axis", "3", "--targets", "849,839,829,819", "--rest-force", "1066", "--gate", "all",
                 "--blip-gate", "--blip-dwell", "0.3", "--pause", "0.5", "--live"], ["CONTINUE"] * 4)
idx = [w[1][3] for w in ws if w[0] == "six"]
ok &= res == ["RESULT DONE"] and idx == [950, 960, 950, 849, 950, 839, 950, 829, 950, 819, 950, 950] and ("one", (1504, 1766)) in ws and ("one", (1504, 2000)) in ws
ok &= o.count("WAITING_FOR_OPERATOR") == 4 and left.ang_set[:4] == [950] * 4 and left.force_set == [2000] * 6
left.contact = [None, None, None, 900, None, None]
res, o, ws = run("index deep target past safety", ["--hand", "left", "--axis", "3", "--targets", "650", "--rest-force", "1066", "--blip-gate",
                 "--blip-dwell", "0.3", "--pause", "0.5", "--live"], ["CONTINUE"])
ok &= res == ["RESULT ABORTED"]
print("MOCK POSITION PRESS TEST", "OK" if ok else "FAILED")
