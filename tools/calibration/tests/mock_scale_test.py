#!/usr/bin/env python3
"""Offline test (simulated hands, no robot) of scripts/scale_test.py (18 Oct): full scale run with 8 s holds (timing, max gap between
changes on the scale, 12 presses, end open at 950 and still, thumbs and the other fingers never written away from their hold, run
packed with sha256), --no-contact-check (only 650..950), a finger with no contact above 650 (skipped), the STOP file mid-run and
Ctrl-C mid-run (open to 950, then no write), and a reading past the safety limit (stop, open to 950)."""
import json
import os
import shutil
import signal
import struct
import subprocess
import sys
import threading
import time

sys.path.insert(0, "/home/unitree/calibration_ws/tests")
sys.path.insert(0, "/home/unitree/h2_20261007/tools")
import test_h2_offline as T  # noqa: E402
from mock_p3_parts import ForceHand  # noqa: E402

S = "/home/unitree/calibration_ws/scripts"
SP = "/tmp/claude-1000/-home-unitree/f20c71cb-b449-4e36-bc97-51c33ddcb58b/scratchpad/mock_scale"
shutil.rmtree(SP, ignore_errors=True)
os.makedirs(SP)
STOP = os.path.join(SP, "STOP")
OPEN = [939, 933, 936, 941, 455, 972]


class CoolHand(ForceHand):
    temp_word = 0x2020                      # 32 C on every axis (the base fake reads 44 to 46 C)

    def read(self, a, n):
        if a == 1618:
            return [self.temp_word] * 3
        return super().read(a, n)


def fresh(contact):
    return CoolHand(list(OPEN), contact=contact, base=[6, 64, 55, 1066, 250, -214])


def run(name, argv, hand, action=None):
    global pl
    T.FRAMES.clear()
    env = dict(os.environ, CW_MOCK_PORTS="right:%d,left:%d" % (pr, pl), CW_MOCK_P3="1", G1RS_PRE_S="1", CW_MOCK_STOP_FILE=STOP, CW_MOCK_PACK_DIR=SP)
    t0 = time.time()
    p = subprocess.Popen([sys.executable, "-B", S + "/scale_test.py"] + argv + ["--live"], env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         text=True, start_new_session=True)
    out = []
    for line in p.stdout:
        out.append(line)
        if action and line.startswith("PRESS index 2/"):
            action(p)
    p.wait()
    dt = time.time() - t0
    o = "".join(out)
    ws = [("six", list(struct.unpack(">6H", f[6:18]))) if f[0] == 0x10 else ("one", struct.unpack(">HH", f[1:5])) for f in T.FRAMES if f[0] in (0x10, 0x06)]
    six = [w[1] for w in ws if w[0] == "six"]
    d = [l.split("dir ")[1].strip() for l in out if l.startswith("RUN ")][0]
    print("     run folder", os.path.basename(d))
    sm = json.load(open(os.path.join(d, "block", "summary.json"))) if os.path.exists(os.path.join(d, "block", "summary.json")) else {}
    res = [l.strip() for l in out if l.startswith("RESULT")][-1:]
    print("%-30s %s in %.0f s | presses %d | max gap %s s | six-value writes %d | end ANGLE_SET %s" % (name, res, dt, len(sm.get("presses", [])),
          sm.get("max_gap_s"), len(six), hand.ang_set))
    for l in out:
        if l.startswith(("CONTACT", "PRESS ", "NO-CONTACT", "STOP", "END", "PACKED", "PRE-FLIGHT FAILED")):
            print("     " + l.rstrip()[:170])
    return res, o, six, sm, dt


right = CoolHand([1000, 549, 552, 550, 450, 962])
hot = ForceHand(list(OPEN))                 # 44 to 46 C: the pre-flight must refuse before any write
pr, pl = T.serve(right), T.serve(hot)
ok = True
T.FRAMES.clear()
res, o, six, sm, dt = run("pre-flight, hand at 46 C", [], hot)
ok &= res == [] or "ABORTED_PREFLIGHT" in o
ok &= "PRE-FLIGHT FAILED" in o and not six
left = fresh([None, None, 880, 859, None, None]); pl = T.serve(left)
res, o, six, sm, dt = run("full scale run (8 s holds)", [], left)
hold = six[0]
ok &= res == ["RESULT DONE"] and len(sm["presses"]) == 12 and sm["max_gap_s"] is not None and sm["max_gap_s"] <= 15
ok &= all(v[4:] == hold[4:] and v[0] == hold[0] and v[1] == hold[1] for v in six) and left.ang_set[2] == 950 and left.ang_set[3] == 950
ok &= left.force_set == [2000] * 6 and left.speed == [1000] * 6 and "PACKED" in o and "verified" in o
print("     total %.0f s; writes never moved thumbs / little / ring: %s" % (dt, all(v[4:] == hold[4:] and v[:2] == hold[:2] for v in six)))
left2 = fresh([None] * 6); pl = T.serve(left2)            # before the scale goes in: nothing under the fingers
res, o, six, sm, dt = run("no-contact check", ["--no-contact-check"], left2)
ok &= res == ["RESULT DONE"] and all(650 <= v[k] <= 950 or v[k] == six[0][k] for v in six for k in (2, 3)) and left2.ang_set[2:4] == [950, 950]
left3 = fresh([None, None, None, 859, None, None]); pl = T.serve(left3)
res, o, six, sm, dt = run("middle no contact (skipped)", ["--hold-s", "1"], left3)
ok &= res == ["RESULT DONE"] and "none above 650" in o and len(sm["presses"]) == 6
left4 = fresh([None, None, 880, 859, None, None]); pl = T.serve(left4)
mark = {}
def stopfile(p):
    if not mark:
        mark["n"] = len(T.FRAMES); open(STOP, "w").write("mock stop")
res, o, six, sm, dt = run("STOP file mid-run", ["--hold-s", "1"], left4, stopfile)
after = [f for f in T.FRAMES[mark.get("n", 0):] if f[0] in (0x10, 0x06)]
ok &= res == ["RESULT ABORTED"] and "OPERATOR_STOP_FILE" in o and len(after) == 1 and left4.ang_set[3] == 950
if os.path.exists(STOP):
    os.remove(STOP)
left5 = fresh([None, None, 880, 859, None, None]); pl = T.serve(left5)
mark2 = {}
def ctrlc(p):
    if not mark2:
        mark2["n"] = len(T.FRAMES); os.killpg(os.getpgid(p.pid), signal.SIGINT)
res, o, six, sm, dt = run("Ctrl-C mid-run", ["--hold-s", "1"], left5, ctrlc)
after = [f for f in T.FRAMES[mark2.get("n", 0):] if f[0] in (0x10, 0x06)]
ok &= res == ["RESULT ABORTED"] and "CTRL_C" in o and len(after) <= 2 and left5.ang_set[3] == 950
left6 = fresh([None, None, 880, 940, None, None]); pl = T.serve(left6)
res, o, six, sm, dt = run("safety limit (rest + 300 for the test)", ["--hold-s", "1", "--safety-g", "300"], left6)
ok &= res == ["RESULT ABORTED"] and "SAFETY_FORCE_AT_OR_ABOVE_LIMIT" in o and left6.ang_set[3] == 950
print("MOCK SCALE TEST", "OK" if ok else "FAILED")
