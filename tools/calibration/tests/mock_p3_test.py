#!/usr/bin/env python3
"""Offline test of the P3 live code (below_rest, press, grip) against simulated hands on localhost (no real hand, no DDS). The fake
hand adds a force model: a finger closing past its contact angle raises FORCE_ACT (base + 4 g per count) and stops with STATUS 3 when
FORCE_ACT reaches its FORCE_SET; with no contact angle it simply reaches its target. Checks the write forms (six-value ANGLE_SET at
1486, single-register SPEED_SET 1522..1528 and FORCE_SET 1498..1504, never 0xFFFF, never a thumb FORCE_SET or thumb target change),
the press gate after level 1, and that the hand ends at its start pose with FORCE_SET and speeds back at their start values.
The locks are bypassed only in mock mode for live_writes; the P3 locks are bypassed here with CW_MOCK_P3=1 (mock only)."""
import os
import struct
import subprocess
import sys
import threading
import time

sys.path.insert(0, "/home/unitree/h2_20261007/tools")
sys.path.insert(0, "/home/unitree/calibration_ws/tests")
import test_h2_offline as T  # noqa: E402
import test_h2_v2_offline as T2  # noqa: E402

PY = sys.executable
S = "/home/unitree/calibration_ws/scripts"


from mock_p3_parts import ForceHand  # noqa: E402


def check_frames():
    bad, n16, n06 = [], 0, 0
    for f in T.FRAMES:
        if f[0] == 3:
            continue
        if f[0] == 0x10:
            n16 += 1
            a, n = struct.unpack(">HH", f[1:5])
            if a != 1486 or n != 6 or max(struct.unpack(">6H", f[6:18])) > 1000:
                bad.append(f.hex())
        elif f[0] == 0x06:
            n06 += 1
            a, v = struct.unpack(">HH", f[1:5])
            if v == 0xFFFF or a % 2 or not (1522 <= a <= 1528 or 1498 <= a <= 1504):
                bad.append(f.hex())
        else:
            bad.append(f.hex())
    return n16, n06, bad


def run(name, argv, hand, gate=None):
    T.FRAMES.clear()
    env = dict(os.environ, CW_MOCK_PORTS="right:%d,left:%d" % (pr, pl), CW_MOCK_P3="1", G1RS_PRE_S="1", CW_HOLD_POST_S="2", G1RS_DW="0.05",
               CW_PAUSE="0.3", CW_GATE_TIMEOUT="20")
    p = subprocess.Popen([PY, "-B"] + argv, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    out = []
    for line in p.stdout:
        out.append(line)
        if gate and line.startswith("WAITING_FOR_OPERATOR"):
            d = line.split("create ")[1].split("/CONTINUE")[0]
            open(os.path.join(d, gate), "w").write("mock scale reading 210 g")
        if line.startswith("HOLDING until"):
            d = line.split("until ")[1].split("/RELEASE")[0]
            threading.Timer(1.0, lambda d=d: open(os.path.join(d, "RELEASE"), "w").close()).start()
    p.wait()
    res = [l.strip() for l in out if l.startswith("RESULT")][-1:]
    n16, n06, bad = check_frames()
    print("%-28s %s six-value %d single %d bad %s | end ANGLE_SET %s SPEED %s FORCE_SET %s" % (name, res, n16, n06, bad, hand.ang_set, hand.speed, hand.force_set))
    for l in out:
        if l.startswith(("PRESS ", "GRIP closed", "RELEASED", "APPROVED", "GATE", "STOP", "LOCKED", "NOTE", "NOTICE")):
            print("     " + l.rstrip())
    return res == ["RESULT DONE"] and not bad, "".join(out)


right = ForceHand([547, 549, 552, 548, 460, 961], contact=[None, None, None, 450, None, None])
left = ForceHand([543, 547, 547, 547, 479, 977], contact=[470, 470, 470, 470, None, None], base=[50, 60, 20, 1050, 230, -130])
pr, pl = T.serve(right), T.serve(left)
ok = True
r, o = run("below_rest right index", [S + "/below_rest.py", "--hand", "right", "--axis", "3", "--floor", "400", "--step", "50", "--live"], right)
ok &= r
r, o = run("press right index (contact 450)", [S + "/press.py", "--hand", "right", "--axis", "3", "--levels", "200,300,400,500", "--close-to", "300", "--live"], right, gate="CONTINUE")
ok &= r
r, o = run("press right index, STOP at gate", [S + "/press.py", "--hand", "right", "--axis", "3", "--levels", "200,300", "--close-to", "300", "--live"], right, gate="STOP")
ok &= r
r, o = run("press right little no contact", [S + "/press.py", "--hand", "right", "--axis", "0", "--levels", "200", "--close-to", "300", "--live"], right)
ok &= r
r, o = run("grip left (contact 470)", [S + "/grip.py", "--hand", "left", "--close-to", "400", "--max-hold-s", "5", "--live"], left)
ok &= r
r, o = run("press right index gate all", [S + "/press.py", "--hand", "right", "--axis", "3", "--levels", "200,300,400", "--close-to", "300", "--gate", "all", "--live"], right, gate="CONTINUE")
ok &= r and o.count("WAITING_FOR_OPERATOR") == 2
right.contact = [None] * 6                          # nothing in the hand from here on
r, o = run("grip right empty hand", [S + "/grip.py", "--hand", "right", "--max-hold-s", "2", "--live"], right)
ok &= r and "GRIP closed: angles [400, 400, 400, 400]" in o
# abort: ERROR 4 on the right index 0.3 s after its first press FORCE_SET write; the block must stop and write nothing after the fault
mark = {}


def inject(k, v):
    if k == 3 and v != 2000 and not mark:
        def go():
            mark["i"] = len(T.FRAMES)
            with right.lock:
                right.err[3] = 4
            time.sleep(1.5)
            with right.lock:
                right.err[3] = 0
        threading.Thread(target=go, daemon=True).start()


right.contact = [None, None, None, 450, None, None]  # the abort leaves the index pressing (STATUS 3)
right.on_force_write = lambda k, v: threading.Timer(0.3, inject, (k, v)).start()
r, o = run("press right index, ERROR abort", [S + "/press.py", "--hand", "right", "--axis", "3", "--levels", "200", "--close-to", "300", "--live"], right)
after = [f for f in T.FRAMES[mark.get("i", 0):] if f[0] != 3]
print("     writes after the fault: %d; RESULT lines %s; stop %s" % (len(after), [l for l in o.splitlines() if l.startswith("RESULT")],
      [l for l in o.splitlines() if "STOP" in l or "ERROR" in l][:3]))
ok &= (not r) and "RESULT ABORTED" in o and "i" in mark and not after
right.on_force_write = None
time.sleep(2.0)
print("     after the abort: STATUS %s ANGLE %s FORCE_SET %s SPEED %s" % (right.status, [round(x) for x in right.pos], right.force_set, right.speed))
r, o = run("release right, no approval", [S + "/release.py", "--hand", "right", "--live"], right)
ok &= (not r) and "STOP: STATUS" in o and check_frames()[:2] == (0, 0)
print("     " + [l for l in o.splitlines() if "STOP" in l][0])
r, o = run("release right, approved 3", [S + "/release.py", "--hand", "right", "--approve-hold-status-3", "--live"], right)
ok &= r and right.force_set == [2000] * 6 and right.speed == [1000] * 6 and abs(right.pos[3] - 550) <= 3
r, o = run("press over 500 refused", [S + "/press.py", "--hand", "right", "--axis", "3", "--levels", "200,600", "--live"], right)
ok &= (r is False and "REFUSED" in o)
print("refusal check:", [l for l in o.splitlines() if "REFUSED" in l or "500" in l][:2])
print("MOCK P3 TEST", "OK" if ok else "FAILED")
