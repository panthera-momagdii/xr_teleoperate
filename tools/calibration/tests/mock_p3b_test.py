#!/usr/bin/env python3
"""Offline test (simulated hands, no robot) of the 13 Oct late additions: below_rest FORCE_SET = rest + 400 g (left middle + 500 g),
restored at the end; a touch during below_rest (STATUS 3) ends the run; thumb steps (steps.py axis 4 and 5, stage t); the operator
STOP file (time from the file to the stop, no write after it; a new run refused while it exists)."""
import json
import os
import struct
import subprocess
import sys
import threading
import time

sys.path.insert(0, "/home/unitree/calibration_ws/tests")
sys.path.insert(0, "/home/unitree/h2_20261007/tools")
import test_h2_offline as T  # noqa: E402
from mock_p3_parts import ForceHand  # noqa: E402

PY = sys.executable
S = "/home/unitree/calibration_ws/scripts"
STOP = "/tmp/claude-1000/-home-unitree/f20c71cb-b449-4e36-bc97-51c33ddcb58b/scratchpad/stoptest/STOP"


def writes():
    out = []
    for f in T.FRAMES:
        if f[0] == 0x10:
            out.append(("six", list(struct.unpack(">6H", f[6:18]))))
        elif f[0] == 0x06:
            out.append(("one", struct.unpack(">HH", f[1:5])))
    return out


def bad(ws, thumbs_may_move, hold_thumbs):
    b = []
    for kind, v in ws:
        if kind == "six" and (max(v) > 1000 or (not thumbs_may_move and v[4:] != hold_thumbs)):
            b.append(v)
        if kind == "one" and (v[1] == 0xFFFF or not (1522 <= v[0] <= 1532 or 1498 <= v[0] <= 1504) or v[0] % 2):
            b.append(v)
    return b


def run(name, argv, hand, on_line=None):
    T.FRAMES.clear()
    env = dict(os.environ, CW_MOCK_PORTS="right:%d,left:%d" % (pr, pl), CW_MOCK_P3="1", G1RS_PRE_S="1", CW_HOLD_POST_S="2", G1RS_DW="0.05",
               CW_MOCK_STOP_FILE=STOP)
    p = subprocess.Popen([PY, "-B"] + argv, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    out = []
    for line in p.stdout:
        out.append(line)
        if on_line:
            on_line(line)
    p.wait()
    o = "".join(out)
    res = [l.strip() for l in out if l.startswith("RESULT")][-1:]
    ws = writes()
    print("%-34s %s writes %d (six %d, single %d) | end ANGLE_SET %s SPEED %s FORCE_SET %s"
          % (name, res, len(ws), sum(1 for w in ws if w[0] == "six"), sum(1 for w in ws if w[0] == "one"), hand.ang_set, hand.speed, hand.force_set))
    return res, o, ws


def rundir(o):
    return [l.split("dir ")[1].strip() for l in o.splitlines() if l.startswith("RUN ")][0]


os.makedirs(os.path.dirname(STOP), exist_ok=True)
right = ForceHand([547, 549, 552, 548, 460, 961])
left = ForceHand([543, 547, 547, 547, 479, 977], base=[50, 60, 20, 1050, 230, -130])
pr, pl = T.serve(right), T.serve(left)
ok = True
H = [460, 976]

# 1 below_rest, nothing touched: FORCE_SET rest + 400 = 20 + 400 on the index, back to 2000
res, o, ws = run("below_rest right index floor 400", [S + "/below_rest.py", "--hand", "right", "--axis", "3", "--floor", "400", "--step", "50", "--pause", "0.3", "--live"], right)
fw = [v for k, v in ws if k == "one" and v[0] == 1504]
print("     FORCE_SET writes (address, value):", fw)
ok &= res == ["RESULT DONE"] and fw == [(1504, 420), (1504, 2000)] and right.force_set == [2000] * 6 and not bad(ws, False, H)

# 2 left middle: rest + 500
res, o, ws = run("below_rest left middle floor 450", [S + "/below_rest.py", "--hand", "left", "--axis", "2", "--floor", "450", "--step", "50", "--pause", "0.3", "--live"], left)
fw = [v for k, v in ws if k == "one" and v[0] == 1502]
print("     FORCE_SET writes (address, value):", fw)
ok &= res == ["RESULT DONE"] and fw == [(1502, 520), (1502, 2000)] and left.force_set == [2000] * 6

# 3 below_rest with a touch at 450: the finger stops at FORCE_SET (STATUS 3) and the run ends, no further writes
right.contact = [None, None, None, 450, None, None]
res, o, ws = run("below_rest right index, touch at 450", [S + "/below_rest.py", "--hand", "right", "--axis", "3", "--floor", "300", "--step", "50", "--pause", "0.3", "--live"], right)
stop = [l for l in o.splitlines() if l.startswith("RESULT ABORTED")]
print("     ", stop[:1], "index angle %d status %d force %d" % (round(right.pos[3]), right.status[3], right.force(3)))
ok &= res == ["RESULT ABORTED"] and "STATUS_NOT_0_1_2" in o and right.force_set[3] == 420 and right.force(3) <= 425
right.contact = [None] * 6
time.sleep(1.0)
res, o, ws = run("release right (approved 3)", [S + "/release.py", "--hand", "right", "--approve-hold-status-3", "--live"], right)
ok &= res == ["RESULT DONE"] and right.force_set == [2000] * 6 and right.speed == [1000] * 6

# 4 thumb bend: +50 toward open and back, speed 100
res, o, ws = run("steps thumb bend right", [S + "/steps.py", "--hand", "right", "--axis", "4", "--pause", "0.5", "--live"], right)
six = [v for k, v in ws if k == "six"]
print("     six-value writes:", six)
print("     single writes:", [v for k, v in ws if k == "one"])
h = six[0]
ok &= res == ["RESULT DONE"] and six[1] == h[:4] + [h[4] + 50, h[5]] and six[2] == h and len(six) == 3 and right.speed == [1000] * 6 and not bad(ws, True, H)

# 5 thumb rotation: fingers 950 first, rotation -50 and back, fingers back
res, o, ws = run("steps thumb rotation right", [S + "/steps.py", "--hand", "right", "--axis", "5", "--pause", "0.5", "--live"], right)
six = [v for k, v in ws if k == "six"]
print("     six-value writes:", six)
print("     single writes:", [v for k, v in ws if k == "one"])
h = six[0]
ok &= (res == ["RESULT DONE"] and six[1] == [950] * 4 + h[4:] and six[2] == [950] * 4 + [h[4], h[5] - 50] and six[3] == [950] * 4 + h[4:]
       and six[4] == h and len(six) == 5 and right.speed == [1000] * 6 and right.ang_set == h)
mv = json.load(open(os.path.join(rundir(o), "block", "summary.json")))["moves"]
print("     moves:", [(m["tag"], m.get("start"), m.get("target"), m.get("settled"), m.get("t_settle_s")) for m in mv])

# 6 refusals: thumb step 60, no write
res, o, ws = run("steps thumb step 60 refused", [S + "/steps.py", "--hand", "right", "--axis", "5", "--thumb-step", "60", "--live"], right)
ok &= not ws and "REFUSED" in o

# 7 operator STOP file during below_rest: stop time, no write after it, next run refused while the file exists
mark = {}


def at_line(line):
    if line.startswith("$") and "g1rs_block.py" in line and not mark:
        def go():
            time.sleep(4.0)
            open(STOP, "w").write("mock operator stop")
            mark["t"], mark["i"] = time.time(), len(T.FRAMES)
        threading.Thread(target=go, daemon=True).start()


res, o, ws = run("below_rest with operator STOP file", [S + "/below_rest.py", "--hand", "right", "--axis", "3", "--floor", "0", "--step", "25", "--pause", "1", "--live"], right, at_line)
d = rundir(o)
sm = json.load(open(os.path.join(d, "block", "summary.json")))
cmds = [json.loads(l) for l in open(os.path.join(d, "block", "commands.jsonl"))]
after = [c for c in cmds if c["t_host"] > mark["t"]]
seen = sm["detail"].get("t_seen") if isinstance(sm["detail"], dict) else None
print("     stop reason %s; file -> seen %.1f ms; last write %.1f ms before the file; writes after the file: %d"
      % (sm["reason"], (seen - mark["t"]) * 1000, (mark["t"] - cmds[-1]["t_host"]) * 1000, len(after)))
ok &= sm["reason"] == "OPERATOR_STOP_FILE" and not after and (seen - mark["t"]) < 0.1
res, o, ws = run("new run while STOP exists", [S + "/below_rest.py", "--hand", "right", "--axis", "3", "--live"], right)
print("     ", [l for l in o.splitlines() if "STOP" in l][:1])
ok &= not ws and "operator stop" in o
os.remove(STOP)
print("MOCK P3B TEST", "OK" if ok else "FAILED")
