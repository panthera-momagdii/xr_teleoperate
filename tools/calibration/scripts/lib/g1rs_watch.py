#!/usr/bin/env python3
"""g1rs_20261012 READ-ONLY watch of one hand (Mohammed 2026-10-12, left hand after its middle finger went to STATUS 7). No write of
any kind (h0_ro.ROModbus, function 0x03 only). STATE groups at full rate until the stop file appears. Everything is logged; it raises
the session STOP_FLAG (which stops writes on the other hand) only on:
  * real motion: an axis more than 5 counts from its reference (median of the first 1 s) on 3 consecutive reads, reached without any
    single-read jump over 10 counts since it was last within 5 counts of the reference;
  * real current: nonzero current below 3000 mA on one axis on 6 consecutive reads.
Logged only (events): one-read jumps over 10 counts (JUMP_GLITCH), out-of-range values and ERROR bursts (GLITCH_BURST, counted per
axis, from g1rs_burst; no stop), STATUS changes (STATUS_CHANGE, e.g. 7), TEMP spikes, temperature medians at or above 48.
   g1rs_watch.py <hand> <out_dir> <stop_file>     (env G1RS_STOP_FLAG = flag file to raise)"""
import collections
import json
import os
import sys
import time

sys.path.insert(0, "/home/unitree/h0_tools")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from h0_ro import STATE, CONFIG  # noqa: E402
from g1rs_read import Reader  # noqa: E402
from g1rs_burst import BurstTracker  # noqa: E402

hand, out, stop_file = sys.argv[1], sys.argv[2], sys.argv[3]
STOP_FLAG = os.environ.get("G1RS_STOP_FLAG")
os.makedirs(out, exist_ok=True)
f = open(os.path.join(out, "state_%s.jsonl" % hand), "x")
r = Reader(hand, f)
r.sample(("ANGLE_SET", "SPEED_SET", "FORCE_SET"), table=CONFIG, event="CONFIG_START")
bursts = BurstTracker(lambda name, **kw: r.log(name, **kw), hand)
for k in range(6):
    pass
ref, first = None, []
t0 = time.monotonic()
last_a = [None] * 6
jumped = [False] * 6
far = [0] * 6
cur_run = [0] * 6
status_prev = None
tsag, tdrv, tfar = {4: False, 5: False}, {4: 0, 5: 0}, {4: 0, 5: 0}
tq = [collections.deque() for _ in range(6)]
hot = [False] * 6
spike = [None] * 6
counts = collections.Counter()
raised = None
last_ok = time.monotonic()
while not os.path.exists(stop_file):
    try:
        rec = r.sample(tuple(STATE))
        last_ok = time.monotonic()
    except Exception as e:  # noqa: BLE001
        r.log("READ_ERROR", detail=repr(e))
        counts["READ_ERROR"] += 1
        try:
            r.c.close()
            r.c.connect()
        except Exception:  # noqa: BLE001
            time.sleep(0.05)
        continue
    d, t = rec["dec"], time.monotonic()
    bursts.update(rec)                                   # log only here (no stop)
    if status_prev is None or d["status"] != status_prev:
        r.log("STATUS_CHANGE", status=d["status"], before=status_prev)
        status_prev = d["status"]
    if ref is None:
        first.append(d["angle_act"])
        if t - t0 >= 1.0:
            ref = [sorted(c)[len(c) // 2] for c in zip(*first)]
            r.log("REFERENCE", angle=ref)
    for k in range(6):
        a = d["angle_act"][k]
        if last_a[k] is not None and abs(a - last_a[k]) > 10:
            r.log("JUMP_GLITCH", axis=k, from_=last_a[k], to=a, raw=rec["raw"])
            counts["JUMP_GLITCH"] += 1
            jumped[k] = True
        last_a[k] = a
        if ref is not None and k >= 4:                     # thumbs: rule 6 (g1rs_20261012b): sag logged, stop at 20 or driven > 10
            dev, c = abs(a - ref[k]), d["current"][k]
            sag = dev > 10 and c == 0
            if sag and not tsag[k]:
                r.log("THUMB_SAG", axis=k, dev=dev, angle=a, reference=ref[k])
            elif tsag[k] and dev <= 10:
                r.log("THUMB_SAG_END", axis=k, dev=dev, angle=a)
            tsag[k] = sag or (tsag[k] and dev > 10)
            tdrv[k] = tdrv[k] + 1 if (dev > 10 and c != 0) else 0
            tfar[k] = tfar[k] + 1 if dev > 20 else 0
            if (tdrv[k] >= 3 or tfar[k] >= 3) and not raised:
                raised = ("THUMB_DRIVEN_GT10" if tdrv[k] >= 3 else "THUMB_GT20_FROM_REFERENCE", {"axis": k, "angle": a, "reference": ref[k], "current": c})
        elif ref is not None:
            dev = abs(a - ref[k])
            if dev <= 5:
                jumped[k], far[k] = False, 0
            else:
                far[k] += 1
                if far[k] >= 3 and not jumped[k] and not raised:
                    raised = ("REAL_MOTION", {"axis": k, "angle": a, "reference": ref[k]})
        c = d["current"][k]
        cur_run[k] = cur_run[k] + 1 if (k < 4 and c != 0 and abs(c) < 3000) else 0     # thumbs: rule 6 above
        if cur_run[k] >= 6 and not raised:
            raised = ("REAL_CURRENT_6_READS", {"axis": k, "current": c})
        q = tq[k]
        q.append((t, d["temp"][k]))
        while t - q[0][0] > 2.0:
            q.popleft()
        med = sorted(x[1] for x in q)[len(q) // 2]
        if abs(d["temp"][k] - med) >= 4:
            if spike[k] is None:
                spike[k] = {"first_seq": rec["seq"], "median": med, "values": [], "raw_first": rec["raw"]["TEMP"]}
            spike[k]["values"].append(d["temp"][k])
        elif spike[k] is not None:
            sp = spike[k]
            r.log("TEMP_SPIKE", axis=k, first_seq=sp["first_seq"], n_reads=len(sp["values"]), values=sp["values"], median=sp["median"], raw=sp["raw_first"])
            counts["TEMP_SPIKE"] += 1
            spike[k] = None
        if t - q[0][0] >= 1.0 and med >= 48 and not hot[k]:
            hot[k] = True
            r.log("TEMP_MEDIAN_GE_48", axis=k, median=med)
    if raised and STOP_FLAG and not os.path.exists(STOP_FLAG):
        open(STOP_FLAG, "w").write("%s %s %s %s" % (hand, raised[0], json.dumps(raised[1]), time.strftime("%H:%M:%S")))
        r.log("STOP_FLAG_RAISED", reason=raised[0], **raised[1])
r.sample(("ANGLE_SET", "SPEED_SET", "FORCE_SET"), table=CONFIG, event="CONFIG_END")
summary = {"hand": hand, "bursts_per_axis": bursts.count, "counts": dict(counts), "raised": raised, "status_end": status_prev, "reference": ref}
json.dump(summary, open(os.path.join(out, "watch_summary_%s.json" % hand), "w"), indent=1, default=str)
r.c.close()
f.close()
print(hand, "watch ended", json.dumps(summary, default=str))
