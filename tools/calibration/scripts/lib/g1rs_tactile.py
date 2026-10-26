#!/usr/bin/env python3
"""g1rs_20261012b run15, READ ONLY: tactile arrays of one hand (h0_ro TOUCH table, the 17 arrays the 7 Oct tool reads, 3000 to 5123),
all arrays per sample, for <seconds>. Raw words per array (int16) with t_mono_ns. Writes <out>/state_<hand>.jsonl and
<out>/tactile_stats_<hand>.json (per array: samples, mean, sd over time of the array mean, min, max, nonzero taxels).
   g1rs_tactile.py <hand> <seconds> <out_dir>"""
import json
import os
import statistics as stt
import sys
import time

sys.path.insert(0, "/home/unitree/h0_tools")
import h0_ro  # noqa: E402
from h0_ro import ROModbus, TOUCH, i16  # noqa: E402

hand, secs, out = sys.argv[1], float(sys.argv[2]), sys.argv[3]
os.makedirs(out, exist_ok=True)
h = h0_ro.HANDS[hand]
c = ROModbus(h["ip"], h["port"], h["unit"], timeout=1.0)
c.connect()
f = open(os.path.join(out, "state_%s.jsonl" % hand), "x")
acc = {n: [] for n in TOUCH}
mn = {n: None for n in TOUCH}
mx = {n: None for n in TOUCH}
nz = {n: 0 for n in TOUCH}
seq, t_end, errs = 0, time.monotonic() + secs, 0
while time.monotonic() < t_end:
    rec = {"t_mono_ns": time.monotonic_ns(), "t_host": round(time.time(), 6), "seq": seq, "hand": hand, "raw": {}, "last_cmd": None, "event": None}
    try:
        for n in TOUCH:
            v = i16(c.read_group(TOUCH, n))
            rec["raw"][n] = v
            acc[n].append(sum(v) / len(v))
            mn[n] = min(v) if mn[n] is None else min(mn[n], min(v))
            mx[n] = max(v) if mx[n] is None else max(mx[n], max(v))
            nz[n] = max(nz[n], sum(1 for x in v if x != 0))
    except Exception as e:  # noqa: BLE001
        errs += 1
        rec["event"] = "READ_ERROR %r" % (e,)
        try:
            c.close()
            c.connect()
        except Exception:  # noqa: BLE001
            time.sleep(0.05)
    f.write(json.dumps(rec, separators=(",", ":")) + "\n")
    seq += 1
c.close()
f.close()
stats = {n: {"samples": len(acc[n]), "mean_of_array_mean": round(stt.mean(acc[n]), 3) if acc[n] else None,
             "sd_of_array_mean": round(stt.pstdev(acc[n]), 3) if acc[n] else None, "min": mn[n], "max": mx[n], "max_nonzero_taxels": nz[n],
             "taxels": TOUCH[n][1]} for n in TOUCH}
json.dump({"hand": hand, "seconds": secs, "samples": seq, "read_errors": errs, "rate_hz": round(seq / secs, 2), "arrays": stats},
          open(os.path.join(out, "tactile_stats_%s.json" % hand), "x"), indent=1)
print(hand, "tactile samples", seq, "rate %.2f Hz" % (seq / secs), "errors", errs)
