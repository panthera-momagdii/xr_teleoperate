#!/usr/bin/env python3
"""calibration_ws tactile cue reader, READ ONLY (h0_ro TOUCH, function 0x03 only). For each pad in turn: prints a cue, logs a CUE
event with t_mono_ns, reads all 17 arrays for --dwell s; 2 s baseline first. Summary per cue: which array rose most over baseline.
Paced mode (env CW_CUE_PACED=1, for a remote operator): before each pad it prints the cue and keeps reading (logged) until the file
NEXT appears in <out_dir>, then records that pad for <dwell_s> s; the file STOP ends the run; 600 s without NEXT ends it.
   cw_tactile_cue.py <hand> <out_dir> <dwell_s> [pads,comma,separated]"""
import json
import os
import sys
import time

sys.path.insert(0, "/home/unitree/h0_tools")
import h0_ro  # noqa: E402
from h0_ro import ROModbus, TOUCH, i16  # noqa: E402

hand, out, dwell = sys.argv[1], sys.argv[2], float(sys.argv[3])
pads = sys.argv[4].split(",") if len(sys.argv) > 4 else list(TOUCH)
h = h0_ro.HANDS[hand]
c = ROModbus(h["ip"], h["port"], h["unit"], timeout=1.0)
c.connect()
f = open(os.path.join(out, "tactile_cue_%s.jsonl" % hand), "x")
seq = 0


def sample(event=None, **kw):
    global seq
    rec = {"t_mono_ns": time.monotonic_ns(), "t_host": round(time.time(), 6), "seq": seq, "hand": hand, "raw": {n: i16(c.read_group(TOUCH, n)) for n in TOUCH},
           "last_cmd": None, "event": event, **kw}
    f.write(json.dumps(rec, separators=(",", ":")) + "\n")
    seq += 1
    return rec


def sums(recs):
    return {n: max(sum(r["raw"][n]) for r in recs) for n in TOUCH}


t_end = time.monotonic() + 2.0
base = [sample("BASELINE")]
while time.monotonic() < t_end:
    base.append(sample())
b = sums(base)
summary = []
PACED = os.environ.get("CW_CUE_PACED") == "1"
NAMES = {"one": "little", "two": "ring", "three": "middle", "four": "index", "five": "thumb"}


def label(pad):
    for k, v in NAMES.items():
        if pad.startswith("finger" + k + "_"):
            return "%s %s" % (v, pad.split("_")[1])
    return pad.replace("_touch", "")


for i, pad in enumerate(pads):
    if PACED:
        nxt, stp = os.path.join(out, "NEXT"), os.path.join(out, "STOP")
        print("CUE %d/%d %s %s (%s): waiting for NEXT, then %.0f s recorded" % (i + 1, len(pads), hand, pad, label(pad), dwell), flush=True)
        sample("CUE_WAIT", pad=pad)
        t_wait = time.monotonic() + 600
        while not os.path.exists(nxt) and not os.path.exists(stp) and time.monotonic() < t_wait:
            sample()
        if not os.path.exists(nxt):
            print("CUE ended before %s (%s)" % (pad, "STOP" if os.path.exists(stp) else "600 s without NEXT"), flush=True)
            break
        os.rename(nxt, nxt + "_used_%d" % (i + 1))
    print("CUE %s %s (%s): press now (%.0f s)" % (hand, pad, label(pad), dwell), flush=True)
    recs = [sample("CUE", pad=pad)]
    t_end = time.monotonic() + dwell
    while time.monotonic() < t_end:
        recs.append(sample())
    m = sums(recs)
    rise = {n: m[n] - b[n] for n in TOUCH}
    top = max(rise, key=rise.get)
    summary.append({"cue": pad, "top_array": top, "top_rise": rise[top], "cued_pad_rise": rise[pad], "match": top == pad})
    print("   top %s rise %d, cued pad rise %d" % (top, rise[top], rise[pad]), flush=True)
c.close()
f.close()
json.dump({"hand": hand, "dwell_s": dwell, "baseline_sum": b, "cues": summary}, open(os.path.join(out, "tactile_cue_summary_%s.json" % hand), "x"), indent=1)
