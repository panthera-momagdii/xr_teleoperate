#!/usr/bin/env python3
"""Offline per-finger table from a g1rs_block run (no robot access), same definitions as h2_20261007/tools/h2_analyze.py:
delay = first sample with |angle - start| >= 3 after the write; t10 / t90 = first sample reaching 10 % / 90 % of (settled - start);
settling = time after which |angle - settled| <= 3 for the rest of the move window; times from the write (t_mono_ns).
   g1rs_block_analyze.py <run_dir>  -> <run_dir>/analysis.json"""
import json
import os
import statistics as st
import sys

d = sys.argv[1]
s = json.load(open(os.path.join(d, "summary.json")))
rows = [json.loads(l) for l in open(os.path.join(d, "state.jsonl"))]
rows = [r for r in rows if r.get("dec")]
ax = s["axis"]
mv = s["moves"]
nexts = [m["t_cmd_mono"] for m in mv[1:]] + [rows[-1]["t_mono_ns"] / 1e9 + 1e-6]


def agg(v):
    v = [x for x in v if x is not None]
    return None if not v else {"mean": round(st.mean(v), 4), "sd": round(st.pstdev(v), 4), "n": len(v), "min": round(min(v), 4), "max": round(max(v), 4)}


def timing(m, t_next):
    w = [r for r in rows if m["t_cmd_mono"] <= r["t_mono_ns"] / 1e9 < t_next]
    a = [r["dec"]["angle_act"][ax] for r in w]
    t = [r["t_mono_ns"] / 1e9 - m["t_cmd_mono"] for r in w]
    s0, s1 = m["start"], m["settled"]
    amp = s1 - s0
    res = {"start": s0, "settled": s1, "target": m["target"], "peak_current": max(abs(r["dec"]["current"][ax]) for r in w)}
    res["delay_s"] = next((ti for ti, ai in zip(t, a) if abs(ai - s0) >= 3), None)
    if amp:
        res["t10_s"] = next((ti for ti, ai in zip(t, a) if (ai - s0) / amp >= 0.1), None)
        res["t90_s"] = next((ti for ti, ai in zip(t, a) if (ai - s0) / amp >= 0.9), None)
    res["settling_s"] = next((t[i] for i in range(len(a)) if all(abs(x - s1) <= 3 for x in a[i:])), None)
    return res


dw = {x["tag"]: x for x in s["dwells"]}
out = {"run": d, "hand": s["hand"], "axis": ax, "finger": s["finger"], "result": s["result"], "reason": s["reason"], "hold": s["hold"][ax]}
ends = {}
for key, tags in (("rest", ["a_rest", "a_back_rest"] + ["a_rest_rep%d" % r for r in range(3)]), ("open_1000", ["a_1000"] + ["a_open_rep%d" % r for r in range(3)])):
    ang = [dw[t]["angle_median"][ax] for t in tags if t in dw]
    pos = [dw[t]["pos_median"][ax] for t in tags if t in dw]
    if ang:
        ends[key] = {"angle": ang, "pos": pos, "angle_spread": max(ang) - min(ang), "pos_spread": max(pos) - min(pos)}
out["endpoints"] = ends
tim = {}
for sp in (100, 500, 1000):
    for di in ("up", "down"):
        res = [timing(m, nx) for m, nx in zip(mv, nexts) if m["tag"].startswith("b_%s_s%d_" % (di, sp))]
        tim["s%d_%s" % (sp, di)] = {k: agg([r.get(k) for r in res]) for k in ("delay_s", "t10_s", "t90_s", "settling_s", "peak_current")}
        tim["s%d_%s" % (sp, di)]["settled"] = [r["settled"] for r in res]
out["timing"] = tim
db, ref = [], dw.get("c_ref", {}).get("angle_median", [None] * 6)[ax]
for sgn in (1, -1):
    for stp in (5, 10, 20):
        a_ = dw.get("c_step_%+d" % (sgn * stp), {}).get("angle_median", [None] * 6)[ax]
        b_ = dw.get("c_back_%+d" % (sgn * stp), {}).get("angle_median", [None] * 6)[ax]
        if a_ is not None and ref is not None:
            db.append({"step": sgn * stp, "change": a_ - ref, "back": b_})
        ref = b_ if b_ is not None else ref
out["deadband"] = db
up = [dw[t]["angle_median"][ax] for t in dw if t.startswith("d_800_from_900")]
lo = [dw[t]["angle_median"][ax] for t in dw if t.startswith("d_800_from_700")]
out["hysteresis"] = {"from_above": up, "from_below": lo, "diff": (st.mean(up) - st.mean(lo)) if up and lo else None}
out["peak_current_all_axes"] = s["peak_current"]
out["max_non_commanded_dev"] = s["max_non_commanded_dev"]
out["temp_median_max"] = s["temp_median_max"]
out["end"] = s["end"]
out["temp_spikes"] = sum(1 for e in s["events"] if e["event"] == "TEMP_SPIKE")
out["glitches"] = [e for e in s["events"] if e["event"] == "GLITCH"]
json.dump(out, open(os.path.join(d, "analysis.json"), "w"), indent=1)
f = lambda x: "-" if not x else "%.3f" % x["mean"]  # noqa: E731
print("%s %s %s: %s %s" % (d, s["hand"], s["finger"], s["result"], s["reason"] or ""))
for k, v in tim.items():
    print("  %-10s delay %s t10 %s t90 %s settling %s peak I %s settled %s" % (k, f(v["delay_s"]), f(v["t10_s"]), f(v["t90_s"]), f(v["settling_s"]),
          None if not v["peak_current"] else v["peak_current"]["mean"], v["settled"]))
print("  endpoints", {k: (v["angle"], v["angle_spread"]) for k, v in ends.items()}, "deadband", [(x["step"], x["change"]) for x in db], "hyst", out["hysteresis"]["diff"])
print("  peak I", s["peak_current"], "max non-cmd dev", s["max_non_commanded_dev"], "temp median max", s["temp_median_max"], "end", s["end"], "temp spikes", out["temp_spikes"], "glitches", len(out["glitches"]))
