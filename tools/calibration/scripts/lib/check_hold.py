#!/usr/bin/env python3
"""Offline: hold run -> STATUS, ERROR, current, ANGLE_ACT at +2 s and +30 s (or the last sample) after the write; prints CLEAN or NOT_CLEAN.
   check_hold.py <hold_dir>"""
import json
import os
import sys

d = sys.argv[1]
s = json.load(open(os.path.join(d, "summary.json")))
rows = [json.loads(l) for l in open(os.path.join(d, "state.jsonl"))]
smp = [r for r in rows if r.get("dec") and "status" in r["dec"]]
tw = json.loads(open(os.path.join(d, "commands.jsonl")).readline())["t_mono_ns"] / 1e9 if s.get("result") != "ABORTED_BEFORE_WRITE" else None
out = {"result": s.get("result"), "flags": s.get("flags"), "values": s.get("values"), "reading": s.get("reading"), "at": {}}
if tw:
    for tt in (2.0, 30.0, "last"):
        r = smp[-1] if tt == "last" else min(smp, key=lambda x: abs(x["t_mono_ns"] / 1e9 - tw - tt))
        out["at"][tt if tt == "last" else "%.0f" % tt] = {"t_s": round(r["t_mono_ns"] / 1e9 - tw, 3), "status": r["dec"]["status"], "error": r["dec"]["error"],
                                   "current": r["dec"]["current"], "angle_act": r["dec"]["angle_act"]}
clean = (s.get("result") == "DONE" and not s.get("flags") and all(v["status"] == [2] * 6 and v["error"] == [0] * 6 for v in out["at"].values()))
out["clean"] = clean
json.dump(out, open(os.path.join(d, "hold_check.json"), "w"), indent=1)
print(json.dumps(out))
print("CLEAN" if clean else "NOT_CLEAN")
