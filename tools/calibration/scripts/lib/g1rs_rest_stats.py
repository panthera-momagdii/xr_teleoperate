#!/usr/bin/env python3
"""Offline stats of a g1rs rest file (no robot access): per axis mean / sigma / min / max of ANGLE_ACT, POS_ACT, FORCE_ACT,
CURRENT, TEMP (lo-first); STATUS / ERROR sets; achieved rate from t_mono_ns; drops = gaps > 50 ms; glitch scan = 1 or 2 read
excursions from the run median (ANGLE > 5, POS > 10, FORCE > 30, CURRENT > 50, TEMP > 4 C or a zero byte, STATUS / ERROR change).
   g1rs_rest_stats.py <state_hand.jsonl> [<state_hand.jsonl> ...]  -> <file>.stats.json"""
import json
import statistics as st
import sys

LIM = {"angle_act": 5, "pos_act": 10, "force_act": 30, "current": 50, "temp": 4}
for path in sys.argv[1:]:
    rows = [json.loads(l) for l in open(path)]
    s = [r for r in rows if r["raw"] and "ANGLE_ACT" in r["raw"]]
    ev = [r for r in rows if r["event"] in ("GLITCH", "READ_ERROR") or (r["event"] or "").startswith("STOP")]
    cfg = [r for r in rows if r["raw"] and "ANGLE_SET" in r["raw"]]
    t = [r["t_mono_ns"] / 1e9 for r in s]
    dts = [b - a for a, b in zip(t, t[1:])]
    out = {"file": path, "samples": len(s), "duration_s": round(t[-1] - t[0], 3), "rate_hz": round((len(s) - 1) / (t[-1] - t[0]), 1),
           "dt_ms_median": round(1000 * st.median(dts), 2), "dt_ms_max": round(1000 * max(dts), 2), "drops_gt_50ms": sum(d > 0.05 for d in dts),
           "online_events": [r["event"] for r in ev], "config_start": cfg[0]["dec"] if cfg else None, "config_end": cfg[-1]["dec"] if cfg else None, "axes": {}, "glitches": []}
    for g in ("angle_act", "pos_act", "force_act", "current", "temp"):
        cols = list(zip(*[r["dec"][g] for r in s]))
        out["axes"][g] = {"mean": [round(st.mean(c), 2) for c in cols], "sigma": [round(st.pstdev(c), 3) for c in cols],
                          "min": [min(c) for c in cols], "max": [max(c) for c in cols]}
        med = [st.median(c) for c in cols]
        for k, c in enumerate(cols):
            run = []
            for i, v in enumerate(c + (med[k],)):
                bad = abs(v - med[k]) > LIM[g] or (g == "temp" and v == 0)
                if bad and i < len(c):
                    run.append(i)
                else:
                    if 0 < len(run) <= 2:
                        out["glitches"].append({"group": g, "axis": k, "seqs": [s[j]["seq"] for j in run], "values": [c[j] for j in run],
                                                "median": med[k], "raw": [s[j]["raw"] for j in run]})
                    run = []
    for g in ("status", "error"):
        out["axes"][g] = sorted({tuple(r["dec"][g]) for r in s})
    json.dump(out, open(path + ".stats.json", "w"), indent=1)
    print(path, out["samples"], "samples", out["rate_hz"], "Hz, dt max", out["dt_ms_max"], "ms, drops", out["drops_gt_50ms"], "glitches", len(out["glitches"]), "events", out["online_events"])
