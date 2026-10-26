#!/usr/bin/env python3
"""Offline Part A table (no robot access). For each run folder given: per speed, step count, t90 / delay / settling mean and spread
(sd, min to max), peak current of the finger, max temperature (2 s median from the block summary); thumb bend / rotation at the hold
and during the block; re-servo on the first ANGLE write of the block; what each SPEED_SET single-register write woke.
   g1rs_partA_table.py <run_dir> [<run_dir> ...]  -> prints markdown, writes <run_dir>/partA_table.json"""
import json
import os
import sys


def rows_of(p):
    return [json.loads(l) for l in open(p)]


def woke_after(smp, t0, t1, before):
    out = []
    for k in range(6):
        w = [r for r in smp if t0 <= r["t_mono_ns"] / 1e9 < t1]
        if not w:
            continue
        st = any(r["dec"]["status"][k] != before["status"][k] for r in w)
        mv = max(abs(r["dec"]["angle_act"][k] - before["angle_act"][k]) for r in w)
        cur = max(abs(r["dec"]["current"][k]) for r in w)
        if st or mv >= 2 or cur > 0:
            out.append({"axis": k, "status_change": st, "moved": mv, "peak_current": cur})
    return out


md = []
for d in sys.argv[1:]:
    b, h = os.path.join(d, "block"), os.path.join(d, "hold_before")
    an = json.load(open(os.path.join(b, "analysis.json")))
    sm = json.load(open(os.path.join(b, "summary.json")))
    hs = json.load(open(os.path.join(h, "summary.json")))
    rows = [r for r in rows_of(os.path.join(b, "state.jsonl")) if r.get("dec")]
    cm = rows_of(os.path.join(b, "commands.jsonl"))
    hrows = [r for r in rows_of(os.path.join(h, "state.jsonl")) if r.get("dec")]
    htw = rows_of(os.path.join(h, "commands.jsonl"))[0]["t_mono_ns"] / 1e9
    hpost = [r for r in hrows if r["t_mono_ns"] / 1e9 >= htw]
    out = {"run": d, "finger": sm["finger"], "result": sm["result"], "reason": sm["reason"], "speeds": {}}
    md += ["### %s (%s): %s %s" % (d, sm["finger"], sm["result"], sm["reason"] or ""), "",
           "| speed | steps | t90 s mean (sd; min to max) | delay s mean (sd; min to max) | settling s mean | peak current mA |", "|---|---|---|---|---|---|"]
    for sp in (500, 1000):
        u, w = an["timing"]["s%d_up" % sp], an["timing"]["s%d_down" % sp]
        def comb(key):
            xs = [x for x in (u.get(key), w.get(key)) if x]
            if not xs:
                return None
            n = sum(x["n"] for x in xs)
            mean = sum(x["mean"] * x["n"] for x in xs) / n
            var = sum(x["n"] * (x["sd"] ** 2 + (x["mean"] - mean) ** 2) for x in xs) / n
            return {"mean": mean, "sd": var ** 0.5, "min": min(x["min"] for x in xs), "max": max(x["max"] for x in xs), "n": n,
                    "up": u.get(key), "down": w.get(key)}
        t90, de, se, pk = comb("t90_s"), comb("delay_s"), comb("settling_s"), comb("peak_current")
        out["speeds"][sp] = {"t90": t90, "delay": de, "settling": se, "peak_current": pk}
        f = lambda x: "-" if not x else "%.3f (%.3f; %.3f to %.3f)" % (x["mean"], x["sd"], x["min"], x["max"])  # noqa: E731
        md.append("| %d | %s | %s | %s | %s | %s |" % (sp, 0 if not t90 else t90["n"], f(t90), f(de), "-" if not se else "%.3f" % se["mean"],
                  "-" if not pk else "%d" % pk["max"]))
    md.append("")
    md.append("Max temperature (2 s median) per axis: %s; max non-commanded deviation %s; end %s" % (sm["temp_median_max"], sm["max_non_commanded_dev"], sm["end"]))
    # thumbs
    th = {"hold_reading": hs["reading"][4:], "hold_sent": hs["values"][4:],
          "hold_window": {k: {"min": min(r["dec"]["angle_act"][k] for r in hpost), "max": max(r["dec"]["angle_act"][k] for r in hpost),
                              "end": hpost[-1]["dec"]["angle_act"][k], "peak_current": max(abs(r["dec"]["current"][k]) for r in hpost)} for k in (4, 5)},
          "block": {k: {"min": min(r["dec"]["angle_act"][k] for r in rows), "max": max(r["dec"]["angle_act"][k] for r in rows),
                        "peak_current": max(abs(r["dec"]["current"][k]) for r in rows)} for k in (4, 5)}}
    a1 = next(c for c in cm if c["register"] == "ANGLE_SET")
    t1 = a1["t_mono_ns"] / 1e9
    pre = [r for r in rows if r["t_mono_ns"] / 1e9 < t1][-1]
    win = [r for r in rows if t1 <= r["t_mono_ns"] / 1e9 < t1 + 1.5]
    th["first_angle_write_reservo"] = {k: {"before": pre["dec"]["angle_act"][k], "min": min(r["dec"]["angle_act"][k] for r in win),
                                           "max": max(r["dec"]["angle_act"][k] for r in win), "peak_current": max(abs(r["dec"]["current"][k]) for r in win)}
                                       for k in (4, 5)}
    out["thumbs"] = th
    md.append("Thumb bend / rotation: hold reading %s, sent %s; after hold %s / %s; during block %s / %s; first ANGLE write re-servo %s / %s" % (
        th["hold_reading"], th["hold_sent"], th["hold_window"][4], th["hold_window"][5], th["block"][4], th["block"][5],
        th["first_angle_write_reservo"][4], th["first_angle_write_reservo"][5]))
    # SPEED_SET single-register writes: what woke until the next write
    sw = []
    for i, c in enumerate(cm):
        if c["register"] != "SPEED_SET":
            continue
        t0 = c["t_mono_ns"] / 1e9
        t_next = cm[i + 1]["t_mono_ns"] / 1e9 if i + 1 < len(cm) else t0 + 1.0
        bef = [r for r in rows if r["t_mono_ns"] / 1e9 < t0]
        if bef:
            sw.append({"value": c["values"][0], "address": c["address"], "window_s": round(t_next - t0, 3),
                       "woke": woke_after(rows, t0, min(t_next, t0 + 1.0), bef[-1]["dec"])})
    out["speed_writes"] = sw
    md.append("SPEED_SET writes (window to the next write, max 1 s): %s" % [(s["value"], [(w["axis"], w["status_change"], w["moved"], w["peak_current"]) for w in s["woke"]]) for s in sw])
    md.append("")
    json.dump(out, open(os.path.join(d, "partA_table.json"), "w"), indent=1)
print("\n".join(md))
