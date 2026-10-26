#!/usr/bin/env python3
"""Offline Part B table (no robot access) from <run>/wake/wake_table.json. Woke = the axis's STATUS changed in the window (from the
value just before the write). Rule A = the written axis and every axis after it in register order (axes >= written, thumbs included);
rule B = the written axis plus both thumbs. Per write: STATUS-woke set, axes that moved >= 2 counts, axes with current; per test: the
sets seen, which rule they fit, first STATUS change delay range.   g1rs_partB_table.py <run_dir> [...]  -> prints markdown"""
import json
import os
import sys

AX = ["little", "ring", "middle", "index", "thumb bend", "thumb rot"]
md = ["| test | writes | axis written | STATUS woke (per write, distinct sets) | moved >= 2 counts | drew current | first STATUS change s (min to max) | fits |",
      "|---|---|---|---|---|---|---|---|"]
for d in sys.argv[1:]:
    wt = json.load(open(os.path.join(d, "wake", "wake_table.json")))
    tests = {}
    for w in wt:
        tests.setdefault(w["test"], []).append(w)
    for t, ws in tests.items():
        ax = ws[0]["axis"]
        sets, moved, cur, delays = [], set(), set(), []
        for w in ws:
            s = tuple(a["axis"] for a in w["axes"] if a["first_status_change_s"] is not None)
            sets.append(s)
            moved |= {a["axis"] for a in w["axes"] if a["counts_moved"] >= 2}
            cur |= {a["axis"] for a in w["axes"] if a["peak_current"] > 0}
            delays += [a["first_status_change_s"] for a in w["axes"] if a["first_status_change_s"] is not None]
        ruleA = tuple(range(ax, 6))
        ruleB = tuple(sorted({ax, 4, 5}))
        distinct = sorted(set(sets))
        fitsA = all(s == ruleA for s in sets)
        fitsB = all(s == ruleB for s in sets)
        fit = "both (same set)" if fitsA and fitsB else "written + every axis after it" if fitsA else "written + both thumbs" if fitsB else (
            "none" if all(len(s) == 0 for s in sets) else "neither")
        md.append("| %s | %d | %d %s | %s | %s | %s | %s | %s |" % (t, len(ws), ax, AX[ax], distinct, sorted(moved), sorted(cur),
                  "-" if not delays else "%.3f to %.3f" % (min(delays), max(delays)), fit))
print("\n".join(md))
