#!/usr/bin/env python3
"""RT7: one arm run, summarised from the session files. CPU only, reads files.

Usage: run_summary.py OUTDIR START_HHMMSS [END_HHMMSS]
  OUTDIR       rt7/out/armN (launcher_pane.log, arm_watch.csv, arm_watch.out, session_watch.csv)
  START        the launch time (first pane line at or after it belongs to this run)
  END          optional; default: the first "launcher exited" line after START, plus 60 s
Stats: the newest ~/xrt/logs/launcher_stats_<pid>.csv created after START.
"""
import csv, glob, os, re, sys, time
import numpy as np

out, start = sys.argv[1], sys.argv[2]
end = sys.argv[3] if len(sys.argv) > 3 else None
sec = lambda hms: int(hms[:2]) * 3600 + int(hms[3:5]) * 60 + int(hms[6:8])
NAMES = ["L sh pitch", "L sh roll", "L sh yaw", "L elbow", "L wr roll", "L wr pitch", "L wr yaw",
         "R sh pitch", "R sh roll", "R sh yaw", "R elbow", "R wr roll", "R wr pitch", "R wr yaw"]

pane = [l.rstrip("\n") for l in open(os.path.join(out, "launcher_pane.log"), errors="replace")]
ev = [(l[:12], l[13:]) for l in pane if re.match(r"\d\d:\d\d:\d\d\.\d{3} ", l) and " INFO " not in l[13:40]]
ev = [(t, x) for t, x in ev if sec(t) >= sec(start)]
def first(pat, after=None):
    for t, x in ev:
        if re.search(pat, x) and (after is None or sec(t) >= sec(after)):
            return t, x
    return None, None
t_exit, x_exit = first(r"launcher exited with")
if end is None:
    end = time.strftime("%H:%M:%S", time.gmtime(sec(t_exit[:8]) + 60)) if t_exit else "23:59:59"
ev = [(t, x) for t, x in ev if sec(t) <= sec(end)]
t_trk, _ = first(r"\[link\] tracking")
t_low, _ = first(r"\[lower\] lowering the arms")
t_done, x_done = first(r"\[lower\] (done|TIMED OUT) in")
holds = [(t, x) for t, x in ev if "[link] HOLD" in x]
reeng = [(t, x) for t, x in ev if "[link] re-engaged" in x]
caps = [(t, x) for t, x in ev if "[cap]" in x and "limit" in x]
ikf = [(t, x) for t, x in ev if "[ik] worker process failed" in x]
keys = [(t, x) for t, x in ev if "[key]" in x]
follow_s = (sec(t_low) - sec(t_trk)) if (t_trk and t_low) else None
print(f"run from {start}: tracking {t_trk}, lowering {t_low}, {x_done or 'no lower result'}, {x_exit or 'no exit line'}")
print(f"  following (first tracking to lowering): {follow_s} s; HOLD events {len(holds)}, re-engaged {len(reeng)}; "
      f"[cap] notices {len(caps)}; IK worker failures {len(ikf)}; keys {[x.split()[1] if len(x.split()) > 1 else x for _, x in keys]}")
for t, x in holds:
    print(f"    {t} {x[:110]}")

# watcher
WCSV = os.environ.get("WATCH_CSV", "arm_watch.csv")
rows = [r for r in csv.DictReader(open(os.path.join(out, WCSV))) if r.get("time", "").count(":") == 2
        and sec(start) <= sec(r["time"]) <= sec(end)]
def colmax(c, rr):
    v = [float(r[c]) for r in rr if r.get(c) not in (None, "", "nan")]
    return max(v) if v else float("nan")
if rows and "spL_housing_C" in rows[0]:
    print(f"  shoulder pitch peak housing [0] L {colmax('spL_housing_C', rows):.0f} R {colmax('spR_housing_C', rows):.0f} C, "
          f"winding [1] L {colmax('spL_winding_C', rows):.0f} R {colmax('spR_winding_C', rows):.0f} C; "
          f"arm max housing {colmax('housing_max_C', rows):.0f}, winding {colmax('winding_max_C', rows):.0f} C")
    first_row = rows[0]
    print(f"  at launch: shoulder pitch housing L {first_row['spL_housing_C']} R {first_row['spR_housing_C']}, "
          f"winding L {first_row['spL_winding_C']} R {first_row['spR_winding_C']} C")
notes = [r["note"] for r in rows if r.get("note")]
print(f"  watcher notes: {notes if notes else 'none'}")
if t_low:
    lw = [r for r in rows if sec(t_low[:8]) <= sec(r["time"]) <= sec((t_done or end)[:8]) + 2]
    if lw and "dqmax_15" in lw[0]:
        pk = [colmax(f"dqmax_{i}", lw) for i in range(15, 29)]
        print("  lowering peak |dq| per joint (rad/s, raw, per-second max of every lowstate sample):")
        print("    " + ", ".join(f"{NAMES[k]} {pk[k]:.2f}" for k in range(14)))
        over = [NAMES[k] for k in range(14) if pk[k] > 0.3]
        print(f"    over 0.3 rad/s: {over if over else 'none'}")
        WR = (19, 20, 21, 26, 27, 28)
        base = [r for r in rows if t_trk and sec(r["time"]) < sec(t_trk[:8])]
        def wstat(rr, key):
            return {i: max((float(r[f"{key}_{i}"]) for r in rr if r.get(f"{key}_{i}") not in (None, "", "nan")), default=float("nan")) for i in WR}
        rv, rb = wstat(lw, "wrist_rev"), wstat(base, "wrist_rev")
        print("  wrist buzz indicators, lowering (before [r] at rest in brackets):")
        print("    dq sign flips per s, max: " + ", ".join(f"{NAMES[i - 15]} {rv[i]:.0f} ({rb[i]:.0f})" for i in WR))
        if "wrist_taupp_19" in lw[0]:
            tv, tb = wstat(lw, "wrist_taupp"), wstat(base, "wrist_taupp")
            print("    tau_est swing per s (Nm), max: " + ", ".join(f"{NAMES[i - 15]} {tv[i]:.2f} ({tb[i]:.2f})" for i in WR))
            bz = sum(1 for r in lw if any(float(r[f"wrist_rev_{i}"]) >= 10 and float(r[f"wrist_taupp_{i}"] or 0) >= 1.0 for i in WR))
            print(f"    buzz seconds (a wrist with >= 10 sign flips and >= 1 Nm swing in the same second): {bz} of {len(lw)}")
    fw = [r for r in rows if t_trk and sec(t_trk[:8]) <= sec(r["time"]) < sec(t_low[:8])]
    if fw and "wrist_rev_19" in fw[0]:
        rv = {i: max(int(r[f'wrist_rev_{i}']) for r in fw) for i in (19, 20, 21, 26, 27, 28)}
        print(f"  (same indicator while following, max: {', '.join(f'{NAMES[i - 15]} {v}' for i, v in rv.items())})")

# launcher stats
cands = [f for f in glob.glob(os.path.expanduser("~/xrt/logs/launcher_stats_*.csv"))]
t0 = time.mktime(time.strptime(time.strftime("%Y-%m-%d ") + start, "%Y-%m-%d %H:%M:%S"))
cands = sorted([f for f in cands if os.path.getmtime(f) >= t0], key=os.path.getmtime)
if cands:
    f = cands[0]
    L = [l.strip().split(",") for l in open(f) if l.strip()]
    hdr = L[0]; R = [dict(zip(hdr, l)) for l in L[1:] if l[0] != "t_mono"]
    fo = [r for r in R if r.get("guard") == "FOLLOW"]
    cf = (sum(int(r["cap_frames_L"] or 0) for r in fo), sum(int(r["cap_frames_R"] or 0) for r in fo))
    hz = [float(r["loop_hz"]) for r in fo]
    tm = [float(r["t_mono"]) for r in R]
    ctrl = []
    for a, b in zip(R, R[1:]):
        if b.get("guard") == "FOLLOW" and b.get("ctrl_steps"):
            ctrl.append(int(b["ctrl_steps"]) / (float(b["t_mono"]) - float(a["t_mono"])))
    frames = int(sum(float(r["loop_hz"]) * 5 for r in fo)) if fo else 0
    print(f"  stats {os.path.basename(f)}: {len(fo)} FOLLOW rows of 5 s; cap frames L {cf[0]} R {cf[1]} "
          f"(of about {frames} frames, {100 * cf[0] / max(frames, 1):.0f} / {100 * cf[1] / max(frames, 1):.0f} %); "
          f"main loop {np.mean(hz) if hz else float('nan'):.1f} Hz; controller {min(ctrl) if ctrl else float('nan'):.1f} to "
          f"{max(ctrl) if ctrl else float('nan'):.1f} Hz (median {np.median(ctrl) if ctrl else float('nan'):.1f}); "
          f"IK p50 {np.median([float(r['ik_p50_ms']) for r in fo]) if fo else float('nan'):.2f} ms, "
          f"p99 max {max([float(r['ik_p99_ms']) for r in fo]) if fo else float('nan'):.2f} ms; "
          f"commanded shoulder ff max L {colmax('sp_ff_cmd_max_L_nm', fo):.2f} R {colmax('sp_ff_cmd_max_R_nm', fo):.2f} Nm "
          f"(IK asked up to {colmax('sp_ff_pred_max_L_nm', fo):.2f} / {colmax('sp_ff_pred_max_R_nm', fo):.2f}); "
          f"margin clips {sum(int(r['margin_clips'] or 0) for r in fo)}")
else:
    print("  stats: no launcher_stats file after START")

# session watcher: websocket RTT and video
sw = os.path.join(out, "session_watch.csv")
if os.path.exists(sw) and t_trk:
    S = [r for r in csv.DictReader(open(sw)) if sec(t_trk[:8]) <= sec(r["time"]) <= sec((t_low or end)[:8])]
    rtt = [float(r["ws_rtt_ms_remote"]) for r in S if r.get("ws_rtt_ms_remote") not in (None, "", "nan") and float(r["ws_rtt_ms_remote"]) > 0]
    tx = [float(r["ws_tx_Bps_remote"]) for r in S if r.get("ws_tx_Bps_remote") not in (None, "", "nan")]
    if rtt:
        print(f"  8012 websocket TCP RTT (kernel, PC2 side): median {np.median(rtt):.0f} ms, p95 {np.percentile(rtt, 95):.0f}, "
              f"max {max(rtt):.0f} ms over {len(rtt)} s; video downlink median {8 * np.median(tx) / 1e6:.2f} Mbit/s")
    else:
        print("  8012 websocket: no remote RTT samples in the window")
