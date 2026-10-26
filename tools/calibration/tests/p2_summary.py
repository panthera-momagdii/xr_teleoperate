#!/usr/bin/env python3
"""Offline P2 summary (no robot access): reads the P2 run folders in calibration_ws/runs and prints the tables for the report."""
import glob
import json
import os

R = "/home/unitree/calibration_ws/runs"


def latest(prefix, n=1, contains=None):
    ds = sorted(d for d in glob.glob(os.path.join(R, prefix + "_2026*")) if not d.endswith(("_dryrun", "_mock")) and "REFUSED" not in d)
    if contains:
        ds = [d for d in ds if contains in open(os.path.join(d, "console.log")).read()]
    return ds[-n:]


out = {}
# preflight
d = latest("preflight")[0]
pf = {}
for h in ("right", "left"):
    o = json.load(open(os.path.join(d, "rest", "state_%s.jsonl.stats.json" % h)))
    pf[h] = {"status": o["axes"]["status"], "error": o["axes"]["error"], "temp": [round(x) for x in o["axes"]["temp"]["mean"]],
             "force": [round(x, 1) for x in o["axes"]["force_act"]["mean"]], "angle": [round(x, 1) for x in o["axes"]["angle_act"]["mean"]], "rate": o["rate_hz"]}
out["preflight"] = pf
# force_zero cold and warm
fz = latest("force_zero", 2)
out["force_zero"] = {os.path.basename(x): json.load(open(os.path.join(x, "force_offsets.json"))) for x in fz}
# tactile
d = latest("tactile_map")[0]
tm = {}
for h in ("right", "left"):
    s = json.load(open(os.path.join(d, "tactile_stats_%s.json" % h)))
    tm[h] = {"samples": s["samples"], "rate": s["rate_hz"], "errors": s["read_errors"],
             "nonzero_arrays": {n: (v["mean_of_array_mean"], v["max"]) for n, v in s["arrays"].items() if v["max"]}}
out["tactile"] = tm
# headcam
d = latest("headcam")[0]
si = json.load(open(os.path.join(d, "frames", "stream_info.json")))
nf = sum(1 for _ in open(os.path.join(d, "frames", "frames.jsonl")))
out["headcam"] = {"frames": nf, "color": {k: si["color"][k] for k in ("width", "height", "fps", "format")}, "color_intr": si["color"]["intrinsics"],
                  "depth": {k: si["depth"][k] for k in ("width", "height", "fps", "format")}, "depth_intr": si["depth"]["intrinsics"]}
# motion runs
mot = {}
for prefix in ("sweep", "steps"):
    for d in latest(prefix, 2):
        cfg = json.load(open(os.path.join(d, "config.json")))
        hand = cfg["args"]["hand"]
        hs = json.load(open(os.path.join(d, "hold", "summary.json")))
        hc = json.load(open(os.path.join(d, "hold", "hold_check.json")))
        bs = json.load(open(os.path.join(d, "block", "summary.json")))
        an = json.load(open(os.path.join(d, "block", "analysis.json"))) if os.path.exists(os.path.join(d, "block", "analysis.json")) else None
        pre = [json.loads(l) for l in open(os.path.join(d, "pre_write_state_%s.jsonl" % hand))]
        pst = [r for r in pre if r.get("dec") and "status" in r["dec"]][-1]["dec"]
        bursts = sum(1 for l in open(os.path.join(d, "block", "state.jsonl")) if "GLITCH_BURST" in l)
        wb = 0
        wd = os.path.join(d, "watch_%s" % ("left" if hand == "right" else "right"))
        for f in glob.glob(os.path.join(wd, "state_*.jsonl")):
            wb += sum(1 for l in open(f) if "GLITCH_BURST" in l)
        rec = {"hand": hand, "result": bs["result"], "reason": bs["reason"], "pre_status": pst["status"], "first_hold": hs["values"][5] == hs["reading"][5] + 15 and pst["status"] == [255] * 6,
               "hold_reading": hs["reading"], "hold_sent": hs["values"], "after_hold_2s": hc["at"]["2"]["angle_act"], "after_hold_last": hc["at"]["last"]["angle_act"],
               "status_after_hold": hc["at"]["last"]["status"], "moves": len(bs["moves"]), "peak_current": bs["peak_current"], "max_dev": bs["max_non_commanded_dev"],
               "temp_max": bs["temp_median_max"], "end": bs["end"], "speed_end": bs["speed_end"], "bursts_block": bursts, "bursts_watch": wb}
        if prefix == "sweep":
            rec["sweep_moves"] = [(m["tag"], m["start"], m["target"], m["settled"], m["t_follow_s"], m["t_settle_s"]) for m in bs["moves"] if m["tag"].startswith("w_")]
        if an:
            rec["timing"] = {k: {x: (None if not v.get(x) else (round(v[x]["mean"], 3), round(v[x]["sd"], 3), v[x]["n"])) for x in ("delay_s", "t90_s", "settling_s", "peak_current")}
                             for k, v in an["timing"].items() if v.get("t90_s")}
        mot["%s_%s" % (prefix, hand)] = rec
out["motion"] = mot
# rest_log
d = latest("rest_log")[0]
rl = {}
for h in ("right", "left"):
    p = os.path.join(d, "watch_%s" % h, "state_%s.jsonl.stats.json" % h)
    if os.path.exists(p):
        o = json.load(open(p))
        rl[h] = {"samples": o["samples"], "rate": o["rate_hz"], "drops": o["drops_gt_50ms"], "angle_mean": [round(x, 1) for x in o["axes"]["angle_act"]["mean"]],
                 "angle_range": [mx - mn for mx, mn in zip(o["axes"]["angle_act"]["max"], o["axes"]["angle_act"]["min"])], "status": o["axes"]["status"],
                 "temp": [round(x) for x in o["axes"]["temp"]["mean"]]}
    ws = os.path.join(d, "watch_%s" % h, "watch_summary_%s.json" % h)
    if os.path.exists(ws):
        rl[h]["watch"] = json.load(open(ws))
out["rest_log"] = rl
json.dump(out, open("/home/unitree/g1rs_20261013_prep/p2_summary.json", "w"), indent=1, default=str)
print(json.dumps(out, indent=1, default=str)[:20000])
