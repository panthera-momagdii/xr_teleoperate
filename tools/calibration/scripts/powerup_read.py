#!/usr/bin/env python3
"""powerup_read: after a power-up, before any hold, reads every register group the 7 Oct tool config names, read only: STATE groups
(5 reads), DEFAULT_SPEED_SET, DEFAULT_FORCE_SET, POS_SET, ANGLE_SET, FORCE_SET, SPEED_SET (3 reads), one frame of the 17 tactile
arrays, both hands. Compares the as-found ANGLE_ACT with the last close values (newest close/state_at_close.json of a g1rs session
unless --last-close is given).
   python scripts/powerup_read.py [--last-close PATH] [--live]"""
import glob
import json
import os
import time
import cw

p = cw.parser(__doc__)
p.add_argument("--last-close", default=None)
a = p.parse_args()
last = a.last_close or max(glob.glob("/home/unitree/g1rs_*/close/state_at_close.json"), key=os.path.getmtime, default=None)
run = cw.Run("powerup_read", a)
cw.print_plan("powerup_read", ["[read only, both hands: STATE x5, CONFIG groups x3, tactile x1; no write]",
                               "[compare as-found ANGLE_ACT with the last close values: %s]" % last])
if not a.live:
    run.finish("DRY_RUN")
else:
    from g1rs_read import Reader
    from h0_ro import STATE, CONFIG, TOUCH, i16
    soc, _ = cw.battery()
    print("BATTERY", soc, "%")
    found = {}
    with open(run.path("state.jsonl"), "w") as f:
        for h in cw.HANDS:
            r = Reader(h, f)
            for i in range(5):
                s = r.sample(tuple(STATE), event="POWERUP_STATE_%d" % i)
            for i in range(3):
                c = r.sample(tuple(CONFIG), table=CONFIG, event="POWERUP_CONFIG_%d" % i)
            t = {n: i16(r.c.read_group(TOUCH, n)) for n in TOUCH}
            f.write(json.dumps({"t_mono_ns": time.monotonic_ns(), "t_host": round(time.time(), 6), "hand": h, "event": "POWERUP_TOUCH", "raw": t}) + "\n")
            r.c.close()
            found[h] = {"state": s["dec"], "config": c["dec"]}
    comp = {}
    lc = json.load(open(last)) if last else None
    for h in cw.HANDS:
        d, cf = found[h]["state"], found[h]["config"]
        print("%-5s STATUS %s ERROR %s TEMP %s FORCE %s" % (h, d["status"], d["error"], d["temp"], d["force_act"]))
        print("      ANGLE_SET %s SPEED %s FORCE_SET %s DEFAULT_SPEED %s DEFAULT_FORCE %s" % (cf["angle_set"], cf["speed_set"], cf["force_set"],
              cf["default_speed_set"], cf["default_force_set"]))
        if lc and h in lc:
            diff = [x - y for x, y in zip(d["angle_act"], lc[h]["angle_act"])]
            comp[h] = {"as_found": d["angle_act"], "last_close": lc[h]["angle_act"], "diff": diff}
            print("      ANGLE as found %s | last close %s | diff %s" % (d["angle_act"], lc[h]["angle_act"], diff))
    json.dump({"last_close_file": last, "comparison": comp, "found": found}, open(run.path("powerup_summary.json"), "w"), indent=1)
    run.finish("DONE")
