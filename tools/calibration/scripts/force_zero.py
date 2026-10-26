#!/usr/bin/env python3
"""force_zero: reads the at-rest force offsets (FORCE_ACT mean and sd per axis) of both hands over --seconds, read only.
--rezero (register 1009, force sensor calibration) is LOCKED (Mohammed: do not write 1009).
   python scripts/force_zero.py [--hands right,left] [--seconds 30] [--live]"""
import json
import os
import subprocess
import cw

p = cw.parser(__doc__)
p.add_argument("--hands", default="right,left")
p.add_argument("--seconds", type=float, default=30.0)
p.add_argument("--rezero", action="store_true", help="LOCKED: would write 1 to register 1009")
a = p.parse_args()
if a.rezero:
    cw.refuse_if_locked("force_rezero_1009", True)
    raise SystemExit("--rezero refused")
hands = a.hands.split(",")
run = cw.Run("force_zero", a)
cw.print_plan("force_zero", ["[read only: FORCE_ACT of %s at full rate for %.0f s; mean and sd per axis]" % (",".join(hands), a.seconds)])
if not a.live:
    run.finish("DRY_RUN")
else:
    soc, _ = cw.battery()
    print("BATTERY", soc, "%")
    ps = [subprocess.Popen([cw.PY, "-B", os.path.join(cw.LIB, "g1rs_read.py"), "rest", h, str(a.seconds), run.dir]) for h in hands]
    for x in ps:
        x.wait()
    cw.run_tool([cw.PY, "-B", os.path.join(cw.LIB, "g1rs_rest_stats.py")] + [run.path("state_%s.jsonl" % h) for h in hands])
    res = {}
    for h in hands:
        o = json.load(open(run.path("state_%s.jsonl.stats.json" % h)))
        res[h] = {"force_mean": o["axes"]["force_act"]["mean"], "force_sd": o["axes"]["force_act"]["sigma"], "status": o["axes"]["status"]}
        print("%-5s FORCE_ACT mean %s sd %s" % (h, [round(x, 1) for x in res[h]["force_mean"]], res[h]["force_sd"]))
    json.dump(res, open(run.path("force_offsets.json"), "w"), indent=1)
    run.finish("DONE")
