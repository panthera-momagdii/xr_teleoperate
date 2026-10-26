#!/usr/bin/env python3
"""preflight: reads both hands (5 as-found reads, then --seconds at full rate): STATUS, ERROR, temperature, force, angles, stored
targets. Read only unless --hold is given; --hold sends a fresh hold on that hand (own reading; first after power-up rotation + 15,
later holds keep the stored thumb targets) and checks STATUS 2 / ERROR 0 on all six.
   python scripts/preflight.py [--hold none|right|left|both] [--seconds 10] [--live]"""
import json
import os
import cw

p = cw.parser(__doc__)
p.add_argument("--hold", default="none", choices=["none", "right", "left", "both"])
p.add_argument("--seconds", type=float, default=10.0)
p.add_argument("--approve-hold-clear-7", action="store_true")
a = p.parse_args()
hands = [] if a.hold == "none" else (list(cw.HANDS) if a.hold == "both" else [a.hold])
if a.live and hands:
    cw.refuse_if_locked("live_writes", True)
run = cw.Run("preflight", a)
L = ["[read only: battery; both hands 5 as-found reads (STATE + ANGLE_SET, SPEED_SET, FORCE_SET), then %.0f s at full rate each]" % a.seconds]
for h in hands:
    L += cw.hold_plan_lines(h)
cw.print_plan("preflight (holds: %s)" % (",".join(hands) or "none"), L)
if not a.live:
    run.finish("DRY_RUN")
else:
    soc, summ = cw.battery()
    print("BATTERY", soc, "%", summ)
    os.makedirs(run.path("asfound"))
    cw.run_tool([cw.PY, "-B", os.path.join(cw.LIB, "g1rs_read.py"), "asfound", run.path("asfound")])
    os.makedirs(run.path("rest"))
    import subprocess
    ps = [subprocess.Popen([cw.PY, "-B", os.path.join(cw.LIB, "g1rs_read.py"), "rest", h, str(a.seconds), run.path("rest")]) for h in cw.HANDS]
    for x in ps:
        x.wait()
    cw.run_tool([cw.PY, "-B", os.path.join(cw.LIB, "g1rs_rest_stats.py")] + [run.path("rest", "state_%s.jsonl" % h) for h in cw.HANDS])
    for h in cw.HANDS:
        o = json.load(open(run.path("rest", "state_%s.jsonl.stats.json" % h)))
        ax = o["axes"]
        print("%-5s STATUS %s ERROR %s TEMP %s FORCE %s ANGLE %s" % (h, ax["status"], ax["error"], [round(x) for x in ax["temp"]["mean"]],
              [round(x, 1) for x in ax["force_act"]["mean"]], [round(x, 1) for x in ax["angle_act"]["mean"]]))
    for h in hands:
        st = cw.check_before_write(h, run, a.approve_hold_clear_7)
        cw.hold(h, run, st, sub="hold_%s" % h)
    soc2, _ = cw.battery()
    print("BATTERY end", soc2, "%")
    run.finish("DONE")
