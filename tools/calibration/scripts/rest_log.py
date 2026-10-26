#!/usr/bin/env python3
"""rest_log: logs both hands (or the given hands) at full rate for N minutes, read only (watch tool, log only). With --hold a fresh
hold is sent on each listed hand right before the log (live writes).
   python scripts/rest_log.py [--hands right,left] [--minutes 5] [--hold] [--live]"""
import os
import time
import cw

p = cw.parser(__doc__)
p.add_argument("--hands", default="right,left")
p.add_argument("--minutes", type=float, default=5.0)
p.add_argument("--hold", action="store_true")
p.add_argument("--approve-hold-clear-7", action="store_true")
a = p.parse_args()
hands = a.hands.split(",")
if a.live and a.hold:
    cw.refuse_if_locked("live_writes", True)
run = cw.Run("rest_log", a)
L = []
if a.hold:
    for h in hands:
        L += cw.hold_plan_lines(h)
L.append("[read only: %s logged at full rate (STATE groups, about 120 Hz each) for %.1f min; nothing moves]" % (",".join(hands), a.minutes))
cw.print_plan("rest_log", L)
if not a.live:
    run.finish("DRY_RUN")
else:
    soc, _ = cw.battery()
    print("BATTERY", soc, "%")
    if a.hold:
        for h in hands:
            st = cw.check_before_write(h, run, a.approve_hold_clear_7)
            cw.hold(h, run, st, sub="hold_%s" % h)
    ws = [cw.Watch(h, run) for h in hands]
    time.sleep(a.minutes * 60)
    for w in ws:
        w.stop()
    cw.run_tool([cw.PY, "-B", os.path.join(cw.LIB, "g1rs_rest_stats.py")] + [run.path("watch_%s" % h, "state_%s.jsonl" % h) for h in hands])
    soc2, _ = cw.battery()
    print("BATTERY end", soc2, "%")
    run.finish("DONE")
