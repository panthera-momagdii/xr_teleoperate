#!/usr/bin/env python3
"""arm_hold (DRY-RUN ONLY, publishes nothing): would hold the arm joints at their current q.
   python scripts/arm_hold.py [--side both] [--kp 40] [--kd 1] [--seconds 10]"""
import cw
import arm_common as ac

p = cw.parser(__doc__)
ac.args(p)
p.add_argument("--seconds", type=float, default=10.0)
a = p.parse_args()
ac.refuse(a)
run = cw.Run("arm_hold", a)
js = ac.joints(a.side)
cw.print_plan("arm_hold (DRY-RUN ONLY)", ["[would read rt/lowstate q of joints %s]" % js,
                                         "[would publish NOTHING here; a live version would send rt/arm_sdk with weight ramp 0 -> 1 over 2 s, "
                                         "q = current q, dq 0, kp %.1f, kd %.1f, tau 0 for %.0f s, then weight 1 -> 0]" % (a.kp, a.kd, a.seconds)])
run.finish("DRY_RUN")
