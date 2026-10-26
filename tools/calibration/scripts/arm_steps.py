#!/usr/bin/env python3
"""arm_steps (DRY-RUN ONLY, publishes nothing): would step one arm joint by +/- delta and back.
   python scripts/arm_steps.py --joint 18 [--delta 0.1] [--reps 3]"""
import cw
import arm_common as ac

p = cw.parser(__doc__)
ac.args(p)
p.add_argument("--joint", required=True, type=int, choices=range(15, 29))
p.add_argument("--delta", type=float, default=0.1)
p.add_argument("--reps", type=int, default=3)
a = p.parse_args()
ac.refuse(a)
run = cw.Run("arm_steps", a)
cw.print_plan("arm_steps (DRY-RUN ONLY)", ["[would publish NOTHING here; a live version would step joint %d by +%.2f / -%.2f rad around its current q, "
                                          "%d repeats, kp %.1f kd %.1f, through rt/arm_sdk with weight ramps]" % (a.joint, a.delta, a.delta, a.reps, a.kp, a.kd)])
run.finish("DRY_RUN")
