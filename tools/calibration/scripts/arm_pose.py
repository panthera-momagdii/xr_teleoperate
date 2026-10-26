#!/usr/bin/env python3
"""arm_pose (DRY-RUN ONLY, publishes nothing): would move the arm joints to a named pose and back.
   python scripts/arm_pose.py --pose elbows_bent [--side both] [--seconds 3]"""
import cw
import arm_common as ac

POSES = {"elbows_bent": {18: 0.6, 25: 0.6}, "hands_forward": {15: -0.4, 22: -0.4, 18: 0.9, 25: 0.9}}
p = cw.parser(__doc__)
ac.args(p)
p.add_argument("--pose", required=True, choices=sorted(POSES))
p.add_argument("--seconds", type=float, default=3.0)
a = p.parse_args()
ac.refuse(a)
run = cw.Run("arm_pose", a)
tgt = {j: v for j, v in POSES[a.pose].items() if j in ac.joints(a.side)}
cw.print_plan("arm_pose (DRY-RUN ONLY)", ["[would read the current q of joints %s]" % ac.joints(a.side),
                                         "[would publish NOTHING here; a live version would interpolate over %.1f s to the offsets %s rad "
                                         "(kp %.1f, kd %.1f), hold 2 s, return the same way]" % (a.seconds, tgt, a.kp, a.kd)])
run.finish("DRY_RUN")
