#!/usr/bin/env python3
"""steps: timed steps on one finger: to lo, then (hi, lo) x reps at each speed (default 500,1000; 100 allowed), blip at the start,
back to the start pose. Direct six-value writes (proven block tool).
   python scripts/steps.py --hand right --axis 3 [--speeds 100,500,1000] [--reps 6] [--range 650,950] [--live]"""
import cw
import motion_common as mc

p = cw.parser(__doc__)
mc.finger_args(p)
p.add_argument("--speeds", default="500,1000")
p.add_argument("--reps", type=int, default=6)
p.add_argument("--thumb-step", type=int, default=50, help="thumbs (axis 4, 5): one step of this many counts and back (max 50)")
p.add_argument("--fingers-open", type=int, default=950, help="thumb rotation: fingers 0..3 opened to this first")
p.add_argument("--pause", type=float, default=2.0, help="thumbs: dwell at the step and back, s")
a = p.parse_args()
if a.axis >= 4 and not (0 < a.thumb_step <= 50 and 900 <= a.fingers_open <= 1000):
    raise SystemExit("REFUSED: --thumb-step 1..50, --fingers-open 900..1000")
run = cw.Run("steps", a)
lo, hi = mc.check_args(a, a.live)
speeds = [int(x) for x in a.speeds.split(",")]
if any(s not in (100, 500, 1000) for s in speeds):
    raise SystemExit("speeds must be from 100, 500, 1000")
cw.print_plan("steps %s axis %d %s" % (a.hand, a.axis, cw.AXIS[a.axis]), mc.plan_thumb(a) if a.axis >= 4 else mc.plan_block(a, lo, hi, speeds, "s"))
if not a.live:
    run.finish("DRY_RUN")
elif a.axis >= 4:
    env = {"CW_THUMB_STEP": str(a.thumb_step), "CW_FINGERS_OPEN": str(a.fingers_open), "CW_PAUSE": str(a.pause), "CW_SPEED_P3": "100"}
    run.finish(mc.live_block(a, run, "t", env))
else:
    env = {"CW_SPEEDS": a.speeds, "CW_REPS": str(a.reps), "CW_RANGE": "%d,%d" % (lo, hi), "CW_BLIP": str(a.blip)}
    run.finish(mc.live_block(a, run, "bs", env))
