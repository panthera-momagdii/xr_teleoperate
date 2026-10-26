#!/usr/bin/env python3
"""below_rest (LOCKED until P3): one finger from its hold value (about 550) toward --floor in --step counts, pausing --pause s at
each, then back to the hold value and its start speed. Blip at the start, idle hand watched read only, block tool stop list.
FORCE_SET of the finger = its at-rest FORCE_ACT (1 s median at the hold) + 400 g (left middle + 500 g) for the run, so a touch stops
the finger (STATUS 3, which ends the run) instead of pushing up to the 2000 g default; start value back at the end (Mohammed, 13 Oct).
   python scripts/below_rest.py --hand right --axis 3 [--floor 0] [--step 25] [--pause 2] [--speed 100] [--live]"""
import cw
import locked_common as lc

p = cw.parser(__doc__)
p.add_argument("--hand", required=True, choices=cw.HANDS)
p.add_argument("--axis", required=True, type=int, choices=range(4))
p.add_argument("--floor", type=int, default=0)
p.add_argument("--step", type=int, default=25)
p.add_argument("--pause", type=float, default=2.0)
p.add_argument("--speed", type=int, default=100, choices=[100, 500, 1000])
p.add_argument("--start", type=int, default=550, help="nominal hold value used for the plan only")
p.add_argument("--force-above-rest", type=int, default=None, help="g above the at-rest FORCE_ACT (default 400; left middle 500)")
p.add_argument("--approve-hold-clear-7", action="store_true")
a = p.parse_args()
if a.force_above_rest is None:
    a.force_above_rest = 500 if (a.hand == "left" and a.axis == 2) else 400
if not (0 <= a.floor <= 540) or a.step < 5 or not (100 <= a.force_above_rest <= 500):
    raise SystemExit("REFUSED: floor 0..540, step >= 5, --force-above-rest 100..500")
if a.live:
    lc.check_live("below_rest")
run = cw.Run("below_rest", a)
k = a.axis
L = cw.hold_plan_lines(a.hand) + ["[read the finger's at-rest FORCE_ACT F0 (1 s median at the hold) and start values FS0, S0]"]
L += [cw.w1("SPEED_SET", k, a.speed, ""), cw.w6(cw.hold_vec({k: "H%d+10" % k}), "blip out"), cw.w6(cw.hold_vec(), "blip back")]
L.append(cw.w1("FORCE_SET", k, "F0+%d (max 3000)" % a.force_above_rest, "a touch stops the finger: STATUS 3 ends the run (no further writes)"))
v = a.start - a.step
while v >= a.floor:
    L.append(cw.w6(cw.hold_vec({k: "H%d-%d (about %d)" % (k, a.start - v, v)}), "step down, pause %.1f s" % a.pause))
    v -= a.step
L += [cw.w6(cw.hold_vec(), "back to the hold value"), cw.w1("FORCE_SET", k, "FS0", "start value back (2000 after power-up)"),
      cw.w6(cw.hold_vec(), "hold value (end check)"), cw.w1("SPEED_SET", k, "S0", "start speed back")]
cw.print_plan("below_rest %s axis %d" % (a.hand, k), L)
if not a.live:
    run.finish("DRY_RUN")
else:
    env = {"CW_FLOOR": str(a.floor), "CW_STEP": str(a.step), "CW_PAUSE": str(a.pause), "CW_SPEED_P3": str(a.speed),
           "CW_FORCE_BELOW_REST": str(a.force_above_rest)}
    run.finish(lc.live_one_finger(a, run, "br", env))
