#!/usr/bin/env python3
"""grip (LOCKED until P3): fingers 0..3 close together toward --close-to with FORCE_SET = each finger's at-rest FORCE_ACT + its level
(200 g; left middle 400 g), hold until released (file RELEASE in the block folder, or --max-hold-s), open to the hold values,
FORCE_SET and speeds back to their start values. Thumbs stay at their hold values (rule 8).
   python scripts/grip.py --hand right [--level 200] [--close-to 400] [--max-hold-s 10] [--speed 100] [--live]"""
import cw
import locked_common as lc

p = cw.parser(__doc__)
p.add_argument("--hand", required=True, choices=cw.HANDS)
p.add_argument("--level", type=int, default=200, help="grams above each finger's at-rest FORCE_ACT (left middle uses --level-left-middle)")
p.add_argument("--level-left-middle", type=int, default=400)
p.add_argument("--close-to", type=int, default=400)
p.add_argument("--max-hold-s", type=float, default=10.0)
p.add_argument("--speed", type=int, default=100, choices=[100, 500, 1000])
p.add_argument("--approve-hold-clear-7", action="store_true")
a = p.parse_args()
levels = [a.level] * 4
if a.hand == "left":
    levels[2] = a.level_left_middle
if any(x <= 0 or x > 500 for x in levels) or not (300 <= a.close_to <= 540):
    raise SystemExit("REFUSED: levels 1..500 g, --close-to 300..540")
if a.live:
    lc.check_live("grip")
run = cw.Run("grip", a)
L = cw.hold_plan_lines(a.hand) + ["[read F0 (at-rest FORCE_ACT, 1 s median at the hold) and start values FS0, S0 of fingers 0..3]"]
L += [cw.w1("SPEED_SET", k, a.speed, "only if not already") for k in range(4)] + [cw.w1("FORCE_SET", k, "F0_%d+%d" % (k, levels[k]), "") for k in range(4)]
L += [cw.w6(cw.hold_vec({3: "H3+10"}), "blip out"), cw.w6(cw.hold_vec(), "blip back"),
      cw.w6([a.close_to] * 4 + ["H4", "H5"], "close fingers 0..3, thumbs at hold; hold until RELEASE (max %.0f s)" % a.max_hold_s),
      cw.w6(cw.hold_vec(), "open to the hold values")]
L += [cw.w1("FORCE_SET", k, "FS0_%d" % k, "start value back") for k in range(4)] + [cw.w1("SPEED_SET", k, "S0_%d" % k, "start speed back") for k in range(4)]
cw.print_plan("grip %s levels %s" % (a.hand, levels), L)
if not a.live:
    run.finish("DRY_RUN")
else:
    run.finish(lc.live_hand(a, run, "cw_grip.py", ["--levels", ",".join(str(x) for x in levels), "--close-to", str(a.close_to),
                                                   "--max-hold-s", str(a.max_hold_s), "--speed", str(a.speed)]))
