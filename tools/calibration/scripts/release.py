#!/usr/bin/env python3
"""release (LOCKED until P3): the way out after an aborted press, grip or below_rest (rule 6: rerun the whole block from rest). Hold
at the hand's own reading, FORCE_SET of fingers 0..3 back to 2000, every finger more than 20 counts below --to opened to --to (opening only, away from
the scale), thumbs at their hold values, speeds of fingers 0..3 back to 1000 (power-up values, both hands, 13 Oct). A finger left at
STATUS 3 (force reached) needs --approve-hold-status-3 (rule 2: Mohammed's approval); STATUS 7 needs --approve-hold-clear-7.
   python scripts/release.py --hand right [--to 550] [--speed 100] [--approve-hold-status-3] [--live]"""
import cw
import locked_common as lc

p = cw.parser(__doc__)
p.add_argument("--hand", required=True, choices=cw.HANDS)
p.add_argument("--to", type=int, default=550)
p.add_argument("--speed", type=int, default=100, choices=[100, 500, 1000])
p.add_argument("--approve-hold-clear-7", action="store_true")
p.add_argument("--approve-hold-status-3", action="store_true")
a = p.parse_args()
if not 500 <= a.to <= 700:
    raise SystemExit("REFUSED: --to 500..700")
if a.live:
    lc.check_live("release")
run = cw.Run("release", a)
L = cw.hold_plan_lines(a.hand) + ["[read start values FS0, S0 of fingers 0..3; a finger k moves only if its hold value Hk < %d]" % (a.to - 20)]
L += [cw.w1("FORCE_SET", k, 2000, "only if FS0_%d is not 2000" % k) for k in range(4)]
L += [cw.w1("SPEED_SET", k, a.speed, "only if finger %d moves and S0_%d is not %d" % (k, k, a.speed)) for k in range(4)]
L += [cw.w6(["%d if H%d < %d else H%d" % (a.to, k, a.to - 20, k) for k in range(4)] + ["H4", "H5"], "open those fingers to %d; thumbs at hold (written only if a finger moves)" % a.to)]
L += [cw.w1("SPEED_SET", k, 1000, "only if not 1000") for k in range(4)]
cw.print_plan("release %s to %d" % (a.hand, a.to), L)
if not a.live:
    run.finish("DRY_RUN")
else:
    run.finish(lc.live_hand(a, run, "cw_grip.py", ["--release-to", str(a.to), "--speed", str(a.speed)]))
