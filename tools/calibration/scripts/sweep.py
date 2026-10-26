#!/usr/bin/env python3
"""sweep: one finger slowly through a range and back (default 650..950 at speed 100), blip at the start, back to the start pose.
   python scripts/sweep.py --hand right --axis 3 [--range 650,950] [--speed 100] [--live]
--record (one finger = one video clip): speed 100, blip +10 held 2 s and back, one slow pass to 950 and back, then FORCE_SET = rest
+ 400 g (left middle + 500 g) and one pass down to the floor + 15 counts and back; FORCE_SET and speed back. Floor: --floor, else
MEASURED_FLOOR, else a floor-finding pass first (speed 100, 25-count steps, FORCE_SET rest + 400 g, stops at the first STATUS 3,
a step it cannot reach, or 0). Live needs live_writes and sweep_wide_range open (Mohammed: GO on the day only).
   python scripts/sweep.py --record --hand left --axis 3 [--floor 35] [--top 950] [--dwell 2] [--live]
--record on a thumb (axis 4 bend, 5 rotation; live needs live_writes and thumb_motion open): fingers 0..3 to 950 first, thumb blip
+10 held 2 s, rotation only: up to 1000 and back, then FORCE_SET rest + 400 g and a floor-finding pass down from the hold value in
25-count steps (stops at the first STATUS 3, a step missed by more than 10, or 0), 2 s there, back to the hold value, fingers back.
   python scripts/sweep.py --record --hand left --axis 4|5 [--dwell 2] [--live]"""
import os
import cw
import motion_common as mc

MEASURED_FLOOR = {("left", 3): 35}       # left index: stopped at 35 with STATUS 2 (below_rest floor 0, 14 Oct 18:37; Mohammed)
p = cw.parser(__doc__)
mc.finger_args(p)
p.add_argument("--speed", type=int, default=100, choices=[100, 500, 1000])
p.add_argument("--record", action="store_true", help="video mode: blip 2 s, pass to --top and back, pass to the floor and back")
p.add_argument("--floor", type=int, default=None, help="record: floor target (default: the measured floor of this finger)")
p.add_argument("--top", type=int, default=950)
p.add_argument("--dwell", type=float, default=2.0, help="record: dwell at each end, s")
p.add_argument("--blip-dwell", type=float, default=2.0)
p.add_argument("--margin", type=int, default=15, help="record: the floor pass ends this many counts above the floor")
p.add_argument("--find-step", type=int, default=25, help="record: step of the floor-finding pass")
p.add_argument("--force-above-rest", type=int, default=None, help="record: g above rest for the floor pass (default 400; left middle 500)")
a = p.parse_args()
THUMB = a.record and a.axis >= 4
if THUMB:
    if a.floor is not None or not (100 <= (a.force_above_rest or 400) <= 500) or not (5 <= a.find_step <= 50):
        raise SystemExit("REFUSED: thumb record: no --floor; --force-above-rest 100..500; --find-step 5..50")
    a.force_above_rest = a.force_above_rest or 400
    if a.live and not (cw.MOCK and os.environ.get("CW_MOCK_P3") == "1"):                # mock tests only
        cw.refuse_if_locked("thumb_motion", True)
    run = cw.Run("sweep_record", a)
    k, st = a.axis, a.find_step
    other = "left" if a.hand == "right" else "right"
    base = {f: 950 for f in range(4)}
    L = ["[battery read; idle %s hand watched read only at full rate, log only]" % other] + cw.hold_plan_lines(a.hand)
    L += ["[block within 30 s of the hold; 135 Hz log; stop list; rule 4 on the other thumb; read F0 (thumb at-rest FORCE_ACT), FS0, S0]"]
    L += [cw.w1("SPEED_SET", f, 100, "only if not already") for f in range(4)]
    L += [cw.w6([950] * 4 + ["H4", "H5"], "fingers 0..3 open to 950 first (wait until still)"), cw.w1("SPEED_SET", k, 100, "only if not already"),
          cw.w6(cw.hold_vec({**base, k: "H%d+10 (max 1000)" % k}), "thumb blip out, held %.0f s (clip marker)" % a.blip_dwell),
          cw.w6(cw.hold_vec(base), "thumb blip back, held %.0f s" % a.blip_dwell)]
    if k == 5:
        L += [cw.w6(cw.hold_vec({**base, 5: 1000}), "rotation up to 1000 (top of its span; lands about 985), dwell %.0f s" % a.dwell),
              cw.w6(cw.hold_vec(base), "back to the hold value, dwell %.0f s" % a.dwell)]
    L += [cw.w1("FORCE_SET", k, "F0+%d (0..3000)" % a.force_above_rest, "floor-finding pass; STATUS 3 allowed here"),
          cw.w6(cw.hold_vec({**base, k: "H%d-%d" % (k, st)}), "step down, then H%d-%d, H%d-%d, ... to 0 (dwell 0.5 s each); stops at the first STATUS 3, "
                "a step missed by more than 10%s, or 0" % (k, 2 * st, k, 3 * st, " (against target - 15)" if k == 5 else "")),
          "[%.0f s at that end]" % a.dwell,
          cw.w6(cw.hold_vec(base), "back to the hold value, dwell %.0f s" % a.dwell),
          cw.w1("FORCE_SET", k, "FS0", "start value back (2000)"),
          cw.w6(cw.hold_vec(), "fingers 0..3 back to their hold values"),
          "[end check, reads only]"]
    L += [cw.w1("SPEED_SET", f, "S0_%d" % f, "start speed back (only if it differs)") for f in (0, 1, 2, 3, k)]
    cw.print_plan("sweep --record %s axis %d %s" % (a.hand, k, cw.AXIS[k]), L)
    if not a.live:
        run.finish("DRY_RUN")
    else:
        env = {"CW_FINGERS_OPEN": "950", "CW_BLIP": str(a.blip), "CW_BLIP_DWELL": str(a.blip_dwell), "CW_PAUSE": str(a.dwell), "CW_SPEED_P3": "100",
               "CW_FORCE_BELOW_REST": str(a.force_above_rest), "CW_STEP": str(st)}
        run.finish(mc.live_block(a, run, "TR", env))
    raise SystemExit(0)
if a.record:
    if a.axis > 3:
        raise SystemExit("REFUSED: --record is for fingers 0..3")
    known = MEASURED_FLOOR.get((a.hand, a.axis))
    floor = a.floor if a.floor is not None else known
    if a.force_above_rest is None:
        a.force_above_rest = 500 if (a.hand == "left" and a.axis == 2) else 400
    if floor is not None and known is not None and floor < known:
        raise SystemExit("REFUSED: --floor %d below the measured floor %d of this finger" % (floor, known))
    if not (550 < a.top <= 1000) or not (100 <= a.force_above_rest <= 500) or (floor is not None and not 0 <= floor <= 540):
        raise SystemExit("REFUSED: --top 551..1000, --force-above-rest 100..500, floor 0..540")
    if not (0 <= a.margin <= 50) or not (5 <= a.find_step <= 50):
        raise SystemExit("REFUSED: --margin 0..50, --find-step 5..50")
    if a.live:
        if not (cw.MOCK and os.environ.get("CW_MOCK_P3") == "1"):       # mock tests only
            cw.refuse_if_locked("sweep_wide_range", True)
run = cw.Run("sweep_record" if a.record else "sweep", a)
if a.record:
    k = a.axis
    fl = str(floor + a.margin) if floor is not None else "FLOOR+%d (FLOOR found above)" % a.margin
    other = "left" if a.hand == "right" else "right"
    L = ["[battery read; idle %s hand watched read only at full rate, log only]" % other] + cw.hold_plan_lines(a.hand)
    L += ["[block within 30 s of the hold; 135 Hz log; stop list; read F0 (at-rest FORCE_ACT, 1 s median), FS0, S0]",
          cw.w1("SPEED_SET", k, 100, "only if not already")]
    if floor is None:
        L += [cw.w1("FORCE_SET", k, "F0+%d (max 3000)" % a.force_above_rest, "floor-finding pass (no measured floor for this finger)"),
              cw.w6(cw.hold_vec({k: "H%d-%d" % (k, a.find_step)}), "step down, then H%d-%d, H%d-%d, ... to 0; STATUS 3 allowed here; stops at the "
                    "first STATUS 3, a step it cannot reach (more than 10 short) or 0: FLOOR = that angle" % (k, 2 * a.find_step, k, 3 * a.find_step)),
              cw.w6(cw.hold_vec(), "back to the hold value, dwell %.0f s" % a.dwell)]
    L += [
          cw.w6(cw.hold_vec({k: "H%d+10" % k}), "blip out, held %.0f s (clip marker)" % a.blip_dwell),
          cw.w6(cw.hold_vec(), "blip back, held %.0f s" % a.blip_dwell),
          cw.w6(cw.hold_vec({k: a.top}), "slow pass up to %d, dwell %.0f s" % (a.top, a.dwell)),
          cw.w6(cw.hold_vec(), "back to the hold value, dwell %.0f s" % a.dwell),
          cw.w1("FORCE_SET", k, "F0+%d (max 3000)" % a.force_above_rest, ("only if not already; " if floor is None else "")
                + "a touch on the way down stops the finger (STATUS 3 ends the run)"),
          cw.w6(cw.hold_vec({k: fl}), "slow pass down to the floor + %d, dwell %.0f s" % (a.margin, a.dwell)),
          cw.w6(cw.hold_vec(), "back to the hold value, dwell %.0f s" % a.dwell),
          cw.w1("FORCE_SET", k, "FS0", "start value back (2000 after power-up)"),
          "[end check, reads only: finger within 10 of the hold value, others within 5, STATUS 2]",
          cw.w1("SPEED_SET", k, "S0", "start speed back (only if it differs)")]
    cw.print_plan("sweep --record %s axis %d %s" % (a.hand, k, cw.AXIS[k]), L)
    if not a.live:
        run.finish("DRY_RUN")
    else:
        env = {"CW_RECORD_TOP": str(a.top), "CW_FLOOR": str(floor if floor is not None else -1), "CW_FLOOR_MARGIN": str(a.margin),
               "CW_STEP": str(a.find_step), "CW_BLIP": str(a.blip), "CW_BLIP_DWELL": str(a.blip_dwell),
               "CW_PAUSE": str(a.dwell), "CW_SPEED_P3": "100", "CW_FORCE_BELOW_REST": str(a.force_above_rest)}
        run.finish(mc.live_block(a, run, "R", env))
else:
    lo, hi = mc.check_args(a, a.live)
    a.reps = 0
    cw.print_plan("sweep %s axis %d %s" % (a.hand, a.axis, cw.AXIS[a.axis]), mc.plan_block(a, lo, hi, [a.speed], "w"))
    if not a.live:
        run.finish("DRY_RUN")
    else:
        env = {"CW_SWEEP_SPEED": str(a.speed), "CW_RANGE": "%d,%d" % (lo, hi), "CW_BLIP": str(a.blip)}
        run.finish(mc.live_block(a, run, "bw", env))
