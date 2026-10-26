#!/usr/bin/env python3
"""press (LOCKED until P3): one fingertip closes toward --close-to with FORCE_SET = the finger's own at-rest FORCE_ACT (1 s median at
the hold) + each level, lowest first; the hand stops the finger at FORCE_SET (STATUS 3, manual 2.6.12) or at the target; pause; open
to the hold value. After level 1 the run waits for the operator: the scale reading decides (file CONTINUE or STOP in the block
folder). FORCE_SET and speed go back to their start values. Levels approved by Mohammed: 200,300,400,500 g above rest (left middle
400,450,500); each level at most 500 g.
   python scripts/press.py --hand right --axis 3 [--levels 200,300,400,500] [--close-to 300] [--pause 2] [--speed 100] [--live]
Scale test from the open pose (Mohammed, 16 Oct): --pose open = hold, fingers 0..3 to --open-to (950) at speed 100 and stay there
(the rest values are saved); then each press run holds the fingers where they are (about 950), so it starts, opens back and ends
at the open pose; --pose rest = hold at the open pose, fingers back down to the saved rest values (only after the scale is removed).
   python scripts/press.py --hand left --pose open [--open-to 950] [--live]
   python scripts/press.py --hand left --axis 2 --levels 200,300,400,500 --gate all --blip-gate --pause 8 --close-to 200 [--live]
   python scripts/press.py --hand left --pose rest [--live]"""
import json
import os
import time
import cw
import locked_common as lc

p = cw.parser(__doc__)
p.add_argument("--hand", required=True, choices=cw.HANDS)
p.add_argument("--axis", type=int, choices=range(4), help="finger to press (not with --pose)")
p.add_argument("--pose", choices=["open", "rest"], help="scale-test pose step instead of a press")
p.add_argument("--open-to", type=int, default=950)
p.add_argument("--levels", default=None, help="grams above the finger's at-rest FORCE_ACT, lowest first, each <= 500")
p.add_argument("--close-to", type=int, default=300)
p.add_argument("--pause", type=float, default=2.0)
p.add_argument("--speed", type=int, default=100, choices=[100, 500, 1000])
p.add_argument("--no-blip", action="store_true", help="position press: no start blip (Mohammed, 16 Oct)")
p.add_argument("--burst-log-only", action="store_true", help="a bad-read burst with an angle/pos change is logged, not a stop (Mohammed, 16 Oct)")
p.add_argument("--gate", choices=["first", "all", "none"], default="first", help="operator gate after level 1 only (Mohammed, P2 answer 4) or after every level")
p.add_argument("--gate-timeout", type=float, default=900.0, help="seconds to wait for CONTINUE or STOP; a timeout counts as STOP")
p.add_argument("--blip-gate", action="store_true", help="blip held --blip-dwell s, then wait for CONTINUE (operator confirms the moving hand)")
p.add_argument("--blip-dwell", type=float, default=2.0)
p.add_argument("--rest-force", type=int, default=None, help="rest force for the limits (g): the closing-pass reading near the plate height "
               "(the reading at the open pose comes from opening and is not used)")
p.add_argument("--targets", default=None, help="position press: angle targets past contact, highest first (e.g. 849,839,829,819); "
               "replaces --levels; the force limit is safety only")
p.add_argument("--safety", type=int, default=700, help="position press: force limit = rest force + this (g); a reading at or above it stops the run")
p.add_argument("--approve-hold-clear-7", action="store_true")
a = p.parse_args()
STATE_DIR = os.environ["CW_MOCK_STATE_DIR"] if cw.MOCK and os.environ.get("CW_MOCK_STATE_DIR") else os.path.join(cw.ROOT, "state")
POSE_FILE = os.path.join(STATE_DIR, "scale_pose_%s.json" % a.hand)
if a.pose:
    if a.axis is not None:
        raise SystemExit("REFUSED: --pose takes no --axis")
    if not 900 <= a.open_to <= 1000:
        raise SystemExit("REFUSED: --open-to 900..1000")
    saved = json.load(open(POSE_FILE)) if os.path.exists(POSE_FILE) else None
    if a.pose == "open" and saved and a.live:
        raise SystemExit("REFUSED: %s exists (the hand is already in the open pose); run --pose rest first" % POSE_FILE)
    if a.pose == "rest" and not saved and a.live:
        raise SystemExit("REFUSED: no saved rest values (%s): the open pose was not set by --pose open" % POSE_FILE)
    if a.live:
        lc.check_live("press")
    run = cw.Run("press_pose_%s" % a.pose, a)
    if a.pose == "open":
        L = cw.hold_plan_lines(a.hand) + [cw.w1("SPEED_SET", f, 100, "only if not already") for f in range(4)]
        L += [cw.w6([a.open_to] * 4 + ["H4", "H5"], "fingers 0..3 open to %d together, thumbs at hold; they stay there for the scale test" % a.open_to),
              cw.w1("SPEED_SET", 0, "S0", "start speeds back (each finger only if it differs)"),
              "[saves the hold values H0..H3 as the rest values to %s]" % POSE_FILE]
        extra = ["--pose-open", str(a.open_to), "--speed", "100"]
    else:
        rv = saved["rest"] if saved else ["R0", "R1", "R2", "R3"]
        L = cw.hold_plan_lines(a.hand) + ["[the hold holds the fingers at the open pose (about %d); scale already removed]" % a.open_to]
        L += [cw.w1("SPEED_SET", f, 100, "only if not already") for f in range(4)]
        L += [cw.w6(list(rv) + ["H4", "H5"], "fingers 0..3 back down to the saved rest values, thumbs at hold"),
              cw.w1("SPEED_SET", 0, 1000, "all finger speeds back to 1000 (power-up value)")]
        extra = ["--pose-rest", ",".join(str(x) for x in rv), "--speed", "100"] if saved else []
    cw.print_plan("press --pose %s %s" % (a.pose, a.hand), L)
    if not a.live:
        run.finish("DRY_RUN")
        raise SystemExit(0)
    res = lc.live_hand(a, run, "cw_grip.py", extra)
    if res == "DONE" and a.pose == "open":
        hs = json.load(open(run.path("hold", "summary.json")))
        os.makedirs(os.path.dirname(POSE_FILE), exist_ok=True)
        json.dump({"hand": a.hand, "rest": hs["values"][:4], "thumbs": hs["values"][4:], "open_to": a.open_to, "run": run.dir,
                   "t_local": time.strftime("%Y-%m-%d %H:%M:%S %Z")}, open(POSE_FILE, "w"), indent=1)
        print("SAVED rest values %s to %s" % (hs["values"][:4], POSE_FILE))
    if res == "DONE" and a.pose == "rest":
        os.rename(POSE_FILE, POSE_FILE.replace(".json", "_used_%s.json" % time.strftime("%Y%m%d_%H%M%S")))
    run.finish(res)
    raise SystemExit(0)
if a.axis is None:
    raise SystemExit("REFUSED: --axis needed for a press")
if a.targets:
    tg = [int(x) for x in a.targets.split(",")]
    if tg != sorted(tg, reverse=True) or len(set(tg)) != len(tg) or not all(300 <= x <= 940 for x in tg) or not (100 <= a.safety <= 1000):
        raise SystemExit("REFUSED: --targets highest first, distinct, each 300..940; --safety 100..1000")
    if a.rest_force is None:
        raise SystemExit("REFUSED: --targets needs --rest-force (the closing-pass reading)")
    if a.live:
        lc.check_live("press")
    run = cw.Run("press_position", a)
    k = a.axis
    L = cw.hold_plan_lines(a.hand) + ["[rest force = --rest-force %d g; safety limit = %d g]" % (a.rest_force, a.rest_force + a.safety)]
    L += [cw.w1("SPEED_SET", k, a.speed, ""), cw.w6(cw.hold_vec({k: "H%d+10" % k}), "blip out" + (", held %.0f s" % a.blip_dwell if a.blip_gate else "")),
          cw.w6(cw.hold_vec(), "blip back" + (", held %.0f s" % a.blip_dwell if a.blip_gate else ""))]
    if a.blip_gate:
        L.append("[WAIT: Brandon confirms which hand moved; CONTINUE or STOP in the block folder; timeout %.0f s = STOP]" % a.gate_timeout)
    L.append(cw.w1("FORCE_SET", k, "%d (rest force + %d; safety only)" % (a.rest_force + a.safety, a.safety), "a reading at or above it (3 reads) stops the run"))
    for i, x in enumerate(tg):
        L += [cw.w6(cw.hold_vec({k: x}), "level %d: position target %d, hold %.0f s" % (i + 1, x, a.pause)), cw.w6(cw.hold_vec(), "open back to the hold value")]
        if i < len(tg) - 1 and (i == 0 or a.gate == "all"):
            L.append("[WAIT: the scale reading decides the next level; CONTINUE or STOP; timeout %.0f s = STOP]" % a.gate_timeout)
    L += [cw.w1("FORCE_SET", k, "FS0", "start value back (2000)"), cw.w6(cw.hold_vec(), "hold value (end)"), cw.w1("SPEED_SET", k, "S0", "start speed back")]
    cw.print_plan("press --targets %s %s axis %d" % (tg, a.hand, k), L)
    if not a.live:
        run.finish("DRY_RUN")
        raise SystemExit(0)
    env = {"CW_TARGETS": ",".join(str(x) for x in tg), "CW_SAFETY": str(a.safety), "CW_PAUSE": str(a.pause), "CW_SPEED_P3": str(a.speed),
           "CW_GATE": a.gate, "CW_GATE_TIMEOUT": str(a.gate_timeout), "CW_BLIP_GATE": "1" if a.blip_gate else "0", "CW_BLIP_DWELL": str(a.blip_dwell),
           "CW_F0_OVERRIDE": str(a.rest_force), "CW_LEVELS": "1", "CW_BURST_MOVE_LOG_ONLY": "1" if a.burst_log_only else "0"}
    run.finish(lc.live_one_finger(a, run, "q" if a.no_blip else "bq", env))
    raise SystemExit(0)
default = "400,450,500" if (a.hand == "left" and a.axis == 2) else "200,300,400,500"
lv = [int(x) for x in (a.levels or default).split(",")]
if a.rest_force is not None and not (-1000 <= a.rest_force <= 2500):
    raise SystemExit("REFUSED: --rest-force -1000..2500 g")
if a.rest_force is not None and any(a.rest_force + x < 0 or a.rest_force + x > 3000 for x in lv):
    raise SystemExit("REFUSED: rest force + level must stay within the register range 0..3000")
if lv != sorted(lv) or any(x <= 0 or x > 500 for x in lv):
    raise SystemExit("REFUSED: levels must go lowest first, each 1..500 g above rest (scale limit 500 g)")
if not (0 <= a.close_to <= 540):
    raise SystemExit("REFUSED: --close-to 0..540")
if a.live:
    lc.check_live("press")
run = cw.Run("press", a)
k = a.axis
L = cw.hold_plan_lines(a.hand) + (["[F0 = --rest-force %d g (closing-pass reading); the reading at the hold is logged, not used]" % a.rest_force]
                                  if a.rest_force is not None else ["[read the finger's at-rest FORCE_ACT F0 (1 s median at the hold) and start values FS0, S0]"])
L += [cw.w1("SPEED_SET", k, a.speed, ""), cw.w6(cw.hold_vec({k: "H%d+10" % k}), "blip out" + (", held %.0f s" % a.blip_dwell if a.blip_gate else "")),
      cw.w6(cw.hold_vec(), "blip back" + (", held %.0f s" % a.blip_dwell if a.blip_gate else ""))]
if a.blip_gate:
    L.append("[WAIT: Brandon confirms which hand moved; file CONTINUE (the expected hand moved) or STOP in the block folder; timeout %.0f s = STOP]" % a.gate_timeout)
for i, x in enumerate(lv):
    L += [cw.w1("FORCE_SET", k, "F0+%d (max 3000)" % x, "level %d g" % x),
          cw.w6(cw.hold_vec({k: a.close_to}), "close toward %d; stops at FORCE_SET (STATUS 3) or at the target; pause %.1f s" % (a.close_to, a.pause)),
          cw.w6(cw.hold_vec(), "open to the hold value")]
    if i < len(lv) - 1 and (i == 0 or a.gate == "all"):
        L.append("[WAIT: the scale reading decides the next level; file CONTINUE or STOP in the block folder; timeout %.0f s = STOP]" % a.gate_timeout)
L += [cw.w1("FORCE_SET", k, "FS0", "start value back (2000)"), cw.w6(cw.hold_vec(), "hold value"), cw.w1("SPEED_SET", k, "S0", "start speed back")]
cw.print_plan("press %s axis %d levels %s" % (a.hand, k, lv), L)
if not a.live:
    run.finish("DRY_RUN")
else:
    env = {"CW_LEVELS": ",".join(str(x) for x in lv), "CW_CLOSE_TO": str(a.close_to), "CW_PAUSE": str(a.pause), "CW_SPEED_P3": str(a.speed), "CW_GATE": a.gate, "CW_GATE_TIMEOUT": str(a.gate_timeout),
           "CW_BLIP_GATE": "1" if a.blip_gate else "0", "CW_BLIP_DWELL": str(a.blip_dwell),
           "CW_F0_OVERRIDE": "" if a.rest_force is None else str(a.rest_force)}
    run.finish(lc.live_one_finger(a, run, "bp", env))
