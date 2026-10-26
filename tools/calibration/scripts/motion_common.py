#!/usr/bin/env python3
"""shared flow for the one-finger motion scripts (sweep, steps): dry-run plan, or a live run: battery, STATUS / ERROR read on the
hand, idle-hand watch (log only), fresh hold (first after power-up: own reading, rotation + 15; later: thumbs keep stored targets),
STATUS 2 / ERROR 0 check, block tool within 30 s (blip, then the stage), hand back at its start pose (hold value, start speed)."""
import os
import cw


def finger_args(p):
    p.add_argument("--hand", required=True, choices=cw.HANDS)
    p.add_argument("--axis", required=True, type=int, choices=range(6), help="0 little, 1 ring, 2 middle, 3 index (4, 5 thumbs: locked)")
    p.add_argument("--range", default="650,950", help="lo,hi inside 650..950 (wider: locked until P3)")
    p.add_argument("--approve-hold-clear-7", action="store_true", help="allow the hold to clear a STATUS 7 (Mohammed's approval)")
    p.add_argument("--blip", type=int, default=10, help="blip size in counts at the start (opening direction)")


def check_args(a, live):
    lo, hi = [int(x) for x in a.range.split(",")]
    if a.axis >= 4:
        if not (cw.MOCK and os.environ.get("CW_MOCK_P3") == "1"):      # mock tests only
            cw.refuse_if_locked("thumb_motion", True if live else False)
        if not live:
            print("NOTE: thumb axis: live is LOCKED (rule 8)")
    if lo < 650 or hi > 950 or lo >= hi:
        lk = cw.refuse_if_locked("sweep_wide_range", live)
        if not live and lk:
            print("NOTE: range %d..%d outside 650..950: live is LOCKED" % (lo, hi))
    return lo, hi


def plan_block(a, lo, hi, speeds, stage):
    other = "left" if a.hand == "right" else "right"
    L = ["[battery read; idle %s hand watched read only at full rate, log only]" % other] + cw.hold_plan_lines(a.hand)
    k = a.axis
    L.append("[block within 30 s of the hold; 135 Hz log; stop list: ERROR 6 reads, positions 3 reads, temp 2 s median, bursts, thumbs rule 4]")
    L.append(cw.w1("SPEED_SET", k, speeds[0], "start speed (start value S0 read first)"))
    L.append(cw.w6(cw.hold_vec({k: "H%d+%d" % (k, a.blip)}), "blip out"))
    L.append(cw.w6(cw.hold_vec(), "blip back"))
    if stage == "s":
        L.append(cw.w6(cw.hold_vec({k: lo}), "to %d" % lo))
        for sp in speeds:
            if sp != speeds[0]:
                L.append(cw.w1("SPEED_SET", k, sp, ""))
            for r in range(a.reps):
                L.append(cw.w6(cw.hold_vec({k: hi}), "speed %d step up %d" % (sp, r + 1)))
                L.append(cw.w6(cw.hold_vec({k: lo}), "speed %d step down %d" % (sp, r + 1)))
        if speeds[-1] != speeds[0]:
            L.append(cw.w1("SPEED_SET", k, speeds[0], ""))
    else:
        L += [cw.w6(cw.hold_vec({k: lo}), "sweep start (dwell 2 s)"), cw.w6(cw.hold_vec({k: hi}), "sweep up, slow (dwell 2 s)"),
              cw.w6(cw.hold_vec({k: lo}), "sweep back (dwell 2 s)")]
    L.append(cw.w6(cw.hold_vec(), "back to the hold value (start pose)"))
    L.append(cw.w1("SPEED_SET", k, "S0", "start speed back (only if it differs)"))
    return L


def plan_thumb(a):
    """stage t: thumb bend (4) one step toward open and back; thumb rotation (5) one step toward the palm and back with fingers 0..3
    opened first and returned after. Speed 100, no blip (the step is the first thumb move)."""
    other = "left" if a.hand == "right" else "right"
    k, st, fo = a.axis, a.thumb_step, a.fingers_open
    L = ["[battery read; idle %s hand watched read only at full rate, log only]" % other] + cw.hold_plan_lines(a.hand)
    L.append("[block within 30 s of the hold; 135 Hz log; stop list; rule 4 on the other thumb; fingers 0..3 within 5 counts]")
    base = {}
    if k == 5:
        L += [cw.w1("SPEED_SET", f, 100, "only if not already") for f in range(4)]
        L.append(cw.w6([fo] * 4 + ["H4", "H5"], "fingers 0..3 open to %d before the thumb turns (wait until still)" % fo))
        base = {f: fo for f in range(4)}
    L.append(cw.w1("SPEED_SET", k, 100, "only if not already"))
    tgt = "H4+%d" % st if k == 4 else "H5-%d" % st
    L.append(cw.w6(cw.hold_vec({**base, k: tgt}), "%s one step %s, dwell %.1f s" % (cw.AXIS[k], "toward open" if k == 4 else "toward the palm (+%d would pass 1000)" % st, a.pause)))
    L.append(cw.w6(cw.hold_vec(dict(base)), "thumb back to its hold value, dwell %.1f s" % a.pause))
    if k == 5:
        L.append(cw.w6(cw.hold_vec(), "fingers 0..3 back to their hold values"))
        L += [cw.w1("SPEED_SET", f, "S0_%d" % f, "start speed back (only if it differs)") for f in range(4)]
    L.append(cw.w1("SPEED_SET", k, "S0_%d" % k, "start speed back (only if it differs)"))
    return L


def live_block(a, run, stage, env):
    cw.refuse_if_locked("live_writes", True)
    soc, summ = cw.battery()
    print("BATTERY", soc, "%", summ)
    st = cw.check_before_write(a.hand, run, a.approve_hold_clear_7)
    other = "left" if a.hand == "right" else "right"
    w = cw.Watch(other, run)
    try:
        hs = cw.hold(a.hand, run, st)
        rc, out = cw.run_tool([cw.PY, "-B", os.path.join(cw.LIB, "g1rs_block.py"), a.hand, str(a.axis), stage, hs, run.path("block")]
                              + cw.hand_net(a.hand) + (["--no-body"] if cw.MOCK else []), env)
        cw.run_tool([cw.PY, "-B", os.path.join(cw.LIB, "g1rs_block_analyze.py"), run.path("block")], log_name=run.path("block", "analysis.txt"))
    finally:
        w.stop()
    soc2, _ = cw.battery()
    print("BATTERY end", soc2, "%")
    return "DONE" if "RESULT DONE" in out else "ABORTED"
