#!/usr/bin/env python3
"""wake_test: single-register ANGLE_SET writes (proven g1rs_wake tool) and which axes wake. Fresh hold, blip, SPEED_SET 500 on the
non-thumb fingers not at 500, 10 s wait, then the tests: resend tests write the stored value 3 times (thumbs resend only), moving
tests write setup 650, 950/650 x3, return to the hold value. WAKE_TABLE.md per test.
   python scripts/wake_test.py --hand right [--tests R0,R1,R2,R3] [--live]"""
import os
import cw

TESTS = {"R0": ("right", 3, "resend"), "R1": ("right", 0, "move"), "R2": ("right", 4, "resend"), "R3": ("right", 5, "resend"),
         "L0": ("left", 3, "resend"), "L1": ("left", 0, "move"), "L2": ("left", 1, "move"), "L3": ("left", 2, "move"),
         "L4": ("left", 3, "move"), "L5": ("left", 4, "resend"), "L6": ("left", 5, "resend")}
p = cw.parser(__doc__)
p.add_argument("--hand", required=True, choices=cw.HANDS)
p.add_argument("--tests", default=None, help="default: all tests of that hand")
p.add_argument("--approve-hold-clear-7", action="store_true")
a = p.parse_args()
tests = (a.tests or ",".join(t for t, v in TESTS.items() if v[0] == a.hand)).split(",")
if any(TESTS[t][0] != a.hand for t in tests):
    raise SystemExit("a test of the other hand was given")
run = cw.Run("wake_test", a)
blip_axis = next((TESTS[t][1] for t in tests if TESTS[t][2] == "move"), 3)
other = "left" if a.hand == "right" else "right"
L = ["[battery read; idle %s hand watched read only, log only]" % other] + cw.hold_plan_lines(a.hand)
L += [cw.w1("SPEED_SET", blip_axis, 100, "blip speed (block tool, start speed restored after)"),
      cw.w6(cw.hold_vec({blip_axis: "H%d+10" % blip_axis}), "blip out"), cw.w6(cw.hold_vec(), "blip back"),
      cw.w1("SPEED_SET", blip_axis, "S0", "start speed back"),
      "[wake tool, within 30 s of the hold] " + cw.w1("SPEED_SET", 0, 500, "only for each finger 0..3 not already at 500, one write each"),
      "[wait 10 s; before every wake write all six axes settled; after it log both hands >= 3 s and until STATUS 2 on all six]"]
for t in tests:
    _, ax, kind = TESTS[t]
    if kind == "resend":
        L += [cw.w1("ANGLE_SET", ax, "T%d" % ax, "%s resend %d: stored value (3 agreeing reads, must equal the hold value)" % (t, i + 1)) for i in range(3)]
    else:
        for i, (k2, v) in enumerate([("setup", 650), ("moving", 950), ("moving", 650), ("moving", 950), ("moving", 650), ("moving", 950), ("moving", 650), ("return", "H%d" % ax)]):
            L.append(cw.w1("ANGLE_SET", ax, v, "%s write %d %s" % (t, i + 1, k2)))
L.append(cw.w1("SPEED_SET", 0, "S0_k", "after the last test: each finger 0..3 whose speed was changed back to its start speed, one write each"))
cw.print_plan("wake_test %s %s" % (a.hand, ",".join(tests)), L)
if not a.live:
    run.finish("DRY_RUN")
else:
    cw.refuse_if_locked("live_writes", True)
    soc, _ = cw.battery()
    print("BATTERY", soc, "%")
    st = cw.check_before_write(a.hand, run, a.approve_hold_clear_7)
    w = cw.Watch(other, run)
    try:
        hs = cw.hold(a.hand, run, st)
        rc, out = cw.run_tool([cw.PY, "-B", os.path.join(cw.LIB, "g1rs_block.py"), a.hand, str(blip_axis), "b", hs, run.path("blip")] + cw.hand_net(a.hand) + (["--no-body"] if cw.MOCK else []), {"CW_SWEEP_SPEED": "100"})
        if "RESULT DONE" not in out:
            raise SystemExit("STOP: blip block not DONE")
        rc, out = cw.run_tool([cw.PY, "-B", os.path.join(cw.LIB, "g1rs_wake.py"), a.hand, ",".join(tests), hs, run.path("wake"), "--no-other"] + cw.hand_net(a.hand) + (["--no-body"] if cw.MOCK else []))
        cw.run_tool([cw.PY, "-B", os.path.join(cw.LIB, "g1rs_partB_table.py"), run.dir], log_name=run.path("partB_table.md"))
    finally:
        w.stop()
    run.finish("DONE" if "RESULT DONE" in out else "ABORTED")
