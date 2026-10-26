#!/usr/bin/env python3
"""scale_test (Mohammed, 18 Oct 2026): the whole kitchen-scale test on the LEFT hand in one command. Dry-run (default) prints the
timeline and touches nothing; --live runs it (locks live_writes and scale_test must be open).
Pre-flight (no motion): STOP file absent, locks open, battery >= 30 %, every temperature on both hands < 45 C, STATUS / ERROR read.
Then a fresh hold on the LEFT hand (clears a STATUS 7: approved in the brief), STATUS 2 and ERROR 0 on all six, and the presses start
within 10 s of the hold. Presses (lib/cw_scale.py): index then middle; open to 950; contact search (limit rest + 200 g, toward 650;
no contact above 650 = finger skipped); targets contact - 10 ... - 60, 8 s each, open to 950 between; safety limit rest + 700 g;
scale kept awake (a change on the scale at least every 15 s; taps when needed and at the switch); 48 C pauses, 52 C stops, STATUS 7
stops, STOP file checked on every read and before every write; Ctrl-C or STOP: open to 950 and stop writing. Every sample is logged
with clock time (block/state.jsonl), press start / end marked. Ends open at 950 and still; the run is packed with sha256.
--no-contact-check: same pre-flight and hold, then index and middle move inside 650..950 only (before the scale goes in).
   python scripts/scale_test.py [--no-contact-check] [--live]"""
import json
import os
import signal
import sys
import time
import cw
import locked_common as lc

p = cw.parser(__doc__)
p.add_argument("--hand", default="left", choices=["left"])
p.add_argument("--no-contact-check", action="store_true")
p.add_argument("--soft-stop", action="store_true", help="new mode (18 Oct): force limit at the factory default during presses, software stop at rest + 700 opens that finger and goes to the next; steps 20,60,100; start at battery >= 27 %%, stop under 25 %%")
p.add_argument("--battery-min", type=int, default=None)
p.add_argument("--rest-index", type=int, default=1066)
p.add_argument("--rest-middle", type=int, default=55)
p.add_argument("--steps", default="10,20,30,40,50,60")
p.add_argument("--hold-s", type=float, default=8.0)
p.add_argument("--safety-g", type=int, default=700)
p.add_argument("--temp-max", type=int, default=45, help="pre-flight: every temperature must be below this (C)")
p.add_argument("--contact-g", type=int, default=200)
a = p.parse_args()
if a.soft_stop and a.steps == "10,20,30,40,50,60":
    a.steps = "20,60,100"
if a.battery_min is None:
    a.battery_min = 27 if a.soft_stop else 30
steps = [int(x) for x in a.steps.split(",")]
name = "scale_test_nocontact" if a.no_contact_check else ("scale_test_soft" if a.soft_stop else "scale_test")


def timeline():
    t, L = 0.0, []
    def add(d, what):
        nonlocal t
        L.append("  t+%5.1f s  %s" % (t, what)); t += d
    add(4, "pre-flight: STOP file, locks, battery >= %d %%, temperatures < %d C, STATUS / ERROR both hands (no motion)" % (a.battery_min, a.temp_max))
    add(9, "fresh hold LEFT (own readings, thumbs stored targets); STATUS 2 / ERROR 0 check at +2 s and +3 s")
    add(2, "tool start (presses begin within 10 s of the hold)")
    for k, nm in ((3, "index"), (2, "middle")):
        if not a.no_contact_check and k == 2:
            add(2.5, "tap with the index at its contact - 5 (scale awake across the switch)")
        add(0.5, "%s: SPEED 100, open to 950" % nm)
        if a.no_contact_check:
            add(3.5, "%s: 950 -> 650, 1 s" % nm)
            add(3.5, "%s: 650 -> 950, 1 s" % nm)
            continue
        add(3.5, "%s: contact search, FORCE_SET rest + 200 g, close toward 650; save the contact angle C" % nm)
        add(1.5, "%s: open to 950; %s" % (nm, "FORCE_SET 2000 (factory); software stop at rest + 700 g" if a.soft_stop else "FORCE_SET rest + 700 g (safety)"))
        for d in steps:
            add(1.2 + a.hold_s + 1.2, "%s: press to C-%d, hold %.0f s, open to 950   (scale: on, then off)" % (nm, d, a.hold_s))
    add(1.5, "FORCE_SET and speeds back; hand open at 950 and still (1 s)")
    add(0, "pack the run: tar.gz + sha256")
    return L, t


if a.live:
    lc.check_live("scale_test")
run = cw.Run(name, a)
L, total = timeline()
print("TIMELINE (%s, estimated): total about %.0f s (%.1f min)" % ("no-contact check" if a.no_contact_check else "scale test", total, total / 60))
print("\n".join(L))
print("rest force index %d g, middle %d g; contact limit rest + 200 g; safety rest + 700 g; steps %s; hold %.0f s" % (a.rest_index, a.rest_middle, steps, a.hold_s))
if not a.live:
    run.finish("DRY_RUN")
    raise SystemExit(0)

# ---- pre-flight (no motion)
def fail(msg):
    print("PRE-FLIGHT FAILED: %s; nothing written" % msg)
    run.finish("ABORTED_PREFLIGHT")
    raise SystemExit(1)

if os.path.exists(cw.STOP_FILE):
    fail("STOP file exists")
soc, _ = cw.battery()
print("BATTERY", soc, "%")
if soc is not None and soc < a.battery_min:
    fail("battery %s %% < %d %%" % (soc, a.battery_min))
for h in ("left", "right"):
    st = cw.read_state(h, run.path("preflight_%s.jsonl" % h))
    print("PRE-FLIGHT %s: STATUS %s ERROR %s TEMP %s ANGLE %s" % (h, st["status"], st["error"], st["temp"], st["angle_act"]))
    if max(st["temp"]) >= a.temp_max:
        fail("%s hand temperature %s C >= %d" % (h, max(st["temp"]), a.temp_max))
st = cw.check_before_write("left", run, approve7=True)
other = cw.Watch("right", run)
os.environ["CW_HOLD_POST_S"] = "3"
if a.soft_stop:
    os.environ.setdefault("G1RS_CLOSE_SOC", "24")                   # under 25 %: the tool stops before its next move, opens to 950
signal.signal(signal.SIGINT, signal.SIG_IGN)            # Ctrl-C goes to the tool (it opens to 950 and stops); the wrapper waits
try:
    hs = cw.hold("left", run, st)
    t_hold = time.time()
    argv = [cw.PY, "-B", os.path.join(cw.LIB, "cw_scale.py"), "left", hs, run.path("block"), "--rest", "3:%d,2:%d" % (a.rest_index, a.rest_middle),
            "--steps", a.steps, "--hold-s", str(a.hold_s), "--safety-g", str(a.safety_g), "--contact-g", str(a.contact_g)] + (["--soft"] if a.soft_stop else []) + (["--no-contact"] if a.no_contact_check else []) + cw.hand_net("left") + (["--no-body"] if cw.MOCK else [])
    rc, out = lc.stream(argv)
finally:
    other.stop()
res = "DONE" if "RESULT DONE" in out else "ABORTED"
soc2, _ = cw.battery()
print("BATTERY end", soc2, "%")
run.finish(res)
import cw_packrun  # noqa: E402
tgt_dir = os.environ.get("CW_MOCK_PACK_DIR") if cw.MOCK else "/home/unitree"
tgt = os.path.join(tgt_dir, "g1rs_%s.tar.gz" % os.path.basename(run.dir))
ok, h, n, msg = cw_packrun.pack(run.dir, tgt)
print("PACKED %s %d bytes sha256 %s (%s)" % (tgt, n, h, msg))
