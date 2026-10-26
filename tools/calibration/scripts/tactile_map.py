#!/usr/bin/env python3
"""tactile_map: records all 17 touch arrays of a hand, read only. Default: baseline for --baseline-s (nobody touches). --cue: one
cue and time mark per pad (prints the pad name, logs a CUE event, reads --dwell s), for a person pressing each pad in turn.
   python scripts/tactile_map.py [--hands right,left] [--baseline-s 60] [--cue --dwell 5 --pads all] [--live]"""
import os
import subprocess
import cw

p = cw.parser(__doc__)
p.add_argument("--hands", default="right,left")
p.add_argument("--baseline-s", type=float, default=60.0)
p.add_argument("--cue", action="store_true")
p.add_argument("--dwell", type=float, default=5.0)
p.add_argument("--pads", default="all")
p.add_argument("--paced", action="store_true", help="with --cue: wait for the file NEXT in the run folder before each pad (STOP ends)")
a = p.parse_args()
hands = a.hands.split(",")
run = cw.Run("tactile_map", a)
from h0_ro import TOUCH  # noqa: E402
pads = list(TOUCH) if a.pads == "all" else a.pads.split(",")
if a.cue:
    L = ["[read only, hands one after the other: 2 s baseline, then for each of %d pads: cue + time mark, %.0f s of all arrays%s]"
         % (len(pads), a.dwell, "; each pad waits for the file NEXT in the run folder (STOP ends)" if a.paced else "")]
else:
    L = ["[read only: all 17 arrays of %s for %.0f s (about 35 Hz), nobody touches]" % (",".join(hands), a.baseline_s)]
cw.print_plan("tactile_map", L)
if not a.live:
    run.finish("DRY_RUN")
else:
    soc, _ = cw.battery()
    print("BATTERY", soc, "%")
    if a.cue:
        for h in hands:
            subprocess.run([cw.PY, "-B", os.path.join(cw.LIB, "cw_tactile_cue.py"), h, run.dir, str(a.dwell), ",".join(pads)],
                           env=dict(os.environ, CW_CUE_PACED="1" if a.paced else "0"))
    else:
        ps = [subprocess.Popen([cw.PY, "-B", os.path.join(cw.LIB, "g1rs_tactile.py"), h, str(a.baseline_s), run.dir]) for h in hands]
        for x in ps:
            x.wait()
    run.finish("DONE")
