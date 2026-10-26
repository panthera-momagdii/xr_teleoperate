#!/usr/bin/env python3
"""headcam: saves head camera (RealSense D435i) frames with host time stamps (monotonic and wall) for --seconds, every --every-th
frameset, color 1280x720@30 RGB8 + depth 848x480@30 Z16, and stream_info.json with the resolution, fps, format and intrinsics of the
streams used. No device option is set, nothing is published, no serial number is written. Fails if another process holds the camera.
   python scripts/headcam.py [--seconds 10] [--every 3] [--live]"""
import json
import os
import subprocess
import cw

p = cw.parser(__doc__)
p.add_argument("--seconds", type=float, default=10.0)
p.add_argument("--every", type=int, default=3)
a = p.parse_args()
run = cw.Run("headcam", a)
n = int(a.seconds * 30 / a.every)
cw.print_plan("headcam", ["[read only camera: bin/headcam, 30 warm-up framesets, then about %d framesets saved (%.0f s, every %d), no hand, no DDS]" % (n, a.seconds, a.every)])
if not a.live:
    run.finish("DRY_RUN")
else:
    os.makedirs(run.path("frames"))
    rc, out = cw.run_tool([os.path.join(cw.ROOT, "scripts", "bin", "headcam"), run.path("frames"), str(a.seconds), str(a.every)])
    try:
        si = json.load(open(run.path("frames", "stream_info.json")))
        for k in ("color", "depth"):
            print("%-5s %dx%d @%d %s fx %.2f fy %.2f ppx %.2f ppy %.2f" % (k, si[k]["width"], si[k]["height"], si[k]["fps"], si[k]["format"],
                  si[k]["intrinsics"]["fx"], si[k]["intrinsics"]["fy"], si[k]["intrinsics"]["ppx"], si[k]["intrinsics"]["ppy"]))
    except Exception as e:  # noqa: BLE001
        print("stream_info not readable:", repr(e))
    run.finish("DONE" if rc == 0 else "FAILED")
