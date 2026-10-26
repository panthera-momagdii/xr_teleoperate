#!/usr/bin/env python3
"""RT6h item 5: mapping candidates (XR_WRIST_XY_SCALE, XR_WRIST_X_OFFSET). CPU only.

Uses rt6d/tools/reach_table.py's chain (televuer transform -> WristMapping -> G1_29_ArmIK,
now with the 0.1 rad joint margin) and its URDF collision clearance. For each mapping:
torso clearance in the chest pose (the smaller of palm down / thumbs up), shoulder pitch
gravity torque in the belly and reach poses (the larger of the two orientations).
Keeps mappings with at least 3 cm torso clearance at the chest, ranked by torque.
"""
import csv, os, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent / "rt6d" / "tools"))
import reach_table as rt                     # noqa: E402  (chdirs into ~/xrt/teleop)
import numpy as np                           # noqa: E402
from teleop.robot_control.wrist_offset import WristMapping            # noqa: E402
from teleop.robot_control.start_pose import ARM_JOINT_NAMES           # noqa: E402

POSES = {"chest": (0.10, 0.12, -0.40), "belly": (0.30, 0.18, -0.55), "reach": (0.45, 0.18, -0.35)}
HANG = (-0.08, 0.22, -0.80)
ch = rt.Chain(); clear = rt.Clearance(ARM_JOINT_NAMES)
rows = []
SCALES = [float(v) for v in os.environ.get("SCALES", "0.6 0.7 0.8 0.9 1.0").split()]
OFFSETS = [float(v) for v in os.environ.get("OFFSETS", "-0.15 -0.10 -0.05 0.0 0.05").split()]
for sc in SCALES:
    for xo in OFFSETS:
        wm = WristMapping(x_offset=xo, xy_scale=sc)
        res = {}
        for rname, roll in rt.ROLLS.items():
            ch.q = np.zeros(ch.model.nq)
            ch.settle(*wm.apply_pair(rt.to_waist(rt.xr_wrist(*HANG, +1, roll), +1), rt.to_waist(rt.xr_wrist(*HANG, -1, roll), -1)), n=40)
            for pn in ("chest", "belly", "reach"):
                f, o, u = POSES[pn]
                lt, rr = wm.apply_pair(rt.to_waist(rt.xr_wrist(f, o, u, +1, roll), +1), rt.to_waist(rt.xr_wrist(f, o, u, -1, roll), -1))
                q, tau, miss, hands, sh = ch.settle(lt, rr, n=30)
                cl = clear(q)
                res[(rname, pn)] = dict(tau=max(abs(tau[0]), abs(tau[7])), torso=min(cl["left"], cl["right"]),
                                        hands=cl["hands"], miss=max(miss))
        r = dict(xy_scale=sc, x_offset=xo,
                 chest_torso_clear_cm=round(100 * min(res[(k, "chest")]["torso"] for k in rt.ROLLS), 1),
                 chest_hands_clear_cm=round(100 * min(res[(k, "chest")]["hands"] for k in rt.ROLLS), 1),
                 chest_tau_Nm=round(max(res[(k, "chest")]["tau"] for k in rt.ROLLS), 2),
                 belly_tau_Nm=round(max(res[(k, "belly")]["tau"] for k in rt.ROLLS), 2),
                 reach_tau_Nm=round(max(res[(k, "reach")]["tau"] for k in rt.ROLLS), 2),
                 reach_miss_cm=round(100 * max(res[(k, "reach")]["miss"] for k in rt.ROLLS), 1))
        rows.append(r)
        print(r, flush=True)
out = Path(__file__).resolve().parent.parent / "out"
with open(out / os.environ.get("OUT_CSV", "mapping_search.csv"), "w", newline="") as fh:
    w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
ok = [r for r in rows if r["chest_torso_clear_cm"] >= 3.0]
ok.sort(key=lambda r: r["belly_tau_Nm"] + r["reach_tau_Nm"])
print("\ncandidates with >= 3 cm torso clearance at the chest, least belly + reach torque first:")
for r in ok[:8]:
    print(r)
sys.stdout.flush()
os._exit(0)
