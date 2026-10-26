#!/usr/bin/env python3
"""RT7 hand prep, CPU only: a FOLD-BACK lowering for the hand run, checked against a table.

Waypoints from the grasp q0: A elbows folded (FOLD rad), wrists 0; B shoulders back (pitch BACK,
roll/yaw rest), elbows folded; C elbows extended to rest, shoulders still back; D low.yaml.
Each segment is the launcher's min-jerk group plan. Table and torso clearance along the
whole path (hppfcl, URDF meshes).

Original notes:

For a table top at height Z (pelvis frame; the shoulder pitch joints are at +0.292 m) with
its near edge X in front of the pelvis origin, the robot grasps an object on it with each
hand (IK, 1:1 mapping, XR_WRIST_X_OFFSET +0.01 is an operator-side shift, so targets are
set directly in the robot frame). Then the automatic exit lowering runs: the launcher's
min-jerk joint plan to poses/low.yaml, shoulders/elbows and wrists on their own clocks
(peaks 0.18 / 0.15 rad/s; the 0.2 rad/s command limit does not bind). Along that path,
the clearance between every arm and hand collision mesh and the table (a solid block from
its top down to the floor) is computed with hppfcl. The URDF carries Dex3 hands; the
robot's Inspire hands are larger, so a spot needs a margin (default 0.06 m).

Also: the object must be inside the head camera view (d435_link, 47.6 deg down, about
54 x 42 deg at 640x480).

Usage: table_spot.py [OUT_CSV]
"""
import csv, os, sys
from pathlib import Path
XRT = Path.home() / "xrt"
sys.path[:0] = [str(XRT), str(XRT / "teleop" / "televuer" / "src")]
os.chdir(XRT / "teleop")
os.environ.pop("XR_IK_JOINT_MARGIN", None)
import numpy as np                                                    # noqa: E402
import pinocchio as pin                                               # noqa: E402
import hppfcl                                                         # noqa: E402
from teleop.robot_control.robot_arm_ik import G1_29_ArmIK             # noqa: E402
from teleop.robot_control.start_pose import load_pose                 # noqa: E402
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent / "rt6d" / "tools"))
import reach_table as rt                                              # noqa: E402

MARGIN = float(os.environ.get("MARGIN", "0.06"))
GAP = float(os.environ.get("GAP", "0.03"))
ARM_NAMES = [f"{s}_{j}_joint" for s in ("left", "right") for j in
             ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")]
LINKS = ("elbow_link", "wrist_roll_link", "wrist_pitch_link", "wrist_yaw_link", "hand_palm_link",
         "hand_thumb_0_link", "hand_thumb_1_link", "hand_thumb_2_link", "hand_middle_0_link",
         "hand_middle_1_link", "hand_index_0_link", "hand_index_1_link")

urdf = str(XRT / "assets" / "g1" / "g1_body29_hand14.urdf")
model, geom = pin.buildModelsFromUrdf(urdf, package_dirs=str(XRT / "assets" / "g1"),
                                      geometry_types=pin.GeometryType.COLLISION)[:2]
data = model.createData()
idx_q = [model.joints[model.getJointId(n)].idx_q for n in ARM_NAMES]
arm_geoms = [i for i, g in enumerate(geom.geometryObjects)
             if any(model.frames[g.parentFrame].name == f"{s}_{l}" for s in ("left", "right") for l in LINKS)]
torso_g = [g for g in geom.geometryObjects if model.frames[g.parentFrame].name == "torso_link"]
gdata = pin.GeometryData(geom)


def full_q(q_arm):
    q = pin.neutral(model)
    for k, i in enumerate(idx_q):
        q[i] = q_arm[k]
    return q


def place(q_arm):
    pin.forwardKinematics(model, data, full_q(q_arm))
    pin.updateGeometryPlacements(model, data, geom, gdata)


# chest plate: the torso mesh's most forward vertex at q = neutral
place(np.zeros(14))
front = -9.0
for g in torso_g:
    gi = geom.getGeometryId(g.name)
    M = gdata.oMg[gi]
    try:
        v = np.asarray(g.geometry.vertices())
        front = max(front, float(np.max((M.rotation @ v.T).T[:, 0] + M.translation[0])))
    except Exception:
        pass
pin.framesForwardKinematics(model, data, full_q(np.zeros(14)))
cam = data.oMf[model.getFrameId("d435_link")]
sh_z = float(data.oMi[model.getJointId("left_shoulder_pitch_joint")].translation[2])
VH, HH = np.radians(21.0), np.radians(27.0)


def in_view(p):
    v = cam.rotation.T @ (np.asarray(p) - cam.translation)        # camera link frame: x forward
    if v[0] <= 0:
        return False
    return abs(np.arctan2(v[2], v[0])) <= VH and abs(np.arctan2(v[1], v[0])) <= HH


def table_clearance(q_path, x_e, z_t):
    """Smallest distance from any arm / hand mesh to the table block along the path."""
    depth, width, height = 0.6, 1.2, 1.5
    box = hppfcl.Box(depth, width, height)
    tf = hppfcl.Transform3f(np.eye(3), np.array([x_e + depth / 2, 0.0, z_t - height / 2]))
    req, worst = hppfcl.DistanceRequest(), (9.0, None, None)
    for k, qa in enumerate(q_path):
        place(qa)
        for gi in arm_geoms:
            M = gdata.oMg[gi]
            res = hppfcl.DistanceResult()
            d = hppfcl.distance(geom.geometryObjects[gi].geometry, hppfcl.Transform3f(M.rotation, M.translation), box, tf, req, res)
            if d < worst[0]:
                worst = (d, k, model.frames[geom.geometryObjects[gi].parentFrame].name)
    return worst


def lowering_path(q0, q_rest, n=80):
    arm = [0, 1, 2, 3, 7, 8, 9, 10]; wr = [4, 5, 6, 11, 12, 13]
    d = q_rest - q0
    Ta = 1.875 * np.max(np.abs(d[arm])) / 0.18; Tw = 1.875 * np.max(np.abs(d[wr])) / 0.15
    T = max(Ta, Tw, 1e-3)
    mj = lambda x: x ** 3 * (10 - 15 * x + 6 * x * x)
    out = []
    for t in np.linspace(0, T, n):
        q = q0.copy()
        q[arm] += d[arm] * mj(min(1.0, t / Ta)) if Ta > 0 else d[arm]
        q[wr] += d[wr] * mj(min(1.0, t / Tw)) if Tw > 0 else d[wr]
        out.append(q)
    return out, T


ik = G1_29_ArmIK()
q_rest = load_pose(str(XRT / "poses" / "low.yaml"))
R_rot = {}
for name, roll in rt.ROLLS.items():                               # hand orientations from the operator mapping
    R_rot[name] = (rt.to_waist(rt.xr_wrist(0.3, 0.2, -0.45, +1, roll), +1)[:3, :3],
                   rt.to_waist(rt.xr_wrist(0.3, 0.2, -0.45, -1, roll), -1)[:3, :3])
pin.framesForwardKinematics(ik.reduced_robot.model, ik.reduced_robot.data, q_rest)
R_rot["hanging hand"] = (ik.reduced_robot.data.oMf[ik.L_hand_id].rotation.copy(),
                         ik.reduced_robot.data.oMf[ik.R_hand_id].rotation.copy())
HAND = ("wrist_yaw_link", "hand_palm_link", "hand_thumb_0_link", "hand_thumb_1_link", "hand_thumb_2_link",
        "hand_middle_0_link", "hand_middle_1_link", "hand_index_0_link", "hand_index_1_link")
hand_geoms = [i for i, g in enumerate(geom.geometryObjects)
              if any(model.frames[g.parentFrame].name == f"{s}_{l}" for s in ("left", "right") for l in HAND)]


def lowest_hand_z(q_arm):
    place(q_arm)
    z = 9.0
    for gi in hand_geoms:
        g = geom.geometryObjects[gi]; M = gdata.oMg[gi]
        try:
            v = np.asarray(g.geometry.vertices())
            z = min(z, float(np.min((M.rotation @ v.T).T[:, 2] + M.translation[2])))
        except Exception:
            z = min(z, float(M.translation[2]))
    return z


def solve(pl, pr, rot, q_start):
    L = np.eye(4); L[:3, :3] = rot[0]; L[:3, 3] = pl
    R = np.eye(4); R[:3, :3] = rot[1]; R[:3, 3] = pr
    ik.smooth_filter._data_queue = []; ik._warm = None
    q = q_start.copy()
    for _ in range(40):
        q, tau = ik.solve_ik(L, R, q.copy(), np.zeros(14)); q = np.asarray(q, dtype=float)
    pin.framesForwardKinematics(ik.reduced_robot.model, ik.reduced_robot.data, q)
    miss = max(np.linalg.norm(ik.reduced_robot.data.oMf[ik.L_hand_id].translation - pl),
               np.linalg.norm(ik.reduced_robot.data.oMf[ik.R_hand_id].translation - pr))
    return q, np.asarray(tau), miss



FOLD = float(os.environ.get("FOLD", "-0.8")); BACK = float(os.environ.get("BACK", "0.6"))
tc = rt.Clearance(ARM_NAMES)


def fold_path(q0):
    A = q0.copy(); A[[3, 10]] = FOLD; A[[4, 5, 6, 11, 12, 13]] = 0.0
    B = A.copy(); B[[0, 7]] = BACK; B[[1, 2, 8, 9]] = q_rest[[1, 2, 8, 9]]
    Cq = B.copy(); Cq[[3, 10]] = q_rest[[3, 10]]
    D = q_rest.copy()
    path, T = [], 0.0
    for a, b in ((q0, A), (A, B), (B, Cq), (Cq, D)):
        p, t = lowering_path(a, b, 60)
        path += p; T += t
    return path, T


print(f"FOLD {FOLD} BACK {BACK}; chest x {front:.3f}, shoulder z {sh_z:.3f}", flush=True)
for z_t in (-0.05, 0.0, 0.05):
    for x_e in (0.20, 0.25, 0.30):
        res = []
        for dx in (0.06, 0.15):
            for name, rot in R_rot.items():
                h = 0.10
                q = q_rest
                for _ in range(2):
                    pl = np.array([x_e + dx, 0.20, z_t + h]); pr = np.array([x_e + dx, -0.20, z_t + h])
                    q, tau, miss = solve(pl, pr, rot, q_rest)
                    h += (z_t + GAP) - lowest_hand_z(q)
                pl = np.array([x_e + dx, 0.20, z_t + h]); pr = np.array([x_e + dx, -0.20, z_t + h])
                q, tau, miss = solve(pl, pr, rot, q_rest)
                if miss > 0.03:
                    continue
                path, T = fold_path(q)
                d0 = table_clearance([q], x_e, z_t)[0]
                d, k, link = table_clearance(path[1:], x_e, z_t)
                torso = min(min(tc(qq)["left"], tc(qq)["right"]) for qq in path[::4])
                view = in_view([x_e + dx + 0.04, 0.20, z_t + 0.04])
                res.append((dx, name, d0, d, link, torso, view, T))
        ok = [r for r in res if r[3] >= min(r[2], MARGIN) - 0.005 and r[3] >= 0.02 and r[5] >= 0.01 and r[6]]
        print(f"top {z_t:+.2f} ({sh_z - z_t:.2f} below shoulder), edge {x_e:.2f} ({x_e - front:+.2f} past chest): "
              f"{len(ok)}/{len(res)} grasps clear; table min {100 * min((r[3] for r in res), default=9):.1f} cm, "
              f"torso min {100 * min((r[5] for r in res), default=9):.1f} cm, lowering {max((r[7] for r in res), default=0):.0f} s; "
              + "; ".join(f"{r[1]} +{r[0]:.2f}: {100 * r[3]:.1f} cm ({r[4]})" for r in res if r not in ok)[:200], flush=True)
sys.stdout.flush()
os._exit(0)
