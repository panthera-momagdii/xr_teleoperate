#!/usr/bin/env python3
"""RT7 hand prep, CPU only: a TABLE REST pose instead of low.yaml for the hand run.

See table_spot2.py (same model, meshes, lowering plan). Here the start and rest pose is a
pose with the forearms forward and the hands GAP above the table top; the question is whether
(1) the start approach from low.yaml, (2) the lowering from a grasp on the table back to the
table rest pose, stay clear of the table, and (3) the objects are in view.

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



def path_between(q0, q1, n=80):
    return lowering_path(q0, q1, n)[0]


print(f"chest plate x {front:.3f} m, shoulder z {sh_z:.3f} m", flush=True)
low = q_rest.copy()
rows = []
for p in (0.0, -0.2, -0.4):
    for e in (-0.3, 0.0, 0.3):
        qtr = low.copy()
        qtr[0] = qtr[7] = p
        qtr[3] = qtr[10] = e
        qtr[[4, 5, 6, 11, 12, 13]] = 0.0
        z_low = lowest_hand_z(qtr)
        z_t = z_low - GAP
        pin.framesForwardKinematics(ik.reduced_robot.model, ik.reduced_robot.data, qtr)
        hx = float(ik.reduced_robot.data.oMf[ik.L_hand_id].translation[0])
        hz = float(ik.reduced_robot.data.oMf[ik.L_hand_id].translation[2])
        g = np.asarray(ik.reduced_robot.model.gravity.linear)
        tau_rest = pin.rnea(ik.reduced_robot.model, ik.reduced_robot.data, qtr, np.zeros(14), np.zeros(14))
        for x_e in np.arange(0.10, hx - 0.03, 0.05):
            d_start, k_s, l_s = table_clearance(path_between(low, qtr)[1:], x_e, z_t)
            d_rest = table_clearance([qtr], x_e, z_t)[0]
            res = []
            for dx in (0.06, 0.15):
                for name, rot in R_rot.items():
                    h = 0.10
                    for _ in range(2):
                        pl = np.array([x_e + dx, 0.20, z_t + h]); pr = np.array([x_e + dx, -0.20, z_t + h])
                        q, tau, miss = solve(pl, pr, rot, qtr)
                        h += (z_t + GAP) - lowest_hand_z(q)
                    pl = np.array([x_e + dx, 0.20, z_t + h]); pr = np.array([x_e + dx, -0.20, z_t + h])
                    q, tau, miss = solve(pl, pr, rot, qtr)
                    if miss > 0.03:
                        continue
                    d0 = table_clearance([q], x_e, z_t)[0]
                    d, k, link = table_clearance(path_between(q, qtr)[1:], x_e, z_t)
                    view = in_view([x_e + dx + 0.04, 0.20, z_t + 0.04]) and in_view([x_e + dx + 0.04, -0.20, z_t + 0.04])
                    res.append((dx, name, d0, d, view, max(abs(tau[0]), abs(tau[7]))))
            ok_g = [r for r in res if r[4] and r[3] >= min(r[2], MARGIN) - 0.005 and r[3] >= 0.02]
            dx_cov = sorted(set(r[0] for r in ok_g))
            ok = d_start >= 0.02 and len(dx_cov) == 2
            line = (f"rest pitch {p:+.1f} elbow {e:+.1f}: hand frame x {hx:.2f} z {hz:+.2f}, table top {z_t:+.2f} "
                    f"({sh_z - z_t:.2f} below shoulder), edge {x_e:.2f} ({x_e - front:+.2f} past chest): "
                    f"{'OK' if ok else '--'} start approach min {100 * d_start:.1f} cm ({l_s}), "
                    f"grasps ok {len(ok_g)}/{len(res)} (dx {dx_cov}), grasp-to-rest min "
                    f"{100 * min((r[3] for r in res), default=9):.1f} cm, shoulder tau at rest {max(abs(tau_rest[0]), abs(tau_rest[7])):.1f} Nm, "
                    f"at grasps up to {max((r[5] for r in res), default=0):.1f} Nm")
            print(line, flush=True)
            rows.append(dict(rest_pitch=p, rest_elbow=e, table_top_z=round(z_t, 3), top_below_shoulder=round(sh_z - z_t, 3),
                             near_edge_x=round(x_e, 3), edge_ahead_of_chest=round(x_e - front, 3), ok=ok,
                             start_min_cm=round(100 * d_start, 1), grasps=len(res), grasps_ok=len(ok_g),
                             rest_tau_Nm=round(max(abs(tau_rest[0]), abs(tau_rest[7])), 1)))
out = sys.argv[1]
with open(out, "w", newline="") as fh:
    w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
sys.stdout.flush()
os._exit(0)
