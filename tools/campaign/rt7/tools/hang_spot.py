#!/usr/bin/env python3
"""RT7 hand prep, CPU only: where to hang two light objects (strings from the gantry bar) in front of
the chest. For each object spot (ahead of the chest plate, below the shoulder joints, out to
each side): IK reach with the operator mapping's hand orientations, shoulder pitch gravity
torque at the grasp, object in the head camera view, and the highest and widest any arm or
hand mesh gets at the grasp and along the exit lowering (what the gantry must clear).

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



def extent(qs):
    """highest z and widest |y| of any arm / hand mesh vertex over the poses"""
    zmax, ymax = -9.0, 0.0
    for qa in qs:
        place(qa)
        for gi in arm_geoms:
            g = geom.geometryObjects[gi]; M = gdata.oMg[gi]
            try:
                v = (M.rotation @ np.asarray(g.geometry.vertices()).T).T + M.translation
            except Exception:
                v = M.translation[None, :]
            zmax = max(zmax, float(v[:, 2].max())); ymax = max(ymax, float(np.abs(v[:, 1]).max()))
    return zmax, ymax


print(f"chest plate x {front:.3f} m, shoulder joints z {sh_z:.3f} m (pelvis frame)", flush=True)
rows = []
for ahead in (0.15, 0.20, 0.25, 0.30):          # object ahead of the chest plate (m)
    for below in (0.15, 0.20, 0.25, 0.30, 0.35):   # object centre below the shoulder joints (m)
        for out in (0.18, 0.22):                    # object centre out from the midline (m)
            x_o, z_o = front + ahead, sh_z - below
            best = None
            for name, rot in R_rot.items():
                pl = np.array([x_o - 0.05, out, z_o]); pr = np.array([x_o - 0.05, -out, z_o])
                q, tau, miss = solve(pl, pr, rot, q_rest)
                if miss > 0.02:
                    continue
                t = max(abs(tau[0]), abs(tau[7]))
                if best is None or t < best[1]:
                    best = (name, t, q)
            view = in_view([x_o, out, z_o]) and in_view([x_o, -out, z_o]) and in_view([x_o, out, z_o + 0.05]) \
                and in_view([x_o, out, z_o - 0.05])
            if best is None:
                print(f"ahead {100 * ahead:.0f} cm, below {100 * below:.0f} cm, out {100 * out:.0f} cm: no orientation reaches", flush=True)
                continue
            path, T = lowering_path(best[2], q_rest)
            zg, yg = extent([best[2]])
            zl, yl = extent(path[::4])
            r = dict(ahead_cm=round(100 * ahead), below_cm=round(100 * below), out_cm=round(100 * out), orientation=best[0],
                     tau_Nm=round(best[1], 2), in_view=view, top_above_shoulder_cm=round(100 * (max(zg, zl) - sh_z), 1),
                     widest_from_mid_cm=round(100 * max(yg, yl), 1), lowering_s=round(T, 1))
            rows.append(r)
            print(r, flush=True)
ok = [r for r in rows if r["in_view"]]
ok.sort(key=lambda r: r["tau_Nm"])
print("\nin view, least shoulder torque first:")
for r in ok[:8]:
    print(r)
out_csv = sys.argv[1]
with open(out_csv, "w", newline="") as fh:
    w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
sys.stdout.flush()
os._exit(0)
