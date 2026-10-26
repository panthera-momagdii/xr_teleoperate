#!/usr/bin/env python3
"""RT6d item 1: predicted shoulder pitch gravity torque for typical operator poses.

CPU only. No DDS, no robot, no headset. Runs the launcher's own chain on synthetic
operator wrist poses:

    OpenXR world matrices (head at 1.5 m, wrists head-relative)
      -> televuer's basis change and head_yaw transform (imported, not copied)
      -> WristMapping (XR_WRIST_XY_SCALE, XR_WRIST_X_OFFSET, ...)
      -> G1_29_ArmIK.solve_ik, repeated until the smoothing filter settles
      -> sol_tauff: the gravity feed-forward the launcher would command

The operator poses are assumptions (an adult of about 1.75 m, the headset origin at
eye level); they are listed in POSES with the numbers used. One check against the
robot: RT6c started with "elbows tucked, forearms level" at 1:1 and measured 3.4 to
3.9 Nm on the shoulder pitch motors.

    python reach_table.py [--out DIR]
"""
import argparse
import csv
import json
import os
import sys
from pathlib import Path

XRT = Path.home() / "xrt"
sys.path[:0] = [str(XRT), str(XRT / "teleop" / "televuer" / "src")]
os.chdir(XRT / "teleop")              # the IK loads its URDF and cache relative to here

import numpy as np                                                    # noqa: E402
from televuer.tv_wrapper import (T_ROBOT_OPENXR, T_OPENXR_ROBOT,      # noqa: E402
                                 T_TO_UNITREE_HUMANOID_LEFT_ARM,
                                 T_TO_UNITREE_HUMANOID_RIGHT_ARM,
                                 transform_IPunitree_Brobot_world_arm_to_head_then_waist)
from teleop.robot_control.wrist_offset import WristMapping            # noqa: E402
from teleop.robot_control.robot_arm_ik import G1_29_ArmIK             # noqa: E402
import pinocchio as pin                                               # noqa: E402

HEAD_Y = 1.5
# Operator wrist, head-relative, robot convention: forward, outward (away from the
# body centre line, per side), up. Metres. Human shoulder taken at (-0.10, 0.19, -0.25),
# upper arm 0.30 m, forearm 0.26 m.
POSES = [
    ("hanging", "arms relaxed at the sides", (-0.08, 0.22, -0.80)),
    ("hands by shoulders", "elbows tucked, forearms vertical", (-0.02, 0.20, -0.30)),
    ("hands at chest", "wrists just in front of the sternum", (0.10, 0.12, -0.40)),
    ("elbows tucked", "elbows at the sides, forearms level", (0.16, 0.20, -0.55)),
    ("hands forward, belly", "elbows a little forward, hands at belly height", (0.30, 0.18, -0.55)),
    ("reach, chest height", "arms reaching forward at chest height", (0.45, 0.18, -0.35)),
    ("arms straight out", "arms straight forward, level", (0.52, 0.19, -0.25)),
]
ROLLS = {"palm down": 0.0, "thumbs up": 90.0}
SP = (0, 7)        # shoulder pitch, left and right, in the 14-joint arm vector
ELB = (3, 10)


def rz(deg):
    a = np.radians(deg)
    m = np.eye(4)
    m[:2, :2] = [[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]
    return m


def xr_wrist(fwd, out, up, side, roll):
    """OpenXR world matrix of one wrist. side +1 left, -1 right. OpenXR: x right, y up,
    -z forward. Identity rotation = palm down, fingers forward (what fake_xr_client sends)."""
    m = rz(roll * side)
    m[:3, 3] = [-out * side, HEAD_Y + up, -fwd]
    return m


HEAD_XR = np.eye(4)
HEAD_XR[1, 3] = HEAD_Y


def to_waist(m_xr, side):
    head = T_ROBOT_OPENXR @ HEAD_XR @ T_OPENXR_ROBOT
    arm = T_ROBOT_OPENXR @ m_xr @ T_OPENXR_ROBOT
    arm = arm @ (T_TO_UNITREE_HUMANOID_LEFT_ARM if side > 0 else T_TO_UNITREE_HUMANOID_RIGHT_ARM)
    return transform_IPunitree_Brobot_world_arm_to_head_then_waist(arm, head, "head_yaw")


BODY = ("torso_link", "logo_link", "head_link", "pelvis", "pelvis_contour_link")
ARM_LINKS = ("elbow_link", "wrist_roll_link", "wrist_pitch_link", "wrist_yaw_link", "hand_palm_link",
             "hand_thumb_0_link", "hand_thumb_1_link", "hand_thumb_2_link", "hand_middle_0_link",
             "hand_middle_1_link", "hand_index_0_link", "hand_index_1_link")


class Clearance:
    """Minimum distance between the forearm/hand collision meshes and the torso, and
    between the two hands, from the URDF's own collision geometry (hppfcl). The URDF hand
    is the Dex3 hand; the robot carries Inspire hands, so this is approximate."""

    def __init__(self, arm_names):
        urdf = str(XRT / "assets" / "g1" / "g1_body29_hand14.urdf")
        self.model, self.geom = pin.buildModelsFromUrdf(urdf, package_dirs=str(XRT / "assets" / "g1"),
                                                        geometry_types=pin.GeometryType.COLLISION)[:2]
        self.data = self.model.createData()
        self.idx_q = [self.model.joints[self.model.getJointId(n)].idx_q for n in arm_names]
        names = [g.name for g in self.geom.geometryObjects]
        def ids(pred):
            return [i for i, g in enumerate(self.geom.geometryObjects)
                    if pred(self.model.frames[g.parentFrame].name)]
        body = ids(lambda f: f in BODY)
        self.pairs = {}
        for side in ("left", "right"):
            arm = ids(lambda f, side=side: any(f == f"{side}_{l}" for l in ARM_LINKS))
            self.pairs[side] = [(a, b) for a in arm for b in body]
        la = ids(lambda f: any(f == f"left_{l}" for l in ARM_LINKS[1:]))
        ra = ids(lambda f: any(f == f"right_{l}" for l in ARM_LINKS[1:]))
        self.pairs["hands"] = [(a, b) for a in la for b in ra]
        for group in self.pairs.values():
            for a, b in group:
                self.geom.addCollisionPair(pin.CollisionPair(a, b))
        self.gdata = pin.GeometryData(self.geom)
        self.index = {(cp.first, cp.second): k for k, cp in enumerate(self.geom.collisionPairs)}

    def __call__(self, q_arm):
        q = pin.neutral(self.model)
        for k, i in enumerate(self.idx_q):
            q[i] = q_arm[k]
        pin.computeDistances(self.model, self.data, self.geom, self.gdata, q)
        out = {}
        for name, group in self.pairs.items():
            out[name] = min(self.gdata.distanceResults[self.index[(a, b)]].min_distance for a, b in group)
        return out


class Chain:
    def __init__(self):
        self.ik = G1_29_ArmIK()
        self.model = self.ik.reduced_robot.model
        self.data = self.model.createData()
        self.lf, self.rf = self.ik.L_hand_id, self.ik.R_hand_id
        self.sh = [self.model.getJointId(n) for n in ("left_shoulder_pitch_joint",
                                                       "right_shoulder_pitch_joint")]
        self.q = np.zeros(self.model.nq)

    def settle(self, lw, rw, n=40):
        """Solve as the launcher does, frame after frame, with the arm following."""
        for _ in range(n):
            q, tau = self.ik.solve_ik(lw, rw, self.q.copy(), np.zeros(14))
            self.q = np.asarray(q, dtype=float).copy()
        pin.framesForwardKinematics(self.model, self.data, self.q)
        res = [float(np.linalg.norm(self.data.oMf[f].translation - w[:3, 3]))
               for f, w in ((self.lf, lw), (self.rf, rw))]
        hands = [self.data.oMf[f].translation.copy() for f in (self.lf, self.rf)]
        shoulders = [self.data.oMi[j].translation.copy() for j in self.sh]
        return self.q.copy(), np.asarray(tau, dtype=float), res, hands, shoulders


MAPPINGS = [("1:1", 1.0, 0.0)] + [(f"0.7 / {x:+.2f}", 0.7, x) for x in (0.0, -0.05, -0.10, -0.15)]


class WhatIf(WristMapping):
    """Offline only: separate x and y scales and a z offset, same order of operations as
    WristMapping.apply (head-relative, scale, then offset). The launcher has XY_SCALE only."""

    def __init__(self, x_scale, y_scale, x_offset, z_offset):
        super().__init__(x_offset=x_offset, xy_scale=1.0, z_offset=z_offset)
        self.xs, self.ys = x_scale, y_scale

    def apply(self, wrist_pose):
        from teleop.robot_control.wrist_offset import HEAD_TO_WAIST_X, HEAD_TO_WAIST_Z
        out = np.array(wrist_pose, dtype=float, copy=True)
        out[0, 3] = (out[0, 3] - HEAD_TO_WAIST_X) * self.xs + self.x_offset + HEAD_TO_WAIST_X
        out[1, 3] = out[1, 3] * self.ys
        out[2, 3] = out[2, 3] + self.z_offset
        return out


WHATIF = [("x0.7 y1.0 / -0.05", (0.7, 1.0, -0.05, 0.0)), ("x0.7 y1.0 / -0.10", (0.7, 1.0, -0.10, 0.0)),
          ("x1.0 y1.0 / -0.10", (1.0, 1.0, -0.10, 0.0)), ("xy0.7 / -0.05 z+0.10", (0.7, 0.7, -0.05, 0.10)),
          ("x0.7 y1.0 / -0.05 z+0.10", (0.7, 1.0, -0.05, 0.10))]


def grid(out):
    """Torque and clearance over operator wrist positions: forward 0..0.45 m, heights
    -0.30..-0.60 m below the headset, 0.18 m out from the centre line, palm down."""
    from teleop.robot_control.start_pose import ARM_JOINT_NAMES
    ch, clear, rows = Chain(), Clearance(ARM_JOINT_NAMES), []
    for mname, sc, xo in (("1:1", 1.0, 0.0), ("0.7 / -0.05", 0.7, -0.05), ("0.7 / -0.10", 0.7, -0.10)):
        wm = WristMapping(x_offset=xo, xy_scale=sc)
        for up in (-0.30, -0.40, -0.50, -0.60):
            ch.q = np.zeros(ch.model.nq)
            for fwd in np.arange(0.0, 0.451, 0.05):
                lt, rt = wm.apply_pair(to_waist(xr_wrist(fwd, 0.18, up, +1, 0.0), +1),
                                       to_waist(xr_wrist(fwd, 0.18, up, -1, 0.0), -1))
                q, tau, res, hands, sh = ch.settle(lt, rt, n=25)
                cl = clear(q)
                rows.append(dict(mapping=mname, op_fwd=round(float(fwd), 2), op_up=up,
                                 tau_sp_L=round(float(tau[SP[0]]), 2), ik_miss_L_m=round(res[0], 3),
                                 clear_torso_L_m=round(cl["left"], 3), clear_hands_m=round(cl["hands"], 3)))
    with open(out / "reach_grid.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    for mname in ("1:1", "0.7 / -0.05", "0.7 / -0.10"):
        print(f"\n{mname}: left shoulder pitch gravity torque (Nm) [torso clearance m]; rows = height below headset, cols = forward")
        print("        " + " ".join(f"{f:>13.2f}" for f in np.arange(0.0, 0.451, 0.05)))
        for up in (-0.30, -0.40, -0.50, -0.60):
            cells = [r for r in rows if r["mapping"] == mname and r["op_up"] == up]
            print(f"{up:+.2f}  " + " ".join(f"{r['tau_sp_L']:6.2f} [{r['clear_torso_L_m']:.3f}]" for r in cells))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(Path(__file__).resolve().parent.parent / "out"))
    ap.add_argument("--whatif", action="store_true", help="offline alternatives (WHATIF), not launcher knobs")
    ap.add_argument("--grid", action="store_true", help="operator wrist grid (forward x height) for 3 mappings")
    args = ap.parse_args()
    if args.grid:
        return grid(Path(args.out))
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    ch = Chain()
    from teleop.robot_control.start_pose import ARM_JOINT_NAMES
    clear = Clearance(ARM_JOINT_NAMES)
    rows = []
    todo = ([(n, None, None, WhatIf(*a)) for n, a in WHATIF] if args.whatif else
            [(n, sc, xo, WristMapping(x_offset=xo, xy_scale=sc)) for n, sc, xo in MAPPINGS])
    for mname, scale, xoff, wm in todo:
        for rname, roll in ROLLS.items():
            ch.q = np.zeros(ch.model.nq)
            lw0, rw0 = to_waist(xr_wrist(*POSES[0][2], +1, roll), +1), to_waist(xr_wrist(*POSES[0][2], -1, roll), -1)
            ch.settle(*wm.apply_pair(lw0, rw0), n=60)        # start from the hanging pose
            for pname, pdesc, (f, o, u) in POSES:
                lw = to_waist(xr_wrist(f, o, u, +1, roll), +1)
                rw = to_waist(xr_wrist(f, o, u, -1, roll), -1)
                lt, rt = wm.apply_pair(lw, rw)
                q, tau, res, hands, sh = ch.settle(lt, rt)
                cl = clear(q)
                rows.append(dict(
                    mapping=mname, xy_scale=scale, x_offset=xoff, hand=rname, pose=pname,
                    op_fwd=f, op_out=o, op_up=u,
                    tgt_fwd_of_shoulder_L=round(float(lt[0, 3] - sh[0][0]), 3),
                    tgt_below_shoulder_L=round(float(sh[0][2] - lt[2, 3]), 3),
                    tau_sp_L=round(float(tau[SP[0]]), 2), tau_sp_R=round(float(tau[SP[1]]), 2),
                    tau_elb_L=round(float(tau[ELB[0]]), 2), tau_elb_R=round(float(tau[ELB[1]]), 2),
                    q_sp_L=round(float(q[SP[0]]), 3), q_elb_L=round(float(q[ELB[0]]), 3),
                    hand_fwd_of_shoulder_L=round(float(hands[0][0] - sh[0][0]), 3),
                    ik_miss_L_m=round(res[0], 3), ik_miss_R_m=round(res[1], 3),
                    clear_torso_L_m=round(cl["left"], 3), clear_torso_R_m=round(cl["right"], 3),
                    clear_hands_m=round(cl["hands"], 3)))
                print(f"{mname:12s} {rname:9s} {pname:22s} tau_sp L {tau[SP[0]]:6.2f} R {tau[SP[1]]:6.2f} Nm"
                      f"  target fwd {lt[0, 3] - sh[0][0]:+.3f} m  miss {res[0]:.3f}/{res[1]:.3f} m"
                      f"  clear torso {cl['left']:.3f}/{cl['right']:.3f} hands {cl['hands']:.3f} m", flush=True)
    with open(out / ("reach_whatif.csv" if args.whatif else "reach_table.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    (out / "reach_poses.json").write_text(json.dumps(
        {"head_y_m": HEAD_Y, "poses": [{"name": n, "what": d, "fwd_out_up_m": p} for n, d, p in POSES],
         "rolls_deg": ROLLS}, indent=1))
    print(f"wrote {len(rows)} rows")


if __name__ == "__main__":
    main()
    sys.stdout.flush()
    from tools._procs import reap_child_processes
    reap_child_processes()
    os._exit(0)
