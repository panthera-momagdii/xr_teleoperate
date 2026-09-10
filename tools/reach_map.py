#!/usr/bin/env python3
"""Sweep the G1_29 arm workspace and record what the IK can actually reach.

Read-only, offline, no DDS, no robot. Answers one question:

    at a given forward distance, how HIGH and how LOW can the wrist actually go?

That number is what explains the 2026-09-09 vertical clipping. Horizontal tracking
followed; up/down did not, because the mapping is head-relative and the operator's
hands sit lower under their head than the G1's do under its own, so targets land near
the bottom of the reachable band and stop.

    cd teleop && python ../tools/reach_map.py --out ../logs/overnight/reach_map.npz

Method
------
NOT `G1_29_ArmIK.solve_ik`. That solver is dual-arm (a bad left target perturbs the
right), stateful (`WeightedMovingFilter` carries between calls, `init_data` warm-starts
from the previous solve), and on failure it SWALLOWS non-convergence and returns the
current joint vector -- all three are right for teleoperation and wrong for a map.

Instead this builds the same reduced model straight from the URDF (same locked joints,
same 5 cm `L_ee`/`R_ee` offsets as `robot_arm_ik.py:44-104`) and runs a plain damped
least-squares IK per arm. The two arms are kinematically independent -- the left
`L_ee` Jacobian is non-zero only in columns 0..6 and the right only in 7..13 -- so each
is solved on its own 7 joints with no coupling at all.

Every sample is solved from several seeds and the best result kept, so a "not reachable"
is a statement about the arm and not about one unlucky initial guess.

"Wrist pointing forward" means target rotation = identity, which at q=0 is what the
wrist frame already is (measured: identity to 2e-4). In the Unitree URDF convention
that is the x-axis pointing from the wrist toward the middle finger, i.e. fingers
forward, palm inboard.
"""

import argparse
import os
import sys
import time

import numpy as np
import pinocchio as pin

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(REPO_ROOT)

LOCKED_JOINTS = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_hand_thumb_0_joint", "left_hand_thumb_1_joint", "left_hand_thumb_2_joint",
    "left_hand_middle_0_joint", "left_hand_middle_1_joint",
    "left_hand_index_0_joint", "left_hand_index_1_joint",
    "right_hand_thumb_0_joint", "right_hand_thumb_1_joint", "right_hand_thumb_2_joint",
    "right_hand_index_0_joint", "right_hand_index_1_joint",
    "right_hand_middle_0_joint", "right_hand_middle_1_joint",
]

POS_TOL = 0.01       # m   -- 1 cm; below the 5 cm grid, well above solver noise
ROT_TOL = 0.15       # rad -- 8.6 deg
LIMIT_EPS = 1e-3     # rad from a bound counts as "at the limit"


def build_model(urdf_path, model_dir):
    robot = pin.RobotWrapper.BuildFromURDF(urdf_path, model_dir)
    reduced = robot.buildReducedRobot(
        list_of_joints_to_lock=LOCKED_JOINTS,
        reference_configuration=np.zeros(robot.model.nq))
    m = reduced.model
    for name, joint in (("L_ee", "left_wrist_yaw_joint"),
                        ("R_ee", "right_wrist_yaw_joint")):
        m.addFrame(pin.Frame(name, m.getJointId(joint),
                             pin.SE3(np.eye(3), np.array([0.05, 0.0, 0.0])),
                             pin.FrameType.OP_FRAME))
    return m


def arm_columns(model, frame_id):
    """The joint columns this frame actually depends on."""
    data = model.createData()
    pin.computeJointJacobians(model, data, np.zeros(model.nq))
    J = pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    return np.flatnonzero(np.abs(J).sum(axis=0) > 1e-9)


def solve_one(model, data, frame_id, cols, target, q_seed, iters=150, damping=1e-4):
    """Damped least squares onto an SE(3) target, restricted to `cols`. Returns q."""
    q = q_seed.copy()
    lower, upper = model.lowerPositionLimit, model.upperPositionLimit
    for _ in range(iters):
        pin.framesForwardKinematics(model, data, q)
        pin.computeJointJacobians(model, data, q)
        oMf = data.oMf[frame_id]
        err = pin.log6(oMf.actInv(target)).vector           # in the frame's local coords
        if np.linalg.norm(err[:3]) < 1e-5 and np.linalg.norm(err[3:]) < 1e-5:
            break
        J = pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.LOCAL)[:, cols]
        JJt = J @ J.T + damping * np.eye(6)
        dq = J.T @ np.linalg.solve(JJt, err)
        q[cols] = np.clip(q[cols] + dq, lower[cols], upper[cols])
    return q


def evaluate(model, data, frame_id, cols, target, q):
    pin.framesForwardKinematics(model, data, q)
    oMf = data.oMf[frame_id]
    pos_err = float(np.linalg.norm(oMf.translation - target.translation))
    rot_err = float(np.linalg.norm(pin.log3(oMf.rotation.T @ target.rotation)))
    at_limit = int(np.sum(
        (q[cols] <= model.lowerPositionLimit[cols] + LIMIT_EPS)
        | (q[cols] >= model.upperPositionLimit[cols] - LIMIT_EPS)))
    return pos_err, rot_err, at_limit


def seeds(model, cols, rng, n_random, warm):
    out = [np.zeros(model.nq)]
    if warm is not None:
        out.append(warm)
    lower, upper = model.lowerPositionLimit, model.upperPositionLimit
    for _ in range(n_random):
        q = np.zeros(model.nq)
        q[cols] = rng.uniform(lower[cols], upper[cols])
        out.append(q)
    return out


def sweep(model, side, xs, ys, zs, n_random, rng):
    frame_id = model.getFrameId("L_ee" if side == "left" else "R_ee")
    cols = arm_columns(model, frame_id)
    data = model.createData()
    shape = (len(xs), len(ys), len(zs))

    ok_full = np.zeros(shape, dtype=bool)     # position AND orientation
    ok_pos = np.zeros(shape, dtype=bool)      # position only
    pos_err = np.full(shape, np.nan)
    rot_err = np.full(shape, np.nan)
    at_limit = np.zeros(shape, dtype=np.int8)
    qs = np.full(shape + (len(cols),), np.nan)

    warm = None
    t0 = time.monotonic()
    for i, x in enumerate(xs):
        for j, y in enumerate(ys):
            for k, z in enumerate(zs):
                target = pin.SE3(np.eye(3), np.array([x, y, z]))
                best = None
                for q_seed in seeds(model, cols, rng, n_random, warm):
                    q = solve_one(model, data, frame_id, cols, target, q_seed)
                    p_e, r_e, n_lim = evaluate(model, data, frame_id, cols, target, q)
                    score = (p_e, r_e)
                    if best is None or score < best[0]:
                        best = (score, q, p_e, r_e, n_lim)
                    if p_e < POS_TOL and r_e < ROT_TOL:
                        break                      # good enough; stop trying seeds
                _, q, p_e, r_e, n_lim = best
                pos_err[i, j, k] = p_e
                rot_err[i, j, k] = r_e
                at_limit[i, j, k] = n_lim
                ok_pos[i, j, k] = p_e < POS_TOL
                ok_full[i, j, k] = (p_e < POS_TOL) and (r_e < ROT_TOL)
                qs[i, j, k] = q[cols]
                warm = q if ok_pos[i, j, k] else None
        print(f"  {side}: x={x:+.2f} done "
              f"({100.0 * ok_full[i].mean():.0f}% reachable at this x, "
              f"{time.monotonic() - t0:.0f}s)", flush=True)
    return dict(ok_full=ok_full, ok_pos=ok_pos, pos_err=pos_err, rot_err=rot_err,
                at_limit=at_limit, q=qs, cols=cols)


def z_range_at(xs, zs, ok, x_query):
    """The reachable z band at the x closest to x_query, over ALL y. (lo, hi) or None."""
    i = int(np.argmin(np.abs(xs - x_query)))
    reachable_z = ok[i].any(axis=0)              # any y works, per z
    idx = np.flatnonzero(reachable_z)
    if idx.size == 0:
        return None, xs[i]
    return (float(zs[idx[0]]), float(zs[idx[-1]])), float(xs[i])


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", default="../assets/g1/g1_body29_hand14.urdf")
    ap.add_argument("--model-dir", default="../assets/g1/")
    ap.add_argument("--step", type=float, default=0.05)
    ap.add_argument("--x-range", type=float, nargs=2, default=(0.10, 0.60))
    ap.add_argument("--y-range", type=float, nargs=2, default=(-0.45, 0.45))
    # [panthera] z defaults to -0.40..0.70, NOT the -0.40..0.40 originally specified.
    # 0.40 provably truncates the workspace: independent forward-kinematics sampling of
    # 20k random arm configurations puts L_ee near x=0.30 anywhere from z=-0.01 to
    # z=+0.62, and the first sweep duly reported a band whose upper edge WAS the grid
    # edge. A map that stops before the top cannot answer "how high can it reach", which
    # is the whole question. The requested range is a subset of this one.
    ap.add_argument("--z-range", type=float, nargs=2, default=(-0.40, 0.70))
    ap.add_argument("--seeds", type=int, default=4, help="random restarts per sample")
    ap.add_argument("--out", default="../logs/overnight/reach_map.npz")
    ap.add_argument("--seed", type=int, default=20260909)
    args = ap.parse_args()

    if not os.path.exists(args.urdf):
        print(f"URDF not found: {args.urdf}\nRun this with cwd = teleop/", file=sys.stderr)
        return 2

    xs = np.round(np.arange(args.x_range[0], args.x_range[1] + 1e-9, args.step), 4)
    ys = np.round(np.arange(args.y_range[0], args.y_range[1] + 1e-9, args.step), 4)
    zs = np.round(np.arange(args.z_range[0], args.z_range[1] + 1e-9, args.step), 4)
    print(f"grid: x {xs[0]}..{xs[-1]} ({len(xs)}), y {ys[0]}..{ys[-1]} ({len(ys)}), "
          f"z {zs[0]}..{zs[-1]} ({len(zs)})  = {len(xs)*len(ys)*len(zs)} points per arm")

    model = build_model(args.urdf, args.model_dir)
    rng = np.random.RandomState(args.seed)

    out = {"x": xs, "y": ys, "z": zs,
           "pos_tol": POS_TOL, "rot_tol": ROT_TOL,
           "joint_lower": model.lowerPositionLimit, "joint_upper": model.upperPositionLimit,
           "joint_names": np.array([model.names[i] for i in range(1, model.njoints)])}

    for side in ("left", "right"):
        print(f"\nsweeping {side} arm...")
        res = sweep(model, side, xs, ys, zs, args.seeds, rng)
        for key, value in res.items():
            out[f"{side}_{key}"] = value

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    np.savez_compressed(args.out, **out)
    print(f"\nsaved {args.out}")

    print("\n=== the number that matters ===")
    for side in ("left", "right"):
        band, x_used = z_range_at(xs, zs, out[f"{side}_ok_full"], 0.30)
        band_p, _ = z_range_at(xs, zs, out[f"{side}_ok_pos"], 0.30)
        if band is None:
            print(f"{side}: NOTHING reachable at x={x_used:.2f}")
            continue
        edge = []
        if abs(band[0] - zs[0]) < 1e-9:
            edge.append("BOTTOM of the grid")
        if abs(band[1] - zs[-1]) < 1e-9:
            edge.append("TOP of the grid")
        print(f"{side} arm at x={x_used:.2f} m (wrist forward, relative to the IK base):")
        print(f"    reachable z: {band[0]:+.2f} .. {band[1]:+.2f} m   "
              f"(span {band[1]-band[0]:.2f} m)")
        if edge:
            print(f"    *** this band touches the {' and the '.join(edge)} -- "
                  f"the true limit is outside the swept range, widen --z-range ***")
        print(f"    position only (any wrist orientation): "
              f"{band_p[0]:+.2f} .. {band_p[1]:+.2f} m")
    return 0


if __name__ == "__main__":
    sys.exit(main())
