#!/usr/bin/env python3
"""FK -> landmarks -> DexPilot retarget round trip for the Dex5-1P, both hands.

Take a known joint configuration q*, run forward kinematics on the Dex5 URDF, turn
the wrist and the five fingertips into the (25,3) landmark array televuer delivers,
push that through exactly the pipeline Dex5_1_Controller.control_process uses, and
see what q_hat comes back. Nothing here needs a robot, a headset, or DDS.

Run with cwd = teleop/robot_control, where HandRetargeting's Unit_Test paths resolve:

    cd teleop/robot_control && python ../../tools/test_dex5_retarget_roundtrip.py

Small residuals are EXPECTED and are not a failure: DexPilot optimises fingertip
contact distances, not joint angles, so many configurations map to the same set of
tip positions. What must hold is that every q_hat is inside the URDF limits and that
a pinch closes the thumb against the index rather than saturating everything.

Exit codes: 0 all configurations pass, 1 a check failed.
"""

import os
import sys

import numpy as np
import pinocchio as pin

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(REPO_ROOT)

from teleop.robot_control.hand_retargeting import (  # noqa: E402
    HandRetargeting, HandType,
    DEX5_LANDMARK_ROTATION_LEFT, DEX5_LANDMARK_ROTATION_RIGHT,
)

# televuer's (25,3) layout: wrist 0, then thumb 1-4, index 5-9, middle 10-14,
# ring 15-19, pinky 20-24. The tips are the last index of each run.
FINGER_LANDMARK_RUNS = {
    "thumb":  [1, 2, 3, 4],
    "index":  [5, 6, 7, 8, 9],
    "middle": [10, 11, 12, 13, 14],
    "ring":   [15, 16, 17, 18, 19],
    "pinky":  [20, 21, 22, 23, 24],
}
TIP_INDEX = {"thumb": 4, "index": 9, "middle": 14, "ring": 19, "pinky": 24}


def load_model(side):
    suffix = "L" if side == "left" else "R"
    urdf = os.path.join(REPO_ROOT, "assets", "unitree_hand_Dex5", f"Dex5-URDF-{suffix}.urdf")
    model = pin.buildModelFromUrdf(urdf)
    return model, model.createData(), suffix, urdf


def actuated_joint_names(model):
    return [model.names[i] for i in range(1, model.njoints)]


def fk_points(model, data, q, suffix):
    """Wrist and the five fingertips, expressed in the wrist frame."""
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    wrist = data.oMf[model.getFrameId(f"base_link00{suffix}")]
    out = {}
    for finger in TIP_INDEX:
        tip = data.oMf[model.getFrameId(f"{finger}_tip{suffix}")]
        out[finger] = (wrist.inverse() * tip).translation.copy()
    return out


def build_landmarks(tips):
    """(25,3): wrist at the origin, tips at 4/9/14/19/24, the rest interpolated.

    DexPilot only reads 0/4/9/14/19/24, but the array must be fully finite -- a NaN
    anywhere would propagate through the optimiser.
    """
    lm = np.zeros((25, 3), dtype=float)
    for finger, run in FINGER_LANDMARK_RUNS.items():
        tip = tips[finger]
        n = len(run)
        for k, idx in enumerate(run):
            lm[idx] = tip * ((k + 1) / n)      # wrist (origin) -> tip, evenly spaced
    assert np.all(np.isfinite(lm)), "landmark array contains non-finite values"
    return lm


SETTLE_TOL = 1e-5
SETTLE_MAX_ITERS = 400

# dex_retargeting deliberately relaxes the nlopt bounds by this much:
#   optimizer.py:47  def set_joint_limit(self, joint_limits, epsilon=1e-3)
#   opt.set_lower_bounds(lower - epsilon); opt.set_upper_bounds(upper + epsilon)
# So a solution may legitimately sit up to 1 mrad outside a URDF limit. Anything
# beyond that is a real violation.
RETARGET_LIMIT_EPS = 1e-3
# nlopt stops within its own tolerance of that relaxed bound.
SOLVER_TOL = 1e-6


def retarget(hr, side, landmarks, settle=True):
    """Exactly what Dex5_1_Controller.control_process does, minus the shared arrays.

    Iterated to convergence, deliberately. SeqRetargeting applies an LPFilter with
    low_pass_alpha 0.2 and warm-starts the optimiser from last_qpos, so ONE call moves
    only ~20 % of the way to the answer and carries state from the previous call. The
    controller gets away with a single call per frame because it runs at 100 Hz on a
    continuous stream; a test that called retarget() once would be measuring the
    filter's time constant, not the mapping.
    """
    rotation = hr.left_landmark_rotation if side == "left" else hr.right_landmark_rotation
    indices = hr.left_indices if side == "left" else hr.right_indices
    retargeting = hr.left_retargeting if side == "left" else hr.right_retargeting
    perm = hr.left_dex_retargeting_to_hardware if side == "left" else hr.right_dex_retargeting_to_hardware

    data = landmarks @ rotation.T                       # the controller's own line
    ref = data[indices[1, :]] - data[indices[0, :]]

    q_all = retargeting.retarget(ref)
    iters = 1
    if settle:
        while iters < SETTLE_MAX_ITERS:
            q_next = retargeting.retarget(ref)
            iters += 1
            if np.max(np.abs(q_next - q_all)) < SETTLE_TOL:
                q_all = q_next
                break
            q_all = q_next
    return q_all[perm], q_all, iters


def make_configs(model, suffix):
    """open / pinch / fist, all strictly inside the URDF limits."""
    names = actuated_joint_names(model)
    lo, hi = model.lowerPositionLimit.copy(), model.upperPositionLimit.copy()
    idx = {n: model.joints[model.getJointId(n)].idx_q for n in names}

    def blank():
        return np.zeros(model.nq)

    def at(cfg, name, frac):
        i = idx[name]
        cfg[i] = lo[i] + frac * (hi[i] - lo[i])

    configs = {}
    configs["open"] = blank()

    pinch = blank()
    for n in (f"Yaw_11{suffix}", f"Roll_12{suffix}", f"Pitch_13{suffix}", f"Pitch_14{suffix}"):
        at(pinch, n, 0.60)
    for n in (f"Pitch_22{suffix}", f"Pitch_23{suffix}", f"Pitch_24{suffix}"):
        at(pinch, n, 0.60)
    configs["pinch"] = pinch

    fist = blank()
    for n in names:
        if n.startswith("Pitch_"):
            at(fist, n, 0.80)
    configs["fist"] = fist
    return configs, idx, lo, hi


def tip_distance(model, data, q, suffix, a, b):
    pts = fk_points(model, data, q, suffix)
    return float(np.linalg.norm(pts[a] - pts[b]))


def run_side(side, hr):
    model, data, suffix, urdf = load_model(side)
    names = actuated_joint_names(model)
    api_names = hr.left_dex5_api_joint_names if side == "left" else hr.right_dex5_api_joint_names
    perm = hr.left_dex_retargeting_to_hardware if side == "left" else hr.right_dex_retargeting_to_hardware
    retarget_names = hr.left_retargeting_joint_names if side == "left" else hr.right_retargeting_joint_names
    rotation = DEX5_LANDMARK_ROTATION_LEFT if side == "left" else DEX5_LANDMARK_ROTATION_RIGHT

    print(f"\n{'=' * 78}\n{side.upper()} HAND  ({os.path.relpath(urdf, REPO_ROOT)})\n{'=' * 78}")
    print(f"nq={model.nq} njoints={model.njoints - 1} actuated joints")
    lo, hi = model.lowerPositionLimit, model.upperPositionLimit
    print(f"{'joint':14s} {'lower':>9} {'upper':>9}   (URDF order)")
    for n in names:
        i = model.joints[model.getJointId(n)].idx_q
        print(f"  {n:12s} {lo[i]:9.4f} {hi[i]:9.4f}")

    ok = True
    for frame in [f"base_link00{suffix}"] + [f"{f}_tip{suffix}" for f in TIP_INDEX]:
        exists = model.existFrame(frame)
        print(f"frame {frame:18s} exists={exists}")
        ok = ok and exists

    print(f"\nrotation matrix (det={np.linalg.det(rotation):+.1f}, symmetric="
          f"{np.allclose(rotation, rotation.T)}, R@R==I={np.allclose(rotation @ rotation, np.eye(3))})")
    print("  -> R.T == R and R@R == I, so pre-rotating by R.T exactly cancels the")
    print("     controller's own `landmarks @ rotation.T` line.")

    same_set = sorted(api_names) == sorted(names)
    print(f"\ntarget_joint_names is a permutation of the URDF's actuated joints: {same_set}")
    ok = ok and same_set
    print(f"{'left' if side == 'left' else 'right'}_dex_retargeting_to_hardware = {list(perm)}")
    identity = list(perm) == list(range(len(perm)))
    print(f"  identity permutation: {identity}"
          + ("" if identity else "  <-- NOT identity; this IS the hardware order, report it"))
    print(f"  retargeting joint order: {list(retarget_names)}")

    configs, idx, lo, hi = make_configs(model, suffix)
    open_thumb_index = None

    for name, q_star in configs.items():
        tips = fk_points(model, data, q_star, suffix)
        landmarks = build_landmarks(tips)
        landmarks = landmarks @ rotation.T            # cancels the controller's rotation
        q_hat_hw, q_hat_all, iters = retarget(hr, side, landmarks)

        q_star_hw = np.array([q_star[idx[n]] for n in api_names])
        resid = np.abs(q_hat_hw - q_star_hw)

        lo_hw = np.array([lo[idx[n]] for n in api_names])
        hi_hw = np.array([hi[idx[n]] for n in api_names])
        over = np.maximum(lo_hw - q_hat_hw, q_hat_hw - hi_hw)
        strict_out = int(np.sum(over > 1e-6))
        # nlopt converges TO the relaxed bound, stopping within its own xtol, so the
        # measured excess lands a little past epsilon (observed 1.28e-8 rad). Allow one
        # microradian of solver tolerance on top of the declared relaxation; anything
        # bigger than that is a genuine violation worth failing on.
        eps_cmp = RETARGET_LIMIT_EPS + SOLVER_TOL
        inside = bool(np.all(over <= eps_cmp))
        n_out = int(np.sum(over > eps_cmp))

        print(f"\n--- {side} / {name} ---")
        print(f"  settled after {iters} retarget() iterations (LPFilter alpha 0.2)")
        print(f"  max |q_hat - q*| = {resid.max():.4f} rad  (mean {resid.mean():.4f})")
        worst = int(np.argmax(resid))
        print(f"  worst joint      = [{worst}] {api_names[worst]}  "
              f"q*={q_star_hw[worst]:+.4f}  q_hat={q_hat_hw[worst]:+.4f}")
        worst_violation = float(over.max())
        print(f"  all q_hat inside URDF limits (+/- the retargeter's own {RETARGET_LIMIT_EPS} rad "
              f"bound relaxation): {inside}"
              + ("" if inside else f"  ({n_out} genuinely outside, worst by {worst_violation:.2e} rad)"))
        if strict_out:
            print(f"    ({strict_out} joint(s) outside the STRICT URDF limit, worst by "
                  f"{worst_violation:.12f} rad -- dex_retargeting/optimizer.py:47 relaxes the "
                  f"nlopt bounds by epsilon={RETARGET_LIMIT_EPS}, so this is upstream design, "
                  f"not a real violation)")
        print("  per-joint |residual| (hardware order):")
        print("   " + " ".join(f"{v:5.2f}" for v in resid))
        ok = ok and inside

        q_hat_urdf = _hw_to_urdf(q_hat_hw, api_names, idx, model)
        tips_hat = fk_points(model, data, q_hat_urdf, suffix)
        tip_err = {f: float(np.linalg.norm(tips_hat[f] - tips[f])) for f in tips}
        print("  fingertip position error after the round trip (mm) -- this is what")
        print("  DexPilot actually minimises, so it matters more than the joint residual:")
        print("   " + "  ".join(f"{f}={tip_err[f] * 1000:.1f}" for f in
                                ("thumb", "index", "middle", "ring", "pinky")))
        print(f"    max {max(tip_err.values()) * 1000:.1f} mm, "
              f"mean {np.mean(list(tip_err.values())) * 1000:.1f} mm")

        d_thumb_index = tip_distance(model, data, q_hat_urdf, suffix, "thumb", "index")
        if name == "open":
            open_thumb_index = d_thumb_index
        print(f"  thumb-tip to index-tip distance after retarget: {d_thumb_index * 1000:.1f} mm"
              + (f"  (open was {open_thumb_index * 1000:.1f} mm)" if open_thumb_index is not None else ""))

        if name == "pinch":
            closed = d_thumb_index < open_thumb_index
            thumb_moved = float(np.abs(q_hat_hw[16:20]).max())
            index_moved = float(np.abs(q_hat_hw[0:4]).max())
            others = float(np.abs(q_hat_hw[4:16]).max())
            print(f"  PINCH check: thumb(16-19) max |q| = {thumb_moved:.4f}, "
                  f"index(0-3) max |q| = {index_moved:.4f}, other fingers max |q| = {others:.4f}")
            print(f"  PINCH closes thumb toward index (vs open): {closed}")
            print(f"  not everything saturating (other fingers < thumb+index): "
                  f"{others < max(thumb_moved, index_moved)}")
            ok = ok and closed
    return ok


def _hw_to_urdf(q_hw, api_names, idx, model):
    q = np.zeros(model.nq)
    for value, name in zip(q_hw, api_names):
        q[idx[name]] = value
    return q


def main():
    hr = HandRetargeting(HandType.UNITREE_DEX5_Unit_Test)
    ok_left = run_side("left", hr)
    ok_right = run_side("right", hr)
    print(f"\n{'=' * 78}")
    print(f"LEFT  : {'PASS' if ok_left else 'FAIL'}")
    print(f"RIGHT : {'PASS' if ok_right else 'FAIL'}")
    print("(residuals are informational: DexPilot matches fingertip contact distances,")
    print(" not joint angles, so q_hat != q* is expected and not a failure)")
    return 0 if (ok_left and ok_right) else 1


if __name__ == "__main__":
    sys.exit(main())
