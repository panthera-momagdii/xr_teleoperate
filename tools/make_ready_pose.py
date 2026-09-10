#!/usr/bin/env python3
"""Derive poses/ready.yaml from wrist targets, check it, and draw it.

Offline: pinocchio + matplotlib only. No DDS, no robot.

    cd teleop && python ../tools/make_ready_pose.py \
        --out ../poses/ready.yaml --png ../logs/overnight/ready_pose.png

The pose asked for is "elbows bent ~90 degrees, upper arms hanging, hands in front of
the belly ~20 cm apart, palms facing each other". Beware a trap: on this URDF the
ELBOW JOINT ANGLE is not the anatomical elbow bend. At q = 0 the elbow joint reads 0
while the arm is already bent about 90 degrees -- the upper arm hangs down and the
forearm points forward. This tool therefore reports the ANATOMICAL angle, the angle
between the shoulder->elbow and elbow->wrist segments, which is what the description
means.

Wrist targets are chosen INSIDE the measured reachable band (logs/overnight/reach_map.npz:
at x = 0.30 both arms reach z = +0.05 .. +0.55), not guessed.
"""

import argparse
import os
import sys

import numpy as np
import pinocchio as pin

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(REPO_ROOT)

from tools.reach_map import build_model, arm_columns, solve_one, evaluate, seeds  # noqa: E402
from teleop.robot_control.start_pose import ARM_JOINT_NAMES  # noqa: E402


def anatomical_elbow_deg(model, data, q, side):
    pin.framesForwardKinematics(model, data, q)
    sh = data.oMi[model.getJointId(f"{side}_shoulder_pitch_joint")].translation
    el = data.oMi[model.getJointId(f"{side}_elbow_joint")].translation
    ee = data.oMf[model.getFrameId("L_ee" if side == "left" else "R_ee")].translation
    u, v = el - sh, ee - el
    cos = np.dot(-u, v) / (np.linalg.norm(u) * np.linalg.norm(v))
    return float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))


def draw(model, data, q, path, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    pin.framesForwardKinematics(model, data, q)
    fig = plt.figure(figsize=(10, 4.6))
    # Labels checked against the rendered output, not assumed: matplotlib's azim/elev
    # convention is easy to get backwards, and a mislabelled view of a robot pose is
    # worse than no view.
    #   elev=0, azim=-90  -> x horizontal, z vertical  = looking along +y  = SIDE
    #   elev=0, azim=  0  -> y horizontal, z vertical  = looking along -x  = FRONT
    views = ((20, -60, "perspective"),
             (0, -90, "side view (looking along +y)"),
             (0, 0, "front view (looking along -x)"))
    for n, (elev, azim, label) in enumerate(views, start=1):
        ax = fig.add_subplot(1, 3, n, projection="3d")
        for side, colour in (("left", "#2f6f4f"), ("right", "#2c6fbb")):
            chain = [np.zeros(3)]
            for j in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
                      "wrist_roll", "wrist_pitch", "wrist_yaw"):
                chain.append(data.oMi[model.getJointId(f"{side}_{j}_joint")].translation)
            chain.append(data.oMf[model.getFrameId(
                "L_ee" if side == "left" else "R_ee")].translation)
            pts = np.array(chain)
            ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], "-o", color=colour, ms=3, lw=2)
            ax.scatter(*pts[-1], color="#c0392b", s=28, zorder=6)
        ax.scatter(0, 0, 0, color="black", s=30, marker="s")
        ax.set_xlim(-0.1, 0.5); ax.set_ylim(-0.4, 0.4); ax.set_zlim(-0.1, 0.5)
        ax.set_box_aspect((0.6, 0.8, 0.6))
        ax.set_xlabel("x fwd", fontsize=7); ax.set_ylabel("y left", fontsize=7)
        ax.set_zlabel("z up", fontsize=7)
        ax.tick_params(labelsize=6)
        ax.view_init(elev=elev, azim=azim)
        if n > 1:
            # In an edge-on view the depth axis collapses into an unreadable smear of
            # overlapping tick labels; hide it rather than ship that.
            (ax.yaxis if n == 2 else ax.xaxis).set_ticklabels([])
            (ax.set_ylabel if n == 2 else ax.set_xlabel)("")
        ax.set_title(label, fontsize=8)
    fig.suptitle(title, fontsize=10)
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", default="../assets/g1/g1_body29_hand14.urdf")
    ap.add_argument("--model-dir", default="../assets/g1/")
    ap.add_argument("--x", type=float, default=0.28, help="wrist forward distance (m)")
    ap.add_argument("--half-width", type=float, default=0.10,
                    help="half the gap between the hands (m); 0.10 => 20 cm apart")
    ap.add_argument("--z", type=float, default=0.18, help="wrist height above the IK base (m)")
    ap.add_argument("--out", default="../poses/ready.yaml")
    ap.add_argument("--png", default="../logs/overnight/ready_pose.png")
    ap.add_argument("--name", default="ready")
    args = ap.parse_args()

    if not os.path.exists(args.urdf):
        print(f"URDF not found: {args.urdf}\nRun with cwd = teleop/", file=sys.stderr)
        return 2

    model = build_model(args.urdf, args.model_dir)
    data = model.createData()
    rng = np.random.RandomState(20260909)

    q = np.zeros(model.nq)
    worst_pos = worst_rot = 0.0
    for side, frame, sign in (("left", "L_ee", +1.0), ("right", "R_ee", -1.0)):
        fid = model.getFrameId(frame)
        cols = arm_columns(model, fid)
        # Palms facing each other. Per the Unitree URDF conventions in
        # tv_wrapper.py's docstring, identity rotation puts the fingers forward with the
        # palms inboard for BOTH hands -- the left/right frames differ by a y/z sign
        # that already encodes the mirroring.
        target = pin.SE3(np.eye(3), np.array([args.x, sign * args.half_width, args.z]))
        best = None
        for q_seed in seeds(model, cols, rng, 12, None):
            cand = solve_one(model, data, fid, cols, target, q_seed, iters=400)
            err = evaluate(model, data, fid, cols, target, cand)
            if best is None or err[:2] < best[0][:2]:
                best = (err, cand)
            if err[0] < 2e-3 and err[1] < 0.02:
                break
        err, cand = best
        q[cols] = cand[cols]
        worst_pos, worst_rot = max(worst_pos, err[0]), max(worst_rot, err[1])
        if err[0] > 5e-3 or err[1] > 0.05:
            print(f"IK did not reach the {side} target "
                  f"(pos {err[0]*1000:.1f} mm, rot {err[1]:.3f} rad). "
                  f"Pick a target inside the reachable band.", file=sys.stderr)
            return 1

    lower, upper = model.lowerPositionLimit, model.upperPositionLimit
    margin = np.minimum(q - lower, upper - q)
    if np.any(margin < 0):
        bad = [ARM_JOINT_NAMES[i] for i in np.flatnonzero(margin < 0)]
        print(f"pose violates URDF limits at {bad}", file=sys.stderr)
        return 1

    pin.framesForwardKinematics(model, data, q)
    l_ee = data.oMf[model.getFrameId("L_ee")].translation
    r_ee = data.oMf[model.getFrameId("R_ee")].translation
    elbow_l = anatomical_elbow_deg(model, data, q, "left")
    elbow_r = anatomical_elbow_deg(model, data, q, "right")

    print(f"IK reached both targets: worst pos {worst_pos*1000:.3f} mm, "
          f"worst rot {worst_rot:.5f} rad")
    print(f"L_ee {np.round(l_ee, 4).tolist()}   R_ee {np.round(r_ee, 4).tolist()}")
    print(f"hands {abs(l_ee[1] - r_ee[1])*100:.1f} cm apart")
    print(f"anatomical elbow bend: left {elbow_l:.0f} deg, right {elbow_r:.0f} deg")
    print(f"smallest distance to any URDF joint limit: {margin.min():.4f} rad "
          f"({ARM_JOINT_NAMES[int(np.argmin(margin))]})")

    lines = [
        f"# {args.name} pose for xr_teleoperate, G1_29 arms.",
        "#",
        "# DERIVED, not measured. tools/make_ready_pose.py solved the IK for wrist",
        f"#   targets x={args.x}, y=+-{args.half_width}, z={args.z} (metres, IK base frame),",
        "#   wrist pointing forward, palms inboard. Those targets sit inside the",
        "#   measured reachable band (logs/overnight/reach_map.npz: at x=0.30 both arms",
        "#   reach z=+0.05..+0.55).",
        "#",
        "# A POSE CAPTURED FROM THE ROBOT BEATS THIS ONE. Put the arms where you want",
        "#   them and run:  python tools/capture_pose.py --out poses/tray.yaml",
        "#",
        f"# Checked: worst IK position error {worst_pos*1000:.3f} mm, worst rotation error",
        f"#   {worst_rot:.5f} rad; every joint inside its URDF limits with at least",
        f"#   {margin.min():.4f} rad to spare; hands {abs(l_ee[1]-r_ee[1])*100:.1f} cm apart;",
        f"#   anatomical elbow bend {elbow_l:.0f}/{elbow_r:.0f} degrees.",
        "#",
        "# NOTE: the elbow JOINT ANGLE is not the anatomical bend on this URDF. At q=0",
        "#   the elbow joint reads 0 while the arm is already bent ~90 degrees.",
        "#",
        "# Rendered: logs/overnight/ready_pose.png",
        "",
        f"name: {args.name}",
        "robot: G1_29",
        "units: radians",
        "# Order is G1_29_JointArmIndex (robot_arm.py:285-302), motors 15..28, but the",
        "# names are what is read -- a reordered list cannot silently mean something else.",
        "joints:",
    ]
    width = max(len(n) for n in ARM_JOINT_NAMES)
    for i, name in enumerate(ARM_JOINT_NAMES):
        lines.append(f"  {name + ':':<{width + 1}} {q[i]:+.6f}")
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w") as fh:
        fh.write("\n".join(lines) + "\n")
    print(f"\nwrote {args.out}")

    draw(model, data, q,
         args.png,
         f"{args.name}: hands {abs(l_ee[1]-r_ee[1])*100:.0f} cm apart at "
         f"x={args.x:.2f} z={args.z:.2f}, elbows {elbow_l:.0f} deg "
         f"(red = L_ee/R_ee, black square = IK base)")
    print(f"wrote {args.png}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
