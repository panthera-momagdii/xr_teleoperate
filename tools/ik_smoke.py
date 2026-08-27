#!/usr/bin/env python3
"""Deterministic IK benchmark for G1_29_ArmIK. Used to measure the hand-mass change.

Run it with cwd = teleop/robot_control, which is where G1_29_ArmIK(Unit_Test=True)
expects to find ../../assets/g1/g1_body29_hand14.urdf:

    cd teleop/robot_control && python ../../tools/ik_smoke.py --steps 200

Two things matter for the before/after comparison to mean anything:

  * The trajectory is SEEDED. robot_arm_ik.py's own __main__ perturbs the oval with
    np.random.normal every step; with an unseeded RNG the two runs would follow
    different paths and the tauff delta would be noise.
  * G1_29_ArmIK pickles the whole pinocchio model -- inertias included -- to
    ./g1_29_model_cache.pkl and reloads it next run. A stale cache makes a URDF mass
    edit completely invisible. This tool refuses to start if one exists.

sol_tauff is pin.rnea(model, data, q, 0, 0), i.e. the pure gravity feed-forward, which
is exactly the quantity a heavier hand should increase.
"""

import argparse
import logging
import os
import sys
import time

import numpy as np
import pinocchio as pin

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(REPO_ROOT)


class ConvergenceCounter(logging.Handler):
    """robot_arm_ik logs 'ERROR in convergence' and falls back to opti.debug values."""

    def __init__(self):
        super().__init__(level=logging.ERROR)
        self.count = 0

    def emit(self, record):
        if "convergence" in record.getMessage().lower():
            self.count += 1


def oval_targets(steps, seed, noise_translation=0.001, noise_rotation=0.01,
                 rotation_speed=0.005):
    """The same slow oval robot_arm_ik.__main__ walks, made reproducible."""
    rng = np.random.RandomState(seed)
    L = pin.SE3(pin.Quaternion(1, 0, 0, 0), np.array([0.25, +0.25, 0.1]))
    R = pin.SE3(pin.Quaternion(1, 0, 0, 0), np.array([0.25, -0.25, 0.1]))
    out = []
    for step in range(steps):
        phase = step % 240
        rot_noise_L = pin.Quaternion(
            np.cos(rng.normal(0, noise_rotation) / 2), 0,
            rng.normal(0, noise_rotation / 2), 0).normalized()
        rot_noise_R = pin.Quaternion(
            np.cos(rng.normal(0, noise_rotation) / 2), 0, 0,
            rng.normal(0, noise_rotation / 2)).normalized()
        if phase <= 120:
            angle = rotation_speed * phase
            sign = +1.0
        else:
            angle = rotation_speed * (240 - phase)
            sign = -1.0
        L.rotation = (rot_noise_L * pin.Quaternion(np.cos(angle / 2), 0, np.sin(angle / 2), 0)).toRotationMatrix()
        R.rotation = (rot_noise_R * pin.Quaternion(np.cos(angle / 2), 0, 0, np.sin(angle / 2))).toRotationMatrix()
        L.translation = L.translation + sign * (np.array([0.001,  0.001, 0.001]) + rng.normal(0, noise_translation, 3))
        R.translation = R.translation + sign * (np.array([0.001, -0.001, 0.001]) + rng.normal(0, noise_translation, 3))
        out.append((L.homogeneous.copy(), R.homogeneous.copy()))
    return out


# One fixed, comfortably reachable pose. Both runs report tauff here, so the two
# 14-vectors are directly comparable.
REACH_L = pin.SE3(pin.Quaternion(1, 0, 0, 0), np.array([0.30, +0.22, 0.05]))
REACH_R = pin.SE3(pin.Quaternion(1, 0, 0, 0), np.array([0.30, -0.22, 0.05]))


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--steps", type=int, default=200)
    parser.add_argument("--seed", type=int, default=20260827)
    parser.add_argument("--label", type=str, default="run")
    args = parser.parse_args()

    cache = "g1_29_model_cache.pkl"
    if os.path.exists(cache):
        print(f"REFUSING TO RUN: {os.path.abspath(cache)} exists. It pickles the model's "
              f"inertias, so a URDF mass edit would be invisible. Delete it first.")
        return 2

    from teleop.robot_control.robot_arm_ik import G1_29_ArmIK

    counter = ConvergenceCounter()
    logging.getLogger().addHandler(counter)
    for name in list(logging.root.manager.loggerDict):
        if "robot_arm_ik" in name:
            logging.getLogger(name).addHandler(counter)

    print(f"=== ik_smoke [{args.label}] steps={args.steps} seed={args.seed} ===")
    urdf_mtime = os.path.getmtime("../../assets/g1/g1_body29_hand14.urdf")
    print(f"urdf mtime: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(urdf_mtime))}")

    t0 = time.monotonic()
    arm_ik = G1_29_ArmIK(Unit_Test=True, Visualization=False)
    print(f"model build: {time.monotonic() - t0:.2f}s")

    # Report the masses actually loaded, so the log proves which URDF the model came from.
    # model.inertias is indexed by JOINT, not by body/frame id, and the reduced model
    # folds every locked hand link into its parent joint -- so the per-link lookup goes
    # through the FULL model's frames, and the totals are the unambiguous check.
    model = arm_ik.reduced_robot.model
    full = arm_ik.robot.model
    for frame_name in ("left_hand_palm_link", "right_hand_palm_link"):
        if full.existFrame(frame_name):
            frame = full.frames[full.getFrameId(frame_name)]
            jid = getattr(frame, "parentJoint", None)
            if jid is None:
                jid = frame.parent
            print(f"model inertia: {frame_name:24s} parent-joint mass = "
                  f"{full.inertias[jid].mass:.8f} kg")
    full_total = sum(full.inertias[i].mass for i in range(1, full.njoints))
    red_total = sum(model.inertias[i].mass for i in range(1, model.njoints))
    print(f"model inertia: full-model total mass    = {full_total:.6f} kg")
    print(f"model inertia: reduced-model total mass = {red_total:.6f} kg")

    # Register handlers again now that the module's logger certainly exists.
    for name in list(logging.root.manager.loggerDict):
        if "robot_arm_ik" in name:
            lg = logging.getLogger(name)
            if counter not in lg.handlers:
                lg.addHandler(counter)

    targets = oval_targets(args.steps, args.seed)
    times = []
    for left, right in targets:                   # warm-started: solve_ik keeps init_data
        t = time.monotonic()
        arm_ik.solve_ik(left, right)
        times.append(time.monotonic() - t)

    times = np.array(times)
    print(f"solve time: mean {times.mean() * 1000:.2f} ms  max {times.max() * 1000:.2f} ms  "
          f"min {times.min() * 1000:.2f} ms  n={len(times)}")
    print(f"non-converged solves: {counter.count} / {len(times)}")

    sol_q, sol_tauff = arm_ik.solve_ik(REACH_L.homogeneous, REACH_R.homogeneous)
    names = [model.names[i + 1] for i in range(model.nq)]
    print("fixed reach pose tauff (N*m), 14 values:")
    for i, (n, tau) in enumerate(zip(names, sol_tauff)):
        print(f"  [{i:2d}] {n:32s} {tau: 10.5f}")
    print("TAUFF_CSV," + ",".join(f"{v:.6f}" for v in sol_tauff))
    print("QSOL_CSV," + ",".join(f"{v:.6f}" for v in sol_q))
    return 0


if __name__ == "__main__":
    sys.exit(main())
