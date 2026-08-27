#!/usr/bin/env python3
"""Evidence for the Dex5 command clamp + slew limiter. DOMAIN 1 ONLY.

Run with cwd = teleop/ (HandRetargeting's non-unit-test paths), with
tools/fake_hand_state.py --n 20 --echo-cmd already publishing:

    cd teleop && python ../tools/test_dex5_clamp_slew.py --domain 1

Three parts, because one of them cannot answer the question on its own:

  A. limit table      the clamp's bounds vs a direct Pinocchio read of the two URDFs
  B. end-to-end       the real Dex5_1_Controller driven over DDS with a synthetic
                      landmark sequence -- 1 s open, a single-frame jump to a fist,
                      then hold -- with every command captured off the cmd topic
  C. limiter directly _limit_command() stepped from open to fist with a SYNTHETIC
                      target step

C exists because SeqRetargeting applies an LPFilter (low_pass_alpha 0.2) to its own
output, so the *target* reaching the controller already ramps over ~20 cycles. That is
fine for B's pass criteria, but it means B alone cannot demonstrate what
DEX5_MAX_STEP_RAD does: with the limiter disabled the command still ramps, because the
target does. C removes the filter from the picture and steps the limiter directly.

Exit codes: 0 all checks pass, 1 a check failed.
"""

import argparse
import csv
import os
import sys
import time

import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(REPO_ROOT)

import logging_mp  # noqa: E402
try:
    # Must precede any getLogger(), which importing hand_config does. Guarded because
    # this module is also imported for its landmark helpers by other tools that may
    # have configured logging already -- logging_mp raises rather than no-opping.
    logging_mp.basicConfig(level=logging_mp.INFO)
except RuntimeError:
    pass

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber  # noqa: E402
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_  # noqa: E402

from teleop.robot_control import hand_config  # noqa: E402
from tools.test_dex5_retarget_roundtrip import (  # noqa: E402  -- reuse, don't duplicate
    load_model, fk_points, build_landmarks, make_configs,
)

# MotorCmd_.q is float32 on the wire, so a value the controller clamped exactly to a
# bound in float64 comes back a few 1e-8 past it. Measured here: worst step excess
# 7.15e-08 rad, worst limit excess 3.85e-08 rad; float32 spacing at 1.7 rad is 1.19e-07.
# 1e-6 rad is ~10x that spacing and four orders of magnitude below anything mechanical
# (1e-6 rad at a 60 mm fingertip is 60 nanometres).
WIRE_TOL_RAD = 1e-6

FINGER_SLOTS = {"index": (0, 4), "middle": (4, 8), "ring": (8, 12),
                "pinky": (12, 16), "thumb": (16, 20)}


def landmarks_for(config_name, side="left"):
    """(25,3) landmark array for a named joint configuration, pre-rotated.

    Pre-rotated by R.T so the controller's own `landmarks @ rotation.T` cancels it
    (both DEX5 rotations are symmetric involutions: R.T == R and R @ R == I).
    """
    model, data, suffix, _urdf = load_model(side)
    configs, _idx, _lo, _hi = make_configs(model, suffix)
    tips = fk_points(model, data, configs[config_name], suffix)
    from teleop.robot_control.hand_retargeting import (
        DEX5_LANDMARK_ROTATION_LEFT, DEX5_LANDMARK_ROTATION_RIGHT)
    rotation = DEX5_LANDMARK_ROTATION_LEFT if side == "left" else DEX5_LANDMARK_ROTATION_RIGHT
    return build_landmarks(tips) @ rotation.T


def check_limit_table(ctrl):
    """A. the clamp's bounds are the URDF's, shrunk by exactly the margin."""
    import pinocchio as pin
    print("=" * 78)
    print("A. limit table vs a direct Pinocchio read of the two URDFs")
    print("=" * 78)
    margin = hand_config.DEX5_LIMIT_MARGIN_RAD
    ok = True
    for side in ("left", "right"):
        suffix = "L" if side == "left" else "R"
        urdf = os.path.join(REPO_ROOT, "assets", "unitree_hand_Dex5", f"Dex5-URDF-{suffix}.urdf")
        m = pin.buildModelFromUrdf(urdf)
        api = getattr(ctrl.hand_retargeting, f"{side}_dex5_api_joint_names")
        idx = {m.names[i]: m.joints[m.getJointId(m.names[i])].idx_q for i in range(1, m.njoints)}
        urdf_lo = np.array([m.lowerPositionLimit[idx[n]] for n in api])
        urdf_hi = np.array([m.upperPositionLimit[idx[n]] for n in api])
        same = (np.allclose(ctrl.q_lower[side], urdf_lo + margin)
                and np.allclose(ctrl.q_upper[side], urdf_hi - margin))
        ok = ok and same
        print(f"  {side:5s}: clamp bounds == URDF limits +/- {margin} rad: {same}")
        print(f"         widest joint  {api[int(np.argmax(urdf_hi - urdf_lo))]:10s} "
              f"travel {float(np.max(urdf_hi - urdf_lo)):.4f} rad")
        print(f"         narrowest     {api[int(np.argmin(urdf_hi - urdf_lo))]:10s} "
              f"travel {float(np.min(urdf_hi - urdf_lo)):.4f} rad")
    import hashlib
    blob = b"".join(np.ascontiguousarray(a, dtype="<f8").tobytes()
                    for side in ("left", "right")
                    for a in (ctrl.q_lower[side], ctrl.q_upper[side]))
    digest = hashlib.sha256(blob).hexdigest()[:16]
    print(f"  limit table sha256[:16] = {digest}   (2 sides x 20 joints x lower/upper)")
    return ok, digest


def capture_mode(out_path, seconds, domain, iface):
    """Subscribe to the left cmd topic and dump (t, q[20]) to JSON. Separate process.

    It has to be a separate PROCESS, not a thread or a forked child of the controller's
    parent. Dex5_1_Controller creates its DDS writers in __init__ and then writes them
    from a forked control_process. Measured on this build: a subscriber living in the
    parent of that fork receives NOTHING from the child, while an independent process
    receives every sample (200/200). That is an intra-participant quirk of writing a
    pre-fork writer from a child, not a delivery failure -- on the robot the hand
    firmware is a separate machine and gets the commands. But an observer inside the
    parent measures zero and looks exactly like a broken controller.
    """
    import json
    ChannelFactoryInitialize(domain, iface)
    rows = []
    sub = ChannelSubscriber(hand_config.TOPIC_LEFT_CMD, HandCmd_)
    sub.Init(lambda msg: rows.append([time.monotonic(), [float(c.q) for c in msg.motor_cmd]]))
    # Readiness handshake. This module imports pinocchio and (via dex_retargeting) torch,
    # so interpreter start-up is several seconds -- far longer than any sleep the parent
    # would guess. The parent waits for this marker instead of guessing, or it runs the
    # whole open->jump->hold sequence before the subscriber exists and captures nothing.
    with open(out_path + ".ready", "w") as fh:
        fh.write(str(time.monotonic()))
    deadline = time.monotonic() + seconds
    stop_path = out_path + ".stop"
    while time.monotonic() < deadline and not os.path.exists(stop_path):
        time.sleep(0.02)
    with open(out_path, "w") as fh:
        json.dump(rows, fh)
    return 0


def start_capture(args):
    """Launch the out-of-process cmd capture BEFORE this process initialises DDS.

    subprocess.Popen from a process that already holds a cyclonedds participant produces
    a child that never receives anything on this build -- measured 0 samples, while the
    identical command launched from a shell with no DDS in its parent receives ~930 over
    the same window. The exec'd child appears to inherit the parent's DDS sockets. So the
    capture goes up first, and only then does this process touch DDS.
    """
    import subprocess
    cap_path = os.path.join(args.csv_dir, "clamp_slew_capture.json")
    for stale in (cap_path, cap_path + ".ready", cap_path + ".stop"):
        if os.path.exists(stale):
            os.remove(stale)
    cap_err = os.path.join(args.csv_dir, "clamp_slew_capture.stderr")
    capture = subprocess.Popen(
        [sys.executable, os.path.abspath(__file__), "--capture-cmd", cap_path,
         "--capture-seconds", str(args.capture_max_seconds), "--domain", str(args.domain)]
        + (["--iface", args.iface] if args.iface else []),
        stdout=subprocess.DEVNULL, stderr=open(cap_err, "w"))
    wait_until = time.monotonic() + 120.0
    while not os.path.exists(cap_path + ".ready"):
        if capture.poll() is not None:
            raise RuntimeError(f"capture process exited early (rc={capture.returncode}); "
                               f"see {cap_err}")
        if time.monotonic() > wait_until:
            capture.kill()
            raise RuntimeError("capture process never signalled ready within 120 s")
        time.sleep(0.1)
    print("capture subscriber ready (out-of-process, started before this process "
          "touched DDS)")
    return capture, cap_path


def check_end_to_end(ctrl_arrays, args, capture_handle):
    """B. drive the real controller over DDS and capture what it actually sent."""
    import json
    left_in, right_in = ctrl_arrays
    print()
    print("=" * 78)
    print("B. end-to-end: open -> single-frame jump to fist -> hold")
    print("=" * 78)

    capture, cap_path = capture_handle

    fist_lm = landmarks_for("fist")
    time.sleep(args.open_seconds)                      # arrays already hold the open pose

    jump_t = time.monotonic()
    with left_in.get_lock():                      # ONE write: a single-frame jump
        left_in[:] = fist_lm.flatten()
    with right_in.get_lock():
        right_in[:] = fist_lm.flatten()
    time.sleep(args.hold_seconds)
    open(cap_path + ".stop", "w").close()
    capture.wait(timeout=30)
    with open(cap_path) as fh:
        received = [(r[0], r[1]) for r in json.load(fh)]

    after = [(t, q) for t, q in received if t >= jump_t]
    if len(after) < 20:
        print(f"  FAIL: only {len(after)} commands captured after the jump "
              f"({len(received)} total)")
        return False, {}
    qs = np.array([q for _t, q in after])
    steps = np.abs(np.diff(qs, axis=0))
    max_step = float(steps.max()) if steps.size else 0.0
    worst_joint = int(np.unravel_index(np.argmax(steps), steps.shape)[1]) if steps.size else -1

    final = qs[-1]
    reached = None
    for i, q in enumerate(qs):
        if np.all(np.abs(q - final) <= hand_config.DEX5_MAX_STEP_RAD):
            reached = i
            break

    lo, hi = ctrl_arrays_limits["left"]
    over = np.maximum(lo - qs, qs - hi)
    outside = int(np.sum(over > WIRE_TOL_RAD))
    outside_exact = int(np.sum(over > 0))

    rate = len(received) / (received[-1][0] - received[0][0]) if len(received) > 1 else 0.0
    print(f"  commands captured: {len(received)} total, {len(qs)} after the jump "
          f"({rate:.1f} Hz)")
    excess = max_step - hand_config.DEX5_MAX_STEP_RAD
    print(f"  max per-cycle step observed: {max_step!r} rad (slot {worst_joint}) "
          f"vs DEX5_MAX_STEP_RAD {hand_config.DEX5_MAX_STEP_RAD}")
    print(f"    excess over the limit: {excess:.3e} rad "
          f"(float32 wire tolerance {WIRE_TOL_RAD:g})")
    print(f"  cycles from the jump until within one step of the final command: {reached}")
    print(f"  commands past a clamped bound at all: {outside_exact}; "
          f"by more than {WIRE_TOL_RAD:g} rad: {outside}")

    step_ok = max_step <= hand_config.DEX5_MAX_STEP_RAD + WIRE_TOL_RAD
    limits_ok = outside == 0
    print(f"  every command moves <= DEX5_MAX_STEP_RAD: {step_ok}")
    print(f"  every command inside the clamped limits : {limits_ok}")
    print("  (the observing subscriber is a plain DDS reader: a DROPPED sample would show "
          "as a doubled step, so it can only make the number too LARGE -- a pass is "
          "unambiguous)")
    return (step_ok and limits_ok), {
        "max_step": max_step, "cycles": reached, "n": len(qs), "rate": rate,
        "rows": qs, "first": received[0][1] if received else None,
    }


def check_limiter_directly(ctrl, args):
    """C. step _limit_command() with a synthetic target -- no LPFilter in the way."""
    print()
    print("=" * 78)
    print("C. the limiter itself: synthetic one-frame target step, open -> fist")
    print("=" * 78)
    model, _data, suffix, _u = load_model("left")
    configs, idx, _lo, _hi = make_configs(model, suffix)
    api = ctrl.hand_retargeting.left_dex5_api_joint_names
    start = np.zeros(hand_config.NUM_JOINTS_EXPECTED)
    target = np.array([configs["fist"][idx[n]] for n in api])

    lo, hi = ctrl.q_lower["left"], ctrl.q_upper["left"]
    last = np.clip(start, lo, hi)
    rows, cycles_to_target = [], None
    for cycle in range(args.max_cycles):
        cmd = ctrl._limit_command(target, last, "left")
        step = np.abs(cmd - last)
        rows.append((cycle, cmd.copy(), step.copy()))
        last = cmd
        if cycles_to_target is None and np.allclose(cmd, np.clip(target, lo, hi), atol=1e-9):
            cycles_to_target = cycle + 1
            break

    all_steps = np.array([r[2] for r in rows])
    max_step = float(all_steps.max())
    cmds = np.array([r[1] for r in rows])
    outside = int(np.sum((cmds < lo - 1e-9) | (cmds > hi + 1e-9)))
    travel = float(np.max(np.abs(np.clip(target, lo, hi) - np.clip(start, lo, hi))))

    print(f"  largest single-joint travel open->fist: {travel:.4f} rad")
    print(f"  DEX5_MAX_STEP_RAD                     : {hand_config.DEX5_MAX_STEP_RAD}")
    print(f"  theoretical minimum cycles            : {int(np.ceil(travel / hand_config.DEX5_MAX_STEP_RAD))}")
    print(f"  cycles to reach the target            : {cycles_to_target}")
    print(f"  max per-cycle step                    : {max_step:.6f} rad")
    print(f"  commands outside the clamped limits   : {outside}")
    step_ok = max_step <= hand_config.DEX5_MAX_STEP_RAD + 1e-12
    print(f"  every step <= DEX5_MAX_STEP_RAD: {step_ok}; all inside limits: {outside == 0}")
    return (step_ok and outside == 0), {"cycles": cycles_to_target, "max_step": max_step,
                                        "rows": cmds}


def write_csv(path, rows, label):
    n = hand_config.NUM_JOINTS_EXPECTED
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["cycle"] + [f"q_{i}" for i in range(n)])
        for i, q in enumerate(rows):
            w.writerow([i] + [f"{v:.6f}" for v in q])
    print(f"  {label} CSV -> {path} ({len(rows)} rows)")


def show_finger(rows, finger, n_rows=30):
    a, b = FINGER_SLOTS[finger]
    print(f"\n  first {n_rows} cycles, {finger} finger (slots {a}..{b - 1}):")
    print("    cycle " + "  ".join(f"q_{i:<8d}" for i in range(a, b)))
    for i, q in enumerate(rows[:n_rows]):
        print(f"    {i:5d} " + "  ".join(f"{q[j]:+.6f}" for j in range(a, b)))


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--domain", type=int, default=1)
    parser.add_argument("--iface", type=str, default=None)
    parser.add_argument("--open-seconds", type=float, default=1.0)
    parser.add_argument("--hold-seconds", type=float, default=2.0)
    parser.add_argument("--max-cycles", type=int, default=400)
    parser.add_argument("--csv-dir", type=str, default=os.path.join(REPO_ROOT, "logs"))
    parser.add_argument("--finger", default="index", choices=sorted(FINGER_SLOTS))
    parser.add_argument("--capture-cmd", type=str, default=None,
                        help="internal: run as the out-of-process cmd capture")
    parser.add_argument("--capture-seconds", type=float, default=10.0)
    parser.add_argument("--capture-max-seconds", type=float, default=120.0)
    parser.add_argument("--expect-start-q", type=float, default=None,
                        help="the constant q the fixture reports; enables check D")
    args = parser.parse_args()

    if args.capture_cmd:
        return capture_mode(args.capture_cmd, args.capture_seconds, args.domain, args.iface)

    print(hand_config.describe())
    os.makedirs(args.csv_dir, exist_ok=True)
    capture_handle = start_capture(args)          # BEFORE any DDS in this process
    ChannelFactoryInitialize(args.domain, args.iface)

    from multiprocessing import Array, Lock
    from teleop.robot_control.robot_hand_unitree import Dex5_1_Controller

    left_in = Array('d', 75, lock=True)
    right_in = Array('d', 75, lock=True)
    # Seed the OPEN pose before constructing. control_process retargets whatever is in
    # these arrays from its first cycle; leaving them all-zero feeds DexPilot a
    # degenerate all-zero landmark set, which is not a state the launcher ever produces
    # (televuer fills them before the controller is built).
    open_lm = landmarks_for("open")
    with left_in.get_lock():
        left_in[:] = open_lm.flatten()
    with right_in.get_lock():
        right_in[:] = open_lm.flatten()
    n = hand_config.NUM_JOINTS_EXPECTED
    ctrl = Dex5_1_Controller(left_in, right_in, Lock(),
                             Array('d', 2 * n, lock=False), Array('d', 2 * n, lock=False),
                             simulation_mode=False)

    global ctrl_arrays_limits
    ctrl_arrays_limits = {"left": (ctrl.q_lower["left"], ctrl.q_upper["left"])}

    ok_a, digest = check_limit_table(ctrl)
    ok_b, res_b = check_end_to_end((left_in, right_in), args, capture_handle)
    ok_c, res_c = check_limiter_directly(ctrl, args)

    ok_d = True
    if args.expect_start_q is not None and res_b.get("first") is not None:
        print()
        print("=" * 78)
        print("D. the first command starts from the MEASURED state, not from zero")
        print("=" * 78)
        first = np.array(res_b["first"])
        measured = np.full(hand_config.NUM_JOINTS_EXPECTED, args.expect_start_q)
        start = np.clip(measured, ctrl.q_lower["left"], ctrl.q_upper["left"])
        delta = np.abs(first - start)
        within = bool(np.all(delta <= hand_config.DEX5_MAX_STEP_RAD + WIRE_TOL_RAD))
        print(f"  fixture reports q = {args.expect_start_q} on every motor")
        print(f"  clamped start (measured, clipped to limits), slots 0..3: "
              f"{np.round(start[:4], 6).tolist()}")
        print(f"  first command captured,               slots 0..3: "
              f"{np.round(first[:4], 6).tolist()}")
        print(f"  max |first - start| = {delta.max():.6f} rad "
              f"(one step is {hand_config.DEX5_MAX_STEP_RAD})")
        print(f"  first command within one step of the measured state: {within}")
        zero_start = np.clip(np.zeros_like(measured), ctrl.q_lower["left"], ctrl.q_upper["left"])
        print(f"  for contrast, |first - ZERO-start| would be "
              f"{np.abs(first - zero_start).max():.6f} rad")
        ok_d = within

    os.makedirs(args.csv_dir, exist_ok=True)
    tag = f"step{hand_config.DEX5_MAX_STEP_RAD:g}"
    if res_b.get("rows") is not None:
        write_csv(os.path.join(args.csv_dir, f"clamp_slew_endtoend_{tag}.csv"),
                  res_b["rows"], "end-to-end")
    write_csv(os.path.join(args.csv_dir, f"clamp_slew_limiter_{tag}.csv"),
              res_c["rows"], "limiter")
    show_finger(res_c["rows"], args.finger)

    print()
    print("=" * 78)
    print(f"A limit table   : {'PASS' if ok_a else 'FAIL'}   (sha256[:16] {digest})")
    print(f"B end-to-end    : {'PASS' if ok_b else 'FAIL'}")
    print(f"C limiter       : {'PASS' if ok_c else 'FAIL'}")
    print(f"DEX5_MAX_STEP_RAD={hand_config.DEX5_MAX_STEP_RAD} "
          f"DEX5_LIMIT_MARGIN_RAD={hand_config.DEX5_LIMIT_MARGIN_RAD}")
    if args.expect_start_q is not None:
        print(f"D first command : {'PASS' if ok_d else 'FAIL'}")
    return 0 if (ok_a and ok_b and ok_c and ok_d) else 1


if __name__ == "__main__":
    rc = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(rc)
