"""[RT6f] Fail-fast start gate for the arms: joint limits, and "arms were moved".

Pure functions plus one bounded DDS read. Runs right after DDS init, before
Enter_Debug_Mode and before the arm controller exists, so a refusal commands nothing.

The problem
-----------
RT6e (22 Oct 2026, 02:00) found the arms limp but not where RT6c had released them:
left shoulder yaw at +2.604 rad, 0.014 rad from its +2.618 rad limit, wrist rolls at
-1.65 and +1.54 rad. Someone had handled the robot (a battery swap). The launcher would
have recorded that pose as the rest pose and, at [q], lowered the arms back into a
joint stop. Nothing checked.

What this does
--------------
1. REFUSE (exit 7) if any of the 14 arm joints is within XR_GATE_LIMIT_MARGIN (rad,
   default 0.05) of a URDF limit, naming each joint, its angle, the limit and the
   distance. Such a pose is never recorded as the rest pose.
2. WARN if any arm joint differs from the last released pose by more than
   XR_GATE_MOVED_TOL (rad, default 0.2): "arms were moved since the last session".
   The last released pose is what the launcher measured when lower_and_release()
   finished; it is kept in XR_LAST_POSE_FILE (start-pose yaml plus `released_at`).
   No file, or XR_LAST_POSE_FILE unset: the warning check is skipped, and says so.

3. [RT6e, 22 Oct, owner] WRIST exception: a wrist joint (roll, pitch, yaw) inside the
   margin passes when the start pose (XR_START_POSE) is clear of its limits and lies on
   the far side, so the start sequence moves the wrist AWAY from the stop, AND a rest
   pose (XR_REST_POSE) is set, so the lowering does not take it back there. Shoulders
   and elbows: no exception. Found on the robot: limp wrist rolls end up at a stop
   (left +1.969 rad, 0.003 rad from +1.972) and hand adjustment did not hold.

Limits come from the URDF the IK uses, read as XML (no pinocchio, no IK build).
"""

import datetime
import os
import time
import xml.etree.ElementTree as ET

import numpy as np

from teleop.robot_control.start_pose import ARM_JOINT_NAMES

ARM_MOTORS = tuple(range(15, 29))     # G1_29_JointArmIndex, same order as ARM_JOINT_NAMES
WRIST = tuple(k for k, n in enumerate(ARM_JOINT_NAMES) if "_wrist_" in n)   # roll, pitch, yaw x 2
DEFAULT_MARGIN = 0.05
DEFAULT_MOVED_TOL = 0.2
URDF = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                    "assets", "g1", "g1_body29_hand14.urdf")


class StartGateError(RuntimeError):
    """A pose the launcher must not start from (or record as the rest pose)."""


def _env_float(name, default):
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return default
    try:
        v = float(raw)
    except ValueError:
        raise ValueError(f"{name}={raw!r} is not a number") from None
    if not np.isfinite(v) or v < 0:
        raise ValueError(f"{name}={raw!r} must be a finite number >= 0")
    return v


def settings_from_env():
    """(margin, moved_tol, last_pose_path or None). Raises ValueError on a bad value."""
    path = os.environ.get("XR_LAST_POSE_FILE", "").strip() or None
    return (_env_float("XR_GATE_LIMIT_MARGIN", DEFAULT_MARGIN),
            _env_float("XR_GATE_MOVED_TOL", DEFAULT_MOVED_TOL), path)


def urdf_arm_limits(urdf_path=URDF):
    """(lower, upper) arrays for the 14 arm joints, in ARM_JOINT_NAMES order."""
    root = ET.parse(urdf_path).getroot()
    lim = {}
    for j in root.iter("joint"):
        name = j.get("name")
        if name in ARM_JOINT_NAMES:
            el = j.find("limit")
            lim[name] = (float(el.get("lower")), float(el.get("upper")))
    missing = [n for n in ARM_JOINT_NAMES if n not in lim]
    if missing:
        raise StartGateError(f"{urdf_path}: no limit for {missing}")
    return (np.array([lim[n][0] for n in ARM_JOINT_NAMES]), np.array([lim[n][1] for n in ARM_JOINT_NAMES]))


def limit_violations(q, lower, upper, margin=DEFAULT_MARGIN):
    """Joints within `margin` of (or past) a limit, as dicts, closest first."""
    q = np.asarray(q, dtype=float)
    out = []
    for k, name in enumerate(ARM_JOINT_NAMES):
        d_lo, d_hi = q[k] - lower[k], upper[k] - q[k]
        side, lim, d = ("lower", lower[k], d_lo) if d_lo < d_hi else ("upper", upper[k], d_hi)
        if d < margin:
            out.append(dict(joint=name, q=float(q[k]), side=side, limit=float(lim), margin=float(d)))
    return sorted(out, key=lambda v: v["margin"])


def describe_violations(viol, margin):
    lines = [f"[gate] REFUSING TO START (exit 7): {len(viol)} arm joint(s) within {margin:g} rad of a limit"]
    for v in viol:
        where = "PAST" if v["margin"] < 0 else f"{v['margin']:.3f} rad from"
        lines.append(f"    {v['joint']} at {v['q']:+.3f} rad is {where} its {v['side']} limit {v['limit']:+.3f} rad")
    lines.append("    The arms were probably moved by hand. Move them clear of the limits (someone at the "
                 "robot), then start again. Nothing was commanded.")
    return "\n".join(lines)


def moved_joints(q, q_last, tol=DEFAULT_MOVED_TOL):
    """[(joint, delta)] for joints that differ from q_last by more than tol, largest first."""
    d = np.asarray(q, dtype=float) - np.asarray(q_last, dtype=float)
    out = [(ARM_JOINT_NAMES[k], float(d[k])) for k in range(len(ARM_JOINT_NAMES)) if abs(d[k]) > tol]
    return sorted(out, key=lambda x: -abs(x[1]))


def describe_moved(moved, tol, released_at):
    lines = [f"[gate] WARNING: arms were moved since the last session (released {released_at}): "
             f"{len(moved)} joint(s) differ by more than {tol:g} rad"]
    lines += [f"    {n} moved {dlt:+.3f} rad" for n, dlt in moved]
    return "\n".join(lines)


def load_last(path):
    """(q (14,), released_at str) or None if there is no file."""
    if not path or not os.path.exists(path):
        return None
    import yaml
    with open(path) as fh:
        doc = yaml.safe_load(fh)
    q = np.array([float(doc["joints"][n]) for n in ARM_JOINT_NAMES])
    return q, str(doc.get("released_at", "unknown time"))


def save_last(path, q, note=""):
    """Write the released pose (start-pose yaml + released_at), atomically."""
    import yaml
    doc = {"name": "last_release", "robot": "G1_29", "units": "radians",
           "released_at": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
           "note": note, "joints": {n: round(float(v), 5) for n, v in zip(ARM_JOINT_NAMES, q)}}
    tmp = f"{path}.tmp{os.getpid()}"
    with open(tmp, "w") as fh:
        yaml.safe_dump(doc, fh, sort_keys=False)
    os.replace(tmp, path)


def read_arm_q(timeout_s=5.0, settle_s=0.3):
    """Mean arm q from rt/lowstate over `settle_s`, waiting up to `timeout_s` for the
    first sample. ChannelFactoryInitialize must already have run. A reader only."""
    from unitree_sdk2py.core.channel import ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    got = []

    def _cb(m):
        got.append([m.motor_state[i].q for i in ARM_MOTORS])
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(_cb, 10)
    try:
        t_end = time.monotonic() + timeout_s
        while not got and time.monotonic() < t_end:
            time.sleep(0.02)
        if not got:
            return None
        time.sleep(settle_s)
        return np.mean(np.array(got, dtype=float), axis=0)
    finally:
        try:
            sub.Close()
        except Exception:
            pass


def wrist_exempt(v, q_start, lower, upper, margin):
    """True if violation `v` is a wrist joint that the start pose moves away from its stop,
    to a target itself clear of both limits by `margin`."""
    k = ARM_JOINT_NAMES.index(v["joint"])
    if k not in WRIST or q_start is None:
        return False
    tgt = float(q_start[k])
    if not (lower[k] + margin <= tgt <= upper[k] - margin):
        return False
    return tgt < v["q"] if v["side"] == "upper" else tgt > v["q"]


def check(q, lower, upper, margin, moved_tol, last_path, q_start=None, rest_pose_set=False):
    """Pure decision. Returns (refuse_text or None, notices[list of str]).
    q_start: the start pose (14,) if XR_START_POSE is set; rest_pose_set: XR_REST_POSE is set."""
    notices = []
    viol = limit_violations(q, lower, upper, margin)
    exempt = [v for v in viol if wrist_exempt(v, q_start, lower, upper, margin)]
    if exempt and not rest_pose_set:
        notices.append("[gate] a wrist at its stop is allowed only with XR_REST_POSE set (otherwise the "
                       "lowering at [q] would take it back to the stop)")
        exempt = []
    for v in exempt:
        k = ARM_JOINT_NAMES.index(v["joint"])
        notices.append(f"[gate] {v['joint']} at {v['q']:+.3f} rad is {v['margin']:.3f} rad from its {v['side']} "
                       f"limit: ALLOWED (wrist): the start pose moves it to {float(q_start[k]):+.3f} rad, away from "
                       f"the stop, and the lowering goes to XR_REST_POSE")
    viol = [v for v in viol if v not in exempt]
    last = load_last(last_path) if last_path else None
    if last is not None:
        moved = moved_joints(q, last[0], moved_tol)
        if moved:
            notices.append(describe_moved(moved, moved_tol, last[1]))
        else:
            notices.append(f"[gate] arms within {moved_tol:g} rad of the last released pose ({last[1]})")
    elif last_path:
        notices.append(f"[gate] no last released pose on file yet ({os.path.basename(last_path)}): moved check skipped")
    else:
        notices.append("[gate] XR_LAST_POSE_FILE not set: moved check off")
    refuse = describe_violations(viol, margin) if viol else None
    if not viol:
        others = [k for k in range(len(q)) if ARM_JOINT_NAMES[k] not in {v["joint"] for v in exempt}]
        closest = min(min(q[k] - lower[k], upper[k] - q[k]) for k in others)
        notices.insert(0, f"[gate] arm joints clear of their limits (closest {closest:.3f} rad, margin {margin:g})"
                       + (f"; {len(exempt)} wrist joint(s) allowed, see below" if exempt else ""))
    return refuse, notices
