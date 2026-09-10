"""Fixed start pose: on [r], go to a known pose first, then blend into the operator.

Pure state machine plus a YAML loader. No DDS, no robot, no threads, no clock of its
own -- every method takes the time it should act on, so the whole thing is testable
without waiting for anything.

The problem
-----------
Today there is no start pose at all. The first command after [r] is the IK of whatever
the operator's wrists happened to be doing on that frame
(`teleop_hand_and_arm.py`, `solve_ik` -> `ctrl_dual_arm`), so the arms jump from
wherever they are to wherever the operator is. Worse, if the XR path is dead, televuer
substitutes `CONST_LEFT_ARM_POSE`/`CONST_RIGHT_ARM_POSE` and the arms move to a fixed
pose with nobody driving them -- which is what happened on 2026-09-09.

With `XR_START_POSE=<yaml>` set, [r] instead runs:

    phase 1  APPROACH   0 .. XR_START_T          current q  ->  the yaml pose
    phase 2  BLEND      .. + XR_BLEND_T          the pose   ->  the operator's IK target
    phase 3  FOLLOW     thereafter               the operator's IK target, untouched

Without the env var the machine is in FOLLOW from the first frame and returns the IK
target unchanged -- behaviour is exactly what it is today.

Rate limiting
-------------
APPROACH is clamped to the same per-joint velocity limit the arm controller enforces
(`XR_ARM_VEL_LIMIT`, `robot_arm.py:167-172`), so the pose is never commanded faster
than the arm is allowed to move. If the pose is far enough away that XR_START_T is not
long enough, the approach simply takes longer -- it does NOT speed up, and it does not
give up. `is_approaching` stays true until the pose is actually reached, so BLEND never
starts from somewhere the arm has not got to.

The clamp here is the same shape as `clip_arm_q_target`: a UNIFORM scale of the whole
14-vector, not a per-joint clip, so the arm slows without changing the path it takes.
"""

import os

import numpy as np

# Canonical order: G1_29_JointArmIndex (robot_arm.py:285-302), motors 15..28. The same
# order pinocchio's reduced model uses, and the order get_current_dual_arm_q() returns.
ARM_JOINT_NAMES = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)
N_ARM_JOINTS = len(ARM_JOINT_NAMES)

DEFAULT_START_T = 3.0
DEFAULT_BLEND_T = 2.0

APPROACH, BLEND, FOLLOW = "approach", "blend", "follow"


class StartPoseError(ValueError):
    """A start-pose file that cannot be trusted. Always fatal: the alternative is
    commanding an arm from a pose file nobody has checked."""


def load_pose(path, joint_limits=None):
    """Read a start-pose YAML and return a validated (14,) float array.

    Schema -- joints are named, never a bare list, because a silently reordered
    14-vector is the exact mistake this file format exists to prevent:

        name: ready
        joints:
          left_shoulder_pitch_joint: 0.1234
          ...                                 # all 14, no more, no fewer

    `joint_limits` is an optional (lower, upper) pair of (14,) arrays; when given, a
    pose outside them is refused rather than clamped.
    """
    import yaml

    try:
        with open(path) as fh:
            doc = yaml.safe_load(fh)
    except OSError as exc:
        raise StartPoseError(f"cannot read start pose {path}: {exc}") from None
    except yaml.YAMLError as exc:
        raise StartPoseError(f"{path} is not valid YAML: {exc}") from None

    if not isinstance(doc, dict) or "joints" not in doc:
        raise StartPoseError(f"{path}: expected a mapping with a 'joints' key")
    joints = doc["joints"]
    if not isinstance(joints, dict):
        raise StartPoseError(f"{path}: 'joints' must be a mapping of name -> radians")

    missing = [n for n in ARM_JOINT_NAMES if n not in joints]
    extra = [n for n in joints if n not in ARM_JOINT_NAMES]
    if missing or extra:
        raise StartPoseError(
            f"{path}: joints must be exactly the {N_ARM_JOINTS} arm joints. "
            + (f"missing {missing}. " if missing else "")
            + (f"unexpected {extra}." if extra else ""))

    q = np.empty(N_ARM_JOINTS, dtype=float)
    for i, name in enumerate(ARM_JOINT_NAMES):
        try:
            q[i] = float(joints[name])
        except (TypeError, ValueError):
            raise StartPoseError(f"{path}: {name}={joints[name]!r} is not a number") from None
    if not np.all(np.isfinite(q)):
        bad = [ARM_JOINT_NAMES[i] for i in np.flatnonzero(~np.isfinite(q))]
        raise StartPoseError(f"{path}: non-finite values for {bad}")

    if joint_limits is not None:
        lower, upper = (np.asarray(x, dtype=float) for x in joint_limits)
        out = np.flatnonzero((q < lower) | (q > upper))
        if out.size:
            detail = ", ".join(
                f"{ARM_JOINT_NAMES[i]}={q[i]:+.4f} not in [{lower[i]:+.4f},{upper[i]:+.4f}]"
                for i in out)
            raise StartPoseError(f"{path}: outside the URDF joint limits: {detail}")
    return q


def _clamp_step(current, target, max_step):
    """One rate-limited step from `current` toward `target`.

    Uniform scaling of the whole delta -- the same shape as
    `G1_29_ArmController.clip_arm_q_target` (robot_arm.py:167-172) -- so the arm slows
    down without bending differently on the way.
    """
    delta = target - current
    biggest = float(np.max(np.abs(delta))) if delta.size else 0.0
    if biggest <= max_step or biggest == 0.0:
        return target.copy()
    return current + delta * (max_step / biggest)


class StartPoseSequencer:
    """APPROACH -> BLEND -> FOLLOW. Deterministic; the caller supplies the time."""

    def __init__(self, pose=None, start_t=None, blend_t=None,
                 velocity_limit=30.0, reach_tol=1e-3):
        self.pose = None if pose is None else np.asarray(pose, dtype=float).copy()
        self.start_t = DEFAULT_START_T if start_t is None else float(start_t)
        self.blend_t = DEFAULT_BLEND_T if blend_t is None else float(blend_t)
        self.velocity_limit = float(velocity_limit)
        self.reach_tol = float(reach_tol)
        if self.start_t < 0 or self.blend_t < 0:
            raise StartPoseError("XR_START_T and XR_BLEND_T must be >= 0")
        if self.velocity_limit <= 0:
            raise StartPoseError("velocity_limit must be > 0")
        self._reset_state()

    def _reset_state(self):
        self.phase = FOLLOW if self.pose is None else APPROACH
        self.t0 = None
        self.last_cmd = None
        self._blend_started_at = None
        self.blend_weight = 1.0 if self.pose is None else 0.0

    @property
    def enabled(self):
        return self.pose is not None

    def start(self, t_now, current_q):
        """Call on every [r]. Restarts the sequence from where the arms are NOW."""
        self._reset_state()
        self.t0 = float(t_now)
        self.last_cmd = np.asarray(current_q, dtype=float).copy()
        return self

    def step(self, t_now, current_q, ik_target, dt):
        """Return the joint vector to command this cycle.

        `dt` is the control period; the rate limit is `velocity_limit * dt` per cycle,
        which is exactly what the arm controller will allow through.
        """
        ik_target = np.asarray(ik_target, dtype=float)
        if not self.enabled:
            return ik_target
        if self.t0 is None:                      # step() before start(): behave as today
            return ik_target
        if self.phase == FOLLOW:
            self.blend_weight = 1.0
            return ik_target

        elapsed = float(t_now) - self.t0
        max_step = self.velocity_limit * float(dt)

        if self.phase == APPROACH:
            self.last_cmd = _clamp_step(self.last_cmd, self.pose, max_step)
            reached = bool(np.all(np.abs(self.last_cmd - self.pose) <= self.reach_tol))
            # BOTH conditions: the clock alone is not enough, because a pose further
            # away than XR_START_T allows at the velocity limit would otherwise start
            # blending from somewhere the arm has not reached.
            if reached and elapsed >= self.start_t:
                self.phase = BLEND
                self._blend_started_at = float(t_now)
            self.blend_weight = 0.0
            return self.last_cmd.copy()

        # BLEND
        if self.blend_t <= 0.0:
            self.phase = FOLLOW
            self.blend_weight = 1.0
            return ik_target
        w = (float(t_now) - self._blend_started_at) / self.blend_t
        w = min(max(w, 0.0), 1.0)
        self.blend_weight = w
        out = (1.0 - w) * self.pose + w * ik_target
        # Still rate-limited: the operator's target can be a long way from the pose,
        # and a blend is not a licence to move faster than the arm may move.
        out = _clamp_step(self.last_cmd, out, max_step)
        self.last_cmd = out.copy()
        if w >= 1.0:
            self.phase = FOLLOW
        return out.copy()

    def describe(self):
        if not self.enabled:
            return "[start-pose] disabled (set XR_START_POSE=<yaml> to enable)"
        return (f"[start-pose] {N_ARM_JOINTS} joints loaded; approach {self.start_t:g}s "
                f"then blend {self.blend_t:g}s, rate-limited to "
                f"{self.velocity_limit:g} rad/s")


def from_env(joint_limits=None, velocity_limit=30.0):
    """Build a sequencer from XR_START_POSE / XR_START_T / XR_BLEND_T.

    Returns a disabled sequencer (pure pass-through) when XR_START_POSE is unset.
    """
    path = os.environ.get("XR_START_POSE")
    pose = None
    if path:
        pose = load_pose(path, joint_limits=joint_limits)

    def _t(name, default):
        raw = os.environ.get(name)
        if raw is None or raw.strip() == "":
            return default
        try:
            value = float(raw)
        except ValueError:
            raise StartPoseError(f"{name}={raw!r} is not a number") from None
        if not np.isfinite(value) or value < 0:
            raise StartPoseError(f"{name}={raw!r} must be a finite number >= 0")
        return value

    return StartPoseSequencer(pose=pose,
                              start_t=_t("XR_START_T", DEFAULT_START_T),
                              blend_t=_t("XR_BLEND_T", DEFAULT_BLEND_T),
                              velocity_limit=velocity_limit)
