"""Operator-to-robot wrist mapping knobs: offset and scale, about the head origin.

Pure functions and module state only. No DDS, no robot, no imports beyond numpy.

The problem this exists to solve
-------------------------------
On 2026-09-09 horizontal tracking followed well and **up/down did not**. The mapping is
head-relative, and an operator's hands sit lower under their head than the G1's do
under its own, so targets land near the bottom of the robot's reach and clip there.

Two separate things make that worse than it needs to be:

1. There is **no scaling at all** today. `G1_29_ArmIK.scale_arms` exists
   (`robot_arm_ik.py:245`, human 0.60 m / robot 0.75 m => 1.25) but the call is
   **commented out** at `robot_arm_ik.py:258`. So a human's arm travel is mapped 1:1
   onto a longer robot arm.
2. The head->waist translation is two fixed constants with no operator adjustment.

This module adds four env knobs. **All four default to the identity**, and at their
defaults the arithmetic is skipped entirely, so the output is bit-identical to the
unmodified code -- not "equal to within floating point", identical. That matters: a
`* 1.0` and a `+ 0.0` are exact for finite values, but `-0.0 + 0.0` is `+0.0`, so a
sign-of-zero difference is possible if the operations are applied unconditionally.

    XR_WRIST_Z_OFFSET   metres, added to wrist z    (positive = robot reaches HIGHER)
    XR_WRIST_X_OFFSET   metres, added to wrist x    (positive = robot reaches FURTHER FORWARD)
    XR_WRIST_XY_SCALE   float,  scales x and y about the head origin
    XR_WRIST_Z_SCALE    float,  scales z about the head origin

Where this runs, and why it is not in televuer
----------------------------------------------
The natural home is
`transform_IPunitree_Brobot_world_arm_to_head_then_waist()`
(`teleop/televuer/src/televuer/tv_wrapper.py:103-120`), between the head-relative step
and the head->waist translation. But `teleop/televuer` is an upstream **git submodule**
(unitreerobotics/televuer, pinned at 766de45): a change there cannot ship in a
parent-repo patch, and `git submodule update` would silently revert it.

So this runs one step later instead, in the launcher, on the wrist pose televuer
returns. That pose is already in the waist frame, so `apply()` subtracts the two
head->waist constants to recover head-relative coordinates, does its work there, and
adds them back. The result is exactly what inserting the code inside the transform
would have produced -- `test_g3_wrist_offset.py` asserts that against a reimplementation
of the transform, and separately asserts these constants still match tv_wrapper's.

`arm_reference_mode` does not matter here: both modes end with the same two constants,
and both leave the pose expressed in the waist frame.
"""

import os

import numpy as np

# ---------------------------------------------------------------------------
# Mirrored from teleop/televuer/src/televuer/tv_wrapper.py:117-118
#
#     IPunitree_Brobot_waist_arm[0, 3] += 0.15
#     IPunitree_Brobot_waist_arm[2, 3] += 0.45
#
# The IK base ("waist") sits 0.15 m BEHIND and 0.45 m BELOW the head. These are
# duplicated rather than imported because televuer is a submodule and this module must
# keep working if it is updated or absent. test_g3_wrist_offset.py parses tv_wrapper.py
# and asserts the numbers still agree, so the duplication cannot drift silently.
# ---------------------------------------------------------------------------
HEAD_TO_WAIST_X = 0.15
HEAD_TO_WAIST_Z = 0.45


def _env_float(name, default):
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return default
    try:
        value = float(raw)
    except ValueError:
        raise ValueError(f"{name}={raw!r} is not a number") from None
    if not np.isfinite(value):
        raise ValueError(f"{name}={raw!r} is not finite")
    return value


class WristMapping:
    """The four knobs, resolved once. Immutable after construction."""

    def __init__(self, z_offset=None, x_offset=None, xy_scale=None, z_scale=None):
        self.z_offset = _env_float("XR_WRIST_Z_OFFSET", 0.0) if z_offset is None else float(z_offset)
        self.x_offset = _env_float("XR_WRIST_X_OFFSET", 0.0) if x_offset is None else float(x_offset)
        self.xy_scale = _env_float("XR_WRIST_XY_SCALE", 1.0) if xy_scale is None else float(xy_scale)
        self.z_scale = _env_float("XR_WRIST_Z_SCALE", 1.0) if z_scale is None else float(z_scale)

        # A non-positive scale would mirror or collapse the workspace. Neither is
        # something an operator means to type, and both are dangerous on a real arm.
        for name, value in (("XR_WRIST_XY_SCALE", self.xy_scale),
                            ("XR_WRIST_Z_SCALE", self.z_scale)):
            if value <= 0.0:
                raise ValueError(f"{name}={value} must be > 0 "
                                 f"(a non-positive scale mirrors or collapses the "
                                 f"workspace)")

    @property
    def is_identity(self):
        """True when this mapping must be a no-op, down to the bit."""
        return (self.z_offset == 0.0 and self.x_offset == 0.0
                and self.xy_scale == 1.0 and self.z_scale == 1.0)

    def describe(self):
        """The single startup line."""
        if self.is_identity:
            return ("[wrist] mapping is 1:1 and unshifted (XR_WRIST_XY_SCALE=1, "
                    "XR_WRIST_Z_SCALE=1, XR_WRIST_X_OFFSET=0, XR_WRIST_Z_OFFSET=0)")
        return (f"[wrist] XY_SCALE={self.xy_scale:g} Z_SCALE={self.z_scale:g} "
                f"X_OFFSET={self.x_offset:+g}m Z_OFFSET={self.z_offset:+g}m "
                f"(scales are about the head origin; offsets are applied after)")

    def apply(self, wrist_pose):
        """Return a new 4x4 wrist pose with the mapping applied.

        `wrist_pose` is what televuer returns: an SE(3) matrix in the IK base ("waist")
        frame. Rotation is never touched -- these knobs move where the hand goes, not
        which way it points.

        At the defaults this returns an unmodified copy: the arithmetic is skipped, so
        the result is bit-identical rather than merely equal.
        """
        if self.is_identity:
            return wrist_pose

        out = np.array(wrist_pose, dtype=float, copy=True)

        # Back to head-relative coordinates, which is where "about the head origin"
        # means what it says.
        x = out[0, 3] - HEAD_TO_WAIST_X
        y = out[1, 3]
        z = out[2, 3] - HEAD_TO_WAIST_Z

        # Scale about the head, then shift. Order matters and is deliberate: the offset
        # is a fixed correction for where the operator's shoulders are relative to the
        # robot's, so it must NOT be multiplied by the reach scale.
        x = x * self.xy_scale + self.x_offset
        y = y * self.xy_scale
        z = z * self.z_scale + self.z_offset

        out[0, 3] = x + HEAD_TO_WAIST_X
        out[1, 3] = y
        out[2, 3] = z + HEAD_TO_WAIST_Z
        return out

    def apply_pair(self, left_wrist_pose, right_wrist_pose):
        """Both wrists, same mapping. Returns (left, right)."""
        return self.apply(left_wrist_pose), self.apply(right_wrist_pose)


def from_env():
    """Build a WristMapping from the environment. Raises ValueError on a bad value."""
    return WristMapping()
