#!/usr/bin/env python3
"""G3: the wrist offset/scale knobs. Pure arithmetic -- no DDS, no robot, no network.

The two claims that matter:

  * at the DEFAULTS the output is BIT-IDENTICAL to the unmodified code -- not "equal to
    within floating point". A `*1.0` and a `+0.0` are exact for finite values, but
    `-0.0 + 0.0` is `+0.0`, so applying the operations unconditionally could still flip
    a sign of zero. apply() skips the arithmetic entirely instead, and this asserts it
    against a byte comparison of the raw buffers.

  * running the knobs one step LATER than the transform (in the launcher, because
    televuer is an upstream submodule) gives exactly what inserting them INSIDE the
    transform would have. Asserted against a local reimplementation of
    transform_IPunitree_Brobot_world_arm_to_head_then_waist().

Run:  python tools/overnight/test_g3_wrist_offset.py
"""

import os
import re
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO))

from teleop.robot_control import wrist_offset  # noqa: E402

results = []


def record(name, ok, detail=""):
    results.append((name, ok, detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  -- {detail}" if detail else ""))


def se3(R, t):
    m = np.eye(4)
    m[:3, :3] = R
    m[:3, 3] = t
    return m


def rand_pose(rng):
    # a random but valid rotation, via QR of a gaussian matrix
    q, r = np.linalg.qr(rng.normal(size=(3, 3)))
    q = q @ np.diag(np.sign(np.diag(r)))
    if np.linalg.det(q) < 0:
        q[:, 0] *= -1
    return se3(q, rng.uniform(-0.6, 0.6, 3))


# --------------------------------------------------------------------- reference impl
def yaw_only(R):
    """Reimplementation of tv_wrapper.get_Brobot_world_head_yaw_rot (tv_wrapper.py:89-101)."""
    x = R[:, 0].copy()
    x[2] = 0.0
    n = np.linalg.norm(x)
    if not np.isfinite(n) or np.isclose(n, 0.0, atol=1e-6):
        return np.eye(3)
    x /= n
    z = np.array([0.0, 0.0, 1.0])
    y = np.cross(z, x)
    y /= np.linalg.norm(y)
    return np.column_stack([x, y, z])


def transform_reference(arm, head, mode, mapping=None):
    """tv_wrapper.transform_IPunitree_Brobot_world_arm_to_head_then_waist (:103-120),
    with the G3 knobs applied where they BELONG -- between the head-relative step and
    the head->waist translation."""
    head_arm = arm.copy()
    if mode == "head_yaw":
        R = yaw_only(head[:3, :3])
        head_arm[:3, :3] = R.T @ arm[:3, :3]
        head_arm[:3, 3] = R.T @ (arm[:3, 3] - head[:3, 3])
    else:
        head_arm[:3, 3] = arm[:3, 3] - head[:3, 3]

    if mapping is not None and not mapping.is_identity:
        head_arm[0, 3] = head_arm[0, 3] * mapping.xy_scale + mapping.x_offset
        head_arm[1, 3] = head_arm[1, 3] * mapping.xy_scale
        head_arm[2, 3] = head_arm[2, 3] * mapping.z_scale + mapping.z_offset

    waist = head_arm.copy()
    waist[0, 3] += 0.15
    waist[2, 3] += 0.45
    return waist


# ======================================================================= 1
print("\n=== (1) the mirrored constants still match tv_wrapper.py ===")
tvw = (REPO / "teleop/televuer/src/televuer/tv_wrapper.py").read_text()
m_x = re.search(r"IPunitree_Brobot_waist_arm\[0, 3\] \+= ([\d.]+)", tvw)
m_z = re.search(r"IPunitree_Brobot_waist_arm\[2, 3\] \+= ([\d.]+)", tvw)
record("tv_wrapper head->waist x matches HEAD_TO_WAIST_X",
       bool(m_x) and float(m_x.group(1)) == wrist_offset.HEAD_TO_WAIST_X,
       f"tv_wrapper={m_x.group(1) if m_x else '?'} module={wrist_offset.HEAD_TO_WAIST_X}")
record("tv_wrapper head->waist z matches HEAD_TO_WAIST_Z",
       bool(m_z) and float(m_z.group(1)) == wrist_offset.HEAD_TO_WAIST_Z,
       f"tv_wrapper={m_z.group(1) if m_z else '?'} module={wrist_offset.HEAD_TO_WAIST_Z}")

# ======================================================================= 2
print("\n=== (2) defaults are BIT-identical, on a saved fixture ===")
# In the repo tree, not logs/, so it ships in the patch and the "saved fixture"
# really is saved rather than regenerated from a seed each run.
FIXTURE = REPO / "tools/overnight/fixtures/g3_wrist_poses.npz"
rng = np.random.RandomState(20260909)
if FIXTURE.exists():
    data = np.load(FIXTURE)
    poses = data["poses"]
    print(f"  (loaded {len(poses)} poses from {FIXTURE.relative_to(REPO)})")
else:
    poses = np.stack([rand_pose(rng) for _ in range(200)])
    # include the awkward cases explicitly
    poses = np.concatenate([poses, np.stack([
        se3(np.eye(3), [0.0, 0.0, 0.0]),
        se3(np.eye(3), [-0.0, -0.0, -0.0]),          # negative zeros
        se3(np.eye(3), [wrist_offset.HEAD_TO_WAIST_X, 0.0, wrist_offset.HEAD_TO_WAIST_Z]),
        se3(np.eye(3), [1e-18, -1e-18, 1e-18]),      # subnormal-ish
        se3(np.eye(3), [1e6, -1e6, 1e6]),            # far out of range
    ])])
    FIXTURE.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(FIXTURE, poses=poses)
    print(f"  (created {FIXTURE.relative_to(REPO)} with {len(poses)} poses)")

for key in ("XR_WRIST_Z_OFFSET", "XR_WRIST_X_OFFSET", "XR_WRIST_XY_SCALE", "XR_WRIST_Z_SCALE"):
    os.environ.pop(key, None)
ident = wrist_offset.from_env()
record("from_env() with nothing set is the identity", ident.is_identity, ident.describe())

bit_identical = True
same_object = True
for pose in poses:
    out = ident.apply(pose)
    if out.tobytes() != pose.tobytes():
        bit_identical = False
    if out is not pose:
        same_object = False
record("identity apply() is bit-identical on every fixture pose", bit_identical,
       f"{len(poses)} poses, byte-for-byte")
record("identity apply() returns the very same object (no copy at all)", same_object)

# explicit negative-zero case, the one a `+ 0.0` would break
neg = se3(np.eye(3), [-0.0, -0.0, -0.0])
out = ident.apply(neg)
record("negative zeros survive the identity path",
       np.signbit(out[0, 3]) and np.signbit(out[1, 3]) and np.signbit(out[2, 3]),
       f"got {out[0,3]}, {out[1,3]}, {out[2,3]}")
# and demonstrate that the naive implementation would NOT have
naive = neg.copy(); naive[0, 3] = naive[0, 3] * 1.0 + 0.0
record("...which a naive `*1.0 + 0.0` would have destroyed (control)",
       not np.signbit(naive[0, 3]), f"naive gives {naive[0,3]}")

# ======================================================================= 3
print("\n=== (3) offset 0.15 and scale 1.2 shift and scale as specified ===")
mp = wrist_offset.WristMapping(z_offset=0.15, xy_scale=1.2, z_scale=1.2)
X, Z = wrist_offset.HEAD_TO_WAIST_X, wrist_offset.HEAD_TO_WAIST_Z

# a wrist exactly at the head origin: scaling about the head must not move it,
# only the offset should.
at_head = se3(np.eye(3), [X, 0.0, Z])
out = mp.apply(at_head)
record("a target AT the head origin only moves by the offset",
       np.allclose(out[:3, 3], [X, 0.0, Z + 0.15]),
       f"{np.round(out[:3,3], 6).tolist()}")

# a wrist 0.30 below and 0.20 in front of the head
head_rel = np.array([0.20, -0.10, -0.30])
pose = se3(np.eye(3), [head_rel[0] + X, head_rel[1], head_rel[2] + Z])
out = mp.apply(pose)
want = np.array([head_rel[0] * 1.2 + X, head_rel[1] * 1.2, head_rel[2] * 1.2 + 0.15 + Z])
record("scale is about the HEAD origin, offset applied after",
       np.allclose(out[:3, 3], want),
       f"got {np.round(out[:3,3],6).tolist()} want {np.round(want,6).tolist()}")

record("rotation is never touched", np.allclose(out[:3, :3], pose[:3, :3]))
record("the input is not mutated", np.allclose(pose[:3, 3],
                                               [head_rel[0] + X, head_rel[1], head_rel[2] + Z]))

# z offset alone raises the target by exactly that much
only_z = wrist_offset.WristMapping(z_offset=0.12)
p0 = se3(np.eye(3), [0.30, 0.20, 0.05])
out = only_z.apply(p0)
record("XR_WRIST_Z_OFFSET=0.12 raises z by exactly 0.12 and changes nothing else",
       np.allclose(out[:3, 3], [0.30, 0.20, 0.17]) and np.allclose(out[:3, :3], np.eye(3)),
       f"{np.round(out[:3,3],6).tolist()}")

# ======================================================================= 4
print("\n=== (4) applying LATE == applying INSIDE the transform ===")
rng = np.random.RandomState(7)
for label, mapping in (("defaults", wrist_offset.WristMapping(0.0, 0.0, 1.0, 1.0)),
                       ("offset 0.15 / scale 1.2", mp),
                       ("z-offset only 0.12", only_z),
                       ("xy 0.8 / z 1.4 / x +0.05", wrist_offset.WristMapping(0.0, 0.05, 0.8, 1.4))):
    worst = 0.0
    for mode in ("head_yaw", "head_position"):
        for _ in range(150):
            arm, head = rand_pose(rng), rand_pose(rng)
            inside = transform_reference(arm, head, mode, mapping)
            late = mapping.apply(transform_reference(arm, head, mode, None))
            worst = max(worst, float(np.max(np.abs(inside - late))))
    record(f"late == inside, {label}", worst < 1e-12, f"max abs diff {worst:.2e}")

# ======================================================================= 5
print("\n=== (5) bad values are refused, not silently accepted ===")
for name, kwargs in (("xy_scale 0", dict(xy_scale=0.0)),
                     ("xy_scale negative", dict(xy_scale=-1.0)),
                     ("z_scale 0", dict(z_scale=0.0)),
                     ("z_scale negative", dict(z_scale=-0.5))):
    try:
        wrist_offset.WristMapping(**kwargs)
        record(f"{name} refused", False, "it was ACCEPTED")
    except ValueError as exc:
        record(f"{name} refused", True, str(exc)[:70])

for name, value in (("not a number", "high"), ("nan", "nan"), ("inf", "inf")):
    os.environ["XR_WRIST_Z_OFFSET"] = value
    try:
        wrist_offset.from_env()
        record(f"XR_WRIST_Z_OFFSET={value!r} refused", False, "it was ACCEPTED")
    except ValueError as exc:
        record(f"XR_WRIST_Z_OFFSET={value!r} refused", True, str(exc)[:60])
os.environ.pop("XR_WRIST_Z_OFFSET", None)

# env plumbing actually works
os.environ["XR_WRIST_Z_OFFSET"] = "0.2"
os.environ["XR_WRIST_Z_SCALE"] = "1.5"
m2 = wrist_offset.from_env()
record("env vars are actually read", m2.z_offset == 0.2 and m2.z_scale == 1.5
       and m2.xy_scale == 1.0 and not m2.is_identity, m2.describe())
os.environ.pop("XR_WRIST_Z_OFFSET"); os.environ.pop("XR_WRIST_Z_SCALE")

# ======================================================================= summary
print("\n=== summary ===")
n_fail = sum(1 for _, ok, _ in results if not ok)
for name, ok, detail in results:
    print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"  -- {detail}" if detail else ""))
print(f"\n{len(results) - n_fail}/{len(results)} passed")
sys.exit(1 if n_fail else 0)
