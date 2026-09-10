# The XR frame chain: Quest world → head → G1 IK base

Every constant on the path from a Quest hand pose to an IK target, with the file and
line it lives on. Written 2026-09-09 against HEAD `99c4680`; line numbers are against
that commit.

The short version, for the vertical clipping:

> The IK base sits **0.45 m below the head**. The G1's shoulder sits **0.2918 m above
> the IK base**, i.e. only **0.158 m below its own head**. An adult operator's shoulder
> is roughly **0.25–0.30 m below** their headset. The mapping is head-relative and
> **unscaled**, so the operator's hands are handed to the robot about **0.10–0.14 m
> lower, relative to the shoulder, than they were on the human** — straight toward the
> bottom of the reachable band. See §6.

---

## 1. The two bases

| convention | axes | used by |
|---|---|---|
| **OpenXR** (`Bxr`) | y up, z back, x right | everything Vuer hands us |
| **Robot** (`Brobot`) | z up, y left, x front | pinocchio, the URDF, the IK |

Change of basis is a similarity transform, `Brobot = T · Bxr · T⁻¹`:

| constant | file:line |
|---|---|
| `T_ROBOT_OPENXR` | [tv_wrapper.py:137-140](../teleop/televuer/src/televuer/tv_wrapper.py#L137-L140) |
| `T_OPENXR_ROBOT` (its inverse) | [tv_wrapper.py:142-145](../teleop/televuer/src/televuer/tv_wrapper.py#L142-L145) |
| `R_ROBOT_OPENXR`, `R_OPENXR_ROBOT` (3×3 forms, hand rotations only) | [tv_wrapper.py:147-153](../teleop/televuer/src/televuer/tv_wrapper.py#L147-L153) |

## 2. Initial-pose conventions (which way is the wrist "unrotated")

OpenXR's wrist frame and Unitree's URDF wrist frame differ by a 90° roll, opposite
signs per side:

| constant | rotation | file:line |
|---|---|---|
| `T_TO_UNITREE_HUMANOID_LEFT_ARM` | Rx(+90°) | [tv_wrapper.py:122-125](../teleop/televuer/src/televuer/tv_wrapper.py#L122-L125) |
| `T_TO_UNITREE_HUMANOID_RIGHT_ARM` | Rx(−90°) | [tv_wrapper.py:127-130](../teleop/televuer/src/televuer/tv_wrapper.py#L127-L130) |
| `T_TO_UNITREE_HAND` (the 25 landmarks, not the wrist) | permutation | [tv_wrapper.py:132-135](../teleop/televuer/src/televuer/tv_wrapper.py#L132-L135) |

Applied **only when the arm pose is valid** — an invalid pose is right-multiplied by
`eye(4)` instead ([tv_wrapper.py:332-333](../teleop/televuer/src/televuer/tv_wrapper.py#L332-L333)).

Controller input skips this step entirely: controller poses already arrive in the
Unitree convention ([tv_wrapper.py:418-420](../teleop/televuer/src/televuer/tv_wrapper.py#L418-L420)).

## 3. World → head → waist: the step that matters

One function, four call sites:
[`transform_IPunitree_Brobot_world_arm_to_head_then_waist`](../teleop/televuer/src/televuer/tv_wrapper.py#L103-L120),
`tv_wrapper.py:103-120`.

```
wrist pose in the Quest's WORLD frame (Robot basis, Unitree initial pose)
  │
  │  ── head-relative step ────────────────────────────────────  line
  │  arm_reference_mode == "head_yaw"   (the ONLY mode reachable from the CLI)
  │      R    = yaw-only part of the head rotation                 89-101
  │      rot   ← Rᵀ · rot                                          109
  │      trans ← Rᵀ · (trans − head_trans)                         110
  │  arm_reference_mode == "head_position"
  │      trans ← trans − head_trans      (no rotation at all)      113
  │
  ▼   ← the wrist mapping knobs act HERE (§5)
  │
  │  ── head → waist ──────────────────────────────────────────
  │      trans.x += 0.15      IK base is 0.15 m BEHIND the head    117
  │      trans.z += 0.45      IK base is 0.45 m BELOW  the head    118
  ▼
wrist target in the IK base ("waist") frame → arm_ik.solve_ik()
```

The launcher hard-codes `arm_reference_mode="head_yaw"`
([teleop_hand_and_arm.py:312](../teleop/teleop_hand_and_arm.py#L312)); `"head_position"`
is unreachable without editing the source.

**`head_yaw` discards head pitch and roll.** Only the yaw component of the head
rotation is used ([tv_wrapper.py:89-101](../teleop/televuer/src/televuer/tv_wrapper.py#L89-L101)),
so looking *down* does not move the targets down — it only changes what the operator
sees. This is deliberate and it is also why "just look where you want to reach" does
not help with the vertical problem.

### Every constant in the chain

| constant | value | meaning | file:line |
|---|---|---|---|
| head→waist x | **+0.15 m** | IK base is 0.15 m behind the head | tv_wrapper.py:117 |
| head→waist z | **+0.45 m** | IK base is 0.45 m below the head | tv_wrapper.py:118 |
| `CONST_HEAD_POSE` | y=1.5, z=−0.2 (OpenXR) | head pose substituted when tracking is invalid | tv_wrapper.py:155-158 |
| `CONST_LEFT_ARM_POSE` | (−0.15, 1.13, −0.30) | **left wrist substituted when tracking is invalid** | tv_wrapper.py:166-169 |
| `CONST_RIGHT_ARM_POSE` | (+0.15, 1.13, −0.30) | right wrist, same | tv_wrapper.py:161-164 |
| `CONST_HAND_ROT` | 25× identity | hand rotations when invalid | tv_wrapper.py:171 |
| `scale_arms` human/robot | 0.60 / 0.75 → **1.25** | **DEAD CODE — the call is commented out** | robot_arm_ik.py:245, 258 |
| `L_ee` / `R_ee` frame offset | +0.05 m along wrist x | the IK's actual target point, 5 cm past the wrist yaw joint | robot_arm_ik.py:81-95 |

**The fallback poses are not a curiosity.** `safe_mat_update`
([tv_wrapper.py:70-75](../teleop/televuer/src/televuer/tv_wrapper.py#L70-L75)) substitutes
`CONST_*_ARM_POSE` whenever the incoming matrix is singular — which is what a *silent*
XR path looks like. That is why, on 2026-09-09 with Vuer unable to bind 8012, pressing
`r` moved the arms to a fixed "default pose": it was these constants, fed through the
transform above. G1's exit 4 exists so that cannot happen again.

**`scale_arms` is disabled.** [robot_arm_ik.py:258](../teleop/robot_control/robot_arm_ik.py#L258)
reads `# left_wrist, right_wrist = self.scale_arms(left_wrist, right_wrist)`. So today
a human's hand travel maps onto the robot **1:1, with no scaling whatsoever**, even
though the robot's arm is about 25% longer than the 0.60 m human arm the function
assumes.

## 4. Where the G1's shoulder actually is

Measured from `assets/g1/g1_body29_hand14.urdf` via the same reduced model the IK
builds (15 leg/waist joints + 14 finger joints locked), at `q = 0`:

| point | position in the IK base frame |
|---|---|
| `left_shoulder_pitch_joint` | `( 0.0000, +0.1002, +0.2918)` |
| `right_shoulder_pitch_joint` | `( 0.0000, −0.1002, +0.2918)` |
| `left_elbow_joint` | `(+0.0158, +0.1468, +0.1052)` |
| `L_ee` at q=0 | `(+0.2498, +0.1487, +0.0952)` |
| `R_ee` at q=0 | `(+0.2498, −0.1486, +0.0952)` |

Reproduce with `tools/reach_map.py`'s `build_model()`.

So, stacking §3 and §4: **head → shoulder is −0.45 + 0.2918 = −0.158 m.** The G1's
shoulder is 15.8 cm below its own head. Shoulder half-width is 0.1002 m.

## 5. The wrist mapping knobs (G3)

[`teleop/robot_control/wrist_offset.py`](../teleop/robot_control/wrist_offset.py), applied
in the launcher between `get_tele_data()` and `solve_ik()`
([teleop_hand_and_arm.py:571-580](../teleop/teleop_hand_and_arm.py#L571-L580)).

| env var | default | meaning |
|---|---|---|
| `XR_WRIST_Z_OFFSET` | `0.0` | metres added to wrist z. **Positive = the robot reaches higher for the same operator pose.** |
| `XR_WRIST_X_OFFSET` | `0.0` | metres added to wrist x (forward) |
| `XR_WRIST_XY_SCALE` | `1.0` | scales x and y **about the head origin** |
| `XR_WRIST_Z_SCALE` | `1.0` | scales z about the head origin |

Order inside `apply()`: recover head-relative coordinates (subtract the two §3
constants) → **scale** → **add offsets** → restore the waist frame. The offset is
applied *after* the scale on purpose: it is a fixed correction for where the operator's
shoulders are relative to the robot's, and multiplying it by the reach scale would make
one knob change the meaning of the other.

At all four defaults `apply()` returns its input **unchanged, without doing any
arithmetic** — so the behaviour is bit-identical to the unmodified code, not merely
equal to within floating point.

**Why it is not inside `transform_...to_head_then_waist()`, where it belongs:**
`teleop/televuer` is an upstream **git submodule** (unitreerobotics/televuer, pinned at
`766de45`). A change there cannot ship in a parent-repo patch, and
`git submodule update` would silently revert it. Running one step later on the same
quantity is exactly equivalent, and `tools/overnight/test_g3_wrist_offset.py` proves it
against a reimplementation of the transform.

## 6. Why up/down clips, in numbers

Take an operator standing with hands at belly height:

| | operator | G1 |
|---|---|---|
| shoulder below head | ~0.25–0.30 m | **0.158 m** |
| hand below head, arms relaxed forward | ~0.45–0.55 m | — |

The mapping is head-relative and unscaled, so a hand 0.50 m below the operator's head
becomes a target 0.50 m below the robot's head, i.e. `z = −0.50 + 0.45 = −0.05` in the
IK base frame. Relative to the *shoulder* — the thing that actually limits reach — that
is `−0.05 − 0.2918 = −0.34 m`, on an arm whose maximum shoulder-to-`L_ee` distance is
**0.4587 m** (measured, §"The measured band" below). At x = 0.30 that leaves nothing:
√(0.30² + 0.34²) = 0.45 m of the 0.4587 m budget before any lateral offset at all, so
the arm is at full stretch pointing down and forward before the operator has asked for
anything unusual.

Raising the operator's hands by 0.20 m moves the target to `z = +0.15`, comfortably
inside the band — which is exactly the "it works if I hold my hands up awkwardly high"
behaviour observed on the robot.

### The measured band

`tools/reach_map.py` swept 11 × 19 × 23 wrist targets per arm (x 0.10–0.60, y ±0.45,
z −0.40–0.70, 5 cm, wrist pointing forward). **At x = 0.30 m, both arms reach**

> **z = +0.05 … +0.55 m**, span 0.50 m, centre **+0.30 m**

and the band is identical for position-only targets, so it is the arm's reach that
binds, not the orientation. Nothing at all is reachable beyond x = 0.45 m; max
`L_ee`-to-shoulder distance is **0.4587 m** (20 000-sample forward-kinematics check).

Now map the operator onto it, with the offset at its default of 0:

| operator posture | hand, head-relative | target z | in the band? |
|---|---|---|---|
| hands at belly, elbows relaxed | −0.50 | **−0.05** | **NO — below the floor** |
| hands at chest / ready pose | −0.35 | +0.10 | yes, but 5 cm off the floor |
| hands at shoulder height | −0.25 | +0.20 | yes |
| hands at head height | 0.00 | +0.45 | yes |

So it is not that targets land "near the bottom" of the reach — **the natural resting
posture lands outside it entirely**, and the whole usable range is squeezed into the
top half of where the operator can comfortably put their hands. That is the vertical
clipping, exactly.

(The hand-height figures are anthropometric estimates and are the one soft input in
this analysis; they are listed in `tools/reach_plot.py:OPERATOR_HAND_Z` so they can be
replaced with measurements.)

### The recommendation

Put the chest/ready posture at the centre of the band:

```
XR_WRIST_Z_OFFSET = +0.30 − (+0.10) = +0.20
```

**Start at `XR_WRIST_Z_OFFSET=0.20`** and adjust by feel. With it, "hands at belly"
maps to +0.15 — inside the band with 10 cm to spare — and the whole comfortable
operator range fits.

Increase `XR_WRIST_Z_SCALE` only if the *range* is still too small once the offset is
right. Offset and scale fix different complaints:

* "everything is too low / I have to hold my hands up" → **offset**
* "I run out of travel before the robot does" → **scale** (the band is 0.50 m tall and
  the operator's comfortable vertical travel is roughly 0.50 m, so scale 1.0 is
  already about right — try the offset first and probably leave the scale alone)

Data: `logs/overnight/reach_map.npz`, figures `logs/overnight/reach_map_*.png`, the
arithmetic above in `logs/overnight/G3_reach_analysis.txt`. All four are copied to
`~/partb_share/`.

## 7. Things that will trip you up

1. **`head_yaw` ignores head pitch.** Looking down does not lower the targets (§3).
2. **The IK target is `L_ee`, not the wrist joint** — 5 cm further along the wrist's
   x-axis (`robot_arm_ik.py:81-95`). Every number here is about `L_ee`.
3. **An invalid XR pose silently becomes a fixed pose**, not an error (§3).
4. **`scale_arms` looks like it is scaling and is not** (§3).
5. **The 25 hand landmarks take a different path** — they are expressed relative to the
   *arm* frame, not the head, and never see the waist offset
   ([tv_wrapper.py:362-374](../teleop/televuer/src/televuer/tv_wrapper.py#L362-L374)).
   The wrist knobs in §5 therefore do not affect finger retargeting at all, which is
   what you want.
