# Episode format

What `--record` writes, with shapes taken from two real episodes recorded on
2026-09-09 (`logs/overnight/example_episode_arm_only/`,
`logs/overnight/example_episode_inspire_ftp/`, 5 s each at 30 Hz).

Written by [`teleop/utils/episode_writer.py`](../teleop/utils/episode_writer.py); the
per-item dictionaries are assembled in the launcher's record block.

---

## On disk

```
<task-dir>/<task-name>/
└── episode_0000/
    ├── data.json      the whole episode: metadata + one entry per timestep
    ├── colors/        <idx>_color_<n>.jpg, one per camera per timestep
    ├── depths/        <idx>_depth_<n>.jpg
    └── audios/        audio_<idx>_<mic>.npy
```

`episode_NNNN` is zero-padded to 4 digits and numbered from the highest existing
directory in `<task-dir>/<task-name>`, so recording into a directory that already has
episodes continues the sequence.

The image directories are created even when nothing is written to them. **An episode
recorded with no image server has empty `colors/`, `depths/` and `audios/` and is still
a valid, useful episode** — the joint data is what matters offline.

## `data.json`

```
{ "info": {...}, "text": {...}, "data": [ {...}, {...}, ... ] }
```

It is streamed: `info` and `text` are written when the episode is created, each item is
appended as it is processed, and `save_episode()` closes the array. **A `data.json`
that does not end in `]\n}` is from a session that was killed rather than stopped
with `[s]`, and `json.load` will fail on it.**

### `info`

| key | type | meaning |
|---|---|---|
| `version` | str | `"1.0.0"` |
| `date` | str | `YYYY-MM-DD` |
| `author` | str | `"unitree"` |
| `image` / `depth` | dict | `width`, `height`, `fps` — from `--frequency` and the writer's `image_size` |
| `audio` | dict | `sample_rate` 16000, `channels` 1, `format` `"PCM"`, `bits` 16 |
| `joint_names` | dict | `left_arm`, `right_arm`, `left_ee`, `right_ee`, `body` — **[panthera] populated; upstream left these empty** |
| `source` | dict | **[panthera] new** — see below |
| `tactile_names` | dict | `left_ee`, `right_ee` — empty; the Inspire touch topics are not recorded yet (`docs/inspire_rh56e2.md`) |
| `sim_state` | str | `""` unless `--sim` |

#### `info.source` — [panthera]

```json
"source": {
    "arm": "G1_29",
    "ee": null,
    "input_mode": "hand",
    "sim": true,
    "end_effector": {"present": false, "name": null, "dof_per_side": 0}
}
```

This exists because `--record` is independent of `--ee`, and **arms-only recording is a
supported and now routine mode**. Without it, an arms-only episode (empty `left_ee.qpos`)
is indistinguishable on disk from an episode where the hand block silently failed to
record. `end_effector.present` is the flag that separates them.

For `--ee inspire_ftp` the same block reads `"ee": "inspire_ftp"`,
`"present": true`, `"dof_per_side": 6`, and `joint_names.left_ee` is
`["little", "ring", "middle", "index", "thumb bend", "thumb rotation"]`.

### `text`

`goal`, `desc`, `steps` — straight from `--task-goal`, `--task-desc`, `--task-steps`.

### `data[i]` — one timestep

| key | type | notes |
|---|---|---|
| `idx` | int | 0-based, contiguous |
| `colors` | dict | camera key → **relative path** (`"colors/000042_color_0.jpg"`). `{}` when there is no image server. |
| `depths` | dict | same shape |
| `states` | dict | measured, see below |
| `actions` | dict | commanded, see below |
| `tactiles` | null | not written yet |
| `audios` | null | not written yet |
| `sim_state` | null / dict | only under `--sim` with the sim running |

`states` and `actions` have identical structure:

```
{"left_arm": {"qpos": [...], "qvel": [], "torque": []},
 "right_arm": {...}, "left_ee": {...}, "right_ee": {...},
 "body": {"qpos": [...]}}
```

**`qvel` and `torque` are always empty.** Nothing populates them.

#### Measured shapes

| field | arms-only | `--ee inspire_ftp` |
|---|---|---|
| `states.left_arm.qpos` | **7** | 7 |
| `states.right_arm.qpos` | **7** | 7 |
| `actions.left_arm.qpos` | **7** | 7 |
| `states.left_ee.qpos` | **0** (`[]`) | **6** |
| `actions.left_ee.qpos` | **0** (`[]`) | **6** |
| `body.qpos` | 0 | 0 |
| items in 5 s at 30 Hz | **150** | **148** |

Arm width is `len(current_lr_arm_q) // 2`, so it follows the robot: 7 for G1_29 and
R1_A7, 4 or 5 for H1 / G1_23 / R1_A5.

End-effector width by `--ee`: `dex5` 20, `dex3` 7, `inspire_ftp` / `inspire_dfx` /
`brainco` 6, `dex1` / `dex1_internal` 1, none 0.

`body.qpos` is non-empty only for `--ee dex1`/`dex1_internal`/`brainco` with
`--input-mode controller` and `--motion`, where it carries the whole-body motor vector
and the locomotion command.

#### states vs actions

`states` is what the robot reported (`get_current_dual_arm_q()`); `actions` is what was
commanded (`sol_q`, the IK solution). They are **not** expected to match: `actions[i]`
is the target, `states[i+k]` is where the arm got to. In the arms-only example the
states sit at 0.15 rad (the fake lowstate) while the actions hover near zero — because
with no XR data televuer substitutes `CONST_LEFT_ARM_POSE` and the IK solves for that.

**[panthera] `actions` is the IK output, before the start-pose sequencer.** With
`XR_START_POSE` set, the joints actually commanded during the approach and blend are
*not* what `actions` records. Anything training on `actions` should discard the first
`XR_START_T + XR_BLEND_T` seconds of an episode.

## Reading one

```python
import json, numpy as np
d = json.load(open("episode_0000/data.json"))
present = d["info"]["source"]["end_effector"]["present"]     # [panthera]
q  = np.array([i["states"]["left_arm"]["qpos"] for i in d["data"]])   # (T, 7)
a  = np.array([i["actions"]["left_arm"]["qpos"] for i in d["data"]])  # (T, 7)
ee = (np.array([i["states"]["left_ee"]["qpos"] for i in d["data"]])   # (T, 6) or absent
      if present else None)
```

## Two failure modes to know about

1. **`data.json` with `"data": []` after a successful-looking save.** Fixed here, and
   worth recognising if you see an old episode like it. `ImageClient` returns a frame
   *object* whose `.bgr` can be `None`; the launcher only checked the object. An empty
   array reached `cv2.imwrite`, which raised, and `EpisodeWriter.process_queue` dropped
   **the whole item** — joint data included. Recording with no image server produced
   150 items, lost all 150, and logged "Episode saved successfully". Now an unwritable
   image costs that image only, and the item is kept.

2. **A truncated `data.json`.** Killing the launcher instead of pressing `[s]` then
   `[q]` leaves the JSON array unclosed. There is no recovery path in the code; append
   `]}` by hand if the items matter.
