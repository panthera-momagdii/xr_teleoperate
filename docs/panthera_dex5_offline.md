# panthera/dex5-1p — offline preparation

Branch `panthera/dex5-1p` = upstream `xr_teleoperate@845b25a` + PR #321 (`39b79bc`,
"[feat] Add Unitree Dex5-1 support", meiander) + eight Panthera changes.

Built and verified 2026-08-27 on the host laptop with **no robot, no headset, no PC2 and
no DDS domain 0**. Everything below was measured on this machine unless marked *pending*.

---

## 1. What the branch contains, gate by gate

| gate | change | files |
|---|---|---|
| G1 | Cherry-pick of PR #321, two conflicts resolved by hand | `teleop/teleop_hand_and_arm.py` + 47 |
| G2 | `hand_config.py` — one place for every hardware-dependent fact | `teleop/robot_control/hand_config.py` |
| G3 | `Dex5_1_Controller` reads `hand_config` and **fails closed** | `robot_hand_unitree.py`, `tools/fake_hand_state.py`, `tools/tool_logging.py`, `tools/test_dex5_failclosed.py` |
| G4 | `XR_ARM_VEL_LIMIT`; `--ee dex5 --sim` refused | `teleop/teleop_hand_and_arm.py` |
| G5 | IK URDF hand mass 0.6965 → 1.10 kg per side | `assets/g1/g1_body29_hand14.urdf`, `tools/ik_smoke.py` |
| G6 | Operator tools: read-only probe, interlocked step test | `tools/hand_probe.py`, `tools/hand_step.py` |
| G7 | Retarget round trip + recorder/DDS shape tests | `tools/test_dex5_retarget_roundtrip.py`, `tools/test_dex5_recorder_shapes.py` |
| G8 | This document and the pending-contract stub | `docs/` |

### The two G1 conflict resolutions

| hunk | resolution |
|---|---|
| `--ee` choices | union of both sides: `['dex1', 'dex1_internal', 'dex3', 'dex5', 'inspire_ftp', 'inspire_dfx', 'brainco']`. `dex1_internal` landed upstream after the PR branched. |
| controller-input guard | HEAD's structure with `"dex5"` added. The PR's duplicated `xr_motion_data_ready = Value('b', False, lock=True)` was **dropped** — HEAD already defines it once at line 150, and re-defining it would shadow the value passed to every other controller. |

---

## 2. Every number measured in this session

### Environment
| item | value |
|---|---|
| python / numpy / pinocchio / casadi | 3.10.21 / 1.26.4 / 3.1.0 / 3.6.7 |
| nlopt / cyclonedds | 2.7.1 / **0.10.2** (matches the robot's advertised version) |
| `unitree_sdk2_python` | `65691c8`, `404fe44` is an ancestor ✓ |
| submodules | televuer `766de45`, teleimager `57cf2a4`, dex-retargeting `d7753d3` |
| rerun-sdk | 0.20.1 (installed cleanly, no fallback needed) |

### IK, before and after the hand-mass change (`tools/ik_smoke.py`, 200 seeded steps)
| quantity | baseline (Dex3 mass) | post (Dex5-1P mass) |
|---|---|---|
| full-model total mass | 30.580234 kg | 31.387142 kg (+0.806908 = 2 × 0.40345) |
| palm parent-joint mass, per side | 0.45741501 kg | 0.86086902 kg |
| non-converged solves | 0 / 200 | 0 / 200 |
| solve time mean / max | 1.60 / 41.62 ms | 1.53 / 19.12 ms |
| `max abs(q_post − q_base)` | — | **0.000000 rad** (identical pose) |

Gravity feed-forward at a fixed reach pose (N·m), the reason the change matters:

| joint | baseline | post | delta |
|---|---|---|---|
| left_shoulder_pitch | −5.25620 | −6.60533 | **+1.34913** |
| left_elbow | −3.23122 | −4.30628 | **+1.07507** |
| right_shoulder_pitch | −5.08206 | −6.39088 | **+1.30883** |
| right_elbow | −3.08893 | −4.12405 | **+1.03512** |

The trajectory is seeded, so the two runs converge to the identical pose and the torque
delta is attributable to the added mass alone.

### Fail-closed behaviour (`tools/test_dex5_failclosed.py`, domain 1)
| case | result |
|---|---|
| no publisher | RuntimeError after **6.46 s** (timeout 6.0 s), names both state topics + 3 causes |
| 7-motor stream | RuntimeError after **0.46 s**: `left: motor_state has 7 entries; 7 = Dex3-1 fitted, expected 20 (Dex5-1P)` |
| 20-motor stream | `Subscribe dds ok.` + `motor_state=20 press_sensor_state=12` on both sides |

### Operator tools (domain 1, against `tools/fake_hand_state.py`)
| check | result |
|---|---|
| `grep -n ChannelPublisher tools/hand_probe.py` | **no output** — the probe cannot publish |
| probe vs a 20-motor hand | `n_motor=20 n_press=12` at 99.11 Hz, 94 non-zero tactile slots |
| probe vs a 7-motor hand | reports `n_motor=7`, exit 0 (the probe reports; only the controller refuses) |
| probe on silence | **exit 3** |
| `hand_step --dry-run` without the env var | exit 0, prints the messages, never imports the publisher class |
| `hand_step` without either | exit 2, "REFUSING: this tool commands a real hand" |
| 0.3 rad step on slot 2 | time-to-90 % **0.485 s**, overshoot **+0.0 %**, neighbour slot 3 range **0.0000 rad** |
| over-temperature (50 °C > 45 °C) | exit 4, CSV records the triggering sample |

### Retargeting round trip (`tools/test_dex5_retarget_roundtrip.py`)
| configuration | max fingertip error | joint residual (max) | limits |
|---|---|---|---|
| open (both hands) | **0.0 mm** | 0.197 rad | all inside |
| pinch (left) | **0.0 mm** | 0.160 rad | all inside |
| pinch (right) | **0.0 mm** | 0.314 rad | all inside |
| fist (both) | 16.8 mm | 1.55 / 1.59 rad | inside (one joint on the retargeter's own 1 mrad relaxed bound) |

Pinch closes thumb-tip↔index-tip from **203.2 mm → 39.3 mm** (left) / **35.6 mm** (right)
with the other three fingers at exactly 0.0000 rad. `left/right_dex_retargeting_to_hardware`
is the **identity** on both hands.

### Recorder and DDS shapes (`tools/test_dex5_recorder_shapes.py`)
`states/actions.{left_ee,right_ee}.qpos` = **20** wide on all three frames, `tactiles`
present, `[:20]/[-20:]` split preserved. `HandCmd_` round-tripped on domain 1 arrives with
**20** `motor_cmd` entries (the SDK factory allocates 7), q/kp/kd intact.

---

## 3. Robot-day run commands (from `teleop/`)

Set the interface first — DDS discovery is per interface, and getting it wrong looks
exactly like unpowered hands:

```bash
export NIC=<the laptop NIC on the robot LAN>     # PENDING: not known yet
source /home/momagdii/Desktop/pantheraaa/dex/env.sh
cd $REPO/teleop
```

**Arms only** (do this first, every session):

```bash
XR_ARM_VEL_LIMIT=5 python teleop_hand_and_arm.py \
    --arm G1_29 --ee dex1 \
    --network-interface "$NIC" \
    --img-server-ip 192.168.123.164
```

**Arms + Dex5-1P hands:**

```bash
XR_ARM_VEL_LIMIT=5 DEX5_TOPIC_PREFIX=rt/dex3 python teleop_hand_and_arm.py \
    --arm G1_29 --ee dex5 \
    --network-interface "$NIC" \
    --img-server-ip 192.168.123.164
```

Never pass `--motion`; it is not used in this project. `--ee dex5 --sim` is refused by
design (the simulator ships a Dex3 hand only).

### Two safety facts about startup

1. **`--sim` does not make startup safe.** `teleop_hand_and_arm.py:149` branches on
   `args.motion`, **not** `args.sim`, so `MotionSwitcher().Enter_Debug_Mode()` runs in
   both cases. `--sim` only changes the DDS domain to 1. Do not run the launcher past
   argument validation on a robot-visible interface unless you intend debug mode.
2. **The launcher swallows startup exceptions and still exits 0.** A wrapper script
   cannot use its exit code to tell a failed start from a good one. Read the log.

---

## 4. Operator sequence for the hand tools

Run these **before** the launcher, in this order. Every tool defaults to `--domain 1`
so a mistyped command cannot reach the robot; `--domain 0` is deliberate.

```bash
cd $REPO

# 1. Is anything there at all? Read-only, cannot move the hands.
python tools/hand_probe.py --domain 0 --iface "$NIC" --seconds 60
#    exit 0 -> both sides streamed; exit 3 -> silence (the 2026-08-24 symptom)
#    Read logs/probe_<ts>.json: n_motor, n_press, per-module nonzero_slots,
#    per-motor q/dq/tau/temperature ranges, and the 4k+3 vs 4k+2 coupling hint.

# 2. If n_motor is 7, a Dex3-1 is fitted. STOP -- do not run --ee dex5.
# 3. If the topics are rt/dex5/* instead, re-run with DEX5_TOPIC_PREFIX=rt/dex5.

# 4. Confirm the refusal paths before trusting them.
cd teleop && python ../tools/test_dex5_failclosed.py --case dex5 --domain 0 --iface "$NIC"

# 5. Dry run the step test and read what it would send.
cd $REPO && python tools/hand_step.py --side left --joint 2 --dry-run

# 6. Only then, the real step. One joint, small, short.
PANTHERA_HAND_CMD_OK=1 python tools/hand_step.py \
    --domain 0 --iface "$NIC" --side left --joint 2 --amplitude 0.3 --hold 5
#    Read the "H2:" line -> did slot 3 move while only slot 2 was commanded?
#    Read time-to-90% and overshoot -> the gain-unit question.
#    Repeat for joints 6, 10, 14, then a thumb slot (16) to compare stiffness.
```

`hand_step.py` exit codes: 0 ok · 2 args/interlock · 3 no state · 4 thermal abort ·
5 motor-count mismatch.

---

## 5. Still unknown until the hands stream

All of these live in `docs/g1_contract_dex5_pending.yaml`, each marked `pending`, with
the tool that answers it and where the answer goes.

| unknown | expected | how we learn it | lands in |
|---|---|---|---|
| `n_motor` | 20 (7 = Dex3-1) | `hand_probe.py` → `sides.*.n_motor` | `hand_config.NUM_JOINTS_EXPECTED` |
| `n_press` + tactile index map | 12 modules × 12 slots | `hand_probe.py` → `press_modules[].nonzero_slots` | `g1_contract.yaml` |
| passive slots 3/7/11/15 | coupled to 2/6/10/14? | `hand_step.py` "H2:" line | `g1_contract.yaml` |
| gain units | thumb N·m vs finger mNm | `hand_step.py` time-to-90 % / overshoot | `hand_config.GAINS` |
| topic prefix | `rt/dex3` today | topic listing on domain 0 | `hand_config.TOPIC_PREFIX` |
| hand power | silent on 2026-08-24 | `hand_probe.py` exit 3 vs 0 | — |
| state rate | 100 Hz | `hand_probe.py` → `rate_hz` | `g1_contract.yaml` |
| PC2 image server | reachable at `192.168.123.164`? | launcher `--img-server-ip` | — |
| laptop NIC name | — | `ip -br link` on the robot LAN | run commands above |

---

## 6. Things that will bite you, learned the hard way here

| trap | what happens | what to do |
|---|---|---|
| `~/.local` user-site | 674 packages shadow the conda env; `numpy` resolves to a foreign build — an ABI hazard for pinocchio | always `source env.sh` (`PYTHONNOUSERSITE=1`, `unset PYTHONPATH`); never bare `conda activate tv` |
| bare `ChannelSubscriber.Read()` | blocks **forever** on a silent topic; `Read(timeout=x)` spams `[Reader] take sample error` | subscribe with `Init(handler)` |
| `logging_mp` | defaults to WARNING, hiding the `n_motor`/`n_press` evidence; `basicConfig()` raises if any `getLogger()` already ran | configure it before importing `hand_config` |
| `g1_29_model_cache.pkl` | pickles the inertias, so a URDF mass edit is invisible | delete it; `ik_smoke.py` refuses to start if one exists |
| `retarget()` called once | measures the LPFilter (alpha 0.2) plus warm-start state, not the mapping | iterate to convergence in tests; the controller is fine because it runs at 100 Hz |
| `os._exit()` | skips stdout flushing; report tails vanish | flush explicitly first |
| `MotorCmd_.q/kp/kd` | float32 on the wire: `0.10` → `0.10000000149011612` | never compare gains at float64 precision |
| `--help` on the launcher | `vuer`/`params_proto` intercept it at import and `SystemExit(0)` before argparse runs | use `--ee bogus` to see the real choice list |
| `MotorState_.temperature` | `array[int16, 2]`, not a scalar | `hand_config.motor_temperature()` returns the hotter of the two |
