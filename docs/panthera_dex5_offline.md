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
| g09 | Hand pre-flight **before** `Enter_Debug_Mode()`; shared refusal wording | `hand_config.py`, `robot_hand_unitree.py`, `teleop_hand_and_arm.py`, `tools/test_dex5_failclosed.py` |
| g10 | Exit 1 on a caught exception | `teleop/teleop_hand_and_arm.py` |
| g11 | This document's exit/state sections | `docs/` |
| g12 | Clamp + slew limit on hand targets | `hand_config.py`, `robot_hand_unitree.py`, `tools/{fake_hand_state,test_dex5_clamp_slew}.py`, `docs/` |

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

There are two independent checks. `hand_config.preflight()` runs at launcher start,
**before** `Enter_Debug_Mode()`. `Dex5_1_Controller` keeps its own identical check for
defence in depth. Both take their wording from `hand_config`, so they cannot drift.

| target | case | result |
|---|---|---|
| preflight | no publisher | RuntimeError at **4.03 s** against a 4.0 s deadline |
| preflight | 7-motor stream | RuntimeError in **0.04 s**, motor-count message |
| preflight | 20-motor stream | passes in **0.04 s**, returns `{'left': {'n_motor': 20, 'n_press': 12}, 'right': …}` |
| preflight | leftover readers | ALIVE `DCPSSubscription` endpoints on the two state topics: 0 → 1 (deliberate subscriber, positive control) → 0 → **0 after preflight** |
| controller | no publisher | RuntimeError after **6.46 s** (timeout 6.0 s), names both state topics + 3 causes |
| controller | 7-motor stream | RuntimeError after **0.46 s**: `left: motor_state has 7 entries; 7 = Dex3-1 fitted, expected 20 (Dex5-1P)` |
| controller | 20-motor stream | `Subscribe dds ok.` + `motor_state=20 press_sensor_state=12` on both sides |

**Why the controller refuses ~0.46 s after its deadline and the pre-flight only ~0.03 s.**
The deadline is `time.monotonic() + STATE_TIMEOUT_S`, set when the *wait* starts, i.e.
after the subscribers exist — which is the earliest moment state could arrive. Anything
built before that is not on the clock. Measured at three timeout values, the offset is
constant, which is what makes it construction cost rather than deadline drift:

| `DEX5_STATE_TIMEOUT_S` | controller refuses at | offset |
|---|---|---|
| 3 | 3.46 s | +0.46 |
| 6 | 6.46 s | +0.46 |
| 10 (default) | 10.46 s | +0.46 |

The 0.46 s is: `ChannelPublisher.Init()` 0.220 s + 0.201 s (the controller builds two
command publishers), `ChannelSubscriber.Init()` 0.005 s, `ChannelFactoryInitialize`
0.001 s, `HandRetargeting` build 0.026 s. The pre-flight builds no publishers and no
retargeter, which is why its overhead is 0.03 s. G3's "6.46 s" was measured with
`DEX5_STATE_TIMEOUT_S=6` set by the test driver; the default is and always was 10 s.

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
| fist (left) | 16.8 mm | 1.55 rad | **inside** |
| fist (right) | 16.8 mm | 1.59 rad | **inside** |

The stated pass criterion was "every q̂ inside URDF limits, and the pinch closes the right
pair" — not the joint residual. Against that criterion, per hand:

| configuration | left: all q̂ inside limits | right: all q̂ inside limits | fingertip error, max (mean) |
|---|---|---|---|
| open | yes | yes | 0.0 mm (0.0) |
| pinch | yes | yes | 0.0 mm (0.0) |
| fist | yes | yes | 16.8 mm (5.4) |

Fist fingertip error per finger, identical on both hands: thumb 8.5, index 0.6,
middle 16.8, ring 0.6, pinky 0.6 mm. On both hands exactly one joint (`Roll_12{L,R}`)
sits on the relaxed bound `dex_retargeting/optimizer.py:47` sets — that file calls
`set_joint_limit(..., epsilon=1e-3)` and nlopt converges *to* `lower - epsilon`,
measured 0.001000012779 rad past the strict URDF limit. That is upstream design, not a
violation. The fist's larger error is the test's own construction: a synthetic
all-pitch fist leaves the thumb roll at 0, which is not a pose DexPilot would choose.

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

Environment variables the hand path reads:

| variable | default | what it does |
|---|---|---|
| `DEX5_TOPIC_PREFIX` | `rt/dex3` | hand DDS topic prefix; try `rt/dex5` if the topics move |
| `DEX5_MAX_STEP_RAD` | `0.05` | max change per joint per control cycle. At 100 Hz that is 5 rad/s. Raise only with a reason; `10` effectively disables the limiter |
| `DEX5_LIMIT_MARGIN_RAD` | `0.001` | URDF limits are shrunk by this before clamping, so a command never sits on a mechanical stop |
| `DEX5_STATE_TIMEOUT_S` | `10` | how long the pre-flight and the controller wait for the first hand state |
| `DEX5_NUM_JOINTS` | `20` | **bench only.** Overrides the fail-closed motor count. Never on the robot |
| `XR_ARM_VEL_LIMIT` | `30.0` | arm joint velocity limit, rad/s. Hardware sessions use `5` |

### What the hand actually receives, and what gets recorded

The retargeted target is **not** sent straight to the hand. Every cycle, per joint:

```
cmd = clip(target,  last_cmd - DEX5_MAX_STEP_RAD, last_cmd + DEX5_MAX_STEP_RAD)
cmd = clip(cmd,     lower + DEX5_LIMIT_MARGIN_RAD, upper - DEX5_LIMIT_MARGIN_RAD)
```

The limits are the two Dex5 URDFs' own, read from the retargeting object so the clamp and
the optimiser cannot disagree. `last_cmd` starts from the **first measured hand state**, so
the first command after startup ramps from where the hand actually is rather than snapping
to the open pose — which is what the unmodified PR does on every start.

**The recorder logs the command that was sent, not the raw retargeted target.**
`actions.{left,right}_ee.qpos` is the clamped, slew-limited value. This is deliberate: an
episode whose recorded `action` never physically happened is worse than no episode, because
anything trained on it learns a hand that can teleport. If you need the pre-clamp target for
analysis, it is not currently recorded — say so before a data session rather than after.

Never pass `--motion`; it is not used in this project. `--ee dex5 --sim` is refused by
design (the simulator ships a Dex3 hand only).

### Three facts about startup

1. **`--sim` does not skip `Enter_Debug_Mode()`.** `teleop_hand_and_arm.py` branches on
   `args.motion`, **not** `args.sim`, so `MotionSwitcher().Enter_Debug_Mode()` runs in
   both cases. What `--sim` changes is the **DDS domain**: 1 instead of 0. That is the
   only thing keeping the call off PC1 — the request goes out on a domain the robot is
   not listening to. It is a consequence of the domain, not a guard in the code, and it
   protects nothing if the domain is wrong.
   **The rule stands: never start the launcher without `--sim` on the robot LAN unless
   you intend debug mode.**

2. **The hand pre-flight runs first, at launcher start.** With `--ee dex5`,
   `hand_config.preflight()` is called immediately after `ChannelFactoryInitialize` —
   before the image client, before `MotionSwitcher`, and long before
   `Dex5_1_Controller`. If the hands are silent or report 7 motors, the launcher
   **exits 1 before `ReleaseMode()`, having moved nothing**: the robot still has its own
   controller, debug mode was never entered, and no go-home is attempted. Before this,
   the same refusal came from the controller, i.e. after the release, and cost a go-home
   with the arms limp and debug mode left active.

3. **Exit codes are meaningful now — a wrapper can branch on them.**

   | code | meaning |
   |---|---|
   | 0 | clean exit: the operator's `q`, or Ctrl-C |
   | 1 | the run ended in a caught exception — pre-flight refusal, DDS failure, anything logged with a traceback |
   | 2 | argparse rejected the arguments (e.g. `--ee dex5 --sim`) |

   Cleanup failures inside `finally` (recorder close, image client close, go-home) are
   logged individually and do **not** flip a clean run to 1.

---

## 4. Operator sequence for the hand tools

Run these **before** the launcher, in this order. Every tool defaults to `--domain 1`
so a mistyped command cannot reach the robot; `--domain 0` is deliberate.

These are still worth running by hand even though the launcher now pre-flights the hands
itself: the launcher's check answers only "are both hands streaming 20 motors?", while
`hand_probe.py` gives the tactile index map, the temperature ranges and the coupling
hint, which is what `docs/g1_contract_dex5_pending.yaml` needs.

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

## 4b. Ending the session — read this before you start one

**After `q` and the go-home, the robot has no controller.** `Enter_Debug_Mode()` released
its own motion control at startup, and the matching `Exit_Debug_Mode()` in the launcher's
`finally` block is **commented out upstream**:

```python
try:
    if not args.motion:
        pass
        # status, result = motion_switcher.Exit_Debug_Mode()
        # logger_mp.info(f"Exit debug mode: {'Success' if status == 3104 else 'Failed'}")
```

That is deliberate and **stays that way**. Do not uncomment it to "tidy up" at the end of
a session; what the robot does on leaving debug mode is not something to discover with a
tired operator at 9 pm.

Consequences, in order of how badly they end:

- **Never `SelectMode('ai')` with the feet off the floor.** Handing control to the AI
  locomotion controller while the robot is on a stand means it tries to balance against
  ground that is not there. This is the single most expensive mistake available here.
- The arms are limp after go-home. Anything the hands were holding will drop.
- Debug mode is still active. Power-cycling is the only thing that reliably clears it
  without choosing a mode.

**Conservative default: power down on the stand.** Go-home, confirm the arms are at rest,
then power off while the robot is still supported.

**The exact exit is a decision for Brandon, taken before the session, not during it.**
Whatever is agreed goes into the written pre-flight checklist alongside the NIC name and
the topic prefix — so the end of the session is a step someone reads, not a judgement
call made while holding a 1.1 kg hand.

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
| `--sim` | does **not** skip `Enter_Debug_Mode()` — it only changes the DDS domain to 1, and it is the domain, not any guard in the code, that keeps the call off PC1 | never start without `--sim` on the robot LAN; the rule stands |
| launcher exit code | used to be 0 even after a logged traceback | now 0 clean / 1 caught exception / 2 bad arguments — wrappers can branch |
| `Exit_Debug_Mode()` | commented out upstream, so the robot has **no controller** after `q` | power down on the stand; never `SelectMode('ai')` with the feet up; see §4b |
| counting DDS readers with `take()` | destructive — a closed reader looks identical to one that was never announced, so a leak check silently reports whatever it likes | use `read()` and filter `sample_info.instance_state` (ALIVE = 16, NOT_ALIVE_DISPOSED = 32) |
| one `cyclonedds.domain.Domain` per domain id per process | building an observer participant before `ChannelFactoryInitialize` makes the SDK fail with "create domain error" | initialise the SDK factory first |
| `MotorState_.temperature` | `array[int16, 2]`, not a scalar | `hand_config.motor_temperature()` returns the hotter of the two |
| a DDS writer created before a `fork` | writes from the child are invisible to a subscriber in the **parent** (measured 0 samples), but reach any other process fine (200/200). `Dex5_1_Controller` does exactly this | put test subscribers in a separate process; do not conclude the controller is mute |
| `subprocess.Popen` from a DDS-initialised process | the exec'd child receives nothing on this build; the same command from a shell receives ~930 over the same window | start helper subscribers **before** this process calls `ChannelFactoryInitialize` |
| comparing commanded q read back off the wire | float32: a value clamped exactly to a bound returns a few 1e-8 past it | compare with a ~1e-6 rad tolerance, not exact |
