"""Single source of truth for the Dex5-1P hand wiring on our G1.

Upstream PR #321 hardcodes the Dex5 topics ("rt/dex5/...") and the 20-joint count
inside robot_hand_unitree.py. Our G1's firmware answers on "rt/dex3/..." instead
(a generic namespace on this firmware, participant g1_dex_protocol_v2.0), so those
two facts have to be settable without editing the vendored controller. Everything
that depends on "which hand is actually bolted on" lives here.

Import-time side effects: none. No ChannelFactoryInitialize, no publishers, no
subscribers. Reading this module must be safe from any process, including the
read-only probe.
"""

import os
import time

from unitree_sdk2py.idl.default import (
    unitree_hg_msg_dds__HandCmd_,
    unitree_hg_msg_dds__MotorCmd_,
)

import logging_mp
logger_mp = logging_mp.getLogger(__name__)


# --- topics ----------------------------------------------------------------
# Default is what the wire showed on our G1 on 2026-08-24, NOT the PR's "rt/dex5".
# Override with DEX5_TOPIC_PREFIX=rt/dex5 if a firmware update moves them.
# --- which hand is fitted ---------------------------------------------------
# The G1 carries Inspire RH56E2-T1 hands (labels read 2026-08-27), not the Dex5-1P this
# module was first written for. The Dex5 lane is kept, not deleted: PR #321 remains
# useful if a Dex5 ever arrives, and deleting a verified path to make room for a new one
# loses the verification. Nothing Dex5 runs unless HAND_MODEL says so.
# Set explicitly in the environment, or None if it was never set. The difference matters:
# an explicit HAND_MODEL that disagrees with --ee is an operator mistake worth stopping
# for, while a defaulted one is just a default and --ee should win.
_HAND_MODEL_ENV = os.environ.get("HAND_MODEL")
_HAND_MODEL_ENV = _HAND_MODEL_ENV.strip().lower() if _HAND_MODEL_ENV else None

HAND_MODEL = _HAND_MODEL_ENV or "inspire_ftp"

MODELS = {
    "inspire_ftp": {
        "label": "Inspire RH56E2-T1 (Modbus TCP via the inspire_sdkpy bridge)",
        "left_cmd": "rt/inspire_hand/ctrl/l",
        "right_cmd": "rt/inspire_hand/ctrl/r",
        "left_state": "rt/inspire_hand/state/l",
        "right_state": "rt/inspire_hand/state/r",
        # 6 actuators; the hand has 12 joints, mechanically coupled.
        "num_joints": 6,
        # RH56DFTP manual: TEMP(m) at register 1618, one byte per DOF, degrees C.
        "temp_limit_c": 45.0,
        "touch_topics": ("rt/inspire_hand/touch/l", "rt/inspire_hand/touch/r"),
    },
    "dex5": {
        "label": "Unitree Dex5-1P (parked -- not the hand on this robot)",
        "left_cmd": None, "right_cmd": None, "left_state": None, "right_state": None,
        "num_joints": 20,
        "temp_limit_c": 45.0,
        "touch_topics": (),
    },
}
if HAND_MODEL not in MODELS:
    raise RuntimeError(
        f"HAND_MODEL={HAND_MODEL!r} is not one of {sorted(MODELS)}. "
        f"This robot's hands are Inspire RH56E2-T1 -> 'inspire_ftp'.")

_MODEL = MODELS[HAND_MODEL]

TOPIC_PREFIX = os.environ.get("DEX5_TOPIC_PREFIX", "rt/dex3")

if HAND_MODEL == "dex5":
    TOPIC_LEFT_CMD    = f"{TOPIC_PREFIX}/left/cmd"
    TOPIC_RIGHT_CMD   = f"{TOPIC_PREFIX}/right/cmd"
    TOPIC_LEFT_STATE  = f"{TOPIC_PREFIX}/left/state"
    TOPIC_RIGHT_STATE = f"{TOPIC_PREFIX}/right/state"
else:
    # Fixed by the bridge, not by a prefix: inspire_sdkpy publishes rt/inspire_hand/*.
    TOPIC_LEFT_CMD    = _MODEL["left_cmd"]
    TOPIC_RIGHT_CMD   = _MODEL["right_cmd"]
    TOPIC_LEFT_STATE  = _MODEL["left_state"]
    TOPIC_RIGHT_STATE = _MODEL["right_state"]

# Tactile. The bridge publishes these; nothing in xr_teleoperate subscribes to them yet.
# T1 is the 17-sensor resistive option, and inspire_hand_touch carries 17 regions. Wiring
# them into the recorder as an extra column is a deliberate future step -- see
# docs/inspire_rh56e2.md -- not something to switch on the day the hands first stream.
TOPIC_TOUCH = _MODEL["touch_topics"]


# --- joint count -----------------------------------------------------------
# Dex5-1P has 20 motors per hand; a Dex3-1 reports 7. The controller refuses to
# run on a mismatch (see Dex5_1_Controller._subscribe_hand_state) rather than
# silently driving the wrong hand.
NUM_JOINTS_EXPECTED = _MODEL["num_joints"]

_NUM_JOINTS_ENV = os.environ.get("DEX5_NUM_JOINTS")
if _NUM_JOINTS_ENV is not None:
    NUM_JOINTS_EXPECTED = int(_NUM_JOINTS_ENV)
    logger_mp.warning(
        "=" * 78 + "\n"
        f"[hand_config] DEX5_NUM_JOINTS={_NUM_JOINTS_ENV} OVERRIDES the expected motor\n"
        f"[hand_config] count. This is a BENCH-ONLY escape hatch. The fail-closed motor\n"
        f"[hand_config] count check is now comparing against {NUM_JOINTS_EXPECTED}, not 20.\n"
        f"[hand_config] Do NOT use this on the robot.\n" + "=" * 78
    )


# --- timeouts and limits ---------------------------------------------------
# How long Dex5_1_Controller waits for the first HandState_ on BOTH sides before
# giving up. Measured on monotonic time, never wall clock.
STATE_TIMEOUT_S = float(os.environ.get("HAND_STATE_TIMEOUT_S",
                                       os.environ.get("DEX5_STATE_TIMEOUT_S", "10")))

# Abort threshold for the operator tools. Not a firmware limit -- a conservative
# bench number until we have real thermal data from the hands.
TEMP_LIMIT_C = float(os.environ.get("HAND_TEMP_LIMIT_C", _MODEL["temp_limit_c"]))


# --- gains -----------------------------------------------------------------
# Author's values from PR #321, unverified on hardware -- do not tune here.
# PR comment (robot_hand_unitree.py): "the thumb motors report in N*m and the four
# fingers in mNm, hence the different scale."
GAINS = {
    "finger": (0.10, 0.001),   # (kp, kd) for slots 0..15
    "thumb":  (1.0, 0.02),     # (kp, kd) for slots 16..19
}

# DDS slot at which the thumb starts. Slots 0..15 are index/middle/ring/pinky in
# groups of four; 16..19 are Yaw_11, Roll_12, Pitch_13, Pitch_14.
THUMB_BASE_INDEX = 16


# --- rate and range limiting -----------------------------------------------
# DexPilot already keeps its output inside the URDF limits (plus its own 1e-3 rad
# bound relaxation), so RANGE is not the hazard. RATE is: when the headset loses the
# hands -- out of view, or during a power grasp -- the landmarks jump, and the
# retargeted target moves the whole way in one 10 ms control cycle. On hardware that
# is a finger snapping shut into an object or into the thumb.
#
# Per cycle, per joint the controller applies:
#     cmd = clip(target, last_cmd - MAX_STEP, last_cmd + MAX_STEP)
#     cmd = clip(cmd,    lower + MARGIN,      upper - MARGIN)
#
# 0.05 rad/cycle at the controller's 100 Hz is 5 rad/s, so a full 1.5708 rad finger
# flexion takes 32 cycles (~0.32 s) -- fast enough to feel direct, slow enough that a
# bad frame cannot become an impact. Measured open->fist: 29 cycles for the widest
# joint (1.4112 rad), 33 cycles end-to-end through the retargeter's own filter.
DEX5_MAX_STEP_RAD = float(os.environ.get("DEX5_MAX_STEP_RAD", "0.05"))

# Shrink the URDF limits by this before clipping, so a command never sits exactly on a
# mechanical stop. Matches the epsilon dex_retargeting relaxes its own bounds by
# (dex_retargeting/optimizer.py:47, set_joint_limit(..., epsilon=1e-3)).
DEX5_LIMIT_MARGIN_RAD = float(os.environ.get("DEX5_LIMIT_MARGIN_RAD", "0.001"))


# Which --ee value implies which hand model. The launcher derives the model from --ee so
# the two cannot disagree silently: choosing an end effector IS choosing a hand, and
# having to remember a matching env var is a way to command a Dex5's topics with an
# Inspire fitted. Values not listed here have no hand model of their own.
EE_TO_MODEL = {
    "dex5": "dex5",
    "inspire_ftp": "inspire_ftp",
}


def apply_model(name):
    """Re-resolve every model-dependent constant in this module for `name`.

    hand_config is imported at launcher start, long before argparse has run, so the model
    picked at import is only a default. This rebinds the module globals once --ee is
    known. Call it before anything reads the topics -- i.e. before preflight and before
    any controller is constructed.
    """
    global HAND_MODEL, _MODEL, TOPIC_LEFT_CMD, TOPIC_RIGHT_CMD
    global TOPIC_LEFT_STATE, TOPIC_RIGHT_STATE, TOPIC_TOUCH
    global NUM_JOINTS_EXPECTED, TEMP_LIMIT_C

    if name not in MODELS:
        raise RuntimeError(f"unknown hand model {name!r}; known: {sorted(MODELS)}")
    HAND_MODEL = name
    _MODEL = MODELS[name]

    if name == "dex5":
        TOPIC_LEFT_CMD    = f"{TOPIC_PREFIX}/left/cmd"
        TOPIC_RIGHT_CMD   = f"{TOPIC_PREFIX}/right/cmd"
        TOPIC_LEFT_STATE  = f"{TOPIC_PREFIX}/left/state"
        TOPIC_RIGHT_STATE = f"{TOPIC_PREFIX}/right/state"
    else:
        TOPIC_LEFT_CMD    = _MODEL["left_cmd"]
        TOPIC_RIGHT_CMD   = _MODEL["right_cmd"]
        TOPIC_LEFT_STATE  = _MODEL["left_state"]
        TOPIC_RIGHT_STATE = _MODEL["right_state"]
    TOPIC_TOUCH = _MODEL["touch_topics"]

    # DEX5_NUM_JOINTS is a bench override and keeps winning if it was set.
    NUM_JOINTS_EXPECTED = (_NUM_JOINTS_ENV if _NUM_JOINTS_ENV is not None
                           else _MODEL["num_joints"])
    TEMP_LIMIT_C = float(os.environ.get("HAND_TEMP_LIMIT_C", _MODEL["temp_limit_c"]))
    return HAND_MODEL


def select_model_for_ee(ee):
    """Derive the hand model from --ee. Returns None on success, or an error string.

    The caller passes the string to parser.error(), so a mismatch exits 2 before any DDS
    init and before Enter_Debug_Mode -- an operator who sets HAND_MODEL=dex5 and then runs
    --ee inspire_ftp is told, rather than being given an Inspire hand configured to listen
    on a Dex5's topics.
    """
    want = EE_TO_MODEL.get(ee)
    if want is None:
        return None                      # this --ee has no hand model of its own
    if _HAND_MODEL_ENV is not None and _HAND_MODEL_ENV != want:
        return (f"--ee {ee} implies HAND_MODEL={want}, but HAND_MODEL={_HAND_MODEL_ENV} "
                f"is set in the environment. These select different hands "
                f"({MODELS[want]['label']} vs {MODELS[_HAND_MODEL_ENV]['label']}) on "
                f"different topics. Unset HAND_MODEL and let --ee decide, or pass the "
                f"--ee that matches.")
    apply_model(want)
    return None


def group_of(idx, thumb_base_index=THUMB_BASE_INDEX):
    """Return "thumb" or "finger" for a DDS motor slot."""
    return "thumb" if idx >= thumb_base_index else "finger"


def read_hand_counts(msg):
    """Return (n_motor, n_press) for a HandState_.

    Both fields are IDL `sequence`s, so their length is whatever the hand actually
    published -- this is the measurement that tells us Dex5-1P (20) from Dex3-1 (7).

    An Inspire inspire_hand_state has no motor_state at all -- its per-DOF array is
    `angle_act` and it carries no press_sensor_state -- so the same accessor answers for
    both hands and the callers do not have to know which is fitted.
    """
    if hasattr(msg, "angle_act"):                       # Inspire RH56
        return (len(msg.angle_act) if msg.angle_act is not None else 0), 0
    n_motor = len(msg.motor_state) if msg.motor_state is not None else 0
    n_press = len(msg.press_sensor_state) if msg.press_sensor_state is not None else 0
    return n_motor, n_press


# RH56 error bits, from the RH56DFTP manual section 2.6.18 and confirmed against
# inspire_sdkpy's own error_descriptions table.
ERROR_BITS = {
    0: "locked rotor",
    1: "over temperature",
    2: "overcurrent",
    3: "abnormal motor operation",
    4: "communication error",
}
ERROR_OVER_TEMPERATURE = 1 << 1

# DOF order, RH56DFTP manual. Same order as the DDS arrays and as
# Inspire_*_Hand_JointIndex.
DOF_NAMES = ("little", "ring", "middle", "index", "thumb bend", "thumb rotation")


def decode_error(value):
    """Bit-decode one RH56 ERROR(m) byte into human-readable causes."""
    value = int(value)
    names = [n for bit, n in ERROR_BITS.items() if value & (1 << bit)]
    return names or [f"unknown code 0x{value:02x}"]


def hand_health_error(side, msg, temp_limit_c=None):
    """Return a RuntimeError if this state message reports a hand that must not be used.

    Two refusals, both from fields upstream reads and discards:

      err != 0        the hand itself is reporting a fault. RH56 register ERROR(m) at
                      1618-6=1606, one byte per DOF, bit-coded. Any non-zero byte means
                      that DOF is not in a state to be commanded.
      over-temperature TEMP(m) at 1618, degrees C per DOF.

    Checked BEFORE Enter_Debug_Mode, so a faulted or hot hand stops the session while the
    robot still has its own controller.
    """
    if temp_limit_c is None:
        temp_limit_c = TEMP_LIMIT_C

    err = list(getattr(msg, "err", []) or [])
    bad = [i for i, e in enumerate(err) if int(e) != 0]
    if bad:
        detail = "; ".join(f"DOF {i} ({DOF_NAMES[i] if i < len(DOF_NAMES) else '?'}) "
                           f"0x{int(err[i]):02x} = {'+'.join(decode_error(err[i]))}"
                           for i in bad)
        # CLEAR_ERROR does NOT clear an over-temperature fault. The manual is explicit:
        # "The over temperature error of the actuator is not clearable. When the
        # temperature falls, such error will be cleared automatically." Advising an
        # operator to write CLEAR_ERROR at a hot hand would have them writing to a
        # register that cannot help, and then wondering why.
        clearable = any(int(e) & ~ERROR_OVER_TEMPERATURE for e in err if int(e))
        overtemp = any(int(e) & ERROR_OVER_TEMPERATURE for e in err if int(e))
        advice = []
        if overtemp:
            advice.append("the over-temperature bit is NOT clearable -- it clears itself "
                          "when the actuator cools, so wait, do not write CLEAR_ERROR")
        if clearable:
            advice.append("the other bits are clearable with CLEAR_ERROR (register 1004) "
                          "ONCE THE CAUSE IS KNOWN -- clearing a locked rotor and "
                          "commanding it again is how a finger gets damaged")
        return RuntimeError(
            f"{side}: hand reports a fault -- {detail}. Commanding it is not safe. "
            + ". ".join(advice) + ".")

    temps = [int(t) for t in (getattr(msg, "temperature", []) or [])]
    if temps and max(temps) > temp_limit_c:
        hot = [i for i, t in enumerate(temps) if t > temp_limit_c]
        return RuntimeError(
            f"{side}: hand over temperature -- DOF {hot} "
            f"({', '.join(DOF_NAMES[i] for i in hot if i < len(DOF_NAMES))}) at "
            f"{[temps[i] for i in hot]} C, limit {temp_limit_c} C (env "
            f"HAND_TEMP_LIMIT_C). This limit is OURS: the RH56DFTP manual states no "
            f"operating or protection temperature, only that TEMP(m) reads 0-100 C and "
            f"that the actuator raises its own over-temperature ERROR bit. 45 C is a "
            f"conservative early warning ahead of the hand's own protection. Let it cool.")
    return None


def check_hand_health(side, msg, temp_limit_c=None):
    """Raise if the hand reports a fault or is over temperature. Fail closed."""
    exc = hand_health_error(side, msg, temp_limit_c)
    if exc is not None:
        raise exc


def describe_state(msg):
    """One-line summary of a hand state message, for the log-once evidence line."""
    if hasattr(msg, "angle_act"):
        parts = []
        for f in ("angle_act", "force_act", "current", "err", "status", "temperature"):
            v = getattr(msg, f, None)
            if v is not None:
                parts.append(f"{f}={[int(x) for x in v]}")
        return "  ".join(parts)
    n_motor, n_press = read_hand_counts(msg)
    return f"motor_state={n_motor} press_sensor_state={n_press}"


def motor_temperature(motor_state):
    """Return the hottest reading for one motor, in C.

    MotorState_.temperature is array[int16, 2] (not a scalar): two sensors per
    motor. Callers that compare against TEMP_LIMIT_C want the worse of the two.
    Returns None when the field is empty.
    """
    temps = getattr(motor_state, "temperature", None)
    if temps is None:
        return None
    try:
        values = [float(t) for t in temps]
    except TypeError:          # a firmware that reports a plain scalar
        return float(temps)
    return max(values) if values else None


def motor_count_error(side, n_motor, expected=None):
    """The single wording for a motor-count mismatch.

    Used by Dex5_1_Controller and by preflight() so the two can never drift apart.
    """
    if expected is None:
        expected = NUM_JOINTS_EXPECTED
    return RuntimeError(
        f"{side}: motor_state has {n_motor} entries; "
        f"7 = Dex3-1 fitted, expected {expected} (Dex5-1P)")


def check_motor_count(side, n_motor, expected=None):
    """Raise motor_count_error unless the count matches. Fail closed."""
    if expected is None:
        expected = NUM_JOINTS_EXPECTED
    if n_motor != expected:
        raise motor_count_error(side, n_motor, expected)


def state_timeout_error(who, timeout_s, seen):
    """The single wording for "no hand state arrived in time".

    `who` is the tag of the caller so a log line says which check gave up. The causes are
    model-specific and must be: for the Inspire hands the usual answer is that the
    Modbus-TCP<->DDS bridge on PC2 is not running, and advising an operator to check
    DEX5_TOPIC_PREFIX would send them somewhere that cannot help.
    """
    saw = sorted(seen) or "nothing"
    head = (f"{who} no hand state on {TOPIC_LEFT_STATE} / {TOPIC_RIGHT_STATE} "
            f"within {timeout_s}s (saw: {saw}). "
            "Note that DDS discovery is per network interface -- the launcher's "
            "--network-interface must be the one the hands are on. ")
    if HAND_MODEL == "dex5":
        return RuntimeError(head + "Three usual causes: "
                            "(1) the hands are unpowered or have not enumerated on the bus; "
                            "(2) wrong DDS domain or interface; "
                            f"(3) wrong topic prefix -- DEX5_TOPIC_PREFIX is currently "
                            f"'{TOPIC_PREFIX}', try the other of rt/dex3 or rt/dex5.")
    return RuntimeError(
        head + "These topics are produced by the Modbus-TCP<->DDS bridge on PC2, NOT by "
        "PC1, so silence usually means the bridge is not running rather than that the "
        "hands are faulty. Four usual causes: "
        "(1) the bridge is not started on PC2 (see docs/next_visit_pc2.md); "
        "(2) the bridge is running but pointed at the wrong hand address -- ours answer "
        "on 192.168.123.210 and .211, not the factory default 192.168.11.210; "
        "(3) wrong DDS domain or interface; "
        "(4) the hands are unpowered -- their power LED reads green when they ARE "
        "powered, and both were green on 2026-08-27.")


def state_type():
    """The DDS type of this model's hand state message.

    Imported lazily and per model, so that importing hand_config still needs neither the
    Unitree channel machinery nor inspire_sdkpy -- the read-only probe and the tools rely
    on that, and on a laptop without the bridge installed the Dex5 lane must still work.
    """
    if HAND_MODEL == "dex5":
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_
        return HandState_
    from inspire_sdkpy import inspire_dds
    return inspire_dds.inspire_hand_state


def ctrl_type():
    """The DDS type of this model's hand command message."""
    if HAND_MODEL == "dex5":
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_
        return HandCmd_
    from inspire_sdkpy import inspire_dds
    return inspire_dds.inspire_hand_ctrl


def preflight(timeout_s=None, log=None):
    """Confirm BOTH hands are streaming a state of the expected shape. Read-only.

    Called by the launcher BEFORE MotionSwitcher().Enter_Debug_Mode(), so that a wrong
    or missing hand stops the session while the robot still has its own controller.
    Once debug mode is entered, refusing costs a go-home with the arms released.

    Subscribes both state topics, waits for the first valid state per side on a
    monotonic deadline, checks the motor count, and CLOSES its subscribers before
    returning either way -- the controller creates its own readers a moment later and
    a leaked reader would sit on the topic for the rest of the process.

    ChannelFactoryInitialize must already have been called by the caller: preflight
    does not choose a DDS domain, and this module must stay importable without one.

    Returns {"left": {"n_motor": n, "n_press": n}, "right": {...}}.
    Raises RuntimeError on timeout or on a motor-count mismatch.
    """
    # Imported here, not at module scope, so that `import hand_config` still touches no
    # DDS machinery -- the read-only probe and the tools rely on that.
    import gc
    from unitree_sdk2py.core.channel import ChannelSubscriber
    HandState_ = state_type()

    if timeout_s is None:
        timeout_s = STATE_TIMEOUT_S
    if log is None:
        log = logger_mp

    counts = {}
    seen = {}
    error = {}

    def handler(side):
        def on_state(msg):
            if side in counts:
                return
            try:
                n_motor, n_press = read_hand_counts(msg)
                counts[side] = {"n_motor": n_motor, "n_press": n_press}
                # The whole first message, once per side. This is the hardware evidence
                # line: on the next visit it is the first real look at these hands.
                log.info(f"[hand preflight] {side} hand first state: "
                         f"{describe_state(msg)}")
                check_motor_count(side, n_motor)
                check_hand_health(side, msg)
                seen[side] = True
            except BaseException as exc:      # surfaced by the wait loop below
                error.setdefault("exc", exc)
        return on_state

    subs = {}
    try:
        for side, topic in (("left", TOPIC_LEFT_STATE), ("right", TOPIC_RIGHT_STATE)):
            sub = ChannelSubscriber(topic, HandState_)
            sub.Init(handler(side))
            subs[side] = sub
        log.info(f"[hand preflight] waiting up to {timeout_s}s for "
                 f"{TOPIC_LEFT_STATE} and {TOPIC_RIGHT_STATE}")

        deadline = time.monotonic() + timeout_s
        last_warning = 0.0
        while True:
            if "exc" in error:
                raise error["exc"]
            if seen.get("left") and seen.get("right"):
                break
            if time.monotonic() >= deadline:
                raise state_timeout_error("[hand preflight]", timeout_s, seen)
            if time.monotonic() - last_warning >= 1.0:
                last_warning = time.monotonic()
                log.warning(f"[hand preflight] waiting for hand state... "
                            f"({deadline - time.monotonic():.0f}s left)")
            time.sleep(0.01)

        log.info(f"[hand preflight] OK: left {counts['left']}, right {counts['right']}")
        return dict(counts)
    finally:
        for side, sub in subs.items():
            try:
                sub.Close()
            except Exception as exc:          # already closed, or never inited
                log.warning(f"[hand preflight] closing {side} subscriber: {exc}")
        # Channel.__Reader.Close() does `del self.__reader`; the DDS entity is released
        # when the object is collected, so collect now rather than whenever.
        gc.collect()


# ---------------------------------------------------------------------------
# [panthera] Generic end-effector state pre-flight, for EVERY --ee family.
# ---------------------------------------------------------------------------
# preflight() above is the rich check, and it only knows the two hands in MODELS.
# The other five --ee families each build a controller that waits for its state topic
# in an UNBOUNDED loop, so a missing end effector hangs the launcher forever instead of
# refusing:
#
#   Dex1_1_Gripper_Controller   robot_hand_unitree.py:589-591   while not ready: sleep
#   Dex3_1_Controller           robot_hand_unitree.py:383-387   while True: if any(...)
#   Inspire_Controller_DFX      robot_hand_inspire.py:47-51     while True: if any(...)
#   Brainco_Controller_ctrl     robot_hand_brainco.py:56-58     while not ready: sleep
#   Brainco_Controller_hand     robot_hand_brainco.py:207-209   while not ready: sleep
#
# That is the 2026-09-09 `--ee dex1` hang: this robot has no Dex1 gripper, so
# "[Dex1_1_Gripper_Controller] Waiting to subscribe dds..." repeated at 100 Hz until
# the operator gave up. The controllers are upstream's and are left alone; the launcher
# now runs this bounded check first and exits 3 instead of hanging.
#
# The topic literals are duplicated here rather than imported, because importing
# robot_hand_unitree pulls in dex_retargeting and its asset files, which hand_config
# must never require -- the read-only probes depend on hand_config staying cheap.
# tools/overnight/test_g1_failfast.py asserts this table equals the controllers'
# own constants, so the duplication cannot drift silently.

EE_STATE_TOPICS = {
    "dex1": {
        "topics": ("rt/dex1/left/state", "rt/dex1/right/state"),
        "idl": "motor_states",
        "who": "Dex1_1_Gripper_Controller",
        "what": "the Dex1 parallel gripper",
    },
    "dex3": {
        "topics": ("rt/dex3/left/state", "rt/dex3/right/state"),
        "idl": "hand_state",
        "who": "Dex3_1_Controller",
        "what": "the Dex3-1 hands",
    },
    "inspire_dfx": {
        # One topic for BOTH hands, unlike every other family.
        "topics": ("rt/inspire/state",),
        "idl": "motor_states",
        "who": "Inspire_Controller_DFX",
        "what": "the Inspire DFX hands",
    },
    "brainco": {
        "topics": ("rt/brainco/left/state", "rt/brainco/right/state"),
        "idl": "motor_states",
        "who": "Brainco_Controller_hand / Brainco_Controller_ctrl",
        "what": "the BrainCo hands",
    },
    # Handled by preflight() instead, which also checks motor counts and health:
    #   "dex5", "inspire_ftp"
    # No end-effector topic of its own -- the gripper is driven from the arm's own
    # motors and its readiness is the arm's lowstate, which G1_29_Arm_Internal_Dex1_
    # Controller already bounds via dds_utils.wait_for_dds(timeout=5.0):
    #   "dex1_internal"
}

# How long to wait for an end-effector state topic before refusing. Separate from
# HAND_STATE_TIMEOUT_S so an operator can lengthen the Inspire bridge's grace period
# without also lengthening this, and vice versa.
EE_STATE_WAIT_S = float(os.environ.get("XR_HAND_WAIT_S", "10"))


def _idl_for(kind):
    """Lazily import the DDS type for one topic family.

    Lazy for the same reason state_type() is: importing hand_config must not require
    the Unitree channel machinery, and on a laptop without inspire_sdkpy the other
    lanes must still work.
    """
    if kind == "motor_states":
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorStates_
        return MotorStates_
    if kind == "hand_state":
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_
        return HandState_
    raise RuntimeError(f"unknown idl kind {kind!r}")


def ee_state_timeout_error(ee, timeout_s, missing, spec):
    """The single wording for "an --ee was asked for and its state never arrived"."""
    topics = ", ".join(missing)
    return RuntimeError(
        f"[ee preflight] --ee {ee}: no state on {topics} within {timeout_s}s. "
        f"{spec['who']} would wait for this forever. "
        f"Either {spec['what']} is not present/powered on this robot, or the DDS "
        f"domain or --network-interface is wrong (discovery is per interface). "
        f"If this robot has no {ee} end effector, run WITHOUT --ee: arms-only "
        f"teleoperation is a supported mode and is what the Sep 9 session used.")


def ee_state_preflight(ee, timeout_s=None, log=None):
    """Confirm an --ee's state topic(s) are live. Read-only, bounded, closes its readers.

    Returns the set of topics seen (empty for an --ee with no topic of its own, and for
    ee=None). Raises RuntimeError naming the silent topics on timeout.

    Called by the launcher BEFORE MotionSwitcher().Enter_Debug_Mode() for exactly the
    reason preflight() is: once debug mode is entered, refusing costs a go-home with
    the arms released.

    ChannelFactoryInitialize must already have been called by the caller.
    """
    import gc
    from unitree_sdk2py.core.channel import ChannelSubscriber

    if log is None:
        log = logger_mp
    if timeout_s is None:
        timeout_s = EE_STATE_WAIT_S

    spec = EE_STATE_TOPICS.get(ee)
    if spec is None:
        return set()                      # ee is None, dex1_internal, or a MODELS lane

    idl = _idl_for(spec["idl"])
    seen = set()

    def handler(topic):
        def on_state(msg):
            if topic not in seen:
                seen.add(topic)
                log.info(f"[ee preflight] first state on {topic}")
        return on_state

    subs = {}
    try:
        for topic in spec["topics"]:
            sub = ChannelSubscriber(topic, idl)
            sub.Init(handler(topic))
            subs[topic] = sub
        log.info(f"[ee preflight] --ee {ee}: waiting up to {timeout_s}s for "
                 f"{', '.join(spec['topics'])}")

        deadline = time.monotonic() + timeout_s
        last_warning = 0.0
        while True:
            if len(seen) == len(spec["topics"]):
                break
            if time.monotonic() >= deadline:
                missing = [t for t in spec["topics"] if t not in seen]
                raise ee_state_timeout_error(ee, timeout_s, missing, spec)
            if time.monotonic() - last_warning >= 1.0:
                last_warning = time.monotonic()
                log.warning(f"[ee preflight] waiting for {ee} state... "
                            f"({deadline - time.monotonic():.0f}s left, "
                            f"seen {len(seen)}/{len(spec['topics'])})")
            time.sleep(0.01)

        log.info(f"[ee preflight] OK: --ee {ee} answering on "
                 f"{', '.join(sorted(seen))}")
        return set(seen)
    finally:
        # Same discipline as preflight(): the controller creates its own readers a
        # moment later, and a leaked reader would sit on the topic for the process's
        # lifetime.
        for topic, sub in subs.items():
            try:
                sub.Close()
            except Exception as exc:
                log.warning(f"[ee preflight] closing {topic} subscriber: {exc}")
        gc.collect()


def make_hand_cmd(n_joints=NUM_JOINTS_EXPECTED, gains=None, thumb_base_index=THUMB_BASE_INDEX):
    """Build a HandCmd_ sized for n_joints, in position-control mode, zeroed.

    unitree_sdk2py's factory allocates motor_cmd for Dex3's 7 motors. The field is
    an IDL sequence, so the fix is to REPLACE the list with a fresh one of the right
    length -- never to resize the factory's list in place (which leaves the shared
    MotorCmd_ instances aliased across messages).
    """
    if gains is None:
        gains = GAINS
    msg = unitree_hg_msg_dds__HandCmd_()
    msg.motor_cmd = [unitree_hg_msg_dds__MotorCmd_() for _ in range(n_joints)]
    for idx in range(n_joints):
        kp, kd = gains[group_of(idx, thumb_base_index)]
        cmd = msg.motor_cmd[idx]
        cmd.mode = 0x01        # position control
        cmd.q = 0.0
        cmd.dq = 0.0
        cmd.tau = 0.0
        cmd.kp = kp
        cmd.kd = kd
    return msg


def describe():
    """One-block summary of the effective config, for logs and tool banners."""
    overridden = " (OVERRIDDEN by DEX5_NUM_JOINTS)" if _NUM_JOINTS_ENV is not None else ""
    lines = [
        "[hand_config] effective hand configuration",
        f"  hand model       : {HAND_MODEL}   (env HAND_MODEL)",
        f"                     {_MODEL['label']}",
        f"    left  cmd/state: {TOPIC_LEFT_CMD} | {TOPIC_LEFT_STATE}",
        f"    right cmd/state: {TOPIC_RIGHT_CMD} | {TOPIC_RIGHT_STATE}",
        f"  DOF expected     : {NUM_JOINTS_EXPECTED}{overridden}",
        f"  state timeout    : {STATE_TIMEOUT_S} s   (env HAND_STATE_TIMEOUT_S)",
        f"  temp limit       : {TEMP_LIMIT_C} C   (env HAND_TEMP_LIMIT_C)",
    ]
    if HAND_MODEL == "dex5":
        lines += [
            f"  topic prefix     : {TOPIC_PREFIX}   (env DEX5_TOPIC_PREFIX)",
            f"  gains            : finger kp/kd {GAINS['finger'][0]}/{GAINS['finger'][1]}, "
            f"thumb kp/kd {GAINS['thumb'][0]}/{GAINS['thumb'][1]} "
            f"(thumb slots >= {THUMB_BASE_INDEX})",
            f"  max step         : {DEX5_MAX_STEP_RAD} rad/cycle   (env DEX5_MAX_STEP_RAD; "
            f"{DEX5_MAX_STEP_RAD * 100.0:g} rad/s at 100 Hz)",
            f"  limit margin     : {DEX5_LIMIT_MARGIN_RAD} rad   (env DEX5_LIMIT_MARGIN_RAD)",
        ]
    else:
        lines += [
            "  DOF order        : 0 little, 1 ring, 2 middle, 3 index, 4 thumb bend, "
            "5 thumb rotation   (RH56DFTP manual)",
            "  angle semantics  : 0-1000 on the wire, 1000 = fully open, 0 = fully bent",
            f"  touch topics     : {list(TOPIC_TOUCH) or 'none'}",
            "                     published by the bridge; NOT subscribed by xr_teleoperate "
            "yet -- see docs/inspire_rh56e2.md",
        ]
    return "\n".join(lines)
