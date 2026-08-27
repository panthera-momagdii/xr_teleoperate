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
TOPIC_PREFIX = os.environ.get("DEX5_TOPIC_PREFIX", "rt/dex3")

TOPIC_LEFT_CMD    = f"{TOPIC_PREFIX}/left/cmd"
TOPIC_RIGHT_CMD   = f"{TOPIC_PREFIX}/right/cmd"
TOPIC_LEFT_STATE  = f"{TOPIC_PREFIX}/left/state"
TOPIC_RIGHT_STATE = f"{TOPIC_PREFIX}/right/state"


# --- joint count -----------------------------------------------------------
# Dex5-1P has 20 motors per hand; a Dex3-1 reports 7. The controller refuses to
# run on a mismatch (see Dex5_1_Controller._subscribe_hand_state) rather than
# silently driving the wrong hand.
NUM_JOINTS_EXPECTED = 20

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
STATE_TIMEOUT_S = float(os.environ.get("DEX5_STATE_TIMEOUT_S", "10"))

# Abort threshold for the operator tools. Not a firmware limit -- a conservative
# bench number until we have real thermal data from the hands.
TEMP_LIMIT_C = 45.0


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


def group_of(idx, thumb_base_index=THUMB_BASE_INDEX):
    """Return "thumb" or "finger" for a DDS motor slot."""
    return "thumb" if idx >= thumb_base_index else "finger"


def read_hand_counts(msg):
    """Return (n_motor, n_press) for a HandState_.

    Both fields are IDL `sequence`s, so their length is whatever the hand actually
    published -- this is the measurement that tells us Dex5-1P (20) from Dex3-1 (7).
    """
    n_motor = len(msg.motor_state) if msg.motor_state is not None else 0
    n_press = len(msg.press_sensor_state) if msg.press_sensor_state is not None else 0
    return n_motor, n_press


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

    `who` is the tag of the caller so a log line says which check gave up; the body --
    both topics, the per-interface discovery note, the three causes -- is identical
    wherever it is raised.
    """
    return RuntimeError(
        f"{who} no HandState_ on {TOPIC_LEFT_STATE} / "
        f"{TOPIC_RIGHT_STATE} within {timeout_s}s "
        f"(saw: {sorted(seen) or 'nothing'}). "
        "Note that DDS discovery is per network interface -- the launcher's "
        "--network-interface must be the one the hands are on. Three usual causes: "
        "(1) the hands are unpowered or have not enumerated on the bus; "
        "(2) wrong DDS domain or interface; "
        f"(3) wrong topic prefix -- DEX5_TOPIC_PREFIX is currently "
        f"'{TOPIC_PREFIX}', try the other of rt/dex3 or rt/dex5.")


def preflight(timeout_s=None, log=None):
    """Confirm both hands are streaming a Dex5-1P-shaped state. Read-only.

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
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_

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
                log.info(f"[hand preflight] {side} hand first state: "
                         f"motor_state={n_motor} press_sensor_state={n_press}")
                check_motor_count(side, n_motor)
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
    return (
        "[hand_config] effective Dex5-1P configuration\n"
        f"  topic prefix     : {TOPIC_PREFIX}   (env DEX5_TOPIC_PREFIX)\n"
        f"    left  cmd/state: {TOPIC_LEFT_CMD} | {TOPIC_LEFT_STATE}\n"
        f"    right cmd/state: {TOPIC_RIGHT_CMD} | {TOPIC_RIGHT_STATE}\n"
        f"  motors expected  : {NUM_JOINTS_EXPECTED}{overridden}   (7 would mean a Dex3-1 is fitted)\n"
        f"  state timeout    : {STATE_TIMEOUT_S} s   (env DEX5_STATE_TIMEOUT_S)\n"
        f"  temp limit       : {TEMP_LIMIT_C} C\n"
        f"  gains            : finger kp/kd {GAINS['finger'][0]}/{GAINS['finger'][1]}, "
        f"thumb kp/kd {GAINS['thumb'][0]}/{GAINS['thumb'][1]} (thumb slots >= {THUMB_BASE_INDEX})"
    )
