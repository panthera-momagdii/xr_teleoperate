"""Link-loss guard: a stale XR pose stream means HOLD; resuming needs a deliberate [r].

Pure state machine. No DDS, no robot, no threads, no clock of its own -- every method
takes the time it should act on, so it is testable without waiting (same shape as
start_pose.py).

The problem (RT4, remote teleop over Tailscale)
-----------------------------------------------
televuer's `motion_data_ready` latches True after the first hand event and never goes
back. When the headset link stalls, the main loop keeps solving IK on whatever pose
sits in shared memory: the arms hold the last target only by accident (the data
stopped changing). When the link comes back, the very next frame carries wherever the
operator's hands are NOW, and the arms jump there. Over a 177 ms median, 259 ms max
WAN link (RT2, 13 Oct 2026) that is the expected failure, not a corner case.

The states
----------
    FOLLOW   stream fresh               output = the caller's command, untouched
    HOLD     no new pose for stale_s    output = the last command sent, frozen
    BLEND    after a deliberate [r]     output = hold pose -> live command over
                                        reengage_t, rate-limited, then FOLLOW

    FOLLOW --(no new pose for stale_s)--> HOLD
    HOLD   --([r] while the stream is fresh)--> BLEND
    HOLD   --([r] while the stream is stale)--> HOLD   (refused, says why)
    HOLD   --(stream comes back, no [r])--> HOLD       (says "press [r]", once)
    BLEND  --(stream stale again)--> HOLD               (at the blended pose)
    BLEND  --(weight 1 and within one step of the command)--> FOLLOW

A stream that is not fresh when [r] starts tracking starts in HOLD at the arms'
MEASURED position, not at the IK of televuer's fallback pose. That is the 2026-09-09
failure (arms moved to CONST_*_ARM_POSE with nobody driving) closed from this side too.

"Fresh" is decided by the caller from the raw wrist data: a new pose is one whose
bytes differ from the previous one. Real hand tracking is never bit-identical from one
event to the next; a stalled websocket is.

Feed-forward torque: in HOLD the last torque is kept (it was computed for exactly the
held pose). In BLEND it is zero, for the same reason start_pose.py zeroes it: a torque
computed for a pose the arm is not commanded to is worse than none.
"""

import os

import numpy as np

FOLLOW, HOLD, BLEND = "FOLLOW", "HOLD", "BLEND"

# Proposed hold time. RT2 measured PC2 -> Giza: median 177 ms, p95 187 ms, max 259 ms,
# jitter 5.24 ms. Hand events arrive every 11-33 ms when the link is healthy. 0.5 s is
# about 2x the worst RTT seen, so normal jitter never trips it, and at 30 Hz it is 15
# missed frames -- short enough that a stall is caught before an operator would notice.
DEFAULT_STALE_S = 0.5
# Re-engage blend. Matches the order of start_pose's blend; long enough to watch.
DEFAULT_REENGAGE_T = 2.0


class LinkGuardError(ValueError):
    pass


def _clamp_step(current, target, max_step):
    """Uniformly scaled step, same shape as start_pose._clamp_step / clip_arm_q_target."""
    delta = target - current
    biggest = float(np.max(np.abs(delta))) if delta.size else 0.0
    if biggest <= max_step or biggest == 0.0:
        return target.copy()
    return current + delta * (max_step / biggest)


class LinkGuard:
    def __init__(self, stale_s=DEFAULT_STALE_S, reengage_t=DEFAULT_REENGAGE_T,
                 velocity_limit=30.0, notify=None):
        self.stale_s = float(stale_s)
        self.reengage_t = float(reengage_t)
        self.velocity_limit = float(velocity_limit)
        if not np.isfinite(self.stale_s) or self.stale_s <= 0:
            raise LinkGuardError("XR_STALE_S must be a finite number > 0")
        if not np.isfinite(self.reengage_t) or self.reengage_t < 0:
            raise LinkGuardError("XR_REENGAGE_T must be a finite number >= 0")
        if not np.isfinite(self.velocity_limit) or self.velocity_limit <= 0:
            raise LinkGuardError("velocity_limit must be > 0")
        self._notify = notify
        self.state = HOLD
        self.started = False
        self.last_fresh_t = None
        self._last_marker = None
        self.hold_q = None
        self.hold_tauff = None
        self.last_q = None
        self.last_tauff = None
        self._blend_t0 = None
        self._said_back = False
        self.blend_weight = 0.0
        self.events = []            # (t, text) -- for tests and the log

    # ------------------------------------------------------------------ helpers
    def _say(self, t_now, text):
        self.events.append((float(t_now), text))
        if self._notify is not None:
            try:
                self._notify(text)
            except Exception:
                pass

    def observe(self, t_now, marker):
        """Feed the raw wrist data of this cycle. Returns True if it is a NEW pose.

        The first call only sets the baseline: initial shared memory is not a pose.
        """
        if marker is None:
            return False
        if self._last_marker is None:
            self._last_marker = marker
            return False
        if marker != self._last_marker:
            self._last_marker = marker
            self.last_fresh_t = float(t_now)
            return True
        return False

    def stream_alive(self, t_now):
        return (self.last_fresh_t is not None
                and float(t_now) - self.last_fresh_t <= self.stale_s)

    @property
    def holding(self):
        """True only in HOLD. Hands are frozen in HOLD and resume in BLEND."""
        return self.state == HOLD

    # ------------------------------------------------------------------ transitions
    def start(self, t_now, current_q):
        """Call when [r] starts tracking. Fresh stream -> FOLLOW, else HOLD where the
        arms ARE."""
        self.started = True
        q = np.asarray(current_q, dtype=float).copy()
        self.last_q = q.copy()
        self.last_tauff = np.zeros_like(q)
        if self.stream_alive(t_now):
            self.state = FOLLOW
            self._say(t_now, "[link] tracking: XR stream is live")
        else:
            self._enter_hold(t_now, q, np.zeros_like(q),
                             "[link] HOLD at start: no live XR pose yet. "
                             "Arms hold where they are. Press [r] once the stream is live.")
        return self.state

    def _enter_hold(self, t_now, q, tauff, text):
        self.state = HOLD
        self.hold_q = np.asarray(q, dtype=float).copy()
        self.hold_tauff = np.asarray(tauff, dtype=float).copy()
        self._blend_t0 = None
        self.blend_weight = 0.0
        self._said_back = False
        self._say(t_now, text)

    def request_reengage(self, t_now):
        """The operator pressed [r] while tracking. Returns 'accepted', 'refused' or
        'ignored'."""
        if not self.started or self.state != HOLD:
            return "ignored"
        if not self.stream_alive(t_now):
            age = ("never" if self.last_fresh_t is None
                   else f"{float(t_now) - self.last_fresh_t:.1f} s ago")
            self._say(t_now, f"[link] re-engage REFUSED: no live XR pose (last {age}). "
                             f"Still holding.")
            return "refused"
        self.state = BLEND
        self._blend_t0 = float(t_now)
        self.blend_weight = 0.0
        self._say(t_now, f"[link] re-engage: blending from the hold pose to the operator "
                         f"over {self.reengage_t:g} s")
        return "accepted"

    # ------------------------------------------------------------------ the step
    def step(self, t_now, cmd_q, cmd_tauff, dt):
        """Return (q, tauff) to command this cycle."""
        cmd_q = np.asarray(cmd_q, dtype=float)
        cmd_tauff = np.asarray(cmd_tauff, dtype=float)
        if not self.started:
            return cmd_q, cmd_tauff
        alive = self.stream_alive(t_now)

        if self.state == FOLLOW:
            if not alive:
                self._enter_hold(t_now, self.last_q, self.last_tauff,
                                 f"[link] HOLD: no new XR pose for {self.stale_s:g} s. "
                                 f"Arms hold the last command. Press [r] to re-engage "
                                 f"once the stream is back.")
                return self.hold_q.copy(), self.hold_tauff.copy()
            self.last_q = cmd_q.copy()
            self.last_tauff = cmd_tauff.copy()
            return cmd_q, cmd_tauff

        if self.state == HOLD:
            if alive and not self._said_back:
                self._said_back = True
                self._say(t_now, "[link] XR stream is back. Still holding: press [r] to "
                                 "re-engage.")
            elif not alive:
                self._said_back = False
            return self.hold_q.copy(), self.hold_tauff.copy()

        # BLEND
        if not alive:
            self._enter_hold(t_now, self.last_q, np.zeros_like(self.last_q),
                             "[link] HOLD during re-engage: the stream went stale again.")
            return self.hold_q.copy(), self.hold_tauff.copy()
        if self.reengage_t <= 0:
            w = 1.0
        else:
            w = min(max((float(t_now) - self._blend_t0) / self.reengage_t, 0.0), 1.0)
        self.blend_weight = w
        target = (1.0 - w) * self.hold_q + w * cmd_q
        max_step = self.velocity_limit * float(dt)
        out = _clamp_step(self.last_q, target, max_step)
        self.last_q = out.copy()
        self.last_tauff = np.zeros_like(out)
        if w >= 1.0 and float(np.max(np.abs(cmd_q - out))) <= max_step:
            self.state = FOLLOW
            self._say(t_now, "[link] re-engaged: following the operator")
        return out, np.zeros_like(out)


def from_env(velocity_limit=30.0, notify=None):
    """Build from XR_STALE_S / XR_REENGAGE_T. Refuses bad values before any DDS."""
    def _f(name, default):
        raw = os.environ.get(name)
        if raw is None or raw.strip() == "":
            return default
        try:
            return float(raw)
        except ValueError:
            raise LinkGuardError(f"{name}={raw!r} is not a number") from None
    return LinkGuard(stale_s=_f("XR_STALE_S", DEFAULT_STALE_S),
                     reengage_t=_f("XR_REENGAGE_T", DEFAULT_REENGAGE_T),
                     velocity_limit=velocity_limit, notify=notify)


def wrist_marker(tvuer):
    """Bytes of both raw wrist poses, read straight from televuer's shared memory."""
    try:
        return (np.asarray(tvuer.left_arm_pose, dtype=float).tobytes()
                + np.asarray(tvuer.right_arm_pose, dtype=float).tobytes())
    except Exception:
        return None
