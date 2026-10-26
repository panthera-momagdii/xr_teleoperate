"""[G0] Follow guard for the G1_29 arm controller's 250 Hz loop.

Pure module: numpy plus a gravity function. No DDS, no threads, no clock of its own
(the caller passes the time).

The problem (RT7, 24 Oct 2026, GAPS 17)
---------------------------------------
The arms did not follow the IK target. Both shoulder pitches ran to -3.09 rad (arms
straight up, at the stop) for about 60 s, the left shoulder yaw to +2.61, the wrist
rolls to +-1.99, while the targets stayed far inside. Cause: the speed limiter
(`clip_arm_q_target`) put each command at most velocity_limit * dt (0.004 rad at
1 rad/s) ahead of the MEASURED pose, so the PD part could push with 0.3 Nm at most,
while the gravity feed-forward was computed for the operator's TARGET pose. Once the
measured pose drifted off the target (the left yaw first), gravity at the measured
pose (about -4 Nm on the left shoulder pitch) no longer matched the feed-forward
(-7 Nm, the cap), and the difference carried the arm into its stops. Nothing pulled
it back, and a tracking error measured against that clipped command read 0.004 rad.
Replayed through the simulated arm (tools/overnight/test_g18_follow_runaway.py) the
same targets put a joint at its stop after 16 s (49 s with the calibrated friction).

What this does, every control step
----------------------------------
1. Rate limit from the LAST COMMAND, never from the measured pose: the command moves
   towards the target at velocity_limit (uniform scale over the 14 joints, as before,
   so the joint-space path is the target's), seeded at the measured pose.
2. Gravity feed-forward at the MEASURED pose. The PD part (kp 80 / 40) then pulls the
   arm onto the command; gravity is cancelled wherever the arm really is.
3. Tracking-error stop, judged against the IK target (what the launcher asked for),
   not against the rate-limited command: max |target - measured| over the 14 joints
   above max_err for longer than grace_s.
4. Joint-at-stop stop: a joint measured within stop_margin of its URDF limit while its
   target is more than stop_gap inside that limit.
A stop latches: from then on the command is the measured pose at the stop (a hold,
gravity still cancelled at the measured pose) until reset(). The launcher ends the
session on it, which lowers and releases the arms.

    XR_TRACK_ERR_MAX     rad, default 0.35 (as panthera-g1-arm's SafetyGuard)
    XR_TRACK_GRACE_S     s, default 1.0
    XR_JOINT_STOP_MARGIN rad, default 0.02
    XR_JOINT_STOP_GAP    rad, default 0.2

The stops are armed only once the launcher sends a target (`armed=True`); before that
the controller holds the pose it measured at start.
"""

import os

import numpy as np

DEFAULT_MAX_ERR = 0.35
DEFAULT_GRACE_S = 1.0
DEFAULT_STOP_MARGIN = 0.02
DEFAULT_STOP_GAP = 0.2


class FollowGuardError(ValueError):
    pass


def _env(name, default):
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return default
    try:
        v = float(raw)
    except ValueError:
        raise FollowGuardError(f"{name}={raw!r} is not a number") from None
    if not np.isfinite(v) or v <= 0.0:
        raise FollowGuardError(f"{name}={raw!r} must be a finite number > 0")
    return v


def settings_from_env():
    """dict(max_err, grace_s, stop_margin, stop_gap). Raises FollowGuardError."""
    return dict(max_err=_env("XR_TRACK_ERR_MAX", DEFAULT_MAX_ERR),
                grace_s=_env("XR_TRACK_GRACE_S", DEFAULT_GRACE_S),
                stop_margin=_env("XR_JOINT_STOP_MARGIN", DEFAULT_STOP_MARGIN),
                stop_gap=_env("XR_JOINT_STOP_GAP", DEFAULT_STOP_GAP))


class FollowGuard:
    def __init__(self, velocity_limit, dt, gravity_fn=None, lower=None, upper=None,
                 max_err=DEFAULT_MAX_ERR, grace_s=DEFAULT_GRACE_S,
                 stop_margin=DEFAULT_STOP_MARGIN, stop_gap=DEFAULT_STOP_GAP):
        self.dt = float(dt)
        self.set_velocity_limit(velocity_limit)
        self.gravity_fn = gravity_fn
        self.lower = None if lower is None else np.asarray(lower, dtype=float).copy()
        self.upper = None if upper is None else np.asarray(upper, dtype=float).copy()
        self.max_err, self.grace_s = float(max_err), float(grace_s)
        self.stop_margin, self.stop_gap = float(stop_margin), float(stop_gap)
        self.q_cmd = None
        self.fault = None              # str, latched
        self.q_hold = None
        self.t_err = None              # when the tracking error went over max_err
        self.err = 0.0                 # max |target - measured| of the last armed step
        self.err_max = 0.0             # since the last read_stats()
        self.clipped = False           # the rate limit cut this step
        self.clipped_arms = (False, False)   # ...and whose own step was over it (left, right)

    def set_velocity_limit(self, velocity_limit):
        v = float(velocity_limit)
        if not np.isfinite(v) or v <= 0.0:
            raise FollowGuardError(f"velocity limit {velocity_limit!r} must be > 0")
        self.velocity_limit = v

    def describe(self):
        return (f"[follow] command rate-limited from the last command at {self.velocity_limit:g} rad/s; "
                f"gravity feed-forward at the measured pose"
                f"{'' if self.gravity_fn is not None else ' (NO gravity model: none sent)'}; "
                f"stop on tracking error > {self.max_err:g} rad for {self.grace_s:g} s (against the IK target) "
                f"or a joint within {self.stop_margin:g} rad of its stop while its target is "
                f"{self.stop_gap:g} rad inside"
                f"{'' if self.lower is not None else ' (NO joint limits: stop check off)'}")

    def reset(self, q_meas):
        """Seed the command at the measured pose; clears a latched stop."""
        self.q_cmd = np.asarray(q_meas, dtype=float).copy()
        self.fault, self.q_hold, self.t_err = None, None, None

    def gravity(self, q):
        if self.gravity_fn is None:
            return np.zeros(len(q))
        return np.asarray(self.gravity_fn(q), dtype=float)

    def _check(self, t, q_target, q_meas):
        e = np.abs(q_target - q_meas)
        k = int(np.argmax(e))
        self.err = float(e[k])
        self.err_max = max(self.err_max, self.err)
        if self.err > self.max_err:
            if self.t_err is None:
                self.t_err = t
            elif t - self.t_err > self.grace_s:
                return (f"tracking error {self.err:.3f} rad on arm joint {k} (target {q_target[k]:+.3f}, "
                        f"measured {q_meas[k]:+.3f}) for more than {self.grace_s:g} s")
        else:
            self.t_err = None
        if self.lower is not None:
            at_lo = (q_meas <= self.lower + self.stop_margin) & (q_target > self.lower + self.stop_gap)
            at_hi = (q_meas >= self.upper - self.stop_margin) & (q_target < self.upper - self.stop_gap)
            hit = np.where(at_lo | at_hi)[0]
            if hit.size:
                j = int(hit[0])
                side = "lower" if at_lo[j] else "upper"
                return (f"arm joint {j} at its {side} stop ({q_meas[j]:+.3f} rad) while its target is "
                        f"{q_target[j]:+.3f} rad")
        return None

    def step(self, t, q_target, q_meas, armed=True):
        """(q_cmd, tau_ff) for this step. armed=False: no stop checks (nothing asked yet)."""
        q_meas = np.asarray(q_meas, dtype=float)
        q_target = np.asarray(q_target, dtype=float)
        if self.q_cmd is None:
            self.reset(q_meas)
        if self.fault is None:
            delta = q_target - self.q_cmd
            lim = self.velocity_limit * self.dt
            scale = float(np.max(np.abs(delta))) / lim
            self.clipped = scale > 1.0
            self.clipped_arms = (self.clipped and float(np.max(np.abs(delta[:7]))) > lim,
                                 self.clipped and float(np.max(np.abs(delta[7:]))) > lim)
            self.q_cmd = self.q_cmd + delta / max(scale, 1.0)
            if armed:
                why = self._check(t, q_target, q_meas)
                if why is not None:
                    self.fault = why
                    self.q_hold = q_meas.copy()
        if self.fault is not None:
            self.clipped, self.clipped_arms = False, (False, False)
            self.q_cmd = self.q_hold.copy()
        return self.q_cmd.copy(), self.gravity(q_meas)

    def read_stats(self):
        """Max tracking error since the last call, then reset."""
        out, self.err_max = self.err_max, 0.0
        return out
