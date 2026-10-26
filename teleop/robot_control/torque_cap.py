"""[RT6d] Shoulder pitch torque cap while following.

Pure module: numpy plus a gravity function. No DDS, no threads, no clock.

The problem
-----------
RT6, RT6b and RT6c (21 and 22 Oct 2026) were all stopped by shoulder pitch temperature.
The measured torque followed the IK's gravity feed-forward to within 0.9 Nm and the PD
part stayed under 0.5 Nm, so the heat came from the pose itself: the operator's hands
went forward, the robot's arms followed, and the shoulder pitch motors held 8 Nm of
gravity torque. At 6 to 8 Nm the reading rose about 1.25 C/s; at 2.5 Nm about 0.1 C/s.

What this does
--------------
Every loop, after the IK, the gravity torque of the IK solution at the two shoulder
pitch joints is checked (it is `sol_tauff`, the feed-forward the launcher commands).
If an arm's |torque| is over the limit, that arm's command stops where the limit is
reached on the way from the last accepted command towards the new IK solution: a scan
plus bisection on the straight joint-space path, using the gravity model only (about
20 us per probe, no extra IK solve). The feed-forward sent is the gravity torque of the
pose actually commanded. When the operator comes back inside the limit the IK solution
passes through untouched (the same object, bit for bit).

So the wrist target is clamped along the operator's own path: the robot stops reaching
at the limit, it is never pulled back towards its body. Each arm is capped on its own.

    XR_SHOULDER_TAU_CAP   Nm, default 5.0 (RT6h; 4.0 before). 0 switches the cap off.

If the accepted pose is itself over the limit (it cannot be, after the first frame, but
the first accepted pose is the measured one), a new solution is accepted only when it
lowers that arm's torque.

[RT6g] SLIDING, not freezing. RT6e held the arms still at 6 Nm for 18 s and then let them
lurch 0.8 rad when the operator came back: the cap refused every move whose straight
path first raised the torque. Now, past the limit:
  1. the arm moves along the operator's path up to the limit,
  2. the rest of the move is projected onto the limit surface: the part along the
     gradient of |shoulder pitch torque| is removed when it would raise it, and the arm
     follows what is left (sideways, up, rotation); repeated, with a line search each
     time, so curvature cannot take it over the limit,
  3. after any capped frame the arm catches up with the IK solution at no more than
     XR_CAP_CATCHUP_VEL (rad/s per joint, default 0.5) until it has caught up: no jump
     when the operator comes back under the limit.

[RT6h] JOINT BOUNDS. In RT6g the slide took the left elbow onto its 2.094 rad stop: the
projection off the torque gradient adds whatever lowers the torque, and folding the
elbow does. With `lower`/`upper` (the IK's bounds, XR_IK_JOINT_MARGIN inside the URDF
limits) every slide step and every catch-up step is kept inside them. A joint that is
already outside (the measured pose at [r]) may only move back in, never further out.
Default limit 5 Nm (RT6g: at 7 Nm the shoulder ran 5 C above the heat model).

[RT6h] COST. Sliding with both arms capped took 1.6 ms a frame (about 280 gravity calls of
6 us, scan 16 + bisect 12 per search), all of it holding the GIL the arm controller's
250 Hz thread needs. Now scan 8 + bisect 10 (the limit is found to 1e-4 of the step),
and a frame whose IK target and accepted pose are unchanged repeats the last answer.
"""

import os

import numpy as np

SHOULDER_PITCH = (0, 7)           # left, right, in the 14-joint arm vector
ARM_SLICES = (slice(0, 7), slice(7, 14))
DEFAULT_LIMIT_NM = 5.0


class TorqueCapError(ValueError):
    pass


class ShoulderTorqueCap:
    def __init__(self, limit_nm, gravity_fn, scan=8, bisect=10, frequency=30.0,
                 catchup_vel=None, slide_iters=3, lower=None, upper=None):
        self.limit = float(limit_nm)
        self.lower = None if lower is None else np.asarray(lower, dtype=float).copy()
        self.upper = None if upper is None else np.asarray(upper, dtype=float).copy()
        self.bound_clips = [0, 0]          # slide/catch-up steps cut by a bound, since the last read
        if catchup_vel is None:
            catchup_vel = float(os.environ.get("XR_CAP_CATCHUP_VEL", "0.5") or 0.5)
        if not np.isfinite(catchup_vel) or catchup_vel <= 0.0:
            raise TorqueCapError(f"XR_CAP_CATCHUP_VEL={catchup_vel!r} must be > 0")
        self.catchup_vel = float(catchup_vel)
        self.catchup_step = self.catchup_vel / float(frequency)
        self.slide_iters = int(slide_iters)
        self.recovering = [False, False]
        if not np.isfinite(self.limit) or self.limit < 0.0:
            raise TorqueCapError(f"XR_SHOULDER_TAU_CAP={limit_nm!r} must be >= 0 (0 = off)")
        self.gravity_fn = gravity_fn
        self.scan, self.bisect = int(scan), int(bisect)
        self.q_acc = None
        self._memo = None                  # (sol_q, q_acc, recovering) -> (q_cmd, tau_cmd, capped)
        self.capped = [False, False]       # this frame, per arm
        self.frames_capped = [0, 0]        # since the last stats read
        self.pred_max = [0.0, 0.0]         # max |IK feed-forward| since the last stats read
        self.cmd_max = [0.0, 0.0]          # max |commanded feed-forward| since the last read

    @property
    def enabled(self):
        return self.limit > 0.0

    def describe(self):
        if not self.enabled:
            return "[cap] shoulder pitch torque cap OFF (XR_SHOULDER_TAU_CAP=0)"
        return (f"[cap] shoulder pitch gravity torque capped at {self.limit:g} Nm while following "
                f"(XR_SHOULDER_TAU_CAP; sliding: the arm keeps following along the limit, only the part "
                f"that raises the torque is held; catch-up {self.catchup_vel:g} rad/s; "
                f"{'kept inside the IK joint bounds' if self.lower is not None else 'NO joint bounds'}; 0 = off)")

    def reset(self, q_accepted):
        """The pose to measure the first step from (the measured arm pose at [r])."""
        self.q_acc = np.asarray(q_accepted, dtype=float).copy()

    def _tau(self, q):
        return np.asarray(self.gravity_fn(q), dtype=float)

    def _search(self, q_from, q_to, arm, j):
        """Largest s in [0, 1] with |tau_j(q_from + s (q_to - q_from))| <= limit, taken
        before the first crossing (only arm `arm` moves). Returns (s, q)."""
        def at(s):
            q = q_from.copy()
            q[ARM_SLICES[arm]] = q_from[ARM_SLICES[arm]] + s * (q_to[ARM_SLICES[arm]] - q_from[ARM_SLICES[arm]])
            return q
        lo = 0.0
        hi = None
        for k in range(1, self.scan + 1):
            s = k / self.scan
            if abs(self._tau(at(s))[j]) > self.limit:
                hi = s
                break
            lo = s
        if hi is None:
            return 1.0, at(1.0)
        for _ in range(self.bisect):
            mid = 0.5 * (lo + hi)
            if abs(self._tau(at(mid))[j]) > self.limit:
                hi = mid
            else:
                lo = mid
        return lo, at(lo)

    def _grad_abs(self, q, a, j, eps=1e-4):
        """Gradient of |tau_j| over arm `a`'s 7 joints at q (finite differences)."""
        sl = ARM_SLICES[a]
        t0 = self._tau(q)[j]
        sgn = 1.0 if t0 >= 0 else -1.0
        g = np.zeros(7)
        for k in range(7):
            qk = q.copy()
            qk[sl.start + k] += eps
            g[k] = (self._tau(qk)[j] - t0) / eps
        return sgn * g

    def _box(self, q_at, a, x):
        """x (arm `a`'s 7-slice) moved into the joint bounds, widened to include q_at's
        slice: a joint outside may stay where it is or move back in, never further out."""
        if self.lower is None:
            return x
        sl = ARM_SLICES[a]
        lo = np.minimum(self.lower[sl], q_at[sl])
        hi = np.maximum(self.upper[sl], q_at[sl])
        y = np.clip(x, lo, hi)
        if np.any(y != x):
            self.bound_clips[a] += 1
        return y

    def _slide(self, q_cmd, tgt, a, j):
        """Move arm `a` in q_cmd towards tgt (its 7-slice): along the path to the limit,
        then along the limit surface. Returns the new q (full 14)."""
        sl = ARM_SLICES[a]
        base = q_cmd.copy()
        target = base.copy()
        target[sl] = tgt
        s1, qb = self._search(base, target, a, j)
        rem = (tgt - base[sl]) * (1.0 - s1)
        for _ in range(self.slide_iters):
            if float(np.max(np.abs(rem))) < 1e-6:
                break
            g = self._grad_abs(qb, a, j)
            gr = float(g @ rem)
            if gr > 0.0 and float(g @ g) > 1e-12:
                rem = rem - gr / float(g @ g) * g
            rem = self._box(qb, a, qb[sl] + rem) - qb[sl]
            nxt = qb.copy()
            nxt[sl] = qb[sl] + rem
            s2, qb = self._search(qb, nxt, a, j)
            rem = rem * (1.0 - s2)
        return qb

    def step(self, sol_q, sol_tauff, q_measured=None):
        """Returns (q, tauff). The inputs come back unchanged (same objects) when neither
        arm is over the limit nor catching up. q_measured seeds the accepted pose on the
        first call when reset() was not called."""
        self.capped = [False, False]
        if not self.enabled:
            return sol_q, sol_tauff
        tau_new = np.asarray(sol_tauff, dtype=float)
        for a, j in enumerate(SHOULDER_PITCH):
            self.pred_max[a] = max(self.pred_max[a], abs(float(tau_new[j])))
        over = [abs(float(tau_new[j])) > self.limit for j in SHOULDER_PITCH]
        if self.q_acc is None:
            self.q_acc = np.asarray(q_measured if q_measured is not None else sol_q, dtype=float).copy()
        if not any(over) and not any(self.recovering):
            self.q_acc = np.asarray(sol_q, dtype=float).copy()
            for a, j in enumerate(SHOULDER_PITCH):
                self.cmd_max[a] = max(self.cmd_max[a], abs(float(tau_new[j])))
            return sol_q, sol_tauff
        q_new = np.asarray(sol_q, dtype=float)
        m = self._memo
        if (m is not None and m[2] == self.recovering and np.array_equal(m[0], q_new)
                and np.array_equal(m[1], self.q_acc)):
            # Same IK target, same accepted pose: the search would repeat itself.
            self.capped = list(m[5])
            for a in range(2):
                self.frames_capped[a] += int(self.capped[a])
                self.cmd_max[a] = max(self.cmd_max[a], abs(float(m[4][SHOULDER_PITCH[a]])))
            self.recovering = list(m[6])
            return m[3].copy(), m[4].copy()
        memo_key = (q_new.copy(), self.q_acc.copy(), list(self.recovering))
        q_cmd = self.q_acc.copy()
        tau_acc = self._tau(self.q_acc)
        for a, j in enumerate(SHOULDER_PITCH):
            sl = ARM_SLICES[a]
            tgt = self._box(q_cmd, a, q_new[sl].copy())
            caught_up = True
            if self.recovering[a]:
                d = tgt - q_cmd[sl]
                m = float(np.max(np.abs(d)))
                if m > self.catchup_step:
                    tgt = q_cmd[sl] + d * (self.catchup_step / m)
                    caught_up = False
            q_t = q_cmd.copy()
            q_t[sl] = tgt
            if abs(float(self._tau(q_t)[j])) <= self.limit:
                q_cmd[sl] = tgt
                if caught_up:
                    self.recovering[a] = False
                continue
            self.capped[a] = True
            self.frames_capped[a] += 1
            self.recovering[a] = True
            if abs(float(tau_acc[j])) > self.limit:
                # Already over (only possible from the first accepted pose): move only if
                # that lowers this arm's torque. [RT6h] The torque of the step actually
                # taken (a catch-up step is not the IK solution), not the IK's.
                if abs(float(self._tau(q_t)[j])) < abs(float(tau_acc[j])):
                    q_cmd[sl] = tgt
                continue
            q_cmd[sl] = self._slide(q_cmd, tgt, a, j)[sl]
        tau_cmd = self._tau(q_cmd)
        self._memo = (*memo_key, q_cmd.copy(), tau_cmd.copy(), list(self.capped), list(self.recovering))
        self.q_acc = q_cmd.copy()
        for a, j in enumerate(SHOULDER_PITCH):
            self.cmd_max[a] = max(self.cmd_max[a], abs(float(tau_cmd[j])))
        return q_cmd, tau_cmd

    def read_stats(self):
        """(pred_max_L, pred_max_R, cmd_max_L, cmd_max_R, frames_capped_L, frames_capped_R)
        since the last call, then reset."""
        out = (*self.pred_max, *self.cmd_max, *self.frames_capped)
        self.pred_max, self.cmd_max, self.frames_capped = [0.0, 0.0], [0.0, 0.0], [0, 0]
        return out


def limit_from_env():
    raw = os.environ.get("XR_SHOULDER_TAU_CAP")
    if raw is None or raw.strip() == "":
        return DEFAULT_LIMIT_NM
    try:
        v = float(raw)
    except ValueError:
        raise TorqueCapError(f"XR_SHOULDER_TAU_CAP={raw!r} is not a number") from None
    if not np.isfinite(v) or v < 0.0:
        raise TorqueCapError(f"XR_SHOULDER_TAU_CAP={raw!r} must be >= 0 (0 = off)")
    return v


def gravity_fn_for(model):
    """Gravity torque of the 14-joint reduced arm model (the IK's), for any q."""
    import pinocchio as pin
    data = model.createData()
    z = np.zeros(model.nv)
    return lambda q: pin.rnea(model, data, np.asarray(q, dtype=float), z, z).copy()
