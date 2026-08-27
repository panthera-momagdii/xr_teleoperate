#!/usr/bin/env python3
"""Single-joint step response on one hand. THIS TOOL COMMANDS THE HARDWARE.

Purpose on robot day:
  H2  command slot 4k+2 and watch slot 4k+3. If the distal joint follows without
      being commanded, it is mechanically coupled (passive), which is what we
      suspect for slots 3, 7, 11, 15.
  gain units  the PR sets finger kp 0.10 / thumb kp 1.0 with a comment that thumb
      motors report in N*m and fingers in mNm. A measured step response is what
      turns that comment into a fact.

Interlocks, because this moves a real hand:
  * refuses to run unless PANTHERA_HAND_CMD_OK=1
  * --amplitude hard maximum 0.5 rad, --hold hard maximum 10 s
  * waits for state with the same fail-closed rules as Dex5_1_Controller: bounded
    by hand_config.STATE_TIMEOUT_S, and refuses a motor count that is not the
    expected one
  * aborts to "hold current q" and exits 4 if any temperature on that hand exceeds
    hand_config.TEMP_LIMIT_C
  * --dry-run prints what it would send and never constructs a publisher

Sequence: hold current q for 1 s, ramp the one joint to q+amp over 0.5 s, hold,
ramp back over 0.5 s, hold 1 s, exit.

  PANTHERA_HAND_CMD_OK=1 python tools/hand_step.py --domain 0 --iface <nic> \
      --side left --joint 2 --amplitude 0.3 --hold 5

Exit codes: 0 ok, 2 bad arguments/interlock, 3 no state, 4 thermal abort,
            5 motor count mismatch.
"""

import argparse
import csv
import logging
import os
import sys
import threading
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(REPO_ROOT)

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber  # noqa: E402
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_  # noqa: E402

from teleop.robot_control import hand_config  # noqa: E402
from tools.tool_logging import setup_tool_logger, timestamp  # noqa: E402

EXIT_OK, EXIT_ARGS, EXIT_NO_STATE, EXIT_THERMAL, EXIT_COUNT = 0, 2, 3, 4, 5

AMPLITUDE_MAX_RAD = 0.5
HOLD_MAX_S = 10.0
RAMP_S = 0.5
SETTLE_S = 1.0


class StateWatcher:
    """Latest state for one side, plus the fail-closed first-message check."""

    def __init__(self, topic, expected_motors):
        self.expected = expected_motors
        self.lock = threading.Lock()
        self.q = None
        self.dq = None
        self.tau = None
        self.temp = None
        self.n_motor = None
        self.n_press = None
        self.error = None
        self.sub = ChannelSubscriber(topic, HandState_)
        self.sub.Init(self._on_state)

    def _on_state(self, msg):
        try:
            n_motor, n_press = hand_config.read_hand_counts(msg)
            with self.lock:
                if self.n_motor is None:
                    self.n_motor, self.n_press = n_motor, n_press
                    if n_motor != self.expected:
                        raise RuntimeError(
                            f"motor_state has {n_motor} entries; 7 = Dex3-1 fitted, "
                            f"expected {self.expected} (Dex5-1P)")
                self.q = [float(m.q) for m in msg.motor_state]
                self.dq = [float(m.dq) for m in msg.motor_state]
                self.tau = [float(m.tau_est) for m in msg.motor_state]
                self.temp = [hand_config.motor_temperature(m) for m in msg.motor_state]
        except BaseException as exc:            # surfaced by wait_for_state
            self.error = exc

    def wait_for_state(self, timeout_s, log):
        deadline = time.monotonic() + timeout_s
        last_warn = 0.0
        while True:
            if self.error is not None:
                raise self.error
            with self.lock:
                if self.q is not None:
                    return
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"no HandState_ within {timeout_s}s. Check hand power, the DDS "
                    f"domain/interface, and DEX5_TOPIC_PREFIX "
                    f"(currently '{hand_config.TOPIC_PREFIX}').")
            if time.monotonic() - last_warn >= 1.0:
                last_warn = time.monotonic()
                log.warning(f"waiting for state... ({deadline - time.monotonic():.0f}s left)")
            time.sleep(0.01)

    def snapshot(self):
        with self.lock:
            return (list(self.q), list(self.dq), list(self.tau), list(self.temp))


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--domain", type=int, default=1,
                        help="DDS domain id (default 1 = this laptop; robot day uses 0)")
    parser.add_argument("--iface", type=str, default=None, help="network interface for DDS")
    parser.add_argument("--prefix", type=str, default=hand_config.TOPIC_PREFIX,
                        help=f"hand topic prefix (default {hand_config.TOPIC_PREFIX})")
    parser.add_argument("--side", choices=["left", "right"], required=True)
    parser.add_argument("--joint", type=int, required=True,
                        help=f"DDS slot to step, 0..{hand_config.NUM_JOINTS_EXPECTED - 1}")
    parser.add_argument("--amplitude", type=float, default=0.3,
                        help=f"step size in rad (hard max {AMPLITUDE_MAX_RAD})")
    parser.add_argument("--hold", type=float, default=5.0,
                        help=f"hold time at the stepped position (hard max {HOLD_MAX_S} s)")
    parser.add_argument("--kp", type=float, default=None, help="override kp (default: hand_config by group)")
    parser.add_argument("--kd", type=float, default=None, help="override kd (default: hand_config by group)")
    parser.add_argument("--rate", type=float, default=100.0, help="command rate in Hz")
    parser.add_argument("--csv", type=str, default=None, help="CSV path (default logs/hand_step_<ts>.csv)")
    parser.add_argument("--dry-run", action="store_true",
                        help="print the messages that would be sent; constructs no publisher")
    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()

    n_joints = hand_config.NUM_JOINTS_EXPECTED
    if not 0 <= args.joint < n_joints:
        parser.error(f"--joint must be in 0..{n_joints - 1}")
    if not 0 < args.amplitude <= AMPLITUDE_MAX_RAD:
        parser.error(f"--amplitude must be in (0, {AMPLITUDE_MAX_RAD}] rad")
    if not 0 < args.hold <= HOLD_MAX_S:
        parser.error(f"--hold must be in (0, {HOLD_MAX_S}] s")

    group = hand_config.group_of(args.joint)
    kp = args.kp if args.kp is not None else hand_config.GAINS[group][0]
    kd = args.kd if args.kd is not None else hand_config.GAINS[group][1]

    log = setup_tool_logger("hand_step")
    log.info(hand_config.describe())
    log.info(f"side={args.side} joint={args.joint} ({group}) amplitude={args.amplitude} rad "
             f"hold={args.hold}s kp={kp} kd={kd} rate={args.rate}Hz dry_run={args.dry_run}")

    # Which columns the CSV records: the commanded joint, its distal neighbour
    # (4k+3 when commanding 4k+2 -- the H2 question), and the thumb base.
    neighbour = None
    if args.joint % 4 == 2 and args.joint + 1 < n_joints:
        neighbour = args.joint + 1
    watched = [args.joint] + ([neighbour] if neighbour is not None else []) + [hand_config.THUMB_BASE_INDEX]
    watched = sorted(set(w for w in watched if w < n_joints))
    log.info(f"CSV columns for slots {watched} "
             f"(commanded={args.joint}, distal neighbour={neighbour}, "
             f"thumb base={hand_config.THUMB_BASE_INDEX})")

    if args.dry_run:
        msg = hand_config.make_hand_cmd(n_joints)
        msg.motor_cmd[args.joint].kp = kp
        msg.motor_cmd[args.joint].kd = kd
        log.info("DRY RUN: no DDS initialisation, no publisher, nothing sent.")
        log.info(f"would publish HandCmd_ on {args.prefix}/{args.side}/cmd with "
                 f"{len(msg.motor_cmd)} motor_cmd entries")
        for idx in watched:
            c = msg.motor_cmd[idx]
            log.info(f"  slot {idx:2d} ({hand_config.group_of(idx):6s}) "
                     f"mode=0x{c.mode:02x} q={c.q} dq={c.dq} tau={c.tau} kp={c.kp} kd={c.kd}")
        log.info("planned sequence: hold current q 1.0 s -> ramp %+0.3f rad over %.1f s -> "
                 "hold %.1f s -> ramp back over %.1f s -> hold %.1f s"
                 % (args.amplitude, RAMP_S, args.hold, RAMP_S, SETTLE_S))
        return EXIT_OK

    if os.environ.get("PANTHERA_HAND_CMD_OK") != "1":
        log.error("REFUSING: this tool commands a real hand. Set PANTHERA_HAND_CMD_OK=1 "
                  "to confirm you intend that, or use --dry-run.")
        return EXIT_ARGS

    # Imported here so that --dry-run provably cannot construct a publisher.
    from unitree_sdk2py.core.channel import ChannelPublisher

    ChannelFactoryInitialize(args.domain, args.iface)
    state = StateWatcher(f"{args.prefix}/{args.side}/state", n_joints)
    try:
        state.wait_for_state(hand_config.STATE_TIMEOUT_S, log)
    except TimeoutError as exc:
        log.error(str(exc))
        return EXIT_NO_STATE
    except RuntimeError as exc:
        log.error(f"{args.side}: {exc}")
        return EXIT_COUNT
    log.info(f"state ok: n_motor={state.n_motor} n_press={state.n_press}")

    pub = ChannelPublisher(f"{args.prefix}/{args.side}/cmd", HandCmd_)
    pub.Init()

    msg = hand_config.make_hand_cmd(n_joints)
    msg.motor_cmd[args.joint].kp = kp
    msg.motor_cmd[args.joint].kd = kd

    q0, _, _, _ = state.snapshot()
    for idx in range(n_joints):
        msg.motor_cmd[idx].q = q0[idx]
    start_q = q0[args.joint]
    target_q = start_q + args.amplitude
    log.info(f"start q[{args.joint}]={start_q:.4f} -> target {target_q:.4f} rad")

    csv_path = args.csv or os.path.join(REPO_ROOT, "logs", f"hand_step_{timestamp()}.csv")
    rows = []
    period = 1.0 / args.rate
    t_start = time.monotonic()

    phases = [("hold_start", SETTLE_S), ("ramp_up", RAMP_S), ("hold_step", args.hold),
              ("ramp_down", RAMP_S), ("hold_end", SETTLE_S)]

    def q_cmd_at(phase, frac):
        if phase == "hold_start":
            return start_q
        if phase == "ramp_up":
            return start_q + args.amplitude * frac
        if phase == "hold_step":
            return target_q
        if phase == "ramp_down":
            return target_q - args.amplitude * frac
        return start_q

    thermal_abort = False
    for phase, duration in phases:
        p_start = time.monotonic()
        while True:
            now = time.monotonic()
            elapsed = now - p_start
            if elapsed >= duration:
                break
            q_cmd = q_cmd_at(phase, elapsed / duration if duration else 1.0)

            q, dq, tau, temp = state.snapshot()
            hot = [(i, t) for i, t in enumerate(temp) if t is not None and t > hand_config.TEMP_LIMIT_C]
            if hot:
                log.error(f"THERMAL ABORT: slots {[i for i, _ in hot]} above "
                          f"{hand_config.TEMP_LIMIT_C} C ({[round(t, 1) for _, t in hot]}). "
                          f"Holding current q and stopping.")
                for idx in range(n_joints):
                    msg.motor_cmd[idx].q = q[idx]
                pub.Write(msg)
                # Record the sample that triggered the abort. Without this the CSV can
                # come back empty -- exactly when it is most wanted as evidence.
                rows.append({
                    "t": round(now - t_start, 4), "phase": "thermal_abort",
                    "joint": args.joint, "q_cmd": round(q[args.joint], 6),
                    **{f"q_{i}": round(q[i], 6) for i in watched},
                    **{f"dq_{i}": round(dq[i], 6) for i in watched},
                    **{f"tau_{i}": round(tau[i], 6) for i in watched},
                    **{f"temp_{i}": temp[i] for i in watched},
                })
                thermal_abort = True
                break

            msg.motor_cmd[args.joint].q = q_cmd
            pub.Write(msg)

            rows.append({
                "t": round(now - t_start, 4), "phase": phase,
                "joint": args.joint, "q_cmd": round(q_cmd, 6),
                **{f"q_{i}": round(q[i], 6) for i in watched},
                **{f"dq_{i}": round(dq[i], 6) for i in watched},
                **{f"tau_{i}": round(tau[i], 6) for i in watched},
                **{f"temp_{i}": temp[i] for i in watched},
            })
            slack = period - (time.monotonic() - now)
            if slack > 0:
                time.sleep(slack)
        if thermal_abort:
            break

    if rows:
        with open(csv_path, "w", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
        log.info(f"CSV written to {csv_path} ({len(rows)} rows)")

    if thermal_abort:
        return EXIT_THERMAL

    # --- analysis ---------------------------------------------------------
    step_rows = [r for r in rows if r["phase"] in ("ramp_up", "hold_step")]
    key = f"q_{args.joint}"
    if step_rows:
        ramp_t0 = next(r["t"] for r in rows if r["phase"] == "ramp_up")
        target_90 = start_q + 0.9 * args.amplitude
        reached = [r for r in step_rows if (r[key] - start_q) >= 0.9 * args.amplitude]
        t90 = (reached[0]["t"] - ramp_t0) if reached else None
        peak = max(r[key] for r in step_rows)
        overshoot = ((peak - target_q) / args.amplitude * 100.0) if args.amplitude else 0.0
        final = step_rows[-1][key]
        log.info(f"commanded slot {args.joint}: start {start_q:.4f} target {target_q:.4f} "
                 f"peak {peak:.4f} final {final:.4f}")
        log.info(f"time to 90% ({target_90:.4f} rad): "
                 f"{f'{t90:.3f} s' if t90 is not None else 'NEVER REACHED'}")
        log.info(f"overshoot: {overshoot:+.1f}% of the commanded step")

    if neighbour is not None:
        nkey = f"q_{neighbour}"
        n_series = [r[nkey] for r in rows]
        n_range = max(n_series) - min(n_series)
        c_series = [r[key] for r in rows]
        c_range = max(c_series) - min(c_series)
        moved = n_range > 1e-3
        log.info(f"H2: distal neighbour slot {neighbour} travelled {n_range:.4f} rad while "
                 f"commanded slot {args.joint} travelled {c_range:.4f} rad -> "
                 f"{'NEIGHBOUR MOVED (coupled/passive)' if moved else 'neighbour flat (independent)'}")
        if moved and c_range > 1e-3:
            log.info(f"H2: coupling ratio distal/proximal = {n_range / c_range:.3f}")
    else:
        log.info(f"H2: slot {args.joint} is not a 4k+2 slot, so no distal neighbour was watched")

    return EXIT_OK


if __name__ == "__main__":
    rc = main()
    logging.shutdown()
    os._exit(rc)
