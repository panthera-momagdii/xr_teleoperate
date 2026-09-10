#!/usr/bin/env python3
"""Read-only full-rate recorder for the G1 arms and the Inspire hands.

Subscribes to rt/lowstate and to both Inspire hand state topics, and writes one row
per lowstate message to a compressed .npz. Built for Part B takes: reach_sweep,
writing, tray_empty, tray_loaded, marker.

    python tools/record_lowstate.py --label reach_sweep
    python tools/record_lowstate.py --label smoke --seconds 20

Read-only by construction: this tool creates readers only. The gate check greps this
file for the SDK's writer class name and must find nothing, which is why that name is
not written out here, not even in prose. It is safe to run alongside a live teleop
session -- it adds a reader to topics PC1 and PC2 already publish and commands nothing.

WHAT IS RECORDED
  Joints 15..28 only -- the 14 arm motors (left arm 15..21, right arm 22..28). The
  legs and waist are deliberately out: on the hoist they are limp until teleop locks
  them, and every Part B label is an arm task. `joint_index` in the file says so.

  Three clocks per row, because they answer different questions:
    t_wall  UTC epoch seconds at receipt -- lines this up with the head camera and
            with anything else timestamped in wall clock.
    t_mono  seconds since this recorder started -- immune to an NTP step mid-take.
    tick    the robot's own counter from LowState, in ms -- the only clock with no
            host-side scheduling jitter in it, and the one that proves no drops.

  Hand angles are held at their last received value (the bridge publishes ~13 Hz
  against lowstate's ~500 Hz), so each row carries the hand sample that was current
  when that lowstate arrived. hand_left_t_mono / hand_right_t_mono carry that
  sample's own arrival time, so the repeats can be collapsed exactly:
      uniq = np.unique(d["hand_left_t_mono"], return_index=True)[1]
  Rows before a side's first sample get angle 0 and t_mono NaN.

FULL RATE, VERIFIED NOT ASSUMED
  The SDK dispatches this tool's handler straight from the DDS listener thread with no
  intermediate queue, and takes one sample per callback. Rather than trust that keeps
  up, the summary reports gaps in the robot's own `tick`: a clean take shows a single
  tick step repeated and 0 gaps. Any dropped sample shows up there as a jump.

Exit codes: 0 rows were recorded and saved, 3 lowstate never streamed.
"""

import argparse
import os
import re
import signal
import sys
import threading
import time

import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(REPO_ROOT)

# No logging_mp.basicConfig() here: this tool logs through stdlib logging. Importing
# hand_config still starts logging_mp's listener lazily, so it is stopped explicitly on
# the way out -- see _stop_logging_mp_listener().
from teleop.robot_control import hand_config  # noqa: E402
from tools.tool_logging import setup_tool_logger  # noqa: E402
from tools._procs import install_reaper, reap_child_processes  # noqa: E402

EXIT_OK, EXIT_SILENT = 0, 3

TOPIC_LOWSTATE = "rt/lowstate"

# Left arm 15..21, right arm 22..28, matching G1_29_JointIndex in
# teleop/robot_control/robot_arm.py. Recorded as a column each, in this order.
JOINT_INDEX = list(range(15, 29))
JOINT_NAMES = [
    "LeftShoulderPitch", "LeftShoulderRoll", "LeftShoulderYaw", "LeftElbow",
    "LeftWristRoll", "LeftWristPitch", "LeftWristYaw",
    "RightShoulderPitch", "RightShoulderRoll", "RightShoulderYaw", "RightElbow",
    "RightWristRoll", "RightWristPitch", "RightWristYaw",
]
N_JOINTS = len(JOINT_INDEX)
N_HAND = 6          # Inspire RH56E2-T1: 6 actuators per hand
N_TEMP = 2          # MotorState_.temperature is array[int16, 2] -- two sensors per motor

# Preallocation. Growing copies the whole buffer on the DDS listener thread, which
# stalls it and drops samples, so takes are sized to never grow.
#
# This is DELIVERED rows per second, not the robot's sample rate. Measured on g1_01
# 2026-09-09: tick advances 1 ms (1 kHz) but ~4 % of rows arrive as a repeated tick,
# so the recorder sees ~1040 rows/s. Sizing on 500 Hz grew the buffer mid-take.
NOMINAL_HZ = 1100.0

# np.zeros is calloc-backed and lazily mapped, so a generous open-ended reservation
# costs address space rather than resident memory. 15 min at ~296 B/row is ~290 MB
# virtual, committed only as rows are actually written.
OPEN_ENDED_PREALLOC_S = 900.0


class Recorder:
    """Fixed-column ring-free buffer. Appends happen on the DDS listener thread."""

    def __init__(self, capacity):
        self.lock = threading.Lock()
        self.n = 0
        self.cap = int(capacity)
        self._alloc(self.cap)

        # Latest hand sample per side, held between hand messages.
        self.hand = {
            "l": {"angle": np.zeros(N_HAND, np.int16), "t": np.nan, "count": 0},
            "r": {"angle": np.zeros(N_HAND, np.int16), "t": np.nan, "count": 0},
        }
        self.short_hand_msgs = 0
        # Delivered rows != distinct robot samples: the wire repeats a tick now and
        # then. Counted here in O(1) so the 10 s line can quote the real rate.
        self.last_tick = None
        self.dup_ticks = 0
        self.gap_events = 0
        self.missed = 0
        self.grows = 0

    def _alloc(self, cap):
        self.t_wall = np.zeros(cap, np.float64)
        self.t_mono = np.zeros(cap, np.float64)
        self.tick = np.zeros(cap, np.uint32)
        self.q = np.zeros((cap, N_JOINTS), np.float32)
        self.dq = np.zeros((cap, N_JOINTS), np.float32)
        self.tau_est = np.zeros((cap, N_JOINTS), np.float32)
        self.temperature = np.zeros((cap, N_JOINTS, N_TEMP), np.int16)
        self.imu_rpy = np.zeros((cap, 3), np.float32)
        self.hand_l = np.zeros((cap, N_HAND), np.int16)
        self.hand_r = np.zeros((cap, N_HAND), np.int16)
        self.hand_l_t = np.zeros(cap, np.float64)
        self.hand_r_t = np.zeros(cap, np.float64)

    def _grow(self):
        self.grows += 1
        old, self.cap = self.cap, self.cap * 2
        keep = {k: getattr(self, k) for k in
                ("t_wall", "t_mono", "tick", "q", "dq", "tau_est", "temperature",
                 "imu_rpy", "hand_l", "hand_r", "hand_l_t", "hand_r_t")}
        self._alloc(self.cap)
        for k, v in keep.items():
            getattr(self, k)[:old] = v

    def on_lowstate(self, msg, t_mono, t_wall):
        with self.lock:
            if self.n >= self.cap:
                self._grow()
            i = self.n
            ms = msg.motor_state
            for j, idx in enumerate(JOINT_INDEX):
                m = ms[idx]
                self.q[i, j] = m.q
                self.dq[i, j] = m.dq
                self.tau_est[i, j] = m.tau_est
                t = m.temperature
                self.temperature[i, j, 0] = t[0]
                self.temperature[i, j, 1] = t[1]
            self.imu_rpy[i] = msg.imu_state.rpy
            tick = msg.tick
            if self.last_tick is not None:
                step = (int(tick) - int(self.last_tick)) & 0xFFFFFFFF
                if step == 0:
                    self.dup_ticks += 1
                elif step > 1:
                    self.gap_events += 1
                    self.missed += step - 1
            self.last_tick = tick
            self.tick[i] = tick
            self.t_mono[i] = t_mono
            self.t_wall[i] = t_wall
            hl, hr = self.hand["l"], self.hand["r"]
            self.hand_l[i] = hl["angle"]
            self.hand_r[i] = hr["angle"]
            self.hand_l_t[i] = hl["t"]
            self.hand_r_t[i] = hr["t"]
            self.n = i + 1

    def on_hand(self, side, msg, t_mono):
        angle = getattr(msg, "angle_act", None)
        if angle is None or len(angle) < N_HAND:
            # The bridge is expected to send 6. A short sequence is worth counting
            # rather than crashing a take that is otherwise fine.
            with self.lock:
                self.short_hand_msgs += 1
            if angle is None or len(angle) == 0:
                return
        with self.lock:
            h = self.hand[side]
            h["angle"] = np.asarray(angle[:N_HAND], np.int16)
            h["t"] = t_mono
            h["count"] += 1

    def snapshot_counts(self):
        with self.lock:
            return (self.n, self.hand["l"]["count"], self.hand["r"]["count"],
                    self.dup_ticks, self.missed)

    def view(self):
        """Trim to the rows actually written. Call only after the readers are done."""
        with self.lock:
            n = self.n
            return {
                "t_wall": self.t_wall[:n], "t_mono": self.t_mono[:n],
                "tick": self.tick[:n], "q": self.q[:n], "dq": self.dq[:n],
                "tau_est": self.tau_est[:n], "temperature": self.temperature[:n],
                "imu_rpy": self.imu_rpy[:n],
                "hand_left_angle": self.hand_l[:n], "hand_right_angle": self.hand_r[:n],
                "hand_left_t_mono": self.hand_l_t[:n],
                "hand_right_t_mono": self.hand_r_t[:n],
            }


def _stop_logging_mp_listener():
    """Reap logging_mp's listener process before os._exit().

    logging_mp forks a NON-DAEMON listener the first time anything calls getLogger --
    which importing hand_config does -- and only ever reaps it from an atexit hook.
    This tool leaves via os._exit() so that DDS teardown cannot hang a finished take,
    and os._exit() skips atexit. Without this the listener is orphaned, and because it
    inherited stdout it also keeps a piped `tail` waiting forever. The stale
    domain0_census processes on this host are exactly that, not census hanging.

    Bounded on every path: the tool is already saved by the time this runs and must
    never be what stops it exiting.
    """
    try:
        import logging_mp
        mgr = getattr(logging_mp, "_internal_manager", None)
        proc = getattr(mgr, "_listener_process", None) if mgr else None
        if proc is None or not proc.is_alive():
            return
        t = threading.Thread(target=mgr._shutdown, daemon=True)
        t.start()
        t.join(2.0)
        if proc.is_alive():
            proc.terminate()
            proc.join(1.0)
        if proc.is_alive():
            proc.kill()
    except Exception:
        pass


def tick_gaps(tick):
    """Return (step_mode, n_gaps, max_gap) over the robot's own counter.

    uint32 differences are taken in int64 so a wrap does not read as a huge jump.
    """
    if tick.size < 2:
        return 0, 0, 0
    d = np.diff(tick.astype(np.int64))
    d = d[d > 0]                      # drop the wrap and any repeat
    if d.size == 0:
        return 0, 0, 0
    vals, counts = np.unique(d, return_counts=True)
    step = int(vals[np.argmax(counts)])
    gaps = d[d > step]
    return step, int(gaps.size), int(gaps.max()) if gaps.size else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--label", required=True,
                        help="take name, e.g. reach_sweep / writing / tray_loaded")
    parser.add_argument("--domain", type=int, default=0, help="DDS domain (default 0)")
    parser.add_argument("--iface", type=str, default="enP7s7",
                        help="network interface (default enP7s7)")
    parser.add_argument("--seconds", type=float, default=0.0,
                        help="stop after N s (default 0 = run until Ctrl+C)")
    parser.add_argument("--out-dir", type=str,
                        default=os.path.join(REPO_ROOT, "logs", "partb"))
    parser.add_argument("--report-every", type=float, default=10.0)
    args = parser.parse_args()

    label = args.label.strip()
    if not re.fullmatch(r"[A-Za-z0-9._-]+", label):
        parser.error("--label must be letters, digits, dot, dash or underscore")

    os.makedirs(args.out_dir, exist_ok=True)
    utc = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    out_path = os.path.join(args.out_dir, f"{label}_{utc}.npz")

    log = setup_tool_logger("record_lowstate",
                            filename=os.path.join("partb", f"{label}_{utc}.log"))
    log.info("READ-ONLY. This tool creates readers only.")
    log.info(f"label={label}  domain={args.domain}  iface={args.iface}")
    log.info(f"output: {out_path}")

    prealloc = int(NOMINAL_HZ * (args.seconds * 1.2 if args.seconds > 0
                                else OPEN_ENDED_PREALLOC_S)) + 1000
    rec = Recorder(prealloc)

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as hg_LowState

    ChannelFactoryInitialize(args.domain, args.iface)
    state_cls = hand_config.state_type()
    topics = {"l": hand_config.TOPIC_LEFT_STATE, "r": hand_config.TOPIC_RIGHT_STATE}
    log.info(f"topics: {TOPIC_LOWSTATE}, {topics['l']}, {topics['r']}")

    t0 = time.monotonic()

    def on_lowstate(msg):
        rec.on_lowstate(msg, time.monotonic() - t0, time.time())

    def hand_handler(side):
        def on_hand(msg):
            rec.on_hand(side, msg, time.monotonic() - t0)
        return on_hand

    # queueLen=0: the handler is called straight from the DDS listener thread with no
    # bounded queue in between, so nothing is silently dropped on a full queue. The
    # handler is a few array writes and stays well clear of the 2 ms budget.
    subs = []
    sub = ChannelSubscriber(TOPIC_LOWSTATE, hg_LowState)
    sub.Init(on_lowstate, 0)
    subs.append(sub)
    for side, topic in topics.items():
        s = ChannelSubscriber(topic, state_cls)
        s.Init(hand_handler(side), 0)
        subs.append(s)

    stop = threading.Event()

    def request_stop(signum, _frame):
        log.info(f"signal {signal.Signals(signum).name} -- finishing the take")
        stop.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    log.info("recording -- Ctrl+C to stop and save")
    last_report = t0
    last_rows = last_hl = last_hr = last_dup = 0
    try:
        while not stop.is_set():
            stop.wait(0.2)
            now = time.monotonic()
            if args.seconds > 0 and (now - t0) >= args.seconds:
                break
            dt = now - last_report
            if dt >= args.report_every:
                rows, hl, hr, dup, missed = rec.snapshot_counts()
                n = rec.n
                hot = int(rec.temperature[:n].max()) if n else 0
                d_rows, d_dup = rows - last_rows, dup - last_dup
                log.info(
                    f"t={now - t0:6.1f}s  rows={rows:<8d} "
                    f"{(d_rows - d_dup) / dt:7.1f} Hz ({d_dup} dup) | "
                    f"hands L {(hl - last_hl) / dt:5.1f} Hz R {(hr - last_hr) / dt:5.1f} Hz | "
                    f"missed {missed} | arm temp max {hot} C"
                )
                last_report, last_rows, last_hl, last_hr, last_dup = (
                    now, rows, hl, hr, dup)
    except KeyboardInterrupt:
        log.info("KeyboardInterrupt -- finishing the take")

    elapsed = time.monotonic() - t0
    for s in subs:
        try:
            s.Close()
        except Exception as exc:
            log.warning(f"reader close: {exc}")

    data = rec.view()
    n = data["t_mono"].size
    rows, hl, hr, dup, missed = rec.snapshot_counts()
    if n == 0:
        log.error(f"no lowstate in {elapsed:.1f}s -- nothing saved. Is PC1 publishing "
                  f"{TOPIC_LOWSTATE} on domain {args.domain} via {args.iface}?")
        return EXIT_SILENT

    step, n_gaps, max_gap = tick_gaps(data["tick"])
    meta = {
        "label": np.array(label),
        "started_utc": np.array(utc),
        "duration_s": np.array(elapsed),
        "joint_index": np.array(JOINT_INDEX, np.int32),
        "joint_names": np.array(JOINT_NAMES),
        "hand_model": np.array(hand_config.HAND_MODEL),
        "topics": np.array([TOPIC_LOWSTATE, topics["l"], topics["r"]]),
        "domain": np.array(args.domain),
        "iface": np.array(args.iface),
        "hand_samples_l": np.array(hl),
        "hand_samples_r": np.array(hr),
        "short_hand_msgs": np.array(rec.short_hand_msgs),
        "tick_step": np.array(step),
        "tick_gaps": np.array(n_gaps),
        "tick_max_gap": np.array(max_gap),
        "buffer_grows": np.array(rec.grows),
        "dup_ticks": np.array(dup),
        "missed_samples": np.array(missed),
    }

    tmp = out_path + ".tmp.npz"
    np.savez_compressed(tmp, **data, **meta)
    os.replace(tmp, out_path)

    tau = data["tau_est"]
    worst = int(np.argmax(np.abs(tau).max(axis=0)))
    log.info(f"saved {out_path}  ({os.path.getsize(out_path) / 1e6:.1f} MB)")
    uniq = n - dup
    expected = uniq + missed
    log.info(f"rows={n} delivered ({n / elapsed:.1f}/s) = {uniq} distinct samples "
             f"({uniq / elapsed:.1f} Hz) + {dup} duplicate tick(s)")
    log.info(f"tick step {step} ms; {n_gaps} gap(s), max {max_gap} ms, "
             f"{missed} sample(s) missed of {expected} "
             f"({100.0 * uniq / expected if expected else 100.0:.2f}% captured)")
    if rec.grows:
        log.warning(f"buffer grew {rec.grows}x mid-take -- each growth stalls the DDS "
                    f"thread and can drop samples. Raise NOMINAL_HZ / "
                    f"OPEN_ENDED_PREALLOC_S, or pass --seconds.")
    if dup:
        log.info("duplicates are kept as-is; drop them with "
                 "d['tick'][np.unique(d['tick'], return_index=True)[1]]")
    log.info(f"hands L={hl} R={hr} samples"
             + (f"  ({rec.short_hand_msgs} short msg)" if rec.short_hand_msgs else ""))
    log.info(f"tau_est range [{tau.min():+.3f}, {tau.max():+.3f}] Nm; "
             f"largest |tau| on {JOINT_NAMES[worst]} (joint {JOINT_INDEX[worst]})")
    log.info(f"arm temp {int(data['temperature'].min())}..{int(data['temperature'].max())} C")
    return EXIT_OK


if __name__ == "__main__":
    install_reaper()                # covers Ctrl-C and `kill`, which os._exit() cannot
    rc = main()
    sys.stdout.flush()
    sys.stderr.flush()
    # [panthera] Was _stop_logging_mp_listener(), a local copy of this. The logic is
    # now shared with every other tool in tools/_procs.py, and reaps any other
    # multiprocessing child as well as logging_mp's listener.
    reap_child_processes()
    # Leave without waiting on DDS teardown, which can block a finished take.
    os._exit(rc)
