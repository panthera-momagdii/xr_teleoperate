#!/usr/bin/env python3
"""READ-ONLY probe of the two hand state topics. Never publishes anything.

This is the tool that answers the questions we could not answer on 2026-08-24,
when the hands were declared on the bus but sent zero samples in 600 s:

  H1  how many motor_state entries (20 = Dex5-1P, 7 = Dex3-1)?
  H1  how many press_sensor_state modules, and which of the 12 slots per module
      actually report pressure -- the tactile index map
  H2  do the distal slots 4k+3 move whenever slot 4k+2 moves (passive coupling)?

Robot day:
    python tools/hand_probe.py --domain 0 --iface <nic> --seconds 60

There is deliberately no publisher anywhere in this file: it imports only
ChannelFactoryInitialize and ChannelSubscriber. The gate check greps this file for
the SDK's publisher class name and must find nothing -- which is why that class name
is not written out here, not even in prose.

Exit codes: 0 both sides streamed, 3 one or both sides were silent, 2 bad arguments.
"""

import argparse
import json
import logging
import os
import sys
import threading
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(REPO_ROOT)

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber  # noqa: E402
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_  # noqa: E402

from teleop.robot_control import hand_config  # noqa: E402
from tools.tool_logging import setup_tool_logger, timestamp  # noqa: E402

EXIT_OK = 0
EXIT_SILENT = 3

# A joint is "moving" if it travels more than this over the capture. Below it we
# cannot distinguish real motion from sensor noise, so we do not claim coupling.
MOVEMENT_EPS_RAD = 1e-3


class SideStats:
    """Per-side accumulator. Everything is derived from received samples only."""

    def __init__(self, side):
        self.side = side
        self.samples = 0
        self.first_mono = None
        self.last_mono = None
        self.n_motor = None
        self.n_press = None
        self.motor = []          # per motor: dict of min/max for q, dq, tau, temp
        self.press = []          # per module: dict with nonzero slot set + lost
        self.q_series = []       # per motor list of q, for the H2 coupling hint
        self.lock = threading.Lock()

    def _ensure(self, n_motor, n_press):
        if self.n_motor is None:
            self.n_motor = n_motor
            self.n_press = n_press
            self.motor = [{"q": [None, None], "dq": [None, None],
                           "tau": [None, None], "temp": [None, None]}
                          for _ in range(n_motor)]
            self.press = [{"nonzero_slots": set(), "slots": 0, "lost_min": None,
                           "lost_max": None, "temp_min": None, "temp_max": None}
                          for _ in range(n_press)]
            self.q_series = [[] for _ in range(n_motor)]

    @staticmethod
    def _upd(pair, value):
        if value is None:
            return
        if pair[0] is None or value < pair[0]:
            pair[0] = value
        if pair[1] is None or value > pair[1]:
            pair[1] = value

    def add(self, msg):
        n_motor, n_press = hand_config.read_hand_counts(msg)
        now = time.monotonic()
        with self.lock:
            self._ensure(n_motor, n_press)
            if n_motor != self.n_motor or n_press != self.n_press:
                # Counts must not change mid-stream; record it rather than crash.
                self.n_motor = min(self.n_motor, n_motor)
                self.n_press = min(self.n_press, n_press)
            self.samples += 1
            if self.first_mono is None:
                self.first_mono = now
            self.last_mono = now

            for idx in range(min(self.n_motor, len(msg.motor_state))):
                ms = msg.motor_state[idx]
                rec = self.motor[idx]
                self._upd(rec["q"], float(ms.q))
                self._upd(rec["dq"], float(ms.dq))
                self._upd(rec["tau"], float(ms.tau_est))
                self._upd(rec["temp"], hand_config.motor_temperature(ms))
                self.q_series[idx].append(float(ms.q))

            for midx in range(min(self.n_press, len(msg.press_sensor_state))):
                mod = msg.press_sensor_state[midx]
                rec = self.press[midx]
                pressure = list(mod.pressure)
                rec["slots"] = len(pressure)
                for slot, value in enumerate(pressure):
                    if value != 0.0:
                        rec["nonzero_slots"].add(slot)
                lost = int(mod.lost)
                rec["lost_min"] = lost if rec["lost_min"] is None else min(rec["lost_min"], lost)
                rec["lost_max"] = lost if rec["lost_max"] is None else max(rec["lost_max"], lost)
                temps = [float(t) for t in mod.temperature]
                if temps:
                    lo, hi = min(temps), max(temps)
                    rec["temp_min"] = lo if rec["temp_min"] is None else min(rec["temp_min"], lo)
                    rec["temp_max"] = hi if rec["temp_max"] is None else max(rec["temp_max"], hi)

    def coupling_hints(self):
        """Slots 4k+3 (distal) that move whenever 4k+2 moves -- the H2 hint."""
        hints = []
        if self.n_motor is None:
            return hints
        for k in range(self.n_motor // 4):
            proximal, distal = 4 * k + 2, 4 * k + 3
            if distal >= self.n_motor:
                continue
            p_series, d_series = self.q_series[proximal], self.q_series[distal]
            if len(p_series) < 2 or len(d_series) < 2:
                continue
            p_range = max(p_series) - min(p_series)
            d_range = max(d_series) - min(d_series)
            hints.append({
                "group": k,
                "proximal_slot": proximal,
                "distal_slot": distal,
                "proximal_range_rad": p_range,
                "distal_range_rad": d_range,
                "proximal_moved": p_range > MOVEMENT_EPS_RAD,
                "distal_moved": d_range > MOVEMENT_EPS_RAD,
                "ratio_distal_over_proximal": (d_range / p_range) if p_range > MOVEMENT_EPS_RAD else None,
            })
        return hints

    def to_dict(self):
        elapsed = (self.last_mono - self.first_mono) if self.samples > 1 else 0.0
        return {
            "side": self.side,
            "samples": self.samples,
            "elapsed_s": round(elapsed, 3),
            "rate_hz": round(self.samples / elapsed, 2) if elapsed > 0 else None,
            "n_motor": self.n_motor,
            "n_press": self.n_press,
            "motors": [
                {"slot": i,
                 "group": hand_config.group_of(i),
                 "q_min": m["q"][0], "q_max": m["q"][1],
                 "dq_min": m["dq"][0], "dq_max": m["dq"][1],
                 "tau_min": m["tau"][0], "tau_max": m["tau"][1],
                 "temp_min": m["temp"][0], "temp_max": m["temp"][1]}
                for i, m in enumerate(self.motor)],
            "press_modules": [
                {"module": i,
                 "slots": p["slots"],
                 "nonzero_slot_count": len(p["nonzero_slots"]),
                 "nonzero_slots": sorted(p["nonzero_slots"]),
                 "lost_min": p["lost_min"], "lost_max": p["lost_max"],
                 "temp_min": p["temp_min"], "temp_max": p["temp_max"]}
                for i, p in enumerate(self.press)],
            "total_nonzero_pressure_slots": sum(len(p["nonzero_slots"]) for p in self.press),
            "coupling_hints_4k3_vs_4k2": self.coupling_hints(),
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--domain", type=int, default=1,
                        help="DDS domain id (default 1 = this laptop; robot day uses 0)")
    parser.add_argument("--iface", type=str, default=None, help="network interface for DDS")
    parser.add_argument("--prefix", type=str, default=hand_config.TOPIC_PREFIX,
                        help=f"hand topic prefix (default {hand_config.TOPIC_PREFIX})")
    parser.add_argument("--seconds", type=float, default=30.0, help="capture duration")
    parser.add_argument("--json", type=str, default=None, help="output path (default logs/probe_<ts>.json)")
    args = parser.parse_args()

    log = setup_tool_logger("hand_probe")
    log.info("READ-ONLY probe. This tool constructs no publisher.")
    log.info(hand_config.describe())
    log.info(f"domain={args.domain} iface={args.iface} prefix={args.prefix} "
             f"seconds={args.seconds}")

    ChannelFactoryInitialize(args.domain, args.iface)

    stats = {side: SideStats(side) for side in ("left", "right")}
    subs = {}
    for side in ("left", "right"):
        topic = f"{args.prefix}/{side}/state"
        sub = ChannelSubscriber(topic, HandState_)
        # Callback, not polling: a bare Read() blocks forever on a silent topic, which
        # is precisely the case this tool has to survive and report.
        sub.Init(lambda msg, s=side: stats[s].add(msg))
        subs[side] = sub
        log.info(f"subscribed {topic}")

    deadline = time.monotonic() + args.seconds
    next_tick = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        time.sleep(0.05)
        if time.monotonic() >= next_tick:
            next_tick += 5.0
            log.info(f"left={stats['left'].samples} right={stats['right'].samples} samples")

    report = {
        "captured_at": timestamp(),
        "domain": args.domain,
        "iface": args.iface,
        "prefix": args.prefix,
        "seconds_requested": args.seconds,
        "topics": {s: f"{args.prefix}/{s}/state" for s in ("left", "right")},
        "sides": {s: stats[s].to_dict() for s in ("left", "right")},
    }

    out_path = args.json or os.path.join(REPO_ROOT, "logs", f"probe_{timestamp()}.json")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w") as fh:
        json.dump(report, fh, indent=2)

    for side in ("left", "right"):
        d = report["sides"][side]
        log.info(f"{side}: samples={d['samples']} rate={d['rate_hz']} Hz "
                 f"n_motor={d['n_motor']} n_press={d['n_press']} "
                 f"nonzero_pressure_slots={d['total_nonzero_pressure_slots']}")
        for hint in d["coupling_hints_4k3_vs_4k2"]:
            if hint["proximal_moved"]:
                log.info(f"  {side} group {hint['group']}: slot {hint['proximal_slot']} moved "
                         f"{hint['proximal_range_rad']:.4f} rad, distal slot "
                         f"{hint['distal_slot']} moved {hint['distal_range_rad']:.4f} rad "
                         f"(ratio {hint['ratio_distal_over_proximal']})")
    log.info(f"report written to {out_path}")

    silent = [s for s in ("left", "right") if stats[s].samples == 0]
    if silent:
        log.error(f"SILENT: no samples on {', '.join(silent)}. "
                  f"Same symptom as 2026-08-24 (topic declared, zero samples). "
                  f"Check hand power, DDS domain/interface, and DEX5_TOPIC_PREFIX.")
        return EXIT_SILENT
    return EXIT_OK


if __name__ == "__main__":
    rc = main()
    logging.shutdown()
    os._exit(rc)
