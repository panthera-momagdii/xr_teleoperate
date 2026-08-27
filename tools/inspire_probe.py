#!/usr/bin/env python3
"""Read-only probe of the Inspire RH56 hand state topics.

The first real look at these hands. Subscribes to both state topics, reports what each
side is publishing and how fast, and writes a JSON record for the contract.

    python tools/inspire_probe.py --domain 1 --seconds 30           # bench, against the fixture
    python tools/inspire_probe.py --domain 0 --iface <nic> --seconds 60   # robot day

Read-only by construction: this tool creates readers only. The gate check greps this file
for the SDK's writer class name and must find nothing, which is why that name is not
written out here, not even in prose.

Requires the bridge to be running -- these topics are produced by the Modbus-TCP<->DDS
bridge on PC2, not by PC1. Silence here means the bridge is not up, not that the hands
are faulty. See docs/inspire_rh56e2.md.

Exit codes: 0 both sides streamed, 3 at least one side was silent.
"""

import argparse
import json
import os
import sys
import time
from collections import defaultdict

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(REPO_ROOT)

import logging_mp  # noqa: E402
try:
    logging_mp.basicConfig(level=logging_mp.INFO)
except RuntimeError:
    pass

from teleop.robot_control import hand_config  # noqa: E402
from tools.tool_logging import setup_tool_logger, timestamp  # noqa: E402

EXIT_OK, EXIT_SILENT = 0, 3

# Fields carried by inspire_hand_state. angle_act is the only one xr_teleoperate reads;
# the rest are exactly what a refusal path needs, so the probe reports all of them.
FIELDS = ("pos_act", "angle_act", "force_act", "current", "err", "status", "temperature")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--domain", type=int, default=1,
                        help="DDS domain (default 1 = this laptop; robot day uses 0)")
    parser.add_argument("--iface", type=str, default=None)
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--out", type=str, default=None,
                        help="JSON output path (default logs/inspire_probe_<ts>.json)")
    args = parser.parse_args()

    log = setup_tool_logger("inspire_probe")
    out_path = args.out or os.path.join(REPO_ROOT, "logs", f"inspire_probe_{timestamp()}.json")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    log.info("READ-ONLY. This tool creates readers only.")
    log.info(hand_config.describe())

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    ChannelFactoryInitialize(args.domain, args.iface)
    state_cls = hand_config.state_type()

    topics = {"left": hand_config.TOPIC_LEFT_STATE, "right": hand_config.TOPIC_RIGHT_STATE}
    samples = defaultdict(int)
    first_t, last_t = {}, {}
    ranges = defaultdict(lambda: defaultdict(lambda: [None, None]))   # side -> field -> [min,max]
    latest = {}
    errors_seen = defaultdict(set)

    def handler(side):
        def on_state(msg):
            now = time.monotonic()
            samples[side] += 1
            first_t.setdefault(side, now)
            last_t[side] = now
            snap = {}
            for f in FIELDS:
                v = getattr(msg, f, None)
                if v is None:
                    continue
                vals = [int(x) for x in v]
                snap[f] = vals
                for i, x in enumerate(vals):
                    lo, hi = ranges[side][f][0], ranges[side][f][1]
                    ranges[side][f][0] = x if lo is None else min(lo, x)
                    ranges[side][f][1] = x if hi is None else max(hi, x)
            for i, e in enumerate(snap.get("err", [])):
                if e:
                    errors_seen[side].add((i, e))
            latest[side] = snap
        return on_state

    subs = {}
    try:
        for side, topic in topics.items():
            sub = ChannelSubscriber(topic, state_cls)
            sub.Init(handler(side))
            subs[side] = sub
        log.info(f"listening on {topics['left']} and {topics['right']} for {args.seconds:g}s")

        started = time.monotonic()
        next_note = started + 5.0
        while time.monotonic() - started < args.seconds:
            if time.monotonic() >= next_note:
                next_note += 5.0
                log.info(f"  t={time.monotonic() - started:5.1f}s  "
                         f"left={samples['left']} right={samples['right']}")
            time.sleep(0.02)
        window = time.monotonic() - started
    finally:
        for sub in subs.values():
            try:
                sub.Close()
            except Exception:
                pass

    report = {"captured_at": timestamp(), "domain": args.domain, "iface": args.iface,
              "window_s": round(window, 3), "hand_model": hand_config.HAND_MODEL,
              "topics": topics, "sides": {}}

    print(f"\n{'=' * 78}\nINSPIRE HAND PROBE -- {window:.1f} s on domain {args.domain}\n{'=' * 78}")
    silent = []
    for side in ("left", "right"):
        n = samples[side]
        span = (last_t.get(side, 0) - first_t.get(side, 0)) if side in first_t else 0.0
        rate = (n / span) if span > 0.2 else (n / window if window else 0.0)
        print(f"\n  {side.upper()}  {topics[side]}")
        if not n:
            print("    SILENT -- 0 samples. The bridge is not publishing this side.")
            silent.append(side)
            report["sides"][side] = {"samples": 0, "rate_hz": 0.0, "silent": True}
            continue
        print(f"    {n} samples, {rate:.1f} Hz")
        for f in FIELDS:
            if f in ranges[side]:
                lo, hi = ranges[side][f]
                cur = latest[side].get(f)
                print(f"      {f:12s} min {lo:>6}  max {hi:>6}   last {cur}")
        if errors_seen[side]:
            print(f"    *** err NON-ZERO: DOF/value {sorted(errors_seen[side])} ***")
        temps = latest[side].get("temperature", [])
        if temps and max(temps) > hand_config.TEMP_LIMIT_C:
            print(f"    *** temperature {max(temps)} C > limit {hand_config.TEMP_LIMIT_C} C ***")
        n_dof, _ = len(latest[side].get("angle_act", [])), 0
        if n_dof != hand_config.NUM_JOINTS_EXPECTED:
            print(f"    *** angle_act is {n_dof} wide, expected "
                  f"{hand_config.NUM_JOINTS_EXPECTED} ***")
        report["sides"][side] = {
            "samples": n, "rate_hz": round(rate, 1), "silent": False, "n_dof": n_dof,
            "ranges": {f: ranges[side][f] for f in ranges[side]},
            "latest": latest[side],
            "err_nonzero": sorted(errors_seen[side]),
        }

    print(f"\n  NOTE: tactile arrives on {list(hand_config.TOPIC_TOUCH) or 'no'} topic(s),")
    print("  which this probe does NOT subscribe to and xr_teleoperate does not record.")
    with open(out_path, "w") as fh:
        json.dump(report, fh, indent=2, default=str)
    print(f"\n  report -> {out_path}")

    if silent:
        print(f"\nRESULT: {', '.join(silent)} SILENT. Is the bridge running on PC2?")
        return EXIT_SILENT
    print("\nRESULT: both sides streaming.")
    return EXIT_OK


if __name__ == "__main__":
    rc = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(rc)
