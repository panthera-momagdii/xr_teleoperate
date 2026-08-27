#!/usr/bin/env python3
"""Publish synthetic Inspire RH56 hand state on DDS. TEST FIXTURE. DOMAIN 1 ONLY.

Stands in for the Modbus-TCP<->DDS bridge so the census, the probe and the controller's
refusal paths can be exercised without a hand, a bridge, or the robot LAN.

    python tools/fake_inspire_state.py --domain 1 --angle 500
    python tools/fake_inspire_state.py --domain 1 --err 2 --side l      # fault on one side
    python tools/fake_inspire_state.py --domain 1 --temp 60             # over-temperature

Angle semantics follow the RH56DFTP manual: 0-1000, where 1000 is fully OPEN and 0 is
fully bent. DOF order is the manual's: 0 little, 1 ring, 2 middle, 3 index, 4 thumb
bending, 5 thumb rotation.

Never run this on the robot's domain: a second publisher on a state topic is a way to
feed a controller a pose the hand is not in.
"""

import argparse
import os
import sys
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(REPO_ROOT)

from tools.tool_logging import setup_tool_logger  # noqa: E402

N_DOF = 6
TOPIC = "rt/inspire_hand/state/{side}"


def build_state(cls, angle, force, current, err, status, temp):
    return cls(
        pos_act=[int(angle * 2) for _ in range(N_DOF)],
        angle_act=[int(angle) for _ in range(N_DOF)],
        force_act=[int(force) for _ in range(N_DOF)],
        current=[int(current) for _ in range(N_DOF)],
        err=[int(err) for _ in range(N_DOF)],
        status=[int(status) for _ in range(N_DOF)],
        temperature=[int(temp) for _ in range(N_DOF)],
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--domain", type=int, default=1)
    parser.add_argument("--iface", type=str, default=None)
    parser.add_argument("--side", choices=["l", "r", "both"], default="both",
                        help="which side(s) to publish; 'l' or 'r' leaves the other silent")
    parser.add_argument("--angle", type=int, default=1000,
                        help="angle_act on every DOF (1000 = fully open, 0 = fully bent)")
    parser.add_argument("--force", type=int, default=0)
    parser.add_argument("--current", type=int, default=0)
    parser.add_argument("--err", type=int, default=0, help="err byte on every DOF")
    parser.add_argument("--status", type=int, default=1)
    parser.add_argument("--temp", type=int, default=25, help="temperature C on every DOF")
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument("--seconds", type=float, default=0.0, help="0 = until interrupted")
    args = parser.parse_args()

    if args.domain == 0:
        parser.error("refusing to run on DDS domain 0: this is a test fixture")

    log = setup_tool_logger("fake_inspire_state")
    log.info("THIS IS A TEST FIXTURE. Never run it on the robot's domain.")
    log.info(f"domain={args.domain} side={args.side} angle={args.angle} err={args.err} "
             f"temp={args.temp} rate={args.rate}Hz")

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
    from inspire_sdkpy import inspire_dds
    ChannelFactoryInitialize(args.domain, args.iface)

    sides = ["l", "r"] if args.side == "both" else [args.side]
    pubs = {}
    for side in sides:
        pub = ChannelPublisher(TOPIC.format(side=side), inspire_dds.inspire_hand_state)
        pub.Init()
        pubs[side] = pub
    log.info(f"publishing on {[TOPIC.format(side=s) for s in sides]}")

    msg = build_state(inspire_dds.inspire_hand_state, args.angle, args.force,
                      args.current, args.err, args.status, args.temp)
    period = 1.0 / args.rate
    started = time.monotonic()
    sent = 0
    try:
        while True:
            loop = time.monotonic()
            for pub in pubs.values():
                pub.Write(msg)
            sent += 1
            if args.seconds and time.monotonic() - started >= args.seconds:
                break
            slack = period - (time.monotonic() - loop)
            if slack > 0:
                time.sleep(slack)
    except KeyboardInterrupt:
        pass
    elapsed = time.monotonic() - started
    log.info(f"done: {sent} samples per side in {elapsed:.2f}s ({sent / elapsed:.1f} Hz)")
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
