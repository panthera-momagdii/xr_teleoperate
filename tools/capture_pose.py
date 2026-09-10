#!/usr/bin/env python3
"""Capture the arms' current pose from the robot into a start-pose YAML. READ ONLY.

    # on the robot, arms already where you want them (hold them there by hand in
    # damping mode, or drive them there and stop)
    python tools/capture_pose.py --domain 0 --iface enP7s7 --out poses/tray.yaml

Averages 2 s of `rt/lowstate` arm joint angles and writes the same YAML format
`poses/ready.yaml` uses, so `XR_START_POSE=poses/tray.yaml` picks it straight up.

WHY DOMAIN 0 IS ALLOWED HERE
----------------------------
Domain 0 is the robot's. Every other offline tool is confined to domain 1 because it
publishes. This one only ever SUBSCRIBES, and reading `rt/lowstate` is exactly as
intrusive as running `ros2 topic echo`: it adds a reader, no writer, and cannot command
anything.

That is not left to good intentions. Before any DDS call, `_forbid_publishers()`
replaces `ChannelPublisher.__init__` with a function that raises, so if anything on the
import path -- now or after some future refactor -- tries to construct a publisher, this
tool dies loudly instead of putting a writer on the robot's domain. `--self-test` proves
the guard fires.

There is also no `--command`, no `--move`, no write path of any kind. The only output is
a YAML file on this host.
"""

import argparse
import os
import sys
import threading
import time

import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(REPO_ROOT)

from tools.tool_logging import setup_tool_logger          # noqa: E402
from tools._procs import install_reaper, reap_child_processes  # noqa: E402
from teleop.robot_control.start_pose import ARM_JOINT_NAMES, N_ARM_JOINTS  # noqa: E402

TOPIC = "rt/lowstate"
# G1_29_JointArmIndex (robot_arm.py:285-302): motors 15..28, left arm then right arm,
# in exactly the order ARM_JOINT_NAMES lists.
ARM_MOTOR_IDS = tuple(range(15, 29))


class PublisherForbidden(RuntimeError):
    pass


def _forbid_publishers():
    """Make constructing a DDS publisher impossible for the rest of this process.

    Returns a zero-argument callable that undoes it, for the self-test.
    """
    from unitree_sdk2py.core import channel as _channel

    original = _channel.ChannelPublisher.__init__

    def refuse(self, *args, **kwargs):
        raise PublisherForbidden(
            "capture_pose.py is READ ONLY and must never create a DDS writer. "
            f"Something tried to construct ChannelPublisher{args!r}. "
            "This guard exists because this is the one tool allowed on domain 0.")

    _channel.ChannelPublisher.__init__ = refuse

    def restore():
        _channel.ChannelPublisher.__init__ = original
    return restore


def self_test():
    """Prove the guard actually fires. No DDS domain is joined."""
    restore = _forbid_publishers()
    try:
        from unitree_sdk2py.core.channel import ChannelPublisher
        from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
        try:
            ChannelPublisher("rt/should_never_exist", String_)
        except PublisherForbidden as exc:
            print("SELF-TEST PASS: ChannelPublisher is blocked")
            print(f"  {exc}")
            return 0
        print("SELF-TEST FAIL: a ChannelPublisher was constructed", file=sys.stderr)
        return 1
    finally:
        restore()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--domain", type=int, default=0,
                    help="DDS domain (default 0 -- the robot's; READ ONLY)")
    ap.add_argument("--iface", type=str, default=None,
                    help="network interface, e.g. enP7s7. DDS discovery is per interface.")
    ap.add_argument("--seconds", type=float, default=2.0, help="how long to average")
    ap.add_argument("--out", default="poses/captured.yaml")
    ap.add_argument("--name", default="captured")
    ap.add_argument("--timeout", type=float, default=10.0,
                    help="give up if no lowstate arrives within this long")
    ap.add_argument("--max-motion", type=float, default=0.05,
                    help="refuse if any joint moves more than this (rad) while capturing")
    ap.add_argument("--self-test", action="store_true",
                    help="prove the no-publisher guard fires, then exit")
    args = ap.parse_args()

    if args.self_test:
        return self_test()

    log = setup_tool_logger("capture_pose")
    install_reaper(log=log)

    # BEFORE any DDS call.
    _forbid_publishers()
    log.info("publisher guard installed: ChannelPublisher now raises if constructed")

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

    ChannelFactoryInitialize(args.domain, args.iface)
    log.info(f"subscribing {TOPIC} on domain {args.domain} iface {args.iface} "
             f"(READ ONLY), averaging {args.seconds}s")

    samples = []
    lock = threading.Lock()
    first = threading.Event()

    def on_state(msg):
        try:
            q = np.array([msg.motor_state[i].q for i in ARM_MOTOR_IDS], dtype=float)
        except (IndexError, AttributeError):
            return
        with lock:
            samples.append(q)
        first.set()

    sub = ChannelSubscriber(TOPIC, LowState_)
    sub.Init(on_state)
    try:
        if not first.wait(timeout=args.timeout):
            log.error(f"no {TOPIC} within {args.timeout}s on domain {args.domain} "
                      f"iface {args.iface}. Is the robot up, and is --iface the "
                      f"interface it is on? DDS discovery is per interface.")
            return 2
        with lock:
            samples.clear()                    # discard anything from before t0
        time.sleep(args.seconds)
        with lock:
            taken = list(samples)
    finally:
        try:
            sub.Close()
        except Exception:
            pass

    if len(taken) < 10:
        log.error(f"only {len(taken)} samples in {args.seconds}s -- too few to average. "
                  f"rt/lowstate should arrive at ~500 Hz.")
        return 2

    arr = np.stack(taken)
    mean = arr.mean(axis=0)
    spread = arr.max(axis=0) - arr.min(axis=0)
    log.info(f"{len(taken)} samples over {args.seconds}s "
             f"({len(taken)/args.seconds:.0f} Hz)")

    # A pose captured while the arms are drifting is not a pose.
    worst = int(np.argmax(spread))
    if spread[worst] > args.max_motion:
        log.error(f"the arms moved while capturing: {ARM_JOINT_NAMES[worst]} spanned "
                  f"{spread[worst]:.4f} rad (limit --max-motion {args.max_motion}). "
                  f"Hold them still and try again.")
        return 3
    log.info(f"steadiest reading: largest joint spread {spread[worst]:.4f} rad "
             f"({ARM_JOINT_NAMES[worst]})")

    lines = [
        f"# {args.name} pose, CAPTURED FROM THE ROBOT.",
        f"# tools/capture_pose.py, domain {args.domain}, iface {args.iface}, "
        f"{len(taken)} samples over {args.seconds}s.",
        f"# Captured {time.strftime('%Y-%m-%d %H:%M:%S %Z')}.",
        f"# Largest joint spread while capturing: {spread[worst]:.4f} rad "
        f"({ARM_JOINT_NAMES[worst]}).",
        "#",
        "# Use with:  XR_START_POSE=" + args.out,
        "",
        f"name: {args.name}",
        "robot: G1_29",
        "units: radians",
        "joints:",
    ]
    width = max(len(n) for n in ARM_JOINT_NAMES)
    for i, name in enumerate(ARM_JOINT_NAMES):
        lines.append(f"  {name + ':':<{width + 1}} {mean[i]:+.6f}")
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w") as fh:
        fh.write("\n".join(lines) + "\n")
    log.info(f"wrote {args.out}")
    for i, name in enumerate(ARM_JOINT_NAMES):
        log.info(f"    {name:<28} {mean[i]:+.5f}  (spread {spread[i]:.4f})")
    return 0


if __name__ == "__main__":
    rc = main()
    sys.stdout.flush()
    sys.stderr.flush()
    reap_child_processes()
    os._exit(rc)
