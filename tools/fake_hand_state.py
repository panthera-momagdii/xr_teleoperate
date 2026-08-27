#!/usr/bin/env python3
"""Publish synthetic Dex5-1P HandState_ on the two state topics. TEST FIXTURE ONLY.

This exists so the fail-closed paths in Dex5_1_Controller and the two operator
tools can be exercised on this laptop, on DDS domain 1, with no robot attached.
It must never be run on domain 0 -- a fake state stream on the robot's domain
would race the real hands.

  # a well-behaved Dex5-1P
  python tools/fake_hand_state.py --n 20 --echo-cmd --seconds 20

  # pretend a Dex3-1 is fitted, to prove the controller refuses it
  python tools/fake_hand_state.py --n 7 --seconds 20
"""

import argparse
import logging
import os
import sys
import threading
import time

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from unitree_sdk2py.core.channel import (  # noqa: E402
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_  # noqa: E402
from unitree_sdk2py.idl.default import (  # noqa: E402
    unitree_hg_msg_dds__HandState_,
    unitree_hg_msg_dds__MotorState_,
    unitree_hg_msg_dds__PressSensorState_,
)

from teleop.robot_control import hand_config  # noqa: E402
from tools.tool_logging import setup_tool_logger  # noqa: E402

PRESS_SLOTS_PER_MODULE = 12   # PressSensorState_.pressure is float32[12]
NONZERO_PRESSURE_SLOTS = 94   # of 12 modules * 12 slots = 144, so the probe's map is partial


def build_state(n_motor, n_press, temp):
    """A HandState_ with sequences resized to the requested counts."""
    msg = unitree_hg_msg_dds__HandState_()
    msg.motor_state = [unitree_hg_msg_dds__MotorState_() for _ in range(n_motor)]
    msg.press_sensor_state = [unitree_hg_msg_dds__PressSensorState_() for _ in range(n_press)]

    for idx, motor in enumerate(msg.motor_state):
        motor.mode = 0x01
        motor.q = 0.0
        motor.dq = 0.0
        motor.tau_est = 0.0
        motor.temperature = [int(temp), int(temp)]

    # Fill only the first NONZERO_PRESSURE_SLOTS of the flattened tactile map, so a
    # probe can tell "slot present but silent" from "slot never reported".
    flat = 0
    for module in msg.press_sensor_state:
        pressure = list(module.pressure)
        temps = list(module.temperature)
        for slot in range(PRESS_SLOTS_PER_MODULE):
            if flat < NONZERO_PRESSURE_SLOTS:
                pressure[slot] = 1.0 + 0.01 * flat
                temps[slot] = float(temp)
            flat += 1
        module.pressure = pressure
        module.temperature = temps
        module.lost = 0
    return msg


class CmdEcho:
    """Mirror commanded q back into the published state after a lag.

    Callback-driven, deliberately. Polling with ChannelSubscriber.Read() is a trap
    here: a bare Read() maps to cyclonedds take_one() with no timeout and BLOCKS
    forever on a silent topic (so the publish loop never runs), while Read(timeout=x)
    makes the SDK print "[Reader] take sample error" on every miss -- hundreds of
    lines a second. Init(handler) has neither problem.
    """

    def __init__(self, prefix, lag_s, n_motor):
        self.lag_s = lag_s
        self.n_motor = n_motor
        self.lock = threading.Lock()
        self.pending = {"left": [], "right": []}   # (apply_at_monotonic, [q...])
        self.current = {"left": [0.0] * n_motor, "right": [0.0] * n_motor}
        self.subs = {}
        for side, topic in (("left", f"{prefix}/left/cmd"), ("right", f"{prefix}/right/cmd")):
            sub = ChannelSubscriber(topic, HandCmd_)
            sub.Init(self._handler(side))
            self.subs[side] = sub

    def _handler(self, side):
        def on_cmd(msg):
            if msg is None or msg.motor_cmd is None:
                return
            q = [float(c.q) for c in msg.motor_cmd][: self.n_motor]
            q += [0.0] * (self.n_motor - len(q))
            with self.lock:
                self.pending[side].append((time.monotonic() + self.lag_s, q))
        return on_cmd

    def settle(self):
        now = time.monotonic()
        with self.lock:
            for side in ("left", "right"):
                ready = [item for item in self.pending[side] if item[0] <= now]
                if ready:
                    self.current[side] = ready[-1][1]
                self.pending[side] = [item for item in self.pending[side] if item[0] > now]
            return dict(self.current)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--domain", type=int, default=1,
                        help="DDS domain id (default 1 = this laptop only; robot day uses 0)")
    parser.add_argument("--iface", type=str, default=None, help="network interface for DDS")
    parser.add_argument("--prefix", type=str, default=hand_config.TOPIC_PREFIX,
                        help=f"hand topic prefix (default {hand_config.TOPIC_PREFIX})")
    parser.add_argument("--n", type=int, default=hand_config.NUM_JOINTS_EXPECTED,
                        help="number of motor_state entries to publish")
    parser.add_argument("--n-press", type=int, default=12,
                        help="number of press_sensor_state modules to publish")
    parser.add_argument("--temp", type=float, default=25.0,
                        help="temperature to report on every motor and tactile module")
    parser.add_argument("--rate", type=float, default=100.0, help="publish rate in Hz")
    parser.add_argument("--seconds", type=float, default=0.0,
                        help="stop after N seconds (0 = run until interrupted)")
    parser.add_argument("--echo-cmd", action="store_true",
                        help="subscribe to the cmd topics and mirror q into the state")
    parser.add_argument("--lag-ms", type=float, default=20.0,
                        help="lag applied to echoed q, in ms (default 20)")
    args = parser.parse_args()

    log = setup_tool_logger("fake_hand_state")
    log.info("THIS IS A TEST FIXTURE. Never run it on the robot's domain.")
    log.info(f"domain={args.domain} iface={args.iface} prefix={args.prefix} "
             f"n_motor={args.n} n_press={args.n_press} temp={args.temp} "
             f"rate={args.rate}Hz echo_cmd={args.echo_cmd} lag={args.lag_ms}ms")

    ChannelFactoryInitialize(args.domain, args.iface)

    pubs = {}
    for side in ("left", "right"):
        pub = ChannelPublisher(f"{args.prefix}/{side}/state", HandState_)
        pub.Init()
        pubs[side] = pub
    log.info(f"publishing on {args.prefix}/left/state and {args.prefix}/right/state")

    echo = CmdEcho(args.prefix, args.lag_ms / 1000.0, args.n) if args.echo_cmd else None
    if echo:
        log.info(f"echoing {args.prefix}/{{left,right}}/cmd -> state q after {args.lag_ms} ms")

    msgs = {side: build_state(args.n, args.n_press, args.temp) for side in ("left", "right")}

    period = 1.0 / args.rate
    started = time.monotonic()
    sent = 0
    try:
        while True:
            loop_start = time.monotonic()
            if args.seconds and (loop_start - started) >= args.seconds:
                break

            if echo:
                current = echo.settle()
                for side in ("left", "right"):
                    for idx, value in enumerate(current[side][: args.n]):
                        msgs[side].motor_state[idx].q = value

            for side in ("left", "right"):
                pubs[side].Write(msgs[side])
            sent += 1

            if sent % int(args.rate * 5) == 0:
                log.info(f"t={loop_start - started:7.2f}s sent={sent} per side")

            slack = period - (time.monotonic() - loop_start)
            if slack > 0:
                time.sleep(slack)
    except KeyboardInterrupt:
        log.info("interrupted")

    elapsed = time.monotonic() - started
    log.info(f"done: {sent} samples per side in {elapsed:.2f}s "
             f"({sent / elapsed if elapsed else 0:.1f} Hz)")
    return 0


if __name__ == "__main__":
    rc = main()
    # os._exit, not sys.exit: importing hand_config starts logging_mp, whose forked
    # listener keeps the interpreter alive at shutdown (observed blocked in pipe_read).
    # Scripted loopback tests need this fixture to actually stop.
    logging.shutdown()
    os._exit(rc)
