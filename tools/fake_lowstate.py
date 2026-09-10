#!/usr/bin/env python3
"""Publish a synthetic G1 rt/lowstate. TEST FIXTURE. DOMAIN 1 ONLY.

Stands in for the robot's own low-level state so the launcher and the arm controllers
can be exercised offline -- without a G1, without PC1, and without ever putting a
writer on the robot's DDS domain.

    python tools/fake_lowstate.py --domain 1 --iface lo
    python tools/fake_lowstate.py --domain 1 --iface lo --seconds 30 --rate 500
    python tools/fake_lowstate.py --domain 1 --iface lo --q-arms 0.2   # arms off zero

Why it exists
-------------
`G1_29_ArmController.__init__` blocks in `dds_utils.wait_for_dds(..., timeout=5.0)`
until a first `rt/lowstate` arrives (teleop/robot_control/robot_arm.py:113). Without
one, the launcher cannot get anywhere near "Press [r]", so none of the offline gates
can test what happens after that point.

The message shape follows `unitree_hg.msg.dds_.LowState_`: 35 motor slots (the IDL is
fixed at 35 for every hg robot; a G1_29 uses the first 29, per
`G1_29_JointIndex`), plus `mode_machine`, which the controller reads once via
`get_mode_machine()` and copies into every LowCmd_ it publishes.

SAFETY
------
This is a WRITER. Running it on domain 0 would put a second publisher on the topic the
real robot publishes, i.e. feed every subscriber on the robot LAN a fabricated body
state. The tool refuses domain 0 outright -- there is no flag to override that.
"""

import argparse
import os
import sys
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(REPO_ROOT)

from tools.tool_logging import setup_tool_logger      # noqa: E402
from tools._procs import reap_child_processes         # noqa: E402

N_MOTOR_SLOTS = 35          # the hg LowState_ IDL is fixed at 35
G1_29_ARM_JOINTS = range(15, 29)   # left/right shoulder..wrist, per G1_29_JointArmIndex
TOPIC = "rt/lowstate"


def build_lowstate(mk, mode_machine, q_arms, q_other, tick):
    msg = mk()
    msg.mode_machine = mode_machine
    msg.mode_pr = 0
    msg.tick = tick & 0xFFFFFFFF
    for i in range(N_MOTOR_SLOTS):
        ms = msg.motor_state[i]
        ms.mode = 1
        ms.q = q_arms if i in G1_29_ARM_JOINTS else q_other
        ms.dq = 0.0
        ms.ddq = 0.0
        ms.tau_est = 0.0
        ms.temperature = [25, 25]
        ms.vol = 48.0
    return msg


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--domain", type=int, default=1,
                    help="DDS domain. MUST be 1; domain 0 is the robot's and is refused.")
    ap.add_argument("--iface", type=str, default="lo",
                    help="network interface for DDS (default lo)")
    ap.add_argument("--rate", type=float, default=500.0, help="publish rate in Hz")
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="stop after N seconds (0 = run until killed)")
    ap.add_argument("--mode-machine", type=int, default=5,
                    help="mode_machine value the controller will echo back")
    ap.add_argument("--q-arms", type=float, default=0.0,
                    help="q reported for the 14 arm joints (radians)")
    ap.add_argument("--q-other", type=float, default=0.0,
                    help="q reported for every other motor slot")
    args = ap.parse_args()

    if args.domain == 0:
        print("REFUSED: fake_lowstate.py is a WRITER and will not run on domain 0. "
              "Domain 0 belongs to the robot; a second rt/lowstate publisher there "
              "would feed the whole robot LAN a fabricated body state.", file=sys.stderr)
        return 2

    log = setup_tool_logger("fake_lowstate")

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_ as mk

    ChannelFactoryInitialize(args.domain, args.iface)
    pub = ChannelPublisher(TOPIC, LowState_)
    pub.Init()
    log.info(f"publishing {TOPIC} on domain {args.domain} iface {args.iface} "
             f"at {args.rate} Hz, mode_machine={args.mode_machine}, "
             f"arm q={args.q_arms}, {N_MOTOR_SLOTS} motor slots")

    period = 1.0 / args.rate
    deadline = time.monotonic() + args.seconds if args.seconds > 0 else None
    tick = 0
    n = 0
    try:
        while True:
            if deadline is not None and time.monotonic() >= deadline:
                break
            pub.Write(build_lowstate(mk, args.mode_machine, args.q_arms,
                                     args.q_other, tick))
            tick += 1
            n += 1
            if n % int(max(args.rate, 1) * 5) == 0:
                log.info(f"published {n} messages")
            time.sleep(period)
    except KeyboardInterrupt:
        log.info("interrupted")
    log.info(f"done, {n} messages published")
    # Bounded, and after everything that matters is already on the wire.
    reap_child_processes(log=log)
    os._exit(0)


if __name__ == "__main__":
    sys.exit(main())
