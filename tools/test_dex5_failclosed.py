#!/usr/bin/env python3
"""Exercise Dex5_1_Controller's fail-closed paths against a fake hand. DOMAIN 1 ONLY.

Three cases, each a separate process because ChannelFactoryInitialize is
once-per-process and the controller forks a control process on success:

  silent  no publisher at all      -> RuntimeError naming both state topics, ~STATE_TIMEOUT_S
  dex3    fake publishing  7 motors -> RuntimeError "motor_state has 7 entries"
  dex5    fake publishing 20 motors -> "Subscribe dds ok", counts logged

Run each with the matching tools/fake_hand_state.py already publishing (except
"silent"). Must be run with cwd = teleop/, which is where HandRetargeting expects
to find ../assets/unitree_hand_Dex5/unitree_dex5.yml.

Exit codes: 0 the case behaved as expected, 1 it did not.
"""

import argparse
import os
import sys
import time
from multiprocessing import Array, Lock

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(REPO_ROOT)

# logging_mp defaults to WARNING, which would hide describe() and the n_motor/n_press
# line -- the whole point of the dex5 case. The launcher does the same at startup.
# It MUST run before anything calls getLogger(), which importing hand_config does.
import logging_mp  # noqa: E402
logging_mp.basicConfig(level=logging_mp.INFO)

from unitree_sdk2py.core.channel import ChannelFactoryInitialize  # noqa: E402
from teleop.robot_control import hand_config  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--case", required=True, choices=["silent", "dex3", "dex5"])
    parser.add_argument("--domain", type=int, default=1)
    parser.add_argument("--iface", type=str, default=None)
    args = parser.parse_args()

    print(f"--- case={args.case} domain={args.domain} "
          f"prefix={hand_config.TOPIC_PREFIX} timeout={hand_config.STATE_TIMEOUT_S}s ---")

    ChannelFactoryInitialize(args.domain, args.iface)
    from teleop.robot_control.robot_hand_unitree import Dex5_1_Controller

    left_in = Array('d', 75, lock=True)
    right_in = Array('d', 75, lock=True)
    n = hand_config.NUM_JOINTS_EXPECTED
    state_out = Array('d', 2 * n, lock=False)
    action_out = Array('d', 2 * n, lock=False)

    started = time.monotonic()
    try:
        ctrl = Dex5_1_Controller(left_in, right_in, Lock(), state_out, action_out,
                                 simulation_mode=False)
        elapsed = time.monotonic() - started
        counts = dict(getattr(ctrl, "_hand_counts", {}))
        print(f"RESULT: constructed OK in {elapsed:.2f}s")
        print(f"RESULT: hand counts (n_motor, n_press) = {counts}")
        ok = args.case == "dex5"
        print(f"RESULT: {'as expected' if ok else 'UNEXPECTED -- should have refused'}")
        sys.stdout.flush()
        os._exit(0 if ok else 1)          # daemon control process is still running
    except RuntimeError as exc:
        elapsed = time.monotonic() - started
        print(f"RESULT: refused after {elapsed:.2f}s")
        print(f"RESULT: RuntimeError: {exc}")
        if args.case == "silent":
            ok = abs(elapsed - hand_config.STATE_TIMEOUT_S) <= 1.0 and "no HandState_" in str(exc)
            print(f"RESULT: within timeout +/-1s and names both topics: {ok}")
        elif args.case == "dex3":
            ok = "motor_state has 7 entries" in str(exc)
            print(f"RESULT: reports the motor-count mismatch: {ok}")
        else:
            ok = False
            print("RESULT: UNEXPECTED -- dex5 case should have succeeded")
        sys.stdout.flush()
        os._exit(0 if ok else 1)


if __name__ == "__main__":
    main()
