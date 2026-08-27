#!/usr/bin/env python3
"""Exercise the Dex5 fail-closed paths against a fake hand. DOMAIN 1 ONLY.

Two targets, because there are now two independent checks:
  --target controller   Dex5_1_Controller.__init__ (built late, after ReleaseMode)
  --target preflight    hand_config.preflight()    (run first, before ReleaseMode)
  --target readers      proves preflight leaves no DDS reader behind

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



def run_preflight_case(args):
    """hand_config.preflight() against the same three situations."""
    logger = logging_mp.getLogger("preflight_case")
    started = time.monotonic()
    try:
        counts = hand_config.preflight(log=logger)
        elapsed = time.monotonic() - started
        print(f"RESULT: preflight passed in {elapsed:.2f}s")
        print(f"RESULT: counts = {counts}")
        shape_ok = all(counts[s]["n_motor"] == hand_config.NUM_JOINTS_EXPECTED
                       for s in ("left", "right"))
        both = set(counts) == {"left", "right"}
        print(f"RESULT: returned both sides: {both}; every n_motor == "
              f"{hand_config.NUM_JOINTS_EXPECTED}: {shape_ok}")
        ok = args.case == "dex5" and both and shape_ok
        print(f"RESULT: {'as expected' if ok else 'UNEXPECTED -- should have refused'}")
        return 0 if ok else 1
    except RuntimeError as exc:
        elapsed = time.monotonic() - started
        print(f"RESULT: preflight refused after {elapsed:.2f}s")
        print(f"RESULT: RuntimeError: {exc}")
        if args.case == "silent":
            ok = (abs(elapsed - hand_config.STATE_TIMEOUT_S) <= 1.0
                  and "no HandState_" in str(exc))
            print(f"RESULT: within timeout +/-1s and names both topics: {ok}")
        elif args.case == "dex3":
            ok = "motor_state has 7 entries" in str(exc)
            print(f"RESULT: reports the motor-count mismatch: {ok}")
        else:
            ok = False
            print("RESULT: UNEXPECTED -- dex5 case should have passed")
        return 0 if ok else 1


def _count_state_readers(observer_reader, prefix, settle_s=1.0):
    """ALIVE DCPSSubscription endpoints on the two hand state topics.

    Uses cyclonedds' builtin discovery topic from a SEPARATE participant, so what it
    sees is what any other process on the domain would see.

    read(), NOT take(). take() is destructive: it consumes the discovery samples, so a
    second call reports only what was announced since the first and a closed reader
    looks identical to one that was never announced. read() keeps the instance history,
    and cyclonedds marks a departed endpoint NOT_ALIVE_DISPOSED (instance_state 32)
    rather than removing it -- ALIVE is 16.
    """
    from cyclonedds.core import InstanceState

    time.sleep(settle_s)                       # let discovery/undiscovery propagate
    topics = (f"{prefix}/left/state", f"{prefix}/right/state")
    alive = {}
    for sample in observer_reader.read(N=500):
        topic = str(getattr(sample, "topic_name", ""))
        if topic not in topics:
            continue
        state = int(sample.sample_info.instance_state)
        key = getattr(sample, "key", None)
        if state == int(InstanceState.Alive):
            alive[key] = topic
        else:
            alive.pop(key, None)
    return alive


def check_no_leftover_readers(args):
    """Prove preflight() closes its subscribers: reader count returns to baseline."""
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsSubscription
    from unitree_sdk2py.core.channel import ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_

    prefix = hand_config.TOPIC_PREFIX

    # Order matters: cyclonedds allows ONE Domain object per domain id per process, and
    # the SDK's factory creates one. Building the observer participant first makes
    # ChannelFactoryInitialize fail with "create domain error".
    ChannelFactoryInitialize(args.domain, args.iface)

    observer = DomainParticipant(args.domain)
    reader = BuiltinDataReader(observer, BuiltinTopicDcpsSubscription)
    time.sleep(1.0)

    base = _count_state_readers(reader, prefix)
    print(f"RESULT: baseline readers on {prefix}/*/state: {len(base)}")

    # Positive control: the observer must be able to SEE a reader, or "0 after" proves nothing.
    probe = ChannelSubscriber(f"{prefix}/left/state", HandState_)
    probe.Init(lambda m: None)
    during_ctrl = _count_state_readers(reader, prefix)
    print(f"RESULT: with one deliberate subscriber open: {len(during_ctrl)} "
          f"(control -- the observer can see readers: {len(during_ctrl) > len(base)})")
    probe.Close()
    import gc
    gc.collect()
    after_ctrl = _count_state_readers(reader, prefix)
    print(f"RESULT: after closing it: {len(after_ctrl)}")

    # The real question.
    try:
        counts = hand_config.preflight(log=logging_mp.getLogger("readers_case"))
        print(f"RESULT: preflight returned {counts}")
    except RuntimeError as exc:
        print(f"RESULT: preflight refused ({exc.__class__.__name__}) -- "
              f"the close path still has to hold")
    after_pf = _count_state_readers(reader, prefix)
    print(f"RESULT: readers on {prefix}/*/state after preflight: {len(after_pf)}")

    control_ok = len(during_ctrl) > len(base)
    closed_ok = len(after_ctrl) <= len(base)
    preflight_ok = len(after_pf) <= len(base)
    print(f"RESULT: observer proven able to see readers: {control_ok}")
    print(f"RESULT: ChannelSubscriber.Close() removes a reader: {closed_ok}")
    print(f"RESULT: preflight leaves NO reader behind: {preflight_ok}")
    return 0 if (control_ok and closed_ok and preflight_ok) else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--case", required=True, choices=["silent", "dex3", "dex5"])
    parser.add_argument("--target", default="controller",
                        choices=["controller", "preflight", "readers"])
    parser.add_argument("--domain", type=int, default=1)
    parser.add_argument("--iface", type=str, default=None)
    args = parser.parse_args()

    print(f"--- target={args.target} case={args.case} domain={args.domain} "
          f"prefix={hand_config.TOPIC_PREFIX} timeout={hand_config.STATE_TIMEOUT_S}s ---")

    if args.target == "readers":
        sys.exit(check_no_leftover_readers(args))

    ChannelFactoryInitialize(args.domain, args.iface)

    if args.target == "preflight":
        sys.exit(run_preflight_case(args))

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
