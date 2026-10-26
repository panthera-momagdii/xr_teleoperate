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
    # [RT5b] An arm that tracks its command. With a constant q the launcher's IK starts
    # cold from that q every solve (solve_ik uses the measured q as its warm start and
    # smoothing anchor): IK p50 7 ms against 2.8 ms when q follows the command, so the
    # full-chain IK numbers on fakes were a fixture artifact. Domain 1 only, like
    # everything here: it reads rt/lowcmd on the SAME domain it publishes on.
    # [RT6b] A rising arm temperature, to test the temperature stop end to end.
    ap.add_argument("--temp-arms", type=float, default=None,
                    help="report this temperature (C) on the 14 arm motors")
    ap.add_argument("--temp-rise", type=float, default=0.0,
                    help="...rising by this many C per second")
    # [RT7] temperature[0] is the housing, [1] the winding (SDK terminations.hpp). Without
    # these, both report --temp-arms / --temp-rise as before.
    ap.add_argument("--temp-winding", type=float, default=None,
                    help="report this winding temperature [1] (C) on the 14 arm motors; --temp-arms is then [0] only")
    ap.add_argument("--temp-winding-rise", type=float, default=0.0,
                    help="...rising by this many C per second")
    ap.add_argument("--follow-cmd", action="store_true",
                    help="report the last rt/lowcmd arm q (same domain) as the arm q")
    # [RT6d] A simulated arm: the 14 arm joints move by forward dynamics under the PD +
    # feed-forward the last rt/lowcmd asks for (tools/arm_physics.py). For measuring
    # joint SPEED, which --follow-cmd cannot show (it reports the command as the state).
    ap.add_argument("--physics", action="store_true",
                    help="simulate the arms (PD + feed-forward from rt/lowcmd, ABA dynamics)")
    ap.add_argument("--pose", type=str, default=None,
                    help="start-pose yaml for the initial arm q (with --physics or alone)")
    ap.add_argument("--armature", type=float, default=0.02,
                    help="--physics: rotor inertia added to every arm joint (kg m^2)")
    ap.add_argument("--viscous", type=float, default=0.05,
                    help="--physics: viscous friction (Nm s/rad)")
    ap.add_argument("--dq-noise", type=float, default=0.0,
                    help="--physics: gaussian noise on the reported dq (rad/s, sigma)")
    ap.add_argument("--friction", type=float, nargs=2, default=None, metavar=("STATIC", "SLIDING"),
                    help="--physics: Stribeck gearbox friction, static and sliding (Nm) [RT6g]")
    ap.add_argument("--friction-joints", choices=("all", "wrists"), default="wrists",
                    help="--physics: which arm joints get --friction (default wrists)")
    ap.add_argument("--friction-arm", type=float, nargs=2, default=None, metavar=("STATIC", "SLIDING"),
                    help="--physics: friction on the shoulders and elbows instead, with --friction on "
                         "the wrists [RT6h; calibrated to RT6g: 2.7 0.81 with --friction 1.7 0.17 "
                         "and --wrist-kd-eff 0.3]")
    ap.add_argument("--wrist-armature", type=float, default=None,
                    help="--physics: rotor inertia of the wrist joints (default --armature)")
    ap.add_argument("--wrist-kd-eff", type=float, default=None,
                    help="--physics: fraction of the commanded kd the wrists really get [RT6h; "
                         "0.3 with the RT6g friction above reproduces RT6g's wrist roll]")
    # [RT7] Fake chatter: listed motors report dq and tau_est alternating sign every sample
    # (what the wrists did in the 23 Oct 20:16 lowering: 300 to 500 flips/s, 25 to 52 Nm).
    ap.add_argument("--chatter", type=int, nargs="+", default=None, metavar="MOTOR",
                    help="motor indices (15..28) that chatter")
    ap.add_argument("--chatter-file", type=str, default=None,
                    help="chatter only while this file exists (default: always)")
    ap.add_argument("--chatter-dq", type=float, default=1.5, help="chatter dq amplitude (rad/s)")
    ap.add_argument("--chatter-tau", type=float, default=20.0, help="chatter tau_est amplitude (Nm)")
    ap.add_argument("--trace", type=str, default=None,
                    help="--physics: CSV of the TRUE simulated joint speed, max per 50 ms "
                         "(t_mono, 14 joints), for tests that must not depend on DDS timing")
    args = ap.parse_args()
    if args.physics and args.follow_cmd:
        ap.error("--physics and --follow-cmd are exclusive")

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
    follow = {"q": None}
    q_init = None
    if args.pose:
        sys.path.append(os.path.join(REPO_ROOT, "teleop"))
        from teleop.robot_control.start_pose import load_pose
        q_init = load_pose(args.pose)
    phys = None
    if args.physics:
        import numpy as np
        from tools.arm_physics import ArmPhysics
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
        _fr, _fj = args.friction, (None if args.friction_joints == "all" else [4, 5, 6, 11, 12, 13])
        if args.friction_arm is not None:
            _wr = [4, 5, 6, 11, 12, 13]
            _fs, _fc = np.array([args.friction_arm[0]] * 14), np.array([args.friction_arm[1]] * 14)
            if args.friction is not None:
                _fs[_wr], _fc[_wr] = args.friction
            else:
                _fs[_wr], _fc[_wr] = 0.0, 0.0
            _fr, _fj = (_fs, _fc), None
        phys = ArmPhysics(q_init if q_init is not None else np.full(14, args.q_arms),
                          armature=args.armature, viscous=args.viscous, friction=_fr, friction_joints=_fj)
        if args.wrist_armature is not None:
            phys.model.armature[[4, 5, 6, 11, 12, 13]] = args.wrist_armature
        if args.wrist_kd_eff is not None:
            phys.kd_eff[[4, 5, 6, 11, 12, 13]] = args.wrist_kd_eff
        _arm = list(G1_29_ARM_JOINTS)

        def _on_cmd_phys(m):
            mc = m.motor_cmd
            phys.set_command([mc[i].q for i in _arm], [mc[i].dq for i in _arm],
                             [mc[i].kp for i in _arm], [mc[i].kd for i in _arm],
                             [mc[i].tau for i in _arm], time.monotonic())
        _psub = ChannelSubscriber("rt/lowcmd", LowCmd_)
        _psub.Init(_on_cmd_phys, 10)
        _rng = np.random.default_rng(0)
        _trace = open(args.trace, "w", buffering=1) if args.trace else None
        if _trace:
            _trace.write("t_mono," + ",".join(f"dq{k}" for k in range(14)) + "\n")
        _tr = {"t": time.monotonic(), "m": np.zeros(14)}
        log.info(f"--physics: arms simulated (armature {args.armature} kg m^2, viscous "
                 f"{args.viscous} Nm s/rad, dq noise {args.dq_noise} rad/s)")
    if args.follow_cmd:
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_

        def _on_cmd(m):
            follow["q"] = {i: float(m.motor_cmd[i].q) for i in G1_29_ARM_JOINTS}
        _sub = ChannelSubscriber("rt/lowcmd", LowCmd_)
        _sub.Init(_on_cmd, 10)
        log.info("--follow-cmd: arm q follows rt/lowcmd on this domain")
    log.info(f"publishing {TOPIC} on domain {args.domain} iface {args.iface} "
             f"at {args.rate} Hz, mode_machine={args.mode_machine}, "
             f"arm q={args.q_arms}, {N_MOTOR_SLOTS} motor slots")

    period = 1.0 / args.rate
    t_start = time.monotonic()
    t_phys = t_start
    deadline = time.monotonic() + args.seconds if args.seconds > 0 else None
    tick = 0
    n = 0
    try:
        while True:
            if deadline is not None and time.monotonic() >= deadline:
                break
            msg = build_lowstate(mk, args.mode_machine, args.q_arms, args.q_other, tick)
            if args.temp_arms is not None:
                _tc = int(args.temp_arms + args.temp_rise * (time.monotonic() - t_start))
                _tw = _tc if args.temp_winding is None else int(
                    args.temp_winding + args.temp_winding_rise * (time.monotonic() - t_start))
                for i in G1_29_ARM_JOINTS:
                    msg.motor_state[i].temperature = [_tc, _tw]
            fq = follow["q"]
            if fq is not None:
                for i, v in fq.items():
                    msg.motor_state[i].q = v
            elif phys is not None:
                _now = time.monotonic()
                phys.advance(_now - t_phys, _now)
                t_phys = _now
                if _trace:
                    _tr["m"] = np.maximum(_tr["m"], phys.dq_max_since_read())
                    if _now - _tr["t"] >= 0.05:
                        _trace.write(f"{_now:.4f}," + ",".join(f"{v:.4f}" for v in _tr["m"]) + "\n")
                        _tr["t"], _tr["m"] = _now, np.zeros(14)
                _dqn = phys.dq + (_rng.normal(0.0, args.dq_noise, 14) if args.dq_noise > 0 else 0.0)
                for k, i in enumerate(G1_29_ARM_JOINTS):
                    ms = msg.motor_state[i]
                    ms.q = float(phys.q[k]); ms.dq = float(_dqn[k]); ms.tau_est = float(phys.tau[k])
            elif q_init is not None:
                for k, i in enumerate(G1_29_ARM_JOINTS):
                    msg.motor_state[i].q = float(q_init[k])
            if args.chatter and (args.chatter_file is None or os.path.exists(args.chatter_file)):
                _sg = 1.0 if tick % 2 == 0 else -1.0
                for i in args.chatter:
                    msg.motor_state[i].dq = _sg * args.chatter_dq
                    msg.motor_state[i].tau_est = _sg * args.chatter_tau
            elif args.chatter:
                for i in args.chatter:
                    if phys is None:
                        msg.motor_state[i].dq = 0.0
                        msg.motor_state[i].tau_est = 0.0
            pub.Write(msg)
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
