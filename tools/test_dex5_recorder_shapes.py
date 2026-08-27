#!/usr/bin/env python3
"""Two shape checks for the Dex5-1P data path. No robot, no headset.

1. EpisodeWriter records 20-wide hand state/action per side. The launcher splits a
   40-wide shared array with [:20] and [-20:]; if EpisodeWriter or the split were
   wrong we would only find out after a recording session, from unusable data.

2. A HandCmd_ built by hand_config.make_hand_cmd() still has 20 motor_cmd entries
   after a real DDS round trip on domain 1. HandCmd_.motor_cmd is an IDL sequence
   and the SDK's factory allocates 7, so this is the proof that the resize survives
   serialisation rather than being silently truncated to the factory length.

    python tools/test_dex5_recorder_shapes.py

Exit codes: 0 both checks pass, 1 a check failed.
"""

import json
import multiprocessing as mp
import os
import shutil
import sys
import tempfile
import time

import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(REPO_ROOT)
sys.path.append(os.path.join(REPO_ROOT, "teleop"))

from teleop.robot_control import hand_config  # noqa: E402

N = hand_config.NUM_JOINTS_EXPECTED
DDS_DOMAIN = 1          # never 0 in this test; it must not touch a robot
ROUND_TRIP_TIMEOUT_S = 15.0


def check_recorder():
    print("=" * 78)
    print("1. EpisodeWriter hand state/action widths")
    print("=" * 78)
    from teleop.utils.episode_writer import EpisodeWriter

    tmp = tempfile.mkdtemp(prefix="dex5_recorder_")
    ok = True
    try:
        writer = EpisodeWriter(task_dir=os.path.join(tmp, "task"), task_goal="shape test",
                               task_desc="dex5 20-wide check", task_steps="n/a",
                               frequency=30, rerun_log=False)
        writer.create_episode()

        for frame in range(3):
            # Exactly the launcher's split of the 40-wide shared arrays.
            dual_hand_state = np.arange(2 * N, dtype=float) + frame * 100.0
            dual_hand_action = np.arange(2 * N, dtype=float) + frame * 100.0 + 0.5
            left_ee_state = list(dual_hand_state[:N])
            right_ee_state = list(dual_hand_state[-N:])
            left_hand_action = list(dual_hand_action[:N])
            right_hand_action = list(dual_hand_action[-N:])

            states = {
                "left_arm":  {"qpos": [0.0] * 7, "qvel": [], "torque": []},
                "right_arm": {"qpos": [0.0] * 7, "qvel": [], "torque": []},
                "left_ee":   {"qpos": left_ee_state,  "qvel": [], "torque": []},
                "right_ee":  {"qpos": right_ee_state, "qvel": [], "torque": []},
                "body":      {"qpos": []},
            }
            actions = {
                "left_arm":  {"qpos": [0.0] * 7, "qvel": [], "torque": []},
                "right_arm": {"qpos": [0.0] * 7, "qvel": [], "torque": []},
                "left_ee":   {"qpos": left_hand_action,  "qvel": [], "torque": []},
                "right_ee":  {"qpos": right_hand_action, "qvel": [], "torque": []},
                "body":      {"qpos": []},
            }
            writer.add_item(colors={}, depths={}, states=states, actions=actions,
                            tactiles={}, audios={})
        writer.save_episode()
        writer.close() if hasattr(writer, "close") else None

        data_json = None
        for _ in range(100):
            hits = []
            for root, _dirs, files in os.walk(tmp):
                if "data.json" in files:
                    hits.append(os.path.join(root, "data.json"))
            if hits:
                data_json = hits[0]
                break
            time.sleep(0.1)

        if data_json is None:
            print("FAIL: no data.json was written")
            return False
        print(f"data.json: {os.path.relpath(data_json, tmp)}")
        with open(data_json) as fh:
            payload = json.load(fh)

        items = payload.get("data", payload)
        if isinstance(items, dict):
            items = items.get("data", [])
        print(f"frames recorded: {len(items)}")
        if len(items) != 3:
            print(f"FAIL: expected 3 frames, got {len(items)}")
            ok = False

        for i, item in enumerate(items):
            for section in ("states", "actions"):
                for side in ("left_ee", "right_ee"):
                    qpos = item[section][side]["qpos"]
                    width = len(qpos)
                    good = width == N
                    ok = ok and good
                    print(f"  frame {i} {section:7s}.{side:9s} width={width:3d} "
                          f"{'OK' if good else f'FAIL (expected {N})'}")
            has_tactiles = "tactiles" in item
            ok = ok and has_tactiles
            print(f"  frame {i} tactiles present: {has_tactiles} "
                  f"(value={item.get('tactiles')!r}; empty is fine today)")

        first = items[0]
        left = first["states"]["left_ee"]["qpos"]
        right = first["states"]["right_ee"]["qpos"]
        split_ok = left[0] == 0.0 and right[-1] == float(2 * N - 1)
        ok = ok and split_ok
        print(f"  [:20] / [-20:] split preserved: {split_ok} "
              f"(left[0]={left[0]}, right[-1]={right[-1]}, expected 0.0 and {float(2 * N - 1)})")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    return ok


def _publisher(ready, done):
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_
    from teleop.robot_control import hand_config as hc

    ChannelFactoryInitialize(DDS_DOMAIN, None)
    pub = ChannelPublisher("rt/dex5_shape_test/cmd", HandCmd_)
    pub.Init()
    msg = hc.make_hand_cmd()
    for idx in range(len(msg.motor_cmd)):
        msg.motor_cmd[idx].q = float(idx) / 100.0
    ready.set()
    deadline = time.monotonic() + ROUND_TRIP_TIMEOUT_S
    while not done.is_set() and time.monotonic() < deadline:
        pub.Write(msg)
        time.sleep(0.02)
    os._exit(0)


def check_dds_round_trip():
    print()
    print("=" * 78)
    print(f"2. HandCmd_ survives a DDS round trip on domain {DDS_DOMAIN} with 20 motor_cmd")
    print("=" * 78)
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_

    factory_len = len(unitree_hg_msg_dds__HandCmd_().motor_cmd)
    built_len = len(hand_config.make_hand_cmd().motor_cmd)
    print(f"SDK factory allocates motor_cmd length : {factory_len}")
    print(f"hand_config.make_hand_cmd() length     : {built_len}")

    ready, done = mp.Event(), mp.Event()
    proc = mp.Process(target=_publisher, args=(ready, done), daemon=True)
    proc.start()
    ready.wait(timeout=10)

    ChannelFactoryInitialize(DDS_DOMAIN, None)
    received = {}
    sub = ChannelSubscriber("rt/dex5_shape_test/cmd", HandCmd_)
    sub.Init(lambda msg: received.setdefault("msg", msg))

    deadline = time.monotonic() + ROUND_TRIP_TIMEOUT_S
    while "msg" not in received and time.monotonic() < deadline:
        time.sleep(0.02)
    done.set()
    proc.join(timeout=5)

    if "msg" not in received:
        print(f"FAIL: nothing received within {ROUND_TRIP_TIMEOUT_S}s")
        return False

    msg = received["msg"]
    got = len(msg.motor_cmd)
    print(f"received motor_cmd length              : {got}")
    ok = got == N
    print(f"  length == {N}: {ok}"
          + ("" if ok else f"  <-- truncated toward the factory's {factory_len}"))

    qs = [round(c.q, 4) for c in msg.motor_cmd]
    q_ok = qs == [round(i / 100.0, 4) for i in range(got)]
    print(f"  q values round-tripped intact: {q_ok}")
    print(f"    {qs}")
    # MotorCmd_.kp/kd/q are float32 in the IDL, so 0.10 comes back as
    # 0.10000000149011612. Compare against the float32 image of the expected value,
    # not the float64 literal.
    def as_f32(x):
        return float(np.float32(x))

    kp = [c.kp for c in msg.motor_cmd]
    kd = [c.kd for c in msg.motor_cmd]
    want_kp = [as_f32(hand_config.GAINS['finger'][0])] * hand_config.THUMB_BASE_INDEX + \
              [as_f32(hand_config.GAINS['thumb'][0])] * (N - hand_config.THUMB_BASE_INDEX)
    want_kd = [as_f32(hand_config.GAINS['finger'][1])] * hand_config.THUMB_BASE_INDEX + \
              [as_f32(hand_config.GAINS['thumb'][1])] * (N - hand_config.THUMB_BASE_INDEX)
    gains_ok = (kp == want_kp) and (kd == want_kd)
    print(f"  kp/kd split finger(0-15)/thumb(16-19) survived: {gains_ok}")
    print(f"    kp[0]={kp[0]!r} kp[16]={kp[16]!r}")
    print(f"    kd[0]={kd[0]!r} kd[16]={kd[16]!r}")
    print(f"    (fields are float32 in the IDL: 0.10 -> {as_f32(0.10)!r}, "
          f"0.001 -> {as_f32(0.001)!r}; exact float64 equality would never hold)")
    return ok and q_ok and gains_ok


def main():
    ok_rec = check_recorder()
    ok_dds = check_dds_round_trip()
    print()
    print("=" * 78)
    print(f"recorder widths : {'PASS' if ok_rec else 'FAIL'}")
    print(f"DDS round trip  : {'PASS' if ok_dds else 'FAIL'}")
    return 0 if (ok_rec and ok_dds) else 1


if __name__ == "__main__":
    rc = main()
    # os._exit skips atexit AND stdout flushing -- flush explicitly or the tail of the
    # report is lost. _exit is needed because importing hand_config starts logging_mp,
    # whose listener otherwise keeps the interpreter alive.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(rc)
