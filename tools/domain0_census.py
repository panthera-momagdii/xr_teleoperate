#!/usr/bin/env python3
"""Read-only census of a DDS domain: who is on it, and who is writing what.

Run this BEFORE anything commands the robot. It answers the questions that turn into
expensive surprises later:

  * who else is on the domain, from which host, and under what process name
  * how many writers each safety-relevant topic has, from which IP, and at what rate
    each individual writer is producing (H3: PC1 publishes rt/lowcmd from TWO writer
    GUIDs summing to ~633 Hz -- a THIRD writer means something else is also driving
    the robot)
  * whether every writer on a topic agrees on the message type, by comparing the
    XTypes type identifier the wire actually advertises. An IDL mismatch found here
    costs a minute; found at the first command it costs a robot.

Robot day:
    python tools/domain0_census.py --domain 0 --iface <nic> --seconds 30

This tool only ever creates readers. It contains no writer of any kind; the gate check
greps this file for the SDK's writer class name and must find nothing, which is why
that name is not written out here, not even in prose.

Single process by design: no multiprocessing, no subprocess. A DDS entity created
before a fork is not usable from the child, and a reader in the parent does not see a
child's writes (measured in g12).

Exit codes: 0 census complete and every verdict OK, 3 a STOP verdict fired,
            4 the domain was empty apart from this tool.
"""

import argparse
import hashlib
import json
import os
import re
import sys
import time
from collections import defaultdict

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(REPO_ROOT)

from cyclonedds.builtin import (  # noqa: E402
    BuiltinDataReader, BuiltinTopicDcpsParticipant, BuiltinTopicDcpsPublication)
from cyclonedds.domain import Domain, DomainParticipant  # noqa: E402
from cyclonedds.qos import Policy, Qos  # noqa: E402
from cyclonedds.sub import DataReader  # noqa: E402
from cyclonedds.topic import Topic  # noqa: E402

from unitree_sdk2py.core.channel_config import (  # noqa: E402
    ChannelConfigAutoDetermine, ChannelConfigHasInterface)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (  # noqa: E402
    LowState_, LowCmd_, HandState_)

from teleop.robot_control import hand_config  # noqa: E402
from tools.tool_logging import timestamp  # noqa: E402
from tools._procs import install_reaper, reap_child_processes  # noqa: E402

EXIT_OK, EXIT_STOP, EXIT_EMPTY = 0, 3, 4

# The address PC1 is expected to write the arm topics from. Anything else on rt/lowcmd
# is a second thing driving the robot. Overridable: hardcoding an address into a safety
# check ages badly, and the offline evidence for the OK path needs it settable.
DEFAULT_EXPECTED_LOWCMD_IP = "192.168.123.161"

# Measured on this robot 2026-08-28: PC1 declares TWO rt/lowcmd writers, one carrying the
# whole stream and one idle at 0.0 Hz. Same shape on rt/lowstate. More than two is a
# different finding from a wrong source address, so they are reported separately.
EXPECTED_LOWCMD_WRITERS = 2

# G1 29-DoF arm joints (teleop/robot_control/robot_arm.py, G1_29_JointArmIndex).
ARM_JOINTS = {
    15: "left_shoulder_pitch", 16: "left_shoulder_roll", 17: "left_shoulder_yaw",
    18: "left_elbow", 19: "left_wrist_roll", 20: "left_wrist_pitch", 21: "left_wrist_yaw",
    22: "right_shoulder_pitch", 23: "right_shoulder_roll", 24: "right_shoulder_yaw",
    25: "right_elbow", 26: "right_wrist_roll", 27: "right_wrist_pitch", 28: "right_wrist_yaw",
}
SHOULDER_PITCH = (15, 22)

# OUR operating limits, not Unitree's. Unitree publishes no numeric arm-motor temperature
# limit in any documentation reachable from here -- only a described "thermal derating"
# behaviour with no threshold. See docs for the reasoning and for the open item to get the
# real number from Unitree. unitree_sdk2_python issue #129 reports G1 shoulder-pitch
# overheating with an Inspire FTP hand on arm_sdk but quotes no temperature at all.
ARM_TEMP_WARN_C = 60
ARM_TEMP_STOP_C = 75
HAND_TEMP_WARN_C = 45

# Deep history and best-effort: a KEEP_LAST(1) reader would undercount rt/lowstate at
# ~999 Hz by orders of magnitude, and a RELIABLE reader would not even match a
# best-effort writer (RxO: a best-effort reader matches both kinds).
CENSUS_QOS = Qos(Policy.Reliability.BestEffort, Policy.History.KeepLast(8192))

POLL_S = 0.002


# 224.0.0.0/4. Cyclone lists the discovery multicast group in __NetworkAddresses, and on
# the robot it lists it FIRST -- e.g.
#   'udp/239.255.0.1:7400@3,udp/192.168.123.161:40310@3'
# Taking found[0] therefore reported EVERY writer as coming from 239.255.0.1, which made
# `foreign` non-empty for every topic and fired a false STOP on rt/lowcmd on 2026-08-28.
# The tell was that rt/lowstate -- which can only come from PC1 -- reported the same
# address. A discovery group is not a source.
_MULTICAST = re.compile(r"^(22[4-9]|23\d)\.")


def _ip_of(network_addresses):
    """The first routable unicast address in __NetworkAddresses.

    'udp/239.255.0.1:7400@3,udp/192.168.123.161:40310@3' -> '192.168.123.161'
    'udp/172.20.10.2:47007@3'                            -> '172.20.10.2'
    'localprocess'                                       -> 'localprocess'
    """
    if not network_addresses:
        return None
    found = [ip for ip in re.findall(r"(\d+\.\d+\.\d+\.\d+)", network_addresses)
             if not _MULTICAST.match(ip) and not ip.startswith("127.")]
    if found:
        return found[0]
    return network_addresses.split(",")[0]


def _participant_info(sample):
    props, name = {}, None
    for policy in (sample.qos or []):
        cls = type(policy).__name__
        if cls == "Property":
            props[policy.key] = policy.value
        elif "EntityName" in cls:
            name = getattr(policy, "name", None)
    return {
        "guid": str(sample.key),
        "name": name,
        "hostname": props.get("__Hostname"),
        "process": props.get("__ProcessName"),
        "pid": props.get("__Pid"),
        "ip": _ip_of(props.get("__NetworkAddresses")),
        "network_addresses": props.get("__NetworkAddresses"),
        "properties": props,
    }


def _type_fingerprint(type_id):
    """Stable short hash of the XTypes type identifier the writer advertises."""
    if type_id is None:
        return None
    try:
        return hashlib.sha256(type_id.serialize()).hexdigest()[:16]
    except Exception:
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--domain", type=int, default=1,
                        help="DDS domain id (default 1 = this laptop; robot day uses 0)")
    parser.add_argument("--iface", type=str, default=None, help="network interface for DDS")
    parser.add_argument("--seconds", type=float, default=10.0, help="census window")
    parser.add_argument("--out", type=str, default=None,
                        help="JSON output path (default logs/census_<ts>.json)")
    parser.add_argument("--expect-lowcmd-ip", type=str, default=DEFAULT_EXPECTED_LOWCMD_IP,
                        help=f"the only IP allowed to write rt/lowcmd "
                             f"(default {DEFAULT_EXPECTED_LOWCMD_IP} = PC1)")
    args = parser.parse_args()

    out_path = args.out or os.path.join(REPO_ROOT, "logs", f"census_{timestamp()}.json")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    print(f"=== DDS census: domain {args.domain}, iface {args.iface or 'auto'}, "
          f"{args.seconds:g} s ===")
    print("READ-ONLY. This tool creates readers only.\n")

    # The Domain is built directly rather than through the SDK's factory. The factory
    # makes its own participant, so going through it would put TWO participants on the
    # domain -- both this tool's -- and a census that miscounts itself is worse than no
    # census. The XML is the SDK's own, so --iface behaves exactly as the launcher's
    # --network-interface does.
    config = (ChannelConfigAutoDetermine if args.iface is None
              else ChannelConfigHasInterface.replace('$__IF_NAME__$', args.iface))
    domain = Domain(args.domain, config)
    participant = DomainParticipant(args.domain)
    self_pid = str(os.getpid())
    print(f"created domain {args.domain} (cyclonedds {type(domain).__module__}) and one "
          f"participant, pid {self_pid}\n")

    part_reader = BuiltinDataReader(participant, BuiltinTopicDcpsParticipant)
    pub_reader = BuiltinDataReader(participant, BuiltinTopicDcpsPublication)

    watched = [
        ("rt/lowstate", LowState_),
        ("rt/lowcmd", LowCmd_),
        ("rt/arm_sdk", LowCmd_),
        (hand_config.TOPIC_LEFT_STATE, HandState_),
        (hand_config.TOPIC_RIGHT_STATE, HandState_),
    ]
    # The Inspire hands are the ones actually fitted. Their IDL lives in inspire_sdkpy,
    # which is optional here: if it is missing the census still runs and says so, rather
    # than refusing to start on a robot where the bridge is not installed yet.
    try:
        from inspire_sdkpy import inspire_dds
        watched += [("rt/inspire_hand/state/l", inspire_dds.inspire_hand_state),
                    ("rt/inspire_hand/state/r", inspire_dds.inspire_hand_state)]
        inspire_available = True
    except Exception as exc:
        inspire_available = False
        print(f"  note: inspire_sdkpy not importable ({exc.__class__.__name__}), so "
              f"rt/inspire_hand/state/* is NOT being watched this run.")
    readers = {}
    for name, kind in watched:
        readers[name] = DataReader(participant, Topic(participant, name, kind), qos=CENSUS_QOS)

    counts = defaultdict(lambda: defaultdict(int))     # topic -> publication_handle -> n
    first_seen, last_seen = {}, {}                     # (topic, handle) -> monotonic
    ticks = defaultdict(list)                          # (topic, handle) -> [tick, ...]
    stamps = defaultdict(list)                         # (topic, handle) -> [source_timestamp]
    arm_temp_max = {}                                  # motor index -> max temperature C
    hand_fields = {}                                   # topic -> last state field snapshot

    started = time.monotonic()
    deadline = started + args.seconds
    next_tick = started + 5.0
    while time.monotonic() < deadline:
        for name, reader in readers.items():
            for sample in reader.take(N=1024):
                info = sample.sample_info
                handle = info.publication_handle
                counts[name][handle] += 1
                key = (name, handle)
                now = time.monotonic()
                first_seen.setdefault(key, now)
                last_seen[key] = now

                # Publisher-side sequencing, so the rate does not depend on how fast
                # THIS process dequeues. See the rate section below.
                tick = getattr(sample, "tick", None)
                if tick is not None:
                    ticks[key].append(int(tick))
                st = getattr(info, "source_timestamp", None)
                if st is not None:
                    stamps[key].append(int(st))

                # Arm motor temperatures. MotorState_.temperature is int16[2] per motor;
                # take the hotter of the two, and the max over the whole window per joint.
                ms = getattr(sample, "motor_state", None)
                if ms is not None:
                    for idx, motor in enumerate(ms):
                        t = getattr(motor, "temperature", None)
                        if t is None:
                            continue
                        hot = max(int(x) for x in t) if hasattr(t, "__iter__") else int(t)
                        if hot > arm_temp_max.get(idx, -999):
                            arm_temp_max[idx] = hot

                # Inspire hand state: the fields upstream throws away are exactly the
                # ones a refusal path needs.
                if hasattr(sample, "angle_act"):
                    hand_fields[name] = {
                        "angle_act": [int(v) for v in sample.angle_act],
                        "force_act": [int(v) for v in getattr(sample, "force_act", [])],
                        "current": [int(v) for v in getattr(sample, "current", [])],
                        "err": [int(v) for v in getattr(sample, "err", [])],
                        "status": [int(v) for v in getattr(sample, "status", [])],
                        "temperature": [int(v) for v in getattr(sample, "temperature", [])],
                    }
        if time.monotonic() >= next_tick:
            next_tick += 5.0
            total = sum(sum(v.values()) for v in counts.values())
            print(f"  t={time.monotonic() - started:5.1f}s  {total} samples so far")
        time.sleep(POLL_S)
    window = time.monotonic() - started

    # --- discovery -------------------------------------------------------
    participants = {}
    for sample in part_reader.read(N=512):
        if sample.sample_info.instance_state != 16:     # ALIVE only
            continue
        info = _participant_info(sample)
        info["is_self"] = (info["pid"] == self_pid
                           and info["hostname"] == os.uname().nodename)
        participants[info["guid"]] = info

    writers = {}            # publication instance handle -> writer record
    for sample in pub_reader.read(N=1024):
        if sample.sample_info.instance_state != 16:
            continue
        writers[sample.sample_info.instance_handle] = {
            "guid": str(sample.key),
            "participant_guid": str(sample.participant_key),
            "topic": sample.topic_name,
            "type_name": sample.type_name,
            "type_fingerprint": _type_fingerprint(sample.type_id),
        }

    # --- report ----------------------------------------------------------
    others = [i for i in participants.values() if not i["is_self"]]
    print(f"\n{'=' * 78}\n1. PARTICIPANTS on domain {args.domain}: {len(participants)} "
          f"({len(others)} besides this tool)\n{'=' * 78}")
    print(f"  {'':<4} {'ip':<16} {'host':<12} {'process':<14} {'pid':<8} {'name':<22} guid")
    for info in sorted(participants.values(), key=lambda d: (d["ip"] or "", d["guid"])):
        print(f"  {'self' if info['is_self'] else '':<4} "
              f"{str(info['ip'] or '?'):<16} {str(info['hostname'] or '?'):<12} "
              f"{str(info['process'] or '?'):<14} {str(info['pid'] or '?'):<8} "
              f"{str(info['name'] or '-'):<22} {info['guid']}")
    named = [i for i in participants.values() if i["name"]]
    if not named:
        print("  (no participant advertises an EntityName QoS; g1_dex_protocol_v2.0 would "
              "appear in the 'name' column if it did)")

    topics_report, verdicts, stop = {}, [], False
    print(f"\n{'=' * 78}\n2. WRITERS per watched topic (window {window:.2f} s)\n{'=' * 78}")
    for name, _kind in watched:
        topic_writers = [w for w in writers.values() if w["topic"] == name]
        rows = []
        for w in topic_writers:
            handle = next((h for h, rec in writers.items() if rec["guid"] == w["guid"]), None)
            n = counts[name].get(handle, 0)
            key = (name, handle)
            span = (last_seen.get(key, 0) - first_seen.get(key, 0)) if key in first_seen else 0.0
            observed = (n / span) if span > 0.2 else (n / window if window else 0.0)
            derived, method, tps, tpsam, med_rate = _derived_rate(
                ticks.get(key, []), stamps.get(key, []), span, n)
            # A quiet writer is trustworthy either way: zero dequeued samples over the
            # window is zero traffic, and there is nothing to derive from.
            rate = derived if (derived and derived > observed) else observed
            dropping = bool(derived and observed < 0.9 * derived)
            pinfo = participants.get(w["participant_guid"], {})
            rows.append({**w, "samples": n, "rate_hz": round(rate, 1),
                         "observed_hz": round(observed, 1),
                         "derived_hz": round(derived, 1) if derived else None,
                         "rate_method": method or "count", "dropping": dropping,
                         "median_hz": round(med_rate, 1) if med_rate else None,
                         "ticks_per_s": round(tps, 1) if tps else None,
                         "ticks_per_sample": round(tpsam, 3) if tpsam else None,
                         "ip": pinfo.get("ip"), "hostname": pinfo.get("hostname"),
                         "process": pinfo.get("process"), "pid": pinfo.get("pid")})
        rows.sort(key=lambda r: r["guid"])

        print(f"\n  {name}: {len(rows)} writer(s)")
        if rows:
            print(f"    {'ip':<16} {'rate Hz':>9} {'via':<6} {'median':>8} {'seen Hz':>8} "
                  f"{'samples':>8}  {'type fp':<16} writer guid")
            for r in rows:
                flag = "  DROPPING" if r["dropping"] else ""
                med = f"{r['median_hz']:>8.1f}" if r["median_hz"] else f"{'-':>8}"
                print(f"    {str(r['ip'] or '?'):<16} {r['rate_hz']:>9.1f} "
                      f"{r['rate_method']:<6} {med} {r['observed_hz']:>8.1f} "
                      f"{r['samples']:>8}  "
                      f"{str(r['type_fingerprint'] or '-'):<16} {r['guid']}{flag}")
            if any(r["rate_method"] == "stamp" for r in rows):
                print("    rate estimator: 1 / p10(source_timestamp deltas) -- the writer's own")
                print("    send times, so it does not depend on how fast this reader dequeues.")
                print("    'median' is 1 / median(same deltas): with no drops the two agree, and")
                print("    every dropped sample pushes the median down while p10 holds.")
            print(f"    total {sum(r['rate_hz'] for r in rows):.1f} Hz across "
                  f"{len(rows)} writer(s); type(s): "
                  f"{sorted({r['type_name'] for r in rows})}")
            if any(r["dropping"] for r in rows):
                print("    DROPPING: this reader saw fewer samples than the publisher sent. "
                      "The rate column is still right (it comes from the publisher's own "
                      "sequencing); it is the census that is behind, not the robot.")
            for r in rows:
                if r["ticks_per_s"]:
                    print(f"      tick: {r['ticks_per_s']:.1f} ticks/s, "
                          f"{r['ticks_per_sample']:.3f} ticks/sample "
                          f"({'a per-publish counter' if 0.9 <= r['ticks_per_sample'] <= 1.1 else 'NOT 1:1 with publishes -- tick is a clock or a multi-step counter, so ticks/s is NOT the publish rate'})")
            if all(r["rate_method"] == "count" for r in rows) and any(r["samples"] for r in rows):
                print("    NOTE: rate is this reader's dequeue count -- a FLOOR, not a "
                      "measurement. No tick field and too few samples to time the publisher.")

        verdict = _verdict(name, rows, args.expect_lowcmd_ip)
        stop = stop or verdict["stop"]
        verdicts.append(verdict)
        print(f"    {verdict['text']}")
        topics_report[name] = {"writers": rows, "verdict": verdict}

    print(f"\n{'=' * 78}\n3. TYPE AGREEMENT\n{'=' * 78}")
    type_report = {}
    for name, _kind in watched:
        rows = topics_report[name]["writers"]
        fps = {r["type_fingerprint"] for r in rows if r["type_fingerprint"]}
        names = {r["type_name"] for r in rows}
        agree = len(fps) <= 1 and len(names) <= 1
        type_report[name] = {"type_names": sorted(names), "fingerprints": sorted(fps),
                             "agree": agree}
        if rows:
            print(f"  {name}: {'all writers agree' if agree else 'MISMATCH'} "
                  f"-- types {sorted(names)}, fingerprints {sorted(fps)}")
            if not agree:
                stop = True
        else:
            print(f"  {name}: no writers")

    print(f"\n{'=' * 78}\n4. TEMPERATURES\n{'=' * 78}")
    if arm_temp_max:
        print("  Arm joint motors, max over the window (rt/lowstate, MotorState_.temperature,")
        print("  int16[2] per motor -- the hotter of the two is taken):")
        print(f"    {'idx':>4}  {'joint':<24} {'max C':>6}")
        for idx in sorted(ARM_JOINTS):
            if idx in arm_temp_max:
                mark = "  <-- shoulder pitch" if idx in SHOULDER_PITCH else ""
                print(f"    {idx:>4}  {ARM_JOINTS[idx]:<24} {arm_temp_max[idx]:>6}{mark}")
        hot = {i: t for i, t in arm_temp_max.items() if t >= ARM_TEMP_WARN_C}
        if hot:
            print(f"    WARNING: {len(hot)} motor(s) at or above {ARM_TEMP_WARN_C} C: "
                  f"{ {ARM_JOINTS.get(i, i): t for i, t in sorted(hot.items())} }")
        sp = [arm_temp_max[i] for i in SHOULDER_PITCH if i in arm_temp_max]
        if sp:
            print(f"    shoulder pitch max: {max(sp)} C   "
                  f"(warn {ARM_TEMP_WARN_C}, stop {ARM_TEMP_STOP_C} -- see docs; these are "
                  f"OUR limits, Unitree publishes none we could find)")
        non_arm = {i: t for i, t in arm_temp_max.items() if i not in ARM_JOINTS}
        if non_arm:
            print(f"    (legs/waist, for context: max {max(non_arm.values())} C across "
                  f"{len(non_arm)} motors)")
    else:
        print("  No rt/lowstate samples with motor_state -- no arm temperatures this run.")

    if hand_fields:
        print("\n  Inspire hand state, last sample per side. angle_act is the only field")
        print("  xr_teleoperate reads; err/status/temperature are what a refusal needs:")
        for topic, f in sorted(hand_fields.items()):
            print(f"    {topic}")
            for k in ("angle_act", "force_act", "current", "err", "status", "temperature"):
                if f.get(k):
                    print(f"      {k:12s} {f[k]}")
            errs = [i for i, e in enumerate(f.get("err", [])) if e]
            if errs:
                print(f"      *** err NON-ZERO on DOF {errs} -- see the RH56 error bits ***")
            temps = f.get("temperature", [])
            if temps and max(temps) >= HAND_TEMP_WARN_C:
                print(f"      *** temperature {max(temps)} C >= {HAND_TEMP_WARN_C} C ***")
    elif inspire_available:
        print("\n  No rt/inspire_hand/state/* samples -- the bridge is not running.")

    report = {
        "captured_at": timestamp(),
        "inspire_topics_watched": inspire_available,
        "arm_motor_temperature_max_c": {str(k): v for k, v in sorted(arm_temp_max.items())},
        "inspire_hand_state_fields": hand_fields,
        "domain": args.domain,
        "iface": args.iface,
        "window_s": round(window, 3),
        "expect_lowcmd_ip": args.expect_lowcmd_ip,
        "participant_count": len(participants),
        "participant_count_excluding_self": len(others),
        "participants": sorted(participants.values(), key=lambda d: d["guid"]),
        "topics": topics_report,
        "type_agreement": type_report,
        "verdicts": verdicts,
        "stop": stop,
    }
    with open(out_path, "w") as fh:
        json.dump(report, fh, indent=2, default=str)
    print(f"\nreport written to {out_path}")

    if not others and not any(topics_report[n]["writers"] for n, _ in watched):
        print(f"\nRESULT: domain {args.domain} is EMPTY apart from this tool "
              f"({len(participants)} participant, all this process).")
        return EXIT_EMPTY
    if stop:
        print("\nRESULT: STOP -- see the verdicts above. Do not proceed to Part B.")
        return EXIT_STOP
    print("\nRESULT: OK -- no unexpected writer, no type mismatch.")
    return EXIT_OK


def _derived_rate(tick_list, stamp_list, span, n_samples):
    """Publish rate from the PUBLISHER's own timing, plus the tick scale as a diagnostic.

    The census's sample count is a FLOOR: if the reader cannot keep up it under-reports
    with no signal. On 2026-08-28 rt/lowcmd (666.5) and rt/lowstate (660.4) landed within
    1% of each other despite being independent streams from different PC1 processes --
    the signature of a consumer-side ceiling.

    PRIMARY METHOD: source_timestamp. Each sample carries the writer's own send time, so
    the inter-sample delta is publisher-side. Taking the 10th percentile of positive
    deltas recovers the true period: a dropped sample doubles a delta, so a low
    percentile still finds the real one as long as ANY two adjacent samples were caught.
    Median would drift with the drop rate; minimum would chase timestamp jitter.

    TICK IS NOT A RATE. LowState_.tick is reported only as ticks/second and
    ticks/sample, never as the publish rate, because its scale is not documented: it may
    be a per-publish counter or a millisecond clock, and those give answers that differ
    by whatever the publish period is. Measured against a fixture publishing at 191.3 Hz
    while advancing tick by 5 each time, treating tick deltas as a rate reported
    960.3 Hz and a false DROPPING. ticks_per_sample tells you which it is: ~1 means a
    per-publish counter, and on a 1 kHz millisecond clock it equals the publish period
    in ms. The 2026-08-24 "999.6 Hz from tick" figure is therefore 999.6 TICKS per
    second, and is only the publish rate if that tick is per-publish -- still unresolved.

    The median is reported alongside p10 because the gap between them IS the drop
    signal. Drops make deltas longer, so the median PERIOD rises and the median RATE
    falls, while p10 stays near the true period. With no drops the two rates agree:
    measured on a 118.4 Hz fixture, p10 gave 120.8 Hz and the median 118.7 Hz.

    Returns (rate, method, ticks_per_s, ticks_per_sample, median_rate).
    """
    ticks_per_s = ticks_per_sample = None
    if len(tick_list) >= 2 and span > 0.2:
        delta = (tick_list[-1] - tick_list[0]) % (1 << 32)          # wrap-safe uint32
        if 0 < delta < (1 << 31):
            ticks_per_s = delta / span
            if n_samples > 1:
                ticks_per_sample = delta / (n_samples - 1)

    if span <= 0.2:
        return None, None, ticks_per_s, ticks_per_sample, None
    if len(stamp_list) >= 8:
        d = sorted(b - a for a, b in zip(stamp_list, stamp_list[1:]) if b > a)
        if d:
            p10 = d[max(0, int(0.10 * len(d)))]
            med = d[len(d) // 2]
            if p10 > 0:
                med_rate = (1e9 / med) if med > 0 else None
                return 1e9 / p10, "stamp", ticks_per_s, ticks_per_sample, med_rate
    return None, None, ticks_per_s, ticks_per_sample, None


def _verdict(topic, rows, expected_lowcmd_ip):
    """One line per topic, and whether it is a stop-the-session finding."""
    ips = sorted({r["ip"] for r in rows if r["ip"]})
    n = len(rows)
    if topic == "rt/arm_sdk":
        # Gate on TRAFFIC, not on the endpoint existing. PC1 declares an idle rt/arm_sdk
        # writer that publishes 0.0 Hz -- seen 2026-08-24 and again 2026-08-28. STOPping
        # on `if n:` meant the census could never pass on this robot.
        live = [r for r in rows if r["rate_hz"] > 0]
        if live:
            live_ips = sorted({r["ip"] for r in live if r["ip"]})
            return {"topic": topic, "stop": True,
                    "text": f"-> STOP: rt/arm_sdk is being WRITTEN by {len(live)} writer(s) "
                            f"from {live_ips} at up to "
                            f"{max(r['rate_hz'] for r in live):.1f} Hz. Something is in "
                            f"motion-control mode; this project never uses --motion."}
        if n:
            return {"topic": topic, "stop": False,
                    "text": f"-> OK: {n} declared writer(s) but 0.0 Hz -- PC1's idle "
                            f"endpoint, expected on this robot."}
        return {"topic": topic, "stop": False, "text": "-> OK: no writer."}

    if topic == "rt/lowcmd":
        if n == 0:
            return {"topic": topic, "stop": False,
                    "text": "-> OK: 0 writers (nothing is commanding the arms yet)."}
        # Two DIFFERENT findings, previously conflated. The old branch fired on any
        # address mismatch and printed "THIRD WRITER ... has 2 writers" in one sentence,
        # because the writer COUNT was never consulted.
        foreign = [ip for ip in ips if ip != expected_lowcmd_ip]
        live = [r for r in rows if r["rate_hz"] > 0]
        findings = []
        if foreign:
            findings.append(f"FOREIGN SOURCE: rt/lowcmd is written from {foreign}, "
                            f"expected only {expected_lowcmd_ip} (PC1)")
        if n > EXPECTED_LOWCMD_WRITERS:
            findings.append(f"UNEXPECTED WRITER COUNT: rt/lowcmd has {n} writers, "
                            f"expected at most {EXPECTED_LOWCMD_WRITERS}")
        if findings:
            return {"topic": topic, "stop": True, "text": "-> STOP: " + "; ".join(findings)}
        return {"topic": topic, "stop": False,
                "text": f"-> OK: {n} writer(s) ({len(live)} carrying traffic), all from "
                        f"{expected_lowcmd_ip}."}

    if n == 0:
        return {"topic": topic, "stop": False, "text": "-> 0 writers."}
    return {"topic": topic, "stop": False,
            "text": f"-> {n} writer(s) from {ips}."}


if __name__ == "__main__":
    # [panthera] Importing hand_config above calls logging_mp.getLogger(), which forks a
    # NON-DAEMON listener process that logging_mp only reaps from an atexit hook. This
    # tool leaves via os._exit(), which skips atexit, so the listener was orphaned on
    # every single run. Four of them were still alive on this host on 2026-09-09 --
    # PPID 1, 2h43m to 4h08m past a --seconds 15..30 budget -- squatting on the DDS
    # domain and holding the inherited stdout open, which is why the symptom looked
    # like "census hung" rather than "census leaked". See tools/_procs.py.
    install_reaper()
    rc = main()
    sys.stdout.flush()
    sys.stderr.flush()
    reap_child_processes()          # os._exit() skips atexit; call it directly
    os._exit(rc)
