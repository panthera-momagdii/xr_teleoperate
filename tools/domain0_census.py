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

EXIT_OK, EXIT_STOP, EXIT_EMPTY = 0, 3, 4

# The address PC1 is expected to write the arm topics from. Anything else on rt/lowcmd
# is a second thing driving the robot. Overridable: hardcoding an address into a safety
# check ages badly, and the offline evidence for the OK path needs it settable.
DEFAULT_EXPECTED_LOWCMD_IP = "192.168.123.161"

# Deep history and best-effort: a KEEP_LAST(1) reader would undercount rt/lowstate at
# ~999 Hz by orders of magnitude, and a RELIABLE reader would not even match a
# best-effort writer (RxO: a best-effort reader matches both kinds).
CENSUS_QOS = Qos(Policy.Reliability.BestEffort, Policy.History.KeepLast(8192))

POLL_S = 0.002


def _ip_of(network_addresses):
    """'udp/172.20.10.2:47007@3' -> '172.20.10.2'; 'localprocess' -> 'localprocess'."""
    if not network_addresses:
        return None
    found = re.findall(r"(\d+\.\d+\.\d+\.\d+)", network_addresses)
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
    readers = {}
    for name, kind in watched:
        readers[name] = DataReader(participant, Topic(participant, name, kind), qos=CENSUS_QOS)

    counts = defaultdict(lambda: defaultdict(int))     # topic -> publication_handle -> n
    first_seen, last_seen = {}, {}                     # (topic, handle) -> monotonic

    started = time.monotonic()
    deadline = started + args.seconds
    next_tick = started + 5.0
    while time.monotonic() < deadline:
        for name, reader in readers.items():
            for sample in reader.take(N=1024):
                handle = sample.sample_info.publication_handle
                counts[name][handle] += 1
                key = (name, handle)
                now = time.monotonic()
                first_seen.setdefault(key, now)
                last_seen[key] = now
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
            rate = (n / span) if span > 0.2 else (n / window if window else 0.0)
            pinfo = participants.get(w["participant_guid"], {})
            rows.append({**w, "samples": n, "rate_hz": round(rate, 1),
                         "ip": pinfo.get("ip"), "hostname": pinfo.get("hostname"),
                         "process": pinfo.get("process"), "pid": pinfo.get("pid")})
        rows.sort(key=lambda r: r["guid"])

        print(f"\n  {name}: {len(rows)} writer(s)")
        if rows:
            print(f"    {'ip':<16} {'rate Hz':>9} {'samples':>8}  {'type fp':<16} writer guid")
            for r in rows:
                print(f"    {str(r['ip'] or '?'):<16} {r['rate_hz']:>9.1f} {r['samples']:>8}  "
                      f"{str(r['type_fingerprint'] or '-'):<16} {r['guid']}")
            print(f"    total {sum(r['rate_hz'] for r in rows):.1f} Hz across "
                  f"{len(rows)} writer(s); type(s): "
                  f"{sorted({r['type_name'] for r in rows})}")

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

    report = {
        "captured_at": timestamp(),
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


def _verdict(topic, rows, expected_lowcmd_ip):
    """One line per topic, and whether it is a stop-the-session finding."""
    ips = sorted({r["ip"] for r in rows if r["ip"]})
    n = len(rows)
    if topic == "rt/arm_sdk":
        if n:
            return {"topic": topic, "stop": True,
                    "text": f"-> STOP: rt/arm_sdk has {n} writer(s) from {ips}. Something is "
                            f"in motion-control mode; this project never uses --motion."}
        return {"topic": topic, "stop": False, "text": "-> OK: no writer (expected)."}

    if topic == "rt/lowcmd":
        if n == 0:
            return {"topic": topic, "stop": False,
                    "text": "-> OK: 0 writers (nothing is commanding the arms yet)."}
        foreign = [ip for ip in ips if ip != expected_lowcmd_ip]
        if foreign:
            return {"topic": topic, "stop": True,
                    "text": f"-> THIRD WRITER {foreign} STOP: rt/lowcmd has {n} writers from "
                            f"{ips}; only {expected_lowcmd_ip} (PC1) is expected."}
        return {"topic": topic, "stop": False,
                "text": f"-> OK: {n} writer(s), all from {expected_lowcmd_ip}."}

    if n == 0:
        return {"topic": topic, "stop": False, "text": "-> 0 writers."}
    return {"topic": topic, "stop": False,
            "text": f"-> {n} writer(s) from {ips}."}


if __name__ == "__main__":
    rc = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(rc)
