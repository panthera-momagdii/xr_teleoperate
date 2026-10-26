#!/usr/bin/env python3
"""RT1 step 0: read-only battery read on DDS domain 0.

Readers only: builtin DCPSPublication reader to find the BMS topic, then a
best-effort reader on it. No writer of any kind is created in this file.
"""
import argparse, json, os, sys, time
from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsPublication
from cyclonedds.domain import Domain, DomainParticipant
from cyclonedds.qos import Policy, Qos
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from unitree_sdk2py.core.channel_config import ChannelConfigHasInterface
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import BmsState_

ap = argparse.ArgumentParser()
ap.add_argument("--iface", default="eth0")
ap.add_argument("--discover-s", type=float, default=4.0)
ap.add_argument("--read-s", type=float, default=6.0)
ap.add_argument("--out", required=True)
a = ap.parse_args()

dom = Domain(0, ChannelConfigHasInterface.replace("$__IF_NAME__$", a.iface))
dp = DomainParticipant(0)
pubs = BuiltinDataReader(dp, BuiltinTopicDcpsPublication)
seen = {}
t_end = time.monotonic() + a.discover_s
while time.monotonic() < t_end:
    for s in pubs.take(N=256):
        try:
            seen.setdefault(s.topic_name, set()).add(s.type_name)
        except Exception:
            pass
    time.sleep(0.05)
bms_topics = sorted(t for t, ty in seen.items() if any("Bms" in x for x in ty) or "bms" in t.lower())
print("topics on domain 0 with Bms in name or type:")
for t in bms_topics:
    print("  ", t, sorted(seen[t]))
result = {"discovered": {t: sorted(seen[t]) for t in bms_topics}, "samples": {}}
qos = Qos(Policy.Reliability.BestEffort, Policy.History.KeepLast(64))
readers = {}
for t in bms_topics:
    if any(x.endswith("BmsState_") for x in seen[t]):
        readers[t] = DataReader(dp, Topic(dp, t, BmsState_), qos=qos)
t_end = time.monotonic() + a.read_s
last, counts = {}, {}
while time.monotonic() < t_end:
    for t, r in readers.items():
        for s in r.take(N=64):
            counts[t] = counts.get(t, 0) + 1
            last[t] = s
    time.sleep(0.05)
for t, s in last.items():
    cur = int(s.current)
    rec = {"n": counts[t], "rate_hz_seen": round(counts[t] / a.read_s, 2),
           "soc_pct": int(s.soc), "soh_pct": int(s.soh), "current_raw": cur,
           "bmsvoltage_raw": [int(v) for v in s.bmsvoltage],
           "temperature_raw": [int(v) for v in s.temperature],
           "cycle": int(s.cycle), "bmsstate_raw": [int(v) for v in s.bmsstate],
           "cell_vol_nonzero_mV": [int(v) for v in s.cell_vol if v]}
    result["samples"][t] = rec
    print(f"{t}: n={rec['n']} soc={rec['soc_pct']} % soh={rec['soh_pct']} % current_raw={cur} "
          f"bmsvoltage_raw={rec['bmsvoltage_raw']} cycle={rec['cycle']} "
          f"temps={rec['temperature_raw']} bmsstate={rec['bmsstate_raw']}")
    cells = rec["cell_vol_nonzero_mV"]
    if cells:
        print(f"   cells: {len(cells)} nonzero, sum {sum(cells)} mV, min {min(cells)} max {max(cells)} mV")
result["captured_at"] = time.strftime("%Y-%m-%d %H:%M:%S %Z")
with open(a.out, "w") as fh:
    json.dump(result, fh, indent=2)
sys.stdout.flush()
os._exit(0)
