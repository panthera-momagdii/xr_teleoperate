#!/usr/bin/env python3
"""RT7: read-only health of every body motor from rt/lowstate on domain 0 for 2 s. READERS ONLY.
Per arm motor: mode, motorstate (error bits), temperature[0] housing / [1] winding (max over
the window), q, |dq| max, tau_est mean, vol. Legs/waist: any nonzero motorstate, max temps.
Usage: arm_health.py OUT_JSON"""
import json, os, sys, time
from cyclonedds.domain import Domain, DomainParticipant
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.qos import Qos, Policy
from unitree_sdk2py.core.channel_config import ChannelConfigHasInterface
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
dom = Domain(0, ChannelConfigHasInterface.replace("$__IF_NAME__$", "eth0")); dp = DomainParticipant(0)
r = DataReader(dp, Topic(dp, "rt/lowstate", LowState_), qos=Qos(Policy.Reliability.BestEffort, Policy.History.KeepLast(64)))
acc = {i: {"mode": set(), "ms": set(), "t0": -99, "t1": -99, "q": [], "dq": 0.0, "tau": [], "vol": []} for i in range(35)}
n, mm, t0 = 0, set(), time.monotonic()
while time.monotonic() - t0 < 2.0:
    for s in r.take(N=64):
        if not hasattr(s, "motor_state"):
            continue
        n += 1; mm.add(int(s.mode_machine))
        for i in range(35):
            m = s.motor_state[i]; a = acc[i]
            a["mode"].add(int(m.mode)); a["ms"].add(int(m.motorstate))
            a["t0"] = max(a["t0"], int(m.temperature[0])); a["t1"] = max(a["t1"], int(m.temperature[1]))
            a["q"].append(m.q); a["dq"] = max(a["dq"], abs(m.dq)); a["tau"].append(m.tau_est); a["vol"].append(m.vol)
    time.sleep(0.01)
out = {"samples": n, "mode_machine": sorted(mm), "motors": {}}
for i in range(35):
    a = acc[i]
    out["motors"][i] = {"mode": sorted(a["mode"]), "motorstate": sorted(a["ms"]), "housing_C": a["t0"], "winding_C": a["t1"],
                        "q": round(sum(a["q"]) / max(len(a["q"]), 1), 4), "dq_absmax": round(a["dq"], 3),
                        "tau_mean": round(sum(a["tau"]) / max(len(a["tau"]), 1), 3), "vol": round(sum(a["vol"]) / max(len(a["vol"]), 1), 2)}
json.dump(out, open(sys.argv[1], "w"), indent=1)
N = ["L sh pitch", "L sh roll", "L sh yaw", "L elbow", "L wr roll", "L wr pitch", "L wr yaw",
     "R sh pitch", "R sh roll", "R sh yaw", "R elbow", "R wr roll", "R wr pitch", "R wr yaw"]
print(f"samples {n}, mode_machine {sorted(mm)}")
for k, i in enumerate(range(15, 29)):
    m = out["motors"][i]
    print(f"{i} {N[k]:10s} mode {m['mode']} motorstate {m['motorstate']} housing {m['housing_C']} winding {m['winding_C']} C "
          f"q {m['q']:+.3f} |dq| {m['dq_absmax']:.3f} tau {m['tau_mean']:+.2f} vol {m['vol']}")
bad = [i for i in range(15) if any(v != 0 for v in out["motors"][i]["motorstate"])]
print(f"legs/waist: nonzero motorstate on {bad if bad else 'none'}; max housing {max(out['motors'][i]['housing_C'] for i in range(15))} "
      f"winding {max(out['motors'][i]['winding_C'] for i in range(15))} C")
sys.stdout.flush(); os._exit(0)
