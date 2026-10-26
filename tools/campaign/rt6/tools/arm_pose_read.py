#!/usr/bin/env python3
"""RT6: read the current arm pose from rt/lowstate on domain 0. READERS ONLY (no writer in this file)."""
import json, os, sys, time
import numpy as np
from cyclonedds.domain import Domain, DomainParticipant
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.qos import Qos, Policy
from unitree_sdk2py.core.channel_config import ChannelConfigHasInterface
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
dom = Domain(0, ChannelConfigHasInterface.replace("$__IF_NAME__$", "eth0")); dp = DomainParticipant(0)
r = DataReader(dp, Topic(dp, "rt/lowstate", LowState_), qos=Qos(Policy.Reliability.BestEffort, Policy.History.KeepLast(64)))
q, dq, tau, t0, mm = [], [], [], time.monotonic(), None
while time.monotonic() - t0 < 2.0:
    for s in r.take(N=64):
        q.append([s.motor_state[i].q for i in range(15, 29)]); dq.append([s.motor_state[i].dq for i in range(15, 29)])
        tau.append([s.motor_state[i].tau_est for i in range(15, 29)]); mm = int(s.mode_machine)
    time.sleep(0.01)
q, dq, tau = np.array(q), np.array(dq), np.array(tau)
out = {"n": len(q), "mode_machine": mm, "arm_q_mean": q.mean(0).round(4).tolist(), "arm_q_ptp": q.ptp(0).round(4).tolist(),
       "arm_dq_absmax": np.abs(dq).max(0).round(3).tolist(), "arm_tau_absmean": np.abs(tau).mean(0).round(2).tolist()}
json.dump(out, open(sys.argv[1], "w"), indent=1); print(json.dumps(out)); os._exit(0)
