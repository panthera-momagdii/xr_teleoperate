#!/usr/bin/env python3
"""RT6b: arm motor temperatures from rt/lowstate on domain 0 for 2 s. READERS ONLY."""
import os, sys, time
from cyclonedds.domain import Domain, DomainParticipant
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.qos import Qos, Policy
from unitree_sdk2py.core.channel_config import ChannelConfigHasInterface
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
dom = Domain(0, ChannelConfigHasInterface.replace("$__IF_NAME__$", "eth0")); dp = DomainParticipant(0)
r = DataReader(dp, Topic(dp, "rt/lowstate", LowState_), qos=Qos(Policy.Reliability.BestEffort, Policy.History.KeepLast(64)))
mx = {}; m0 = {}; m1 = {}; t0 = time.monotonic()
while time.monotonic() - t0 < 2.0:
    for s in r.take(N=64):
        for i in range(35):
            mx[i] = max(mx.get(i, -99), max(s.motor_state[i].temperature))
            m0[i] = max(m0.get(i, -99), s.motor_state[i].temperature[0])
            m1[i] = max(m1.get(i, -99), s.motor_state[i].temperature[1])
    time.sleep(0.02)
arm = {i: mx[i] for i in range(15, 29)}
a0 = {i: m0[i] for i in range(15, 29)}; a1 = {i: m1[i] for i in range(15, 29)}
# RT7: temperature[0] = housing, [1] = winding (SDK terminations.hpp)
print(f"{time.strftime('%T')} shoulder_pitch housing[0] L {m0[15]} R {m0[22]} C, winding[1] L {m1[15]} R {m1[22]} C | "
      f"arm max housing {max(a0.values())} C (motor {max(a0, key=a0.get)}), winding {max(a1.values())} C (motor {max(a1, key=a1.get)}) | "
      f"legs/waist max {max(mx[i] for i in range(0, 15))} C", flush=True)
os._exit(0)
