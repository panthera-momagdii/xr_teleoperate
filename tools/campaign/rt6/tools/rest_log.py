#!/usr/bin/env python3
"""RT6f: rest log, one row per second. DOMAIN 0, READERS ONLY (no writer in this file).

    rest_log.py OUT_CSV [SECONDS] [IFACE]        default 3600 s, eth0

Per second: both shoulder pitch temperatures (max of the motor's two values over the
second), the hottest arm motor (15..28) and which, mean tau_est on both shoulder pitch
joints, the lowstate sample count, battery % from rt/lf/bmsstate (last value seen).
If rt/lowstate goes silent for 5 s (robot powered off), the row says so and the log
ends. A summary line goes to stdout at the end.
"""
import os, sys, time
import numpy as np
from cyclonedds.domain import Domain, DomainParticipant
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.qos import Qos, Policy
from unitree_sdk2py.core.channel_config import ChannelConfigHasInterface
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, BmsState_

out = sys.argv[1]
seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 3600.0
iface = sys.argv[3] if len(sys.argv) > 3 else "eth0"
dom = Domain(0, ChannelConfigHasInterface.replace("$__IF_NAME__$", iface))
dp = DomainParticipant(0)
qos = Qos(Policy.Reliability.BestEffort, Policy.History.KeepLast(256))
rs = DataReader(dp, Topic(dp, "rt/lowstate", LowState_), qos=qos)
rb = DataReader(dp, Topic(dp, "rt/lf/bmsstate", BmsState_), qos=qos)
ARM = range(15, 29)

fh = open(out, "w", buffering=1)
fh.write("time,lowstate_n,spL_temp_C,spR_temp_C,arm_max_C,arm_max_motor,spL_tau_est_Nm,spR_tau_est_Nm,battery_pct,note\n")
t0 = time.monotonic(); nxt = t0 + 1.0; last_rx = t0; soc = ""
first = last = None
while time.monotonic() - t0 < seconds:
    n = 0; tl = tr = am = -99; amj = -1; tauL = []; tauR = []
    while time.monotonic() < nxt:
        for s in rs.take(N=256):
            if not hasattr(s, "motor_state"):
                continue
            n += 1
            m = s.motor_state
            tl = max(tl, max(m[15].temperature)); tr = max(tr, max(m[22].temperature))
            for i in ARM:
                v = max(m[i].temperature)
                if v > am:
                    am, amj = v, i
            tauL.append(m[15].tau_est); tauR.append(m[22].tau_est)
        for b in rb.take(N=64):
            if hasattr(b, "soc"):
                soc = int(b.soc)
        time.sleep(0.02)
    nxt += 1.0
    now = time.monotonic()
    if n:
        last_rx = now
        row = (tl, tr, am, amj, float(np.mean(tauL)), float(np.mean(tauR)))
        first = first or (time.strftime("%T"), row)
        last = (time.strftime("%T"), row)
        fh.write(f"{time.strftime('%T')},{n},{tl},{tr},{am},{amj},{row[4]:.3f},{row[5]:.3f},{soc},\n")
    else:
        silent = now - last_rx
        note = "no lowstate" if silent < 5.0 else f"STREAM STOPPED: no rt/lowstate for {silent:.0f} s (last sample about {time.strftime('%T', time.localtime(time.time() - silent))})"
        fh.write(f"{time.strftime('%T')},0,,,,,,,{soc},{note}\n")
        if silent >= 5.0:
            print(note, flush=True)
            break
fh.close()
print(f"rest log end {time.strftime('%T')}: first {first}, last {last}", flush=True)
os._exit(0)
