#!/usr/bin/env python3
"""RT5: per-second log of what the launcher commands on DOMAIN 1 / lo (readers only) and of
the 8012 websocket traffic (from `ss -tin`, read-only). No addresses: the peer is written as
local or remote."""
import os, re, subprocess, sys, time
import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from inspire_sdkpy import inspire_dds
out_path = sys.argv[1]
ChannelFactoryInitialize(1, "lo")
buf = {"lowcmd": [], "l": [], "r": []}
def cb(name, fn):
    return lambda m: buf[name].append(fn(m))
subs = []
for topic, cls, name, fn in (("rt/lowcmd", LowCmd_, "lowcmd", lambda m: np.array([m.motor_cmd[i].q for i in range(15, 29)])),
                             ("rt/inspire_hand/ctrl/l", inspire_dds.inspire_hand_ctrl, "l", lambda m: tuple(m.angle_set)),
                             ("rt/inspire_hand/ctrl/r", inspire_dds.inspire_hand_ctrl, "r", lambda m: tuple(m.angle_set))):
    s = ChannelSubscriber(topic, cls); s.Init(cb(name, fn), 10); subs.append(s)
def ss8012():
    out = subprocess.run(["ss", "-tinH", "state", "established", "( sport = :8012 )"], capture_output=True, text=True).stdout
    tot = {"remote": [0, 0, 0], "local": [0, 0, 0]}
    rows = out.split("\n")
    for i, row in enumerate(rows):
        m = re.match(r"\s*\d+\s+\d+\s+(\S+):8012\s+(\S+)", row)
        if not m:
            continue
        side = "local" if m.group(2).startswith(("127.", "[::1]")) else "remote"
        info = rows[i + 1] if i + 1 < len(rows) else ""
        rx = re.search(r"bytes_received:(\d+)", info); tx = re.search(r"bytes_acked:(\d+)", info); rtt = re.search(r" rtt:([\d.]+)/", info)
        tot[side][0] += int(rx.group(1)) if rx else 0
        tot[side][1] += int(tx.group(1)) if tx else 0
        tot[side][2] = max(tot[side][2], float(rtt.group(1)) if rtt else 0.0)
    n = sum(1 for r in rows if ":8012" in r)
    return tot, n
fh = open(out_path, "a", buffering=1)
fh.write("time,lowcmd_n,arm_q_maxabs,arm_q_step_max,ctrl_l_n,ctrl_l_distinct,ws_conns,ws_rx_Bps_remote,ws_tx_Bps_remote,ws_rtt_ms_remote\n")
last_q, last_tot, last_t = None, None, time.monotonic()
while True:
    time.sleep(1.0)
    now = time.monotonic(); dt = now - last_t; last_t = now
    lc, l = buf["lowcmd"], buf["l"]; buf["lowcmd"], buf["l"], buf["r"] = [], [], []
    qmax = float(np.max(np.abs(lc[-1]))) if lc else float("nan")
    step = float(np.max(np.abs(lc[-1] - last_q))) if lc and last_q is not None else 0.0
    if lc:
        last_q = lc[-1]
    tot, n = ss8012()
    if last_tot is None:
        rx = tx = 0.0
    else:
        rx = (tot["remote"][0] - last_tot["remote"][0]) / dt; tx = (tot["remote"][1] - last_tot["remote"][1]) / dt
    last_tot = tot
    fh.write(f"{time.strftime('%T')},{len(lc)},{qmax:.4f},{step:.4f},{len(l)},{len(set(l))},{n},{max(rx, 0):.0f},{max(tx, 0):.0f},{tot['remote'][2]:.1f}\n")
