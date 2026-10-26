#!/usr/bin/env python3
"""RT5: watch DDS domain 0 for the whole session, READERS ONLY, and flag any writer from
this host. Only builtin readers (DCPSParticipant, DCPSPublication) are created; there is
no writer of any kind in this file. Hosts are written as labels (this-host / other-host),
never as addresses."""
import os, re, subprocess, sys, time
from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsParticipant, BuiltinTopicDcpsPublication
from cyclonedds.domain import Domain, DomainParticipant
from unitree_sdk2py.core.channel_config import ChannelConfigHasInterface

iface, out_path, period = sys.argv[1], sys.argv[2], float(sys.argv[3]) if len(sys.argv) > 3 else 10.0
own = set(re.findall(r"inet (\d+\.\d+\.\d+\.\d+)", subprocess.run(["ip", "-4", "-o", "addr"], capture_output=True, text=True).stdout))
dom = Domain(0, ChannelConfigHasInterface.replace("$__IF_NAME__$", iface))
dp = DomainParticipant(0)
pr, pb = BuiltinDataReader(dp, BuiltinTopicDcpsParticipant), BuiltinDataReader(dp, BuiltinTopicDcpsPublication)
parts, pubs = {}, {}
def label(qos_or_addr):
    ips = [ip for ip in re.findall(r"(\d+\.\d+\.\d+\.\d+)", str(qos_or_addr)) if not ip.startswith(("239.", "127."))]
    if not ips:
        return "this-host" if "localprocess" in str(qos_or_addr) else "unknown"
    return "this-host" if ips[0] in own else "other-host"
fh = open(out_path, "a", buffering=1)
fh.write(f"# d0_watch start {time.strftime('%F %T')} domain 0 iface {iface} readers only\n")
fh.write("time,participants_this_host,participants_other_host,writers_this_host,writers_other_host,alert\n")
me = None
nxt = time.monotonic() + period
while True:
    for s in pr.take(N=256):
        try:
            k = str(s.key)
            props = {getattr(q, "key", None): getattr(q, "value", None) for q in (s.qos or []) if type(q).__name__ == "Property"}
            parts[k] = label(props.get("__NetworkAddresses", "") or props)
            if props.get("__Pid") == str(os.getpid()) or (props.get("__ProcessName") == "python" and props.get("__Pid") == str(os.getpid())):
                me = k
        except Exception:
            pass
    for s in pb.take(N=1024):
        try:
            pk = str(s.participant_key)
            pubs[str(s.key)] = (pk, s.topic_name)
        except Exception:
            pass
    if time.monotonic() >= nxt:
        nxt += period
        own_parts = [k for k, v in parts.items() if v == "this-host" and k != me]
        w_own = sorted({t for k, (pk, t) in pubs.items() if parts.get(pk) == "this-host"})
        w_oth = sum(1 for k, (pk, t) in pubs.items() if parts.get(pk) != "this-host")
        alert = ("ALERT this-host writers: " + " ".join(w_own)) if w_own else ""
        fh.write(f"{time.strftime('%T')},{len(own_parts)},{sum(1 for v in parts.values() if v == 'other-host')},{len(w_own)},{w_oth},{alert}\n")
    time.sleep(0.05)
