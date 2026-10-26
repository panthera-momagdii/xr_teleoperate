#!/usr/bin/env python3
"""RT5b step 2: launcher IK time on fakes (domain 1, lo, --sim) for one config.
Config comes from env (thread limits, video knobs) plus optional --taskset CPUS for the
launcher. Records launcher stats (IK p50/p99/max over FOLLOW windows) and CPU per process
and per launcher thread. Assumes the image server is already up."""
import json, os, re, signal, subprocess, sys, time, uuid
sys.path[:0] = ["/home/unitree/xrt/tools"]
from fake_xr_client import FakeXR
label, secs = sys.argv[1], float(sys.argv[2]); taskset = sys.argv[3] if len(sys.argv) > 3 else ""
PY = "/home/unitree/miniconda3/envs/xrt/bin/python"; O = "/home/unitree/panthera/remote-teleop-20261012/rt5b/out"
TCK = os.sysconf("SC_CLK_TCK")
def ipc(cmd):
    import zmq; c = zmq.Context(); s = c.socket(zmq.REQ); s.setsockopt(zmq.RCVTIMEO, 4000); s.setsockopt(zmq.LINGER, 0)
    s.connect("ipc://@xr_teleoperate_data.ipc"); s.send_json({"reqid": str(uuid.uuid4()), "cmd": cmd})
    try: s.recv_json()
    except Exception: pass
    s.close(); c.term()
def jiffies(path):
    try:
        f = open(path).read().rsplit(")", 1)[1].split(); return int(f[11]) + int(f[12])
    except Exception: return None
def snapshot(pids):
    out = {}
    for pid in pids:
        out[("p", pid)] = jiffies(f"/proc/{pid}/stat")
        try:
            for tid in os.listdir(f"/proc/{pid}/task"):
                out[("t", pid, tid)] = jiffies(f"/proc/{pid}/task/{tid}/stat")
        except Exception: pass
    return out
def name(pid):
    try: a = open(f"/proc/{pid}/cmdline", "rb").read().split(b"\0")
    except Exception: return "?"
    s = " ".join(x.decode(errors="replace") for x in a[:4])
    for k, v in (("fake_lowstate", "fake_lowstate"), ("fake_inspire", "fake_inspire"), ("image_server", "image_server"),
                 ("teleop_hand_and_arm", "launcher")):
        if k in s: return v
    return s[:40]
env = dict(os.environ, PYTHONUNBUFFERED="1", XR_ARM_VEL_LIMIT="3", XR_STATS_S="5")
fx = [subprocess.Popen([PY, "/home/unitree/xrt/tools/fake_lowstate.py", "--domain", "1", "--iface", "lo", "--rate", "1000"] + (["--follow-cmd"] if os.environ.get("RT5B_FOLLOW") == "1" else []), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True, env=env),
      subprocess.Popen([PY, "/home/unitree/xrt/tools/fake_inspire_state.py", "--domain", "1", "--iface", "lo"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True, env=env)]
time.sleep(3)
cmd = ([ "taskset", "-c", taskset] if taskset else []) + [PY, "teleop_hand_and_arm.py", "--sim", "--network-interface", "lo", "--ipc", "--ee", "inspire_ftp", "--img-server-ip", ".".join(["127", "0", "0", "1"])]
lg = open(f"{O}/ik_{label}_launcher.log", "w")
L = subprocess.Popen(cmd, cwd="/home/unitree/xrt/teleop", env=env, stdin=subprocess.DEVNULL, stdout=lg, stderr=subprocess.STDOUT, start_new_session=True)
xr = FakeXR(rate=60.0).start()
for _ in range(600):
    if "Press [r]" in open(lg.name, errors="replace").read(): break
    time.sleep(0.2)
time.sleep(3); ipc("CMD_START"); time.sleep(8)
ps = subprocess.run(["ps", "-eo", "pid=,sid=,args="], capture_output=True, text=True).stdout
pids = [int(r.split()[0]) for r in ps.splitlines() if r.split()[1] == str(L.pid)]
pids += [int(r.split()[0]) for r in ps.splitlines() if "teleimager.image_server" in r and "g1visit" in r]
pids += [f.pid for f in fx]
a = snapshot(pids); ta = time.monotonic(); time.sleep(secs); b = snapshot(pids); dt = time.monotonic() - ta
procs = {}
for k, v in b.items():
    if k[0] == "p" and a.get(k) is not None and v is not None:
        procs.setdefault(name(k[1]), 0.0); procs[name(k[1])] += 100 * (v - a[k]) / TCK / dt
threads = {}
for k, v in b.items():
    if k[0] == "t" and k[1] == L.pid and a.get(k) is not None and v is not None:
        try: comm = open(f"/proc/{k[1]}/task/{k[2]}/comm").read().strip()
        except Exception: comm = "?"
        key = "MainThread" if k[2] == str(L.pid) else comm
        threads[key] = threads.get(key, 0.0) + 100 * (v - a[k]) / TCK / dt
nthreads = len([k for k in b if k[0] == "t" and k[1] == L.pid])
stats = f"/home/unitree/xrt/logs/launcher_stats_{L.pid}.csv"
rows = [l.strip().split(",") for l in open(stats) if l[:1].isdigit()] if os.path.exists(stats) else []
fol = [r for r in rows if r[-1] == "FOLLOW"]
import numpy as np
p50 = float(np.median([float(r[3]) for r in fol])) if fol else None
p99 = max(float(r[4]) for r in fol) if fol else None
mx = max(float(r[5]) for r in fol) if fol else None
hz = float(np.mean([float(r[1]) for r in fol])) if fol else None
res = {"label": label, "taskset": taskset, "threads_env": {k: env.get(k) for k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "XR_TORCH_THREADS")},
       "video": {k: env.get(k) for k in ("XR_DISPLAY_FPS", "XR_JPEG_QUALITY", "XR_DISPLAY_SCALE")},
       "follow_windows": len(fol), "loop_hz": hz, "ik_p50_ms_median": p50, "ik_p99_ms_worst": p99, "ik_max_ms": mx,
       "cpu_pct_core_by_process": {k: round(v, 1) for k, v in sorted(procs.items(), key=lambda x: -x[1])},
       "launcher_threads": nthreads, "launcher_thread_cpu_top": {k: round(v, 1) for k, v in sorted(threads.items(), key=lambda x: -x[1])[:6]}}
print(json.dumps(res), flush=True)
ipc("CMD_STOP")
try: L.wait(timeout=30)
except subprocess.TimeoutExpired: os.killpg(L.pid, signal.SIGKILL)
xr.stop()
for f in fx:
    try: os.killpg(f.pid, signal.SIGTERM)
    except OSError: pass
for f in fx:
    try: f.wait(timeout=5)
    except subprocess.TimeoutExpired: os.killpg(f.pid, signal.SIGKILL)
