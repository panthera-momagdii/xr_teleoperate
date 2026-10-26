"""RT4 debug: launcher via ipc on domain 1/lo; print per-thread SigBlk for SIGHUP; then kill -HUP and time it."""
import os, signal, subprocess, sys, time, uuid
sys.path.insert(0, "/home/unitree/xrt/tools")
from fake_xr_client import FakeXR
PY = sys.executable
fx = [subprocess.Popen([PY, "/home/unitree/xrt/tools/fake_lowstate.py", "--domain", "1", "--iface", "lo", "--rate", "500"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)]
time.sleep(3)
log = open("/home/unitree/panthera/remote-teleop-20261012/rt4/out/sigmask_probe_launcher.log", "w")
p = subprocess.Popen([PY, "teleop_hand_and_arm.py", "--sim", "--network-interface", "lo", "--ipc"], cwd="/home/unitree/xrt/teleop",
                     env=dict(os.environ, PYTHONUNBUFFERED="1"), stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
xr = FakeXR().start()
for _ in range(600):
    if "Press [r]" in open(log.name).read(): break
    time.sleep(0.2)
time.sleep(2)
import zmq; c = zmq.Context(); s = c.socket(zmq.REQ); s.connect("ipc://@xr_teleoperate_data.ipc"); s.send_json({"reqid": str(uuid.uuid4()), "cmd": "CMD_START"}); s.recv_json(); s.close(); c.term()
time.sleep(3)
blocked = []
for tid in os.listdir(f"/proc/{p.pid}/task"):
    st = open(f"/proc/{p.pid}/task/{tid}/status").read()
    blk = int([l for l in st.splitlines() if l.startswith("SigBlk")][0].split()[1], 16)
    name = open(f"/proc/{p.pid}/task/{tid}/comm").read().strip()
    blocked.append((tid, name, bool(blk & 1), hex(blk)))
print("threads:", len(blocked), " blocking SIGHUP:", sum(b[2] for b in blocked))
for b in blocked:
    if b[0] == str(p.pid) or not b[2]: print("  ", b)
t0 = time.monotonic(); os.kill(p.pid, signal.SIGHUP)
try: rc = p.wait(timeout=90)
except subprocess.TimeoutExpired: rc = None
print("SIGHUP -> exit", rc, "after", round(time.monotonic() - t0, 1), "s")
xr.stop()
for f in [p] + fx:
    try: os.killpg(f.pid, signal.SIGKILL)
    except OSError: pass
