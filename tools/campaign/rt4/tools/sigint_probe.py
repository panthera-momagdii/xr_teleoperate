"""RT4 debug: SIGINT the launcher N times (domain 1, lo, --ee inspire_ftp, fake headset); dump stacks on a hang."""
import os, signal, subprocess, sys, time, uuid
sys.path.insert(0, "/home/unitree/xrt/tools")
from fake_xr_client import FakeXR
PY = sys.executable; D = "/home/unitree/panthera/remote-teleop-20261012/rt4/out"
fx = [subprocess.Popen([PY, "/home/unitree/xrt/tools/fake_lowstate.py", "--domain", "1", "--iface", "lo", "--rate", "500"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True),
      subprocess.Popen([PY, "/home/unitree/xrt/tools/fake_inspire_state.py", "--domain", "1", "--iface", "lo"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)]
time.sleep(3)
def ipc(cmd):
    import zmq; c = zmq.Context(); s = c.socket(zmq.REQ); s.setsockopt(zmq.RCVTIMEO, 4000); s.setsockopt(zmq.LINGER, 0)
    s.connect("ipc://@xr_teleoperate_data.ipc"); s.send_json({"reqid": str(uuid.uuid4()), "cmd": cmd})
    try: s.recv_json()
    except Exception: pass
    s.close(); c.term()
N = int(sys.argv[1]) if len(sys.argv) > 1 else 4
for i in range(N):
    stk = f"{D}/sigint_stacks_{i}.txt"; lg = f"{D}/sigint_launcher_{i}.log"
    shim = (f"import faulthandler, signal, runpy, sys; faulthandler.register(signal.SIGUSR1, file=open('{stk}', 'w'), all_threads=True, chain=False); "
            "sys.argv = sys.argv[1:]; runpy.run_path(sys.argv[0], run_name='__main__')")
    p = subprocess.Popen([PY, "-c", shim, "teleop_hand_and_arm.py", "--sim", "--network-interface", "lo", "--ipc", "--ee", "inspire_ftp"],
                         cwd="/home/unitree/xrt/teleop", env=dict(os.environ, PYTHONUNBUFFERED="1", XR_ARM_VEL_LIMIT="3"),
                         stdin=subprocess.DEVNULL, stdout=open(lg, "w"), stderr=subprocess.STDOUT, start_new_session=True)
    xr = FakeXR().start()
    for _ in range(600):
        if "Press [r]" in open(lg).read(): break
        time.sleep(0.2)
    time.sleep(2); ipc("CMD_START"); time.sleep(3 + i * 0.37)
    t0 = time.monotonic(); os.kill(p.pid, signal.SIGINT)
    try: rc = p.wait(timeout=10)
    except subprocess.TimeoutExpired:
        rc = None; os.kill(p.pid, signal.SIGUSR1); time.sleep(1)
    print(f"run {i}: exit {rc} after {time.monotonic() - t0:.1f} s", flush=True)
    xr.stop()
    try: os.killpg(p.pid, signal.SIGKILL)
    except OSError: pass
    time.sleep(2)
for f in fx: os.killpg(f.pid, signal.SIGTERM)
