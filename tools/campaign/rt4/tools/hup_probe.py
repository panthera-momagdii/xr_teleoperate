"""RT4 debug: launcher on a pty, [r], hang up, dump all thread stacks 8 s later. Domain 1, lo."""
import os, pty, signal, subprocess, sys, threading, time
sys.path.insert(0, "/home/unitree/xrt/tools"); sys.path.insert(0, "/home/unitree/xrt")
from fake_xr_client import FakeXR
PY = sys.executable; T = "/home/unitree/xrt/teleop"; D = "/home/unitree/panthera/remote-teleop-20261012/rt4/out"
fx = [subprocess.Popen([PY, "/home/unitree/xrt/tools/fake_lowstate.py", "--domain", "1", "--iface", "lo", "--rate", "500"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True),
      subprocess.Popen([PY, "/home/unitree/xrt/tools/fake_inspire_state.py", "--domain", "1", "--iface", "lo"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)]
time.sleep(3)
shim = ("import fcntl, termios, faulthandler, signal, runpy, sys; fcntl.ioctl(0, termios.TIOCSCTTY, 0); "
        f"faulthandler.register(signal.SIGUSR1, file=open('{D}/hup_stacks.txt', 'w'), all_threads=True); "
        "sys.argv = sys.argv[1:]; runpy.run_path(sys.argv[0], run_name='__main__')")
m, s = pty.openpty()
env = dict(os.environ, XR_ARM_VEL_LIMIT="3", PYTHONUNBUFFERED="1")
p = subprocess.Popen([PY, "-c", shim, "teleop_hand_and_arm.py", "--sim", "--network-interface", "lo", "--ee", "inspire_ftp"],
                     cwd=T, env=env, stdin=s, stdout=s, stderr=s, start_new_session=True)
os.close(s)
out = []
def rd():
    while True:
        try: c = os.read(m, 4096)
        except OSError: break
        if not c: break
        out.append(c)
threading.Thread(target=rd, daemon=True).start()
xr = FakeXR().start()
t0 = time.monotonic()
while b"Press [r]" not in b"".join(out) and time.monotonic() - t0 < 120: time.sleep(0.2)
time.sleep(3); os.write(m, b"r"); time.sleep(3)
print("hangup at", time.strftime("%T")); th = time.monotonic(); os.close(m)
time.sleep(3)
st = open(f"/proc/{p.pid}/status").read()
print("pid", p.pid, [l for l in st.splitlines() if l.split(":")[0] in ("ShdPnd","SigPnd","SigBlk","SigIgn","SigCgt")])
pend = []
for tid in os.listdir(f"/proc/{p.pid}/task"):
    t = open(f"/proc/{p.pid}/task/{tid}/status").read()
    d = {l.split(":")[0]: l.split()[1] for l in t.splitlines() if l.split(":")[0] in ("SigPnd","SigBlk")}
    if int(d["SigPnd"],16) or int(d["SigBlk"],16) & 1: pend.append((tid, open(f"/proc/{p.pid}/task/{tid}/comm").read().strip(), d))
print("threads with pending or SIGHUP-blocked:", pend[:6])
time.sleep(5)
print("alive after 8 s:", p.poll() is None)
if p.poll() is None: os.kill(p.pid, signal.SIGUSR1)
try: rc = p.wait(timeout=60)
except subprocess.TimeoutExpired: rc = None
print("exit", rc, "after", round(time.monotonic() - th, 1), "s")
xr.stop()
try: os.killpg(p.pid, signal.SIGKILL)
except OSError: pass
for f in fx: os.killpg(f.pid, signal.SIGTERM)
