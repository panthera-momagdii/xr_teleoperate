#!/usr/bin/env python3
"""G2: tools reap their children and release their ports. DOMAIN 1, LOOPBACK ONLY.

Every case starts a tool in its own PROCESS GROUP, kills it with a specific signal, and
then asks two questions:

  1. is anything of ours still alive?     (pgrep -f, scoped to the tool's own path)
  2. is the port bindable within 1 s?     (a real bind, not a /proc reading)

The port question is the one that matters. An orphaned logging_mp listener inherits the
parent's LISTENING SOCKET, so "the process is gone" and "the port is free" are different
facts, and only the second one lets the next run start.

Run:  python tools/overnight/test_g2_hygiene.py
"""

import os
import signal
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent.parent
PY = sys.executable

results = []


def record(name, ok, detail=""):
    results.append((name, ok, detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  -- {detail}" if detail else ""))


def port_free(port, host="0.0.0.0"):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind((host, port))
        return True
    except OSError:
        return False
    finally:
        s.close()


def wait_port_free(port, timeout=1.0):
    """The gate's own criterion: bindable within one second."""
    deadline = time.monotonic() + timeout
    while True:
        if port_free(port):
            return True, time.monotonic() - (deadline - timeout)
        if time.monotonic() >= deadline:
            return False, timeout
        time.sleep(0.02)


def survivors(pattern):
    """PIDs whose cmdline contains `pattern`, excluding this test and its shell."""
    out = subprocess.run(["pgrep", "-af", pattern], capture_output=True, text=True)
    pids = []
    for line in out.stdout.splitlines():
        pid, _, cmd = line.partition(" ")
        if not pid.isdigit() or int(pid) in (os.getpid(), os.getppid()):
            continue
        if "test_g2_hygiene" in cmd or "pgrep" in cmd:
            continue
        pids.append((int(pid), cmd))
    return pids


def cleanup(pattern):
    for pid, _ in survivors(pattern):
        for sig in (signal.SIGKILL,):
            try:
                os.kill(pid, sig)
            except OSError:
                pass
    time.sleep(0.3)


def run_and_signal(argv, sig, settle=4.0, kill_after=1.5, cwd=None):
    """Start a tool, let it settle, signal its process GROUP, return its output."""
    with tempfile.NamedTemporaryFile("w+", suffix=".log", delete=False) as fh:
        log_path = fh.name
    with open(log_path, "wb") as sink:
        proc = subprocess.Popen(argv, cwd=cwd or str(REPO), stdout=sink,
                                stderr=subprocess.STDOUT, start_new_session=True)
        time.sleep(settle)
        try:
            os.killpg(os.getpgid(proc.pid), sig)
        except OSError:
            pass
        try:
            proc.wait(timeout=kill_after + 3)
        except subprocess.TimeoutExpired:
            pass
    out = Path(log_path).read_text(errors="replace")
    os.unlink(log_path)
    return proc.returncode, out


# =========================================================================== 1
# tools/_procs.py itself: the ladder must survive a child that ignores SIGTERM.
print("\n=== (1) tools/_procs.py reaps a SIGTERM-ignoring child ===")
probe = r"""
import multiprocessing as mp, os, signal, sys, time
sys.path.insert(0, %r)
from tools._procs import reap_child_processes

def stubborn():
    signal.signal(signal.SIGTERM, signal.SIG_IGN)   # exactly what the leaked fixtures did
    while True:
        time.sleep(0.1)

if __name__ == "__main__":
    p = mp.Process(target=stubborn, daemon=False)
    p.start()
    print("CHILD", p.pid, flush=True)
    t0 = time.monotonic()
    ok = reap_child_processes()
    print("REAPED", ok, "alive", p.is_alive(), "secs", round(time.monotonic()-t0, 2), flush=True)
""" % str(REPO)
proc = subprocess.run([PY, "-c", probe], capture_output=True, text=True, timeout=120)
line = [l for l in proc.stdout.splitlines() if l.startswith("REAPED")]
child = [l for l in proc.stdout.splitlines() if l.startswith("CHILD")]
ok = bool(line) and line[0].startswith("REAPED True") and "alive False" in line[0]
record("reap_child_processes kills a child that IGNORES SIGTERM", ok,
       line[0] if line else proc.stderr[-200:])
if child:
    pid = int(child[0].split()[1])
    time.sleep(0.3)
    gone = not Path(f"/proc/{pid}").exists()
    record("that child is really gone from /proc", gone, f"pid {pid}")

# =========================================================================== 2
print("\n=== (2) quest_link_check.py: SIGINT and SIGTERM leave nothing behind ===")
QLC = str(REPO / "tools" / "quest_link_check.py")
for signame, sig in (("SIGINT", signal.SIGINT), ("SIGTERM", signal.SIGTERM)):
    cleanup("quest_link_check")
    free_before = port_free(8012)
    rc, out = run_and_signal([PY, QLC, "--timeout", "60", "--seconds", "2"], sig)
    left = survivors("quest_link_check")
    freed, secs = wait_port_free(8012, timeout=1.0)
    record(f"quest_link_check + {signame}: no leftover process", not left,
           f"exit={rc}" + (f", survivors={left}" if left else ""))
    record(f"quest_link_check + {signame}: 8012 bindable within 1 s", freed,
           f"free_before={free_before}")
    if left:
        cleanup("quest_link_check")

# =========================================================================== 3
print("\n=== (3) quest_link_check.py --port 8013 runs alongside a squatter on 8012 ===")
cleanup("quest_link_check")
squat = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
squat.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
try:
    squat.bind(("0.0.0.0", 8012))
    squat.listen(1)
    # Start it WITHOUT signalling, so the bind can be observed while it is live. The
    # URL saying 8013 proves nothing on its own -- vuer's bind happens in an aiohttp
    # thread and could silently still be on 8012. The only proof is that 8013 is
    # actually occupied while the tool runs, and free again once it stops.
    with tempfile.NamedTemporaryFile("w+", suffix=".log", delete=False) as fh:
        live_log = fh.name
    with open(live_log, "wb") as sink:
        live = subprocess.Popen([PY, QLC, "--port", "8013", "--timeout", "60",
                                 "--seconds", "2"],
                                cwd=str(REPO), stdout=sink, stderr=subprocess.STDOUT,
                                start_new_session=True)
        bound_8013 = False
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            if not port_free(8013):
                bound_8013 = True
                break
            time.sleep(0.2)
        record("--port 8013: vuer REALLY binds 8013 (not just prints it)", bound_8013)
        try:
            os.killpg(os.getpgid(live.pid), signal.SIGINT)
            live.wait(timeout=10)
        except (OSError, subprocess.TimeoutExpired):
            try:
                os.killpg(os.getpgid(live.pid), signal.SIGKILL)
            except OSError:
                pass
    out = Path(live_log).read_text(errors="replace")
    os.unlink(live_log)
    freed_8013, _ = wait_port_free(8013, timeout=1.0)
    record("--port 8013: 8013 bindable again within 1 s of exit", freed_8013)
    record("--port 8013 starts while 8012 is held",
           ":8013/?ws=wss://" in out and "refusing to start" not in out,
           "printed an 8013 URL" if ":8013/?ws=wss://" in out else out[-200:])
    record("--port 8013 advertises 8013, never 8012",
           ":8013/?ws=wss://" in out and ":8012/?ws=wss://" not in out)
    # and the opposite: asking for the busy port is refused, not fought over
    rc2, out2 = run_and_signal([PY, QLC, "--port", "8012", "--timeout", "10"],
                               signal.SIGINT, settle=4.0)
    record("--port 8012 while busy -> refuses with exit 4",
           rc2 == 4 and "already in use" in out2, f"exit={rc2}")
finally:
    squat.close()
    cleanup("quest_link_check")
freed, _ = wait_port_free(8012, timeout=1.0)
record("8012 bindable again after case (3)", freed)

# =========================================================================== 4
print("\n=== (4) domain0_census.py on DOMAIN 1 leaves nothing behind ===")
CENSUS = str(REPO / "tools" / "domain0_census.py")
for signame, sig in (("SIGINT", signal.SIGINT), ("SIGTERM", signal.SIGTERM)):
    before = {p for p, _ in survivors("domain0_census")}
    rc, out = run_and_signal(
        [PY, CENSUS, "--domain", "1", "--iface", "lo", "--seconds", "3"], sig, settle=5.0)
    after = {p for p, _ in survivors("domain0_census")}
    new = after - before
    record(f"domain0_census + {signame}: no NEW leftover process", not new,
           f"exit={rc}" + (f", new={sorted(new)}" if new else ""))
    for pid in new:
        try:
            os.kill(pid, signal.SIGKILL)
        except OSError:
            pass

# a clean, un-signalled run must also leave nothing
before = {p for p, _ in survivors("domain0_census")}
proc = subprocess.run([PY, CENSUS, "--domain", "1", "--iface", "lo", "--seconds", "3"],
                      cwd=str(REPO), capture_output=True, text=True, timeout=180)
time.sleep(1.0)
after = {p for p, _ in survivors("domain0_census")}
new = after - before
record("domain0_census clean run: no NEW leftover process", not new,
       f"exit={proc.returncode}" + (f", new={sorted(new)}" if new else ""))
for pid in new:
    try:
        os.kill(pid, signal.SIGKILL)
    except OSError:
        pass

# =========================================================================== 5
print("\n=== (5) fake_lowstate.py (writer) reaps too, and refuses domain 0 ===")
FAKE = str(REPO / "tools" / "fake_lowstate.py")
proc = subprocess.run([PY, FAKE, "--domain", "0"], capture_output=True, text=True, timeout=60)
record("fake_lowstate refuses domain 0", proc.returncode == 2 and "REFUSED" in proc.stdout + proc.stderr,
       f"exit={proc.returncode}")
before = {p for p, _ in survivors("fake_lowstate")}
rc, out = run_and_signal([PY, FAKE, "--domain", "1", "--iface", "lo"], signal.SIGTERM,
                         settle=4.0)
after = {p for p, _ in survivors("fake_lowstate")}
record("fake_lowstate + SIGTERM: no leftover process", not (after - before),
       f"exit={rc}" + (f", new={sorted(after - before)}" if after - before else ""))
for pid in after - before:
    try:
        os.kill(pid, signal.SIGKILL)
    except OSError:
        pass

# =========================================================================== summary
print("\n=== summary ===")
n_fail = sum(1 for _, ok, _ in results if not ok)
for name, ok, detail in results:
    print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"  -- {detail}" if detail else ""))
print(f"\n{len(results) - n_fail}/{len(results)} passed")
sys.exit(1 if n_fail else 0)
