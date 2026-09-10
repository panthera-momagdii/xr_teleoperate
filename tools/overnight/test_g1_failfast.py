#!/usr/bin/env python3
"""G1: the launcher refuses instead of hanging. DOMAIN 1, LOOPBACK ONLY.

Never starts a DDS writer on domain 0 and never runs the launcher against the robot:
every launcher invocation here is `--sim`, which makes teleop_hand_and_arm.py call
ChannelFactoryInitialize(1, ...) -- see teleop_hand_and_arm.py:132-135.

Cases
  a  --ee inspire_ftp with no fixture      -> exit 3 in ~XR_HAND_WAIT_S
  a2 --ee dex1 with no fixture             -> exit 3 (the 2026-09-09 hang; unbounded before)
  a3 every other --ee family               -> exit 3, none of them hangs
  b  port 8012 pre-held by a dummy         -> exit 4 in < 2 s
  c  no --ee                               -> gets PAST the ee pre-flight
  d  EE_STATE_TOPICS matches the controllers' own constants
  e  cert line is printed and names the resolved file

Run:  python tools/overnight/test_g1_failfast.py
"""

import os
import pathlib
import re
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent.parent
TELEOP = REPO / "teleop"
PY = sys.executable

results = []


def notices(text):
    """The launcher's plain-text operator lines, unwrapped.

    teleop_hand_and_arm.notice() prints these to stderr verbatim, precisely because
    logging_mp/rich mangles anything containing a path or a DDS topic. A test that
    matched the rich-rendered copy would be testing the log formatter; matching these
    tests what an operator can actually read.
    """
    keep = []
    for line in text.splitlines():
        if (line.startswith(("[cert]", "[xr]", "[ee]", "----", "🟢", "🟡", "🔵", "🔴", "⚠"))
                or line.startswith("    ")):
            keep.append(line.strip())
    return "\n".join(keep)


def flat(text):
    """Collapse whitespace.

    logging_mp renders through rich, which word-WRAPS every message into a narrow
    column and prefixes each physical line with a timestamp gutter. A message is
    therefore split across lines at arbitrary points, and a naive substring search for
    the wording fails on a line break that is purely cosmetic. Flattening restores the
    logical message before matching.
    """
    return re.sub(r"\s+", " ", text)


def record(name, ok, detail=""):
    results.append((name, ok, detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  -- {detail}" if detail else ""))


def run_launcher(extra_args, timeout, env_extra=None, wait_s="6"):
    """Run the launcher on domain 1 (--sim) and return (rc, elapsed, output).

    The launcher is started in its own PROCESS GROUP and the whole group is killed on
    timeout. This is not fussiness: the launcher inherits logging_mp's non-daemon
    listener fork, that fork inherits the listening socket on 8012, and killing only
    the direct child leaves the fork holding the port. Four such orphans accumulated
    during this gate's own development and made the next run fail the exit-4 check --
    the very failure mode G1 exists to catch. G2 fixes the launcher; the test must not
    depend on that fix having landed.
    """
    env = dict(os.environ)
    env["XR_HAND_WAIT_S"] = wait_s
    env.setdefault("PYTHONUNBUFFERED", "1")
    if env_extra:
        env.update(env_extra)
    cmd = [PY, "teleop_hand_and_arm.py", "--sim", "--network-interface", "lo"] + extra_args
    t0 = time.monotonic()
    # Output goes to a FILE, never a pipe. The launcher's logging_mp listener fork
    # inherits stdout, so with a pipe `communicate()` blocks until that ORPHAN closes
    # it -- which is never -- and returns empty output even after the direct child is
    # killed. SPARK_HOST.md says the same thing about these tools. With a file the
    # output is on disk the moment it is written, whatever happens to the processes.
    with tempfile.NamedTemporaryFile("w+", suffix=".log", delete=False) as fh:
        log_path = fh.name
    try:
        with open(log_path, "wb") as sink:
            proc = subprocess.Popen(cmd, cwd=TELEOP, env=env, stdout=sink,
                                    stderr=subprocess.STDOUT, start_new_session=True)
            try:
                rc = proc.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                _kill_group(proc)
                rc = None                              # None == still running
        out = pathlib.Path(log_path).read_text(errors="replace")
        return rc, time.monotonic() - t0, out
    finally:
        try:
            os.unlink(log_path)
        except OSError:
            pass


def _kill_group(proc):
    """SIGKILL the process group, then wait. Bounded, never raises."""
    import signal
    for sig in (signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(os.getpgid(proc.pid), sig)
        except (ProcessLookupError, PermissionError, OSError):
            return
        try:
            proc.wait(timeout=3)
            return
        except subprocess.TimeoutExpired:
            continue


def require_8012_free(where):
    """Refuse to run a case while something still holds 8012 -- and say what."""
    sys.path.insert(0, str(REPO))
    from tools import port_guard
    if port_guard.port_is_free(8012):
        return True
    print(f"  !! 8012 is busy before {where}:")
    print("     " + port_guard.describe_busy(8012).replace("\n", "\n     "))
    return False


# --------------------------------------------------------------------------- a
record("8012 is free before the suite starts", require_8012_free("the suite"))

print("\n=== (a) --ee with no end effector present -> exit 3, bounded ===")
WAIT = 6.0
# dex5 is excluded here on purpose: `--ee dex5 --sim` is refused by argparse (exit 2)
# before any of this, because unitree_sim_isaaclab ships Dex3 only. Its exit-3 path is
# the same hand_config.preflight() the fail-closed suite already exercises on domain 1.
for ee in ("inspire_ftp", "dex1", "dex3", "inspire_dfx", "brainco"):
    rc, secs, out = run_launcher(["--ee", ee], timeout=WAIT + 25, wait_s=str(WAIT))
    if rc is None:
        record(f"--ee {ee}", False, f"HUNG (still running after {secs:.1f}s)")
        continue
    note = notices(out)
    bounded = secs <= WAIT + 20
    named = bool(re.search(r"no (hand |)state on ", note))
    # the message must name at least one INTACT topic -- not a token rich split in half
    topics_named = bool(re.search(r"rt/\S*(state|/l\b|/r\b)", note))
    record(f"--ee {ee} -> exit 3, bounded, names the topic",
           rc == 3 and bounded and named and topics_named,
           f"exit={rc} in {secs:.1f}s (wait was {WAIT}s), names topic={named and topics_named}")

# dex5 under --sim: a DIFFERENT, also-correct refusal that must not regress to a hang
rc, secs, out = run_launcher(["--ee", "dex5"], timeout=60, wait_s="6")
record("--ee dex5 --sim -> exit 2 (argparse), not a hang",
       rc == 2 and secs < 30 and "no simulation target" in flat(out),
       f"exit={rc} in {secs:.1f}s")

# --------------------------------------------------------------------------- b
print("\n=== (b) port 8012 held -> exit 4, fast ===")
record("8012 is free before case (b)", require_8012_free("case (b)"))
squat = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
squat.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
try:
    squat.bind(("0.0.0.0", 8012))
    squat.listen(1)
    rc, secs, out = run_launcher([], timeout=60)
    named_pid = f"held by PID {os.getpid()}" in notices(out)
    record("port busy -> exit 4", rc == 4 and secs < 25,
           f"exit={rc} in {secs:.1f}s")
    record("port busy names the holding PID and cmdline", named_pid,
           f"looked for 'held by PID {os.getpid()}'")
    record("port busy refuses in < 2 s of launcher work",
           "[xr] port 8012 is free" not in notices(out) and rc == 4,
           "no 'port is free' line was printed")
finally:
    squat.close()

# re-check the port is released, so later cases are not poisoned
time.sleep(0.3)
probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    probe.bind(("0.0.0.0", 8012))
    record("8012 released after the squatter closed", True)
except OSError as exc:
    record("8012 released after the squatter closed", False, str(exc))
finally:
    probe.close()

# --------------------------------------------------------------------------- c
print("\n=== (c) no --ee still gets past the ee pre-flight ===")
rc, secs, out = run_launcher([], timeout=90, wait_s="3")
past_ee = "[ee preflight]" not in flat(out) and "[hand preflight]" not in flat(out)
got_port = "[xr] port 8012 is free" in notices(out)
record("no --ee: no ee pre-flight runs at all", past_ee, f"exit={rc} in {secs:.1f}s")
record("no --ee: port check ran and passed", got_port)
record("no --ee: does NOT exit 3 or 4", rc not in (3, 4), f"exit={rc}")
NO_EE_OUT = out
NO_EE_RC = rc

# ... and with a fake rt/lowstate it gets all the way to the [r] prompt. Without one
# G1_29_ArmController times out in dds_utils.wait_for_dds (robot_arm.py:113), which is
# the only thing between here and the prompt on a box with no robot.
print("\n--- (c2) no --ee + tools/fake_lowstate.py -> reaches 'Press [r]' ---")
fake = subprocess.Popen(
    [PY, str(REPO / "tools" / "fake_lowstate.py"),
     "--domain", "1", "--iface", "lo", "--seconds", "60", "--rate", "500"],
    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
try:
    time.sleep(3.0)                       # let discovery settle
    rc2, secs2, out2 = run_launcher([], timeout=35, wait_s="3")
    flat2 = flat(out2)
    # matched against notices(), not the rich-rendered log: see notices.__doc__
    reached = "Press [r] to start syncing the robot" in notices(out2)
    # rc None == still running at the timeout, which is exactly right: the launcher
    # is parked in `while not START and not STOP` waiting for a keypress.
    parked = rc2 is None
    record("no --ee + fake lowstate: reaches 'Press [r]'", reached,
           f"exit={rc2} after {secs2:.1f}s")
    record("no --ee + fake lowstate: then waits for the keypress (does not exit)",
           parked and reached, f"exit={rc2}")
    record("no --ee + fake lowstate: arm controller came up",
           "Initialize G1_29_ArmController" in flat2)
    record("no --ee + fake lowstate: prompt is readable in one piece (not rich-wrapped)",
           any(l == "🟢  Press [r] to start syncing the robot with your movements."
               for l in out2.splitlines()))
finally:
    fake.terminate()
    try:
        fake.wait(timeout=5)
    except subprocess.TimeoutExpired:
        fake.kill()
        fake.wait(timeout=5)

# --------------------------------------------------------------------------- d
print("\n=== (d) EE_STATE_TOPICS matches the controllers' own constants ===")
probe_src = r"""
import sys
sys.path.insert(0, %r)
from teleop.robot_control import hand_config as hc
from teleop.robot_control import robot_hand_unitree as u
from teleop.robot_control import robot_hand_brainco as b
from teleop.robot_control import robot_hand_inspire as i
want = {
    "dex1":        (u.kTopicGripperLeftState, u.kTopicGripperRightState),
    "dex3":        (u.kTopicDex3LeftState, u.kTopicDex3RightState),
    "brainco":     (b.kTopicbraincoLeftState, b.kTopicbraincoRightState),
    "inspire_dfx": (i.kTopicInspireDFXState,),
}
bad = []
for ee, topics in want.items():
    got = tuple(hc.EE_STATE_TOPICS[ee]["topics"])
    if got != topics:
        bad.append((ee, got, topics))
print("MISMATCH" if bad else "MATCH", bad)
""" % str(REPO)
proc = subprocess.run([PY, "-c", probe_src], cwd=TELEOP, capture_output=True, text=True,
                      timeout=120)
record("topic table equals the controllers' constants",
       "MATCH" in proc.stdout and "MISMATCH" not in proc.stdout,
       proc.stdout.strip().splitlines()[-1] if proc.stdout.strip() else proc.stderr[-200:])

# every --ee choice is either in the table, a MODELS lane, or explicitly exempt
choices_src = r"""
import re, sys
sys.path.insert(0, %r)
from teleop.robot_control import hand_config as hc
src = open(%r).read()
m = re.search(r"add_argument\('--ee'.*?choices=\[(.*?)\]", src, re.S)
choices = [c.strip().strip("'\"") for c in m.group(1).split(",")]
covered = set(hc.EE_STATE_TOPICS) | set(hc.EE_TO_MODEL) | {"dex1_internal"}
missing = [c for c in choices if c not in covered]
print("CHOICES", choices)
print("UNCOVERED", missing)
""" % (str(REPO), str(TELEOP / "teleop_hand_and_arm.py"))
proc = subprocess.run([PY, "-c", choices_src], cwd=TELEOP, capture_output=True, text=True,
                      timeout=120)
uncovered = [l for l in proc.stdout.splitlines() if l.startswith("UNCOVERED")]
record("every --ee choice is covered or explicitly exempt",
       uncovered == ["UNCOVERED []"], uncovered[0] if uncovered else proc.stderr[-200:])

# --------------------------------------------------------------------------- e
print("\n=== (e) the cert line is printed at startup ===")
flat_no_ee = notices(NO_EE_OUT)
m = re.search(r"\[cert\].*?SAN=\[[^\]]*\]", flat_no_ee)
record("cert line printed", bool(m), m.group(0)[:140] if m else "no [cert] line found")
record("cert line carries a sha256 and a SAN list",
       bool(m) and "sha256=" in m.group(0) and "SAN=[" in m.group(0))
# and it must name the file televuer will really open, not a guess
import subprocess as _sp
_res = _sp.run([PY, str(REPO / "tools" / "cert_info.py"), "--oneline"],
               capture_output=True, text=True, timeout=60)
_resolved = _res.stdout.split()[1] if _res.returncode == 0 and _res.stdout.split() else ""
record("cert line names the path televuer actually resolves",
       bool(_resolved) and _resolved in flat_no_ee, _resolved)

# --------------------------------------------------------------------------- summary
print("\n=== summary ===")
n_fail = sum(1 for _, ok, _ in results if not ok)
for name, ok, detail in results:
    print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"  -- {detail}" if detail else ""))
print(f"\n{len(results) - n_fail}/{len(results)} passed")
if NO_EE_RC not in (3, 4):
    print(f"note: case (c) exited {NO_EE_RC}; without a robot or an image server the "
          f"launcher cannot reach 'Press [r]', which is expected -- what (c) proves is "
          f"that it is NOT the ee pre-flight or the port check that stopped it.")
sys.exit(1 if n_fail else 0)
