#!/usr/bin/env python3
"""G7 (RT4): link-loss guard and exit hygiene. DOMAIN 1, LOOPBACK ONLY.

Never touches domain 0 and never runs the launcher against the robot: every launcher
here is `--sim --network-interface lo`, so it calls ChannelFactoryInitialize(1, "lo").
The fixtures are tools/fake_lowstate.py and tools/fake_inspire_state.py on domain 1,
and tools/fake_xr_client.py, which speaks vuer's websocket protocol to 127.0.0.1:8012
the way the Quest browser does.

Layers
  (1) link_guard.LinkGuard as pure arithmetic, with a simulated clock
  (2) the REAL launcher, --ee inspire_ftp, driven by the fake headset; the commands it
      publishes (rt/lowcmd, rt/inspire_hand/ctrl/{l,r}) are captured off the wire:
        stream live          -> arms follow
        stream stops         -> HOLD within XR_STALE_S, arms and hands frozen
        stream back, no [r]  -> still frozen, says "press [r]"
        [r]                  -> blend, every step inside XR_ARM_VEL_LIMIT / frequency
        [r] while stale      -> refused
        [q]                  -> go home, exit 0
  (3) exit paths: [q] (above), SIGINT, SIGTERM, SIGHUP from a real hung-up terminal
      (a pty whose master is closed), SIGKILL of the launcher. For each: nothing of
      ours publishes afterwards, no process of the launcher's session is left, 8012
      is bindable.

Run:  python tools/overnight/test_g7_link_guard.py
"""

import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time
import uuid
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "tools"))
PY = sys.executable
TELEOP = REPO / "teleop"

STALE_S, REENGAGE_T, VEL, FREQ = 0.5, 2.0, 3.0, 30.0
MAX_STEP = VEL / FREQ

results = []


def record(name, ok, detail=""):
    results.append((name, bool(ok), detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  -- {detail}" if detail else ""),
          flush=True)


# ======================================================================== (1)
print("=== (1) LinkGuard, pure ===", flush=True)
from teleop.robot_control import link_guard as lg   # noqa: E402

g = lg.LinkGuard(stale_s=STALE_S, reengage_t=REENGAGE_T, velocity_limit=VEL)
q0 = np.zeros(14)
record("first observe() is only a baseline", g.observe(0.0, b"a") is False)
record("start() with no fresh pose -> HOLD at the measured q",
       g.start(0.0, q0 + 0.1) == lg.HOLD and np.allclose(g.step(0.0, q0 + 9, q0, 1 / FREQ)[0], 0.1))
g = lg.LinkGuard(stale_s=STALE_S, reengage_t=REENGAGE_T, velocity_limit=VEL)
g.observe(0.0, b"a"); g.observe(0.01, b"b")
record("start() with a fresh pose -> FOLLOW", g.start(0.02, q0) == lg.FOLLOW)
out, tau = g.step(0.03, q0 + 0.2, q0 + 1.5, 1 / FREQ)
record("FOLLOW passes q and tauff through untouched",
       np.array_equal(out, q0 + 0.2) and np.array_equal(tau, q0 + 1.5))
out, tau = g.step(0.01 + STALE_S + 0.02, q0 + 5.0, q0, 1 / FREQ)
record("no new pose for stale_s -> HOLD on the LAST command, not the new one",
       g.state == lg.HOLD and np.allclose(out, 0.2) and np.allclose(tau, 1.5))
record("[r] while stale is refused", g.request_reengage(1.0) == "refused" and g.state == lg.HOLD)
g.observe(1.1, b"c")
out, _ = g.step(1.1, q0 + 5.0, q0, 1 / FREQ)
record("stream back without [r] -> still holding",
       g.state == lg.HOLD and np.allclose(out, 0.2)
       and any("press [r]" in e for _, e in g.events))
t, steps, last = 1.12, [], out.copy()
g.observe(t, b"d")
record("[r] with a live stream is accepted", g.request_reengage(t) == "accepted")
target = q0 + 1.2
while g.state != lg.FOLLOW and t < 10.0:
    t += 1 / FREQ
    g.observe(t, f"m{t}".encode())
    out, tau = g.step(t, target, q0 + 7, 1 / FREQ)
    steps.append(float(np.max(np.abs(out - last))))
    last = out.copy()
    if g.state == lg.BLEND:
        record_tau_zero = bool(np.all(tau == 0))
record("blend never steps more than velocity_limit * dt",
       max(steps) <= MAX_STEP + 1e-12, f"max step {max(steps):.4f} rad vs {MAX_STEP:.4f}")
record("blend lasts at least reengage_t", t - 1.12 >= REENGAGE_T - 1e-9, f"{t - 1.12:.2f} s")
record("feed-forward torque is zero during the blend", record_tau_zero)
record("blend ends in FOLLOW on the live command", g.state == lg.FOLLOW and np.allclose(out, target))
g2 = lg.LinkGuard(stale_s=STALE_S, reengage_t=REENGAGE_T, velocity_limit=VEL)
g2.observe(0, b"a"); g2.observe(0.01, b"b"); g2.start(0.02, q0); g2.step(0.03, q0, q0, 0.033)
g2.step(1.0, q0 + 1, q0, 0.033); g2.observe(1.1, b"c"); g2.request_reengage(1.1)
g2.step(1.2, q0 + 1, q0, 0.033)
g2.step(1.2 + STALE_S + 0.1, q0 + 1, q0, 0.033)
record("stream lost DURING the blend -> HOLD at the blended pose", g2.state == lg.HOLD)
for bad in ("0", "-1", "nan", "x"):
    os.environ["XR_STALE_S"] = bad
    try:
        lg.from_env()
        ok = False
    except lg.LinkGuardError:
        ok = True
    record(f"XR_STALE_S={bad!r} refused", ok)
os.environ.pop("XR_STALE_S", None)

# ======================================================================== shared rig
from fake_xr_client import FakeXR                          # noqa: E402
from tools import port_guard                               # noqa: E402
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber  # noqa: E402
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_  # noqa: E402
from inspire_sdkpy import inspire_dds                      # noqa: E402

ChannelFactoryInitialize(1, "lo")


class Capture:
    """Readers only. Every sample is (monotonic arrival time, payload)."""

    def __init__(self):
        self.lowcmd, self.ctrl_l, self.ctrl_r = [], [], []
        self._subs = []
        for topic, cls, sink, fn in (
                ("rt/lowcmd", LowCmd_, self.lowcmd,
                 lambda m: np.array([m.motor_cmd[i].q for i in range(15, 29)])),
                ("rt/inspire_hand/ctrl/l", inspire_dds.inspire_hand_ctrl, self.ctrl_l,
                 lambda m: tuple(m.angle_set)),
                ("rt/inspire_hand/ctrl/r", inspire_dds.inspire_hand_ctrl, self.ctrl_r,
                 lambda m: tuple(m.angle_set))):
            sub = ChannelSubscriber(topic, cls)
            sub.Init(lambda m, s=sink, f=fn: s.append((time.monotonic(), f(m))), 10)
            self._subs.append(sub)

    @staticmethod
    def window(sink, t0, t1):
        return [v for t, v in list(sink) if t0 <= t < t1]


cap = Capture()


def ipc_send(cmd, timeout_ms=4000):
    import zmq
    ctx = zmq.Context()
    sock = ctx.socket(zmq.REQ)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.RCVTIMEO, timeout_ms)
    sock.setsockopt(zmq.SNDTIMEO, timeout_ms)
    try:
        sock.connect("ipc://@xr_teleoperate_data.ipc")
        sock.send_json({"reqid": str(uuid.uuid4()), "cmd": cmd})
        return sock.recv_json()
    except Exception as exc:
        return {"status": "error", "msg": f"{type(exc).__name__}: {exc}"}
    finally:
        sock.close()
        ctx.term()


# XR_SHOULDER_TAU_CAP=0: the RT6d shoulder torque cap is off here. The fake headset's wrists
# sit about 8 Nm out front, so with the cap on the "operator moved during the outage" jump is
# held at the 4 Nm limit and there is nothing to blend (that path is test_g10's job).
# [G0] XR_TRACK_ERR_MAX=10: the follow guard's tracking stop is out of reach here. This
# fake's arms never move (constant q = 0), so against the moving headset the guard stops
# the session after 1 s, before any HOLD (that path is test_g18's job).
LAUNCH_ENV = {"XR_ARM_VEL_LIMIT": str(VEL), "XR_STALE_S": str(STALE_S),
              "XR_REENGAGE_T": str(REENGAGE_T), "XR_HAND_WAIT_S": "15",
              "PYTHONUNBUFFERED": "1", "XR_SHOULDER_TAU_CAP": "0", "XR_TRACK_ERR_MAX": "10"}
LAUNCH_ARGS = ["teleop_hand_and_arm.py", "--sim", "--network-interface", "lo",
               "--ee", "inspire_ftp"]


class Launcher:
    def __init__(self, label, pty_mode=False):
        self.label = label
        self.pty_mode = pty_mode
        self.lines = []           # (monotonic, text)
        self.master = None
        self._stop_reading = threading.Event()
        env = dict(os.environ); env.update(LAUNCH_ENV)
        if pty_mode:
            import pty
            self.master, slave = pty.openpty()
            # Popen's start_new_session does the setsid() in C; the shim then takes the
            # pty as its controlling terminal and execs the launcher, which keeps the
            # PID -- so the launcher is the session leader the kernel hangs up.
            shim = ("import fcntl, os, sys, termios; fcntl.ioctl(0, termios.TIOCSCTTY, 0); "
                    "os.execv(sys.executable, [sys.executable] + sys.argv[1:])")
            self.proc = subprocess.Popen([PY, "-c", shim] + LAUNCH_ARGS, cwd=str(TELEOP),
                                         env=env, stdin=slave, stdout=slave, stderr=slave,
                                         start_new_session=True, close_fds=True)
            os.close(slave)
            self._reader = threading.Thread(target=self._read_pty, daemon=True)
        else:
            self.proc = subprocess.Popen([PY] + LAUNCH_ARGS + ["--ipc"], cwd=str(TELEOP),
                                         env=env, stdin=subprocess.DEVNULL,
                                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                         start_new_session=True)
            self._reader = threading.Thread(target=self._read_pipe, daemon=True)
        self.sid = self.proc.pid
        self._reader.start()

    def _read_pipe(self):
        for raw in iter(self.proc.stdout.readline, b""):
            self.lines.append((time.monotonic(), raw.decode(errors="replace").rstrip()))

    def _read_pty(self):
        # select() with a timeout, never a bare blocking read: a thread parked in
        # os.read(master) keeps the master open after os.close(), so the hang-up the
        # test is about would not happen until the next byte of output arrived.
        import select
        buf = b""
        while not self._stop_reading.is_set():
            try:
                ready, _, _ = select.select([self.master], [], [], 0.1)
                if not ready:
                    continue
                chunk = os.read(self.master, 4096)
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self.lines.append((time.monotonic(), line.decode(errors="replace").rstrip("\r")))

    def find(self, pattern, after=0.0):
        rx = re.compile(pattern)
        for t, line in list(self.lines):
            if t >= after and rx.search(line):
                return t, line
        return None

    def wait_for(self, pattern, timeout, after=0.0):
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            hit = self.find(pattern, after)
            if hit:
                return hit
            if self.proc.poll() is not None:
                return self.find(pattern, after)
            time.sleep(0.05)
        return None

    def press(self, key):
        if self.pty_mode:
            os.write(self.master, key.encode())
            return {"status": "ok"}
        return ipc_send({"r": "CMD_START", "q": "CMD_STOP"}[key])

    def session_left(self):
        out = subprocess.run(["ps", "-eo", "pid=,sid=,args="], capture_output=True,
                             text=True).stdout
        left = []
        for row in out.splitlines():
            parts = row.split(None, 2)
            if len(parts) >= 2 and parts[1] == str(self.sid) and parts[0] != str(self.sid):
                left.append(row.strip()[:120])
        if self.proc.poll() is None:
            left.append(f"{self.sid} (the launcher itself)")
        return left

    def hang_up(self):
        """Close the pty master for real, as sshd does when the SSH session dies."""
        self._stop_reading.set()
        self._reader.join(timeout=2)
        os.close(self.master)

    def kill_all(self):
        try:
            os.killpg(self.sid, signal.SIGKILL)
        except OSError:
            pass


def fixtures():
    env = dict(os.environ)
    procs = [subprocess.Popen([PY, str(REPO / "tools/fake_lowstate.py"), "--domain", "1",
                               "--iface", "lo", "--rate", "500"], cwd=str(REPO), env=env,
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                              start_new_session=True),
             subprocess.Popen([PY, str(REPO / "tools/fake_inspire_state.py"), "--domain", "1",
                               "--iface", "lo", "--angle", "500"], cwd=str(REPO), env=env,
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                              start_new_session=True)]
    time.sleep(3.0)
    return procs


def stop_fixtures(procs):
    for p in procs:
        try:
            os.killpg(p.pid, signal.SIGTERM)
        except OSError:
            pass
    for p in procs:
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(p.pid, signal.SIGKILL)
            except OSError:
                pass


def bring_up(label, pty_mode=False):
    """Launcher up, fake headset streaming, [r] pressed, commands flowing."""
    if not port_guard.port_is_free(8012):
        record(f"{label}: 8012 free before start", False, port_guard.describe_busy(8012))
    L = Launcher(label, pty_mode=pty_mode)
    xr = FakeXR("wss://127.0.0.1:8012/", rate=60.0).start()
    hit = L.wait_for(r"Press \[r\]", timeout=120)
    if not hit:
        tail = " | ".join(l for _, l in L.lines[-5:])
        return L, xr, f"no [r] prompt (exit={L.proc.poll()}): {tail[-300:]}"
    t_end = time.monotonic() + 20
    while time.monotonic() < t_end and xr.sent < 60:
        time.sleep(0.1)
    time.sleep(1.0)
    L.press("r")
    if not L.wait_for(r"\[link\] tracking", timeout=10):
        return L, xr, "no '[link] tracking' line after [r]"
    time.sleep(2.0)
    return L, xr, None


def silence_after(L, t_trigger, t_exit, label, expect_home):
    time.sleep(4.0)
    before = [len(Capture.window(s, t_trigger - 1.5, t_trigger))
              for s in (cap.lowcmd, cap.ctrl_l, cap.ctrl_r)]
    after = [len(Capture.window(s, t_exit + 1.0, t_exit + 4.0))
             for s in (cap.lowcmd, cap.ctrl_l, cap.ctrl_r)]
    record(f"{label}: commands were flowing before (lowcmd, ctrl/l, ctrl/r)",
           all(n > 0 for n in before), f"{before} samples in the 1.5 s before")
    record(f"{label}: NOTHING published 1..4 s after (lowcmd, ctrl/l, ctrl/r)",
           after == [0, 0, 0], f"{after} samples")
    left = L.session_left()
    record(f"{label}: no process of the launcher's session left", not left,
           "; ".join(left) if left else "none")
    record(f"{label}: 8012 bindable", port_guard.port_is_free(8012),
           "TIME-WAIT is allowed, a listener is not")
    if expect_home:
        last = [v for t, v in cap.lowcmd if t < t_exit + 1.0][-1:]
        record(f"{label}: last arm command is home (all 14 q = 0)",
               bool(last) and float(np.max(np.abs(last[0]))) < 1e-6,
               f"max |q| {float(np.max(np.abs(last[0]))):.4f}" if last else "no sample")


fx = fixtures()
try:
    # ==================================================================== (2)
    print("\n=== (2) the launcher on fakes: hold, no auto-resume, blended re-engage ===",
          flush=True)
    L, xr, err = bring_up("guard")
    if err:
        record("guard: launcher up and tracking", False, err)
    else:
        record("guard: launcher up, fake headset tracking", True)
        t_f = time.monotonic()
        time.sleep(3.0)
        follow = Capture.window(cap.lowcmd, t_f - 2.0, t_f + 3.0)
        spread = float(np.max(np.ptp(np.array(follow), axis=0))) if follow else 0.0
        record("FOLLOW: arms follow the moving headset", spread > 1e-3,
               f"largest joint range {spread:.4f} rad over 5 s")
        hand_follow = Capture.window(cap.ctrl_l, t_f - 2.0, t_f + 3.0)
        hand_moves = len(set(hand_follow)) > 1
        print(f"  (info) left hand command values seen while following: {len(set(hand_follow))}")

        xr.pause()
        t_p = time.monotonic()
        hit = L.wait_for(r"\[link\] HOLD: no new XR pose", timeout=5, after=t_p)
        lat = (hit[0] - t_p) if hit else None
        record("stream stops -> HOLD is declared within XR_STALE_S + 0.3 s",
               hit is not None and lat <= STALE_S + 0.3,
               f"after {lat:.2f} s (XR_STALE_S={STALE_S})" if hit else "no HOLD line")
        time.sleep(3.0)
        t_h = (hit[0] if hit else t_p) + 0.2
        hold = Capture.window(cap.lowcmd, t_h, time.monotonic())
        hold_q = hold[-1] if hold else None
        record("HOLD: arm command frozen",
               bool(hold) and float(np.max(np.ptp(np.array(hold), axis=0))) == 0.0,
               f"{len(hold)} samples, range {float(np.max(np.ptp(np.array(hold), axis=0))) if hold else 'n/a'}")
        hh = Capture.window(cap.ctrl_l, t_h + 0.5, time.monotonic())
        record("HOLD: hand command frozen", bool(hh) and len(set(hh)) == 1,
               f"{len(set(hh))} distinct values in {len(hh)} samples")

        xr.set_offset([0.0, 0.10, 0.10])     # the operator moved while the link was down
        xr.resume()
        t_r = time.monotonic()
        back = L.wait_for(r"\[link\] XR stream is back", timeout=3, after=t_r)
        record("stream back -> says 'press [r]'", back is not None,
               f"after {back[0] - t_r:.2f} s" if back else "no line")
        time.sleep(3.0)
        after_back = Capture.window(cap.lowcmd, t_r, time.monotonic())
        dev = (float(np.max(np.abs(np.array(after_back) - hold_q)))
               if after_back and hold_q is not None else None)
        record("stream back WITHOUT [r]: arms do not move (no auto-resume, no jump)",
               dev is not None and dev == 0.0, f"max |q - hold| {dev} over 3 s")

        t_e = time.monotonic()
        L.press("r")
        bl = L.wait_for(r"\[link\] re-engage: blending", timeout=3, after=t_e)
        record("[r] with a live stream -> blend starts", bl is not None)
        done = L.wait_for(r"\[link\] re-engaged", timeout=REENGAGE_T + 6, after=t_e)
        record("blend takes at least XR_REENGAGE_T",
               done is not None and done[0] - t_e >= REENGAGE_T - 0.1,
               f"{done[0] - t_e:.2f} s" if done else "never re-engaged")
        time.sleep(1.5)
        seg = Capture.window(cap.lowcmd, t_e, time.monotonic())
        distinct = [seg[0]] if seg else []
        for v in seg[1:]:
            if not np.array_equal(v, distinct[-1]):
                distinct.append(v)
        steps = [float(np.max(np.abs(b - a))) for a, b in zip(distinct, distinct[1:])]
        worst = max(steps) if steps else None
        record("every re-engage step <= XR_ARM_VEL_LIMIT / frequency",
               worst is not None and worst <= MAX_STEP + 1e-6,
               f"worst {worst:.4f} rad vs {MAX_STEP:.4f} ({len(steps)} steps)" if steps else "no steps")
        gap = float(np.max(np.abs(seg[-1] - hold_q))) if seg and hold_q is not None else 0.0
        record("...and the gap it closed was a real jump (> 3 steps)", gap > 3 * MAX_STEP,
               f"hold -> live target {gap:.3f} rad")

        xr.pause()
        t_s = time.monotonic()
        L.wait_for(r"\[link\] HOLD: no new XR pose", timeout=5, after=t_s)
        t_rr = time.monotonic()
        L.press("r")
        ref = L.wait_for(r"re-engage REFUSED", timeout=3, after=t_rr)
        record("[r] while the stream is stale is refused", ref is not None,
               ref[1].strip()[:100] if ref else "no refusal line")

        xr.resume()
        time.sleep(1.0)
        t_q = time.monotonic()
        L.press("q")
        try:
            rc = L.proc.wait(timeout=40)
        except subprocess.TimeoutExpired:
            rc = None
        record("[q]: exit 0", rc == 0, f"exit={rc}")
        xr.stop()
        silence_after(L, t_q, time.monotonic(), "[q]", expect_home=True)
        L.kill_all()

    # ==================================================================== (3)
    print("\n=== (3) exit paths: SIGINT, SIGTERM, SIGHUP (hung-up terminal), SIGKILL ===",
          flush=True)
    # SIGINT three times: CasADi swallows a KeyboardInterrupt that lands inside IPOPT,
    # so a single SIGINT passes or fails on timing (2 of 5 hung before RT4's fix).
    for label, how, want_rc, home in (("SIGINT", signal.SIGINT, 0, True),
                                      ("SIGINT#2", signal.SIGINT, 0, True),
                                      ("SIGINT#3", signal.SIGINT, 0, True),
                                      ("SIGTERM", signal.SIGTERM, 6, True),
                                      ("SIGHUP", "pty", 5, True),
                                      ("SIGKILL", signal.SIGKILL, -9, False)):
        L, xr, err = bring_up(label, pty_mode=(how == "pty"))
        if err:
            record(f"{label}: launcher up and tracking", False, err)
            xr.stop(); L.kill_all()
            continue
        t_x = time.monotonic()
        if how == "pty":
            L.hang_up()                    # the terminal goes away, as when SSH dies
        else:
            os.kill(L.proc.pid, how)
        try:
            rc = L.proc.wait(timeout=40)
        except subprocess.TimeoutExpired:
            rc = None
        t_done = time.monotonic()
        record(f"{label}: launcher exited with {want_rc}", rc == want_rc,
               f"exit={rc} after {t_done - t_x:.1f} s")
        if how == "pty":
            hup_log = REPO / "logs" / f"launcher_hup_{L.proc.pid}.log"
            text = hup_log.read_text(errors="replace") if hup_log.exists() else ""
            record("SIGHUP: shutdown continued into logs/launcher_hup_<pid>.log",
                   "orderly stop" in text, str(hup_log.name) if text else "no log file")
        xr.stop()
        silence_after(L, t_x, t_done, label, expect_home=home)
        L.kill_all()
finally:
    stop_fixtures(fx)

# ======================================================================== summary
print("\n=== summary ===")
n_fail = sum(1 for _, ok, _ in results if not ok)
for name, ok, detail in results:
    print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"  -- {detail}" if detail else ""))
print(f"\n{len(results) - n_fail}/{len(results)} passed", flush=True)
from tools._procs import reap_child_processes  # noqa: E402
reap_child_processes()
os._exit(1 if n_fail else 0)
