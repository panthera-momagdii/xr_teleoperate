"""[panthera] RT6h. The G1_29 arm IK in its own process.

Why: CasADi holds the Python GIL for the whole IPOPT solve. In-process, every solve
(8 to 13 ms, 30 times a second) stalls G1_29_ArmController's 250 Hz publish thread.
Measured on PC2 with a 250 Hz sleep thread: 246 Hz idle, 209 Hz with the IK, 172 Hz
with the IK and both arms capped; RT6g on the robot: 145 to 190 Hz. Here the solve
runs in a child process; the launcher's main thread waits on a socket, which releases
the GIL, so the publish thread keeps its rate.

`IKWorker()` starts the child; `attach(local_ik)` waits for it and returns the proxy,
which has G1_29_ArmIK's interface. The launcher starts it before DDS is initialized
(no DDS threads to fork past) and attaches once its own G1_29_ArmIK exists. `solve_ik` goes to the child; every
other attribute (reduced_robot, ik_lower, ik_upper, joint_margin) is the launcher's own
in-process G1_29_ArmIK, which is also the fallback: if the child fails to start, dies,
raises, or answers later than XR_IK_WORKER_TIMEOUT_S (0.5 s), the proxy kills it, logs
once, and solves in-process from then on (the old path, at the old rate).

The child:
  * is started with subprocess (fork+exec, safe with the launcher's threads), in a new
    session (no terminal signals: the launcher alone decides when it stops), stdin from
    /dev/null (a child on the launcher's tty breaks sshkeyboard, RT5c);
  * arms PR_SET_PDEATHSIG=SIGKILL and exits on EOF, so it cannot outlive the launcher
    even after a SIGKILL or os._exit;
  * imports robot_arm_ik with a stand-in for logging_mp, whose getLogger forks a
    listener process that only atexit reaps.
"""
import os
import socket
import subprocess
import sys
import time

import numpy as np
from multiprocessing.connection import Connection

DEFAULT_TIMEOUT_S = 0.5
DEFAULT_READY_S = 20.0


def enabled_from_env():
    return os.environ.get("XR_IK_WORKER", "1") == "1"


class IKWorker:
    def __init__(self, local_ik=None, timeout_s=None, ready_s=DEFAULT_READY_S, log=None):
        self.local = None
        self.timeout_s = float(timeout_s if timeout_s is not None
                               else os.environ.get("XR_IK_WORKER_TIMEOUT_S", DEFAULT_TIMEOUT_S))
        self.log = log
        self.proc = None
        self.conn = None
        self.failed = None
        self.solves = 0
        parent_sock, child_sock = socket.socketpair()
        repo = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        env = dict(os.environ)
        env["PYTHONPATH"] = repo + (os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
        try:
            self.proc = subprocess.Popen(
                [sys.executable, "-u", os.path.abspath(__file__), str(child_sock.fileno()), str(os.getpid())],
                pass_fds=(child_sock.fileno(),), stdin=subprocess.DEVNULL, cwd=os.getcwd(), env=env,
                start_new_session=True)
        except Exception as exc:
            child_sock.close()
            parent_sock.close()
            self._fail(f"start: {exc}")
            return
        child_sock.close()
        self.conn = Connection(parent_sock.detach())
        if local_ik is not None:
            self.attach(local_ik, ready_s)

    def attach(self, local_ik, ready_s=DEFAULT_READY_S):
        """Wait for the child to be ready (or fail over to `local_ik`); returns self."""
        self.local = local_ik
        if not self.active:
            return self
        try:
            if not self.conn.poll(ready_s):
                raise TimeoutError(f"not ready after {ready_s:g} s")
            msg = self.conn.recv()
            if msg[0] != "ready":
                raise RuntimeError(f"start: {msg}")
            lo, hi = np.asarray(msg[1]), np.asarray(msg[2])
            if not (np.allclose(lo, local_ik.ik_lower) and np.allclose(hi, local_ik.ik_upper)):
                raise RuntimeError("its joint bounds differ from the launcher's")
        except Exception as exc:
            self._fail(f"start: {exc}")
        return self

    @property
    def active(self):
        return self.failed is None

    @property
    def pid(self):
        return self.proc.pid if self.proc is not None else None

    def describe(self):
        if self.active:
            return f"[ik] solving in a worker process (pid {self.pid}); in-process fallback after {self.timeout_s:g} s"
        return f"[ik] worker process off ({self.failed}); solving in-process"

    def __getattr__(self, name):
        if name == "local":
            raise AttributeError(name)
        return getattr(self.local, name)

    def solve_ik(self, left_wrist, right_wrist, current_lr_arm_motor_q=None, current_lr_arm_motor_dq=None):
        if self.active:
            try:
                self.conn.send((left_wrist, right_wrist, current_lr_arm_motor_q, current_lr_arm_motor_dq))
                if not self.conn.poll(self.timeout_s):
                    raise TimeoutError(f"no answer in {self.timeout_s:g} s")
                ok, a, b = self.conn.recv()
                if not ok:
                    raise RuntimeError(a)
                self.solves += 1
                return _native(a), _native(b)
            except Exception as exc:
                self._fail(f"solve {self.solves + 1}: {type(exc).__name__}: {exc}")
        return self.local.solve_ik(left_wrist, right_wrist, current_lr_arm_motor_q, current_lr_arm_motor_dq)

    def _fail(self, why):
        self.failed = why
        if self.log is not None:
            try:
                self.log(f"[ik] worker process failed ({why}); solving in-process from now on")
            except Exception:
                pass
        self.close()

    def close(self, wait_s=1.0):
        if self.conn is not None:
            try:
                self.conn.close()
            except Exception:
                pass
            self.conn = None
        p = self.proc
        if p is None:
            return
        try:
            p.wait(timeout=wait_s)              # EOF ends it
        except Exception:
            try:
                p.kill()
                p.wait(timeout=wait_s)
            except Exception:
                pass


def _native(x):
    """A copy with numpy's own float64 dtype instance. An unpickled array's dtype is an
    equal but separate instance, so np.asarray(x, dtype=float) returns a NEW array, and
    the launcher's `cmd_q is sol_q` test (start_pose.step) then reads the IK target as
    replaced and zeroes the gravity feed-forward (G10 B2/B3 caught it)."""
    out = np.empty(np.shape(x))
    out[...] = x
    return out


def _child(fd, ppid):
    import ctypes
    import logging
    import signal
    import types
    try:
        ctypes.CDLL("libc.so.6", use_errno=True).prctl(1, signal.SIGKILL)   # PR_SET_PDEATHSIG
    except Exception:
        pass
    if os.getppid() != ppid:
        os._exit(0)
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    logging.basicConfig(level=logging.INFO, stream=sys.stderr,
                        format="%(asctime)s [ik-worker] %(levelname)s %(message)s")
    shim = types.ModuleType("logging_mp")
    for name in ("getLogger", "basicConfig", "DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"):
        setattr(shim, name, getattr(logging, name))
    sys.modules["logging_mp"] = shim
    conn = Connection(fd)
    try:
        from teleop.robot_control.robot_arm_ik import G1_29_ArmIK
        ik = G1_29_ArmIK()
        conn.send(("ready", ik.ik_lower, ik.ik_upper))
    except Exception as exc:
        try:
            conn.send(("error", f"{type(exc).__name__}: {exc}"))
        finally:
            os._exit(1)
    while True:
        try:
            req = conn.recv()
        except (EOFError, OSError):
            break
        try:
            q, tau = ik.solve_ik(*req)
            conn.send((True, q, tau))
        except Exception as exc:
            try:
                conn.send((False, f"{type(exc).__name__}: {exc}", None))
            except Exception:
                break
    sys.stderr.flush()
    os._exit(0)


if __name__ == "__main__":
    _child(int(sys.argv[1]), int(sys.argv[2]))
