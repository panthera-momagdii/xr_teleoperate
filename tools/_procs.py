"""Child-process hygiene shared by every operator tool. No DDS, no robot, no imports
beyond the standard library.

The problem
-----------
`logging_mp.getLogger()` -- which importing `hand_config` triggers -- forks a
**non-daemon** listener process, and only ever reaps it from an `atexit` hook. Several
tools here leave via `os._exit()` so that a DDS teardown cannot hang a take that has
already been saved, and `os._exit()` skips `atexit`. The listener is therefore
orphaned. Because it inherited the parent's stdout it also keeps a piped reader (a
`tail`, a `$(...)` capture) waiting forever, which is why the symptom usually looks
like "the tool hung" rather than "the tool leaked".

Measured on this host, 2026-09-09 17:16 -- four orphans from `tools/domain0_census.py`,
PPID 1, no children of their own, alive 2h43m to 4h08m past a `--seconds 15..30`
budget, argv still reading as the parent's because `fork()` copies it:

    547669  1  Wed Sep  9 13:07:40  04:08:06  python tools/domain0_census.py ... --seconds 30
    578665  1  Wed Sep  9 13:55:49  03:19:57  python tools/domain0_census.py ... --seconds 20
    583434  1  Wed Sep  9 14:03:27  03:12:19  python tools/domain0_census.py ... --seconds 15
    598921  1  Wed Sep  9 14:32:22  02:43:24  python tools/domain0_census.py ... --seconds 30

They squat on the DDS domain and hang the *next* run.

The ladder
----------
`terminate()` alone is not enough: on 2026-09-09 two leaked fixtures ignored SIGTERM
for at least 2 s and only died on SIGKILL. So every reap here is
graceful-shutdown -> terminate -> join -> kill, each step bounded, and the whole thing
wrapped so that reaping can never itself be what stops a tool from exiting.

Usage
-----
    from tools._procs import reap_child_processes, install_reaper

    reap_child_processes(log=log)          # immediately before os._exit()
    install_reaper(log=log)                # atexit + SIGINT + SIGTERM, once at startup
"""

import atexit
import os
import signal
import threading

__all__ = ["reap_child_processes", "install_reaper", "stop_logging_mp_listener",
           "install_parent_death_signal"]

_installed = False
_pdeath_installed = False
_lock = threading.Lock()


def install_parent_death_signal(sig=signal.SIGKILL,
                                reset_in_child=(signal.SIGHUP, signal.SIGTERM)):
    """Every multiprocessing child started after this dies when its parent dies. RT4.

    Nothing above covers SIGKILL: no handler, no finally and no atexit runs, so the
    children of a killed launcher -- the hand controllers' control loops, which publish
    rt/inspire_hand/ctrl/*, televuer's vuer server, which holds 8012, and logging_mp's
    listener -- were left running with PPID 1. Linux's PR_SET_PDEATHSIG makes the
    kernel send `sig` to the child the moment its parent goes, whatever killed it.

    How: BaseProcess.start() records the parent PID; BaseProcess._bootstrap(), which
    runs first thing in the child, calls prctl(PR_SET_PDEATHSIG) and then checks the
    parent is still the one that forked it (a parent that died between fork and prctl
    would otherwise be missed).

    `reset_in_child`: the child also puts these signals back to SIG_DFL. A fork copies
    the parent's Python signal handlers, and the launcher's SIGHUP/SIGTERM handler is
    an orderly-stop for the PARENT; in a child it would only set a flag nobody reads,
    and the child would survive the hang-up that should end it.

    Caveat, and the reason for the main-thread check: the kernel fires the signal when
    the parent THREAD that forked the child exits, not the process. A child started
    from a worker thread would be killed when that thread ends, so only children
    started from the main thread are armed. Every child the launcher has is.

    Linux only; a no-op elsewhere. Idempotent. Must run BEFORE the first child is
    started -- for the launcher that means before `import logging_mp`.
    """
    global _pdeath_installed
    import sys
    if not sys.platform.startswith("linux"):
        return False
    with _lock:
        if _pdeath_installed:
            return True
        _pdeath_installed = True
    import ctypes
    import multiprocessing.process as _mpp

    try:
        libc = ctypes.CDLL(None, use_errno=True)
    except OSError:
        return False
    PR_SET_PDEATHSIG = 1
    orig_start = _mpp.BaseProcess.start
    orig_bootstrap = _mpp.BaseProcess._bootstrap

    def start(self, *args, **kwargs):
        if threading.current_thread() is threading.main_thread():
            self._panthera_parent_pid = os.getpid()
        else:
            self._panthera_parent_pid = None
        return orig_start(self, *args, **kwargs)

    def _bootstrap(self, *args, **kwargs):
        parent = getattr(self, "_panthera_parent_pid", None)
        if parent is not None:
            try:
                libc.prctl(PR_SET_PDEATHSIG, int(sig), 0, 0, 0)
                if os.getppid() != parent:
                    os._exit(1)
            except Exception:
                pass
        for s in reset_in_child:
            try:
                signal.signal(s, signal.SIG_DFL)
            except Exception:
                pass
        return orig_bootstrap(self, *args, **kwargs)

    _mpp.BaseProcess.start = start
    _mpp.BaseProcess._bootstrap = _bootstrap
    return True


def _quiet(log, level, msg):
    if log is None:
        return
    try:
        getattr(log, level)(msg)
    except Exception:
        pass


def _kill_ladder(proc, log, graceful=None, graceful_timeout=2.0,
                 term_timeout=1.0, name="child"):
    """graceful -> terminate -> join -> kill. Every step bounded. Never raises.

    `graceful` is an optional zero-argument callable for a shutdown the process
    understands (logging_mp has one); it runs on a daemon thread so a shutdown that
    itself hangs cannot hang us.
    """
    try:
        if not proc.is_alive():
            return True
        if graceful is not None:
            t = threading.Thread(target=graceful, daemon=True)
            t.start()
            t.join(graceful_timeout)
            if not proc.is_alive():
                return True
        proc.terminate()
        proc.join(term_timeout)
        if proc.is_alive():
            _quiet(log, "warning", f"[_procs] {name} pid {proc.pid} ignored SIGTERM; killing")
            proc.kill()
            proc.join(term_timeout)
        return not proc.is_alive()
    except Exception as exc:
        _quiet(log, "warning", f"[_procs] reaping {name}: {exc}")
        return False


def stop_logging_mp_listener(log=None):
    """Reap logging_mp's forked listener. Returns True if nothing is left alive.

    Reaches into `logging_mp._internal_manager._listener_process` because logging_mp
    exposes no public shutdown that works from a non-atexit path. Guarded throughout:
    if the internals move, this degrades to a no-op rather than breaking every tool.
    """
    try:
        import logging_mp
    except Exception:
        return True
    try:
        mgr = getattr(logging_mp, "_internal_manager", None)
        proc = getattr(mgr, "_listener_process", None) if mgr else None
        if proc is None:
            return True
        shutdown = getattr(mgr, "_shutdown", None)
        return _kill_ladder(proc, log, graceful=shutdown,
                            name="logging_mp listener")
    except Exception as exc:
        _quiet(log, "warning", f"[_procs] stop_logging_mp_listener: {exc}")
        return False


def reap_child_processes(log=None):
    """Reap logging_mp's listener AND any other multiprocessing child. Bounded.

    Call immediately before `os._exit()`, and from signal handlers. Safe to call more
    than once and safe to call when there is nothing to reap.

    Returns True when nothing of ours is left alive.
    """
    ok = stop_logging_mp_listener(log=log)
    try:
        import multiprocessing
        for proc in multiprocessing.active_children():
            ok = _kill_ladder(proc, log, name=f"child {proc.name}") and ok
    except Exception as exc:
        _quiet(log, "warning", f"[_procs] active_children: {exc}")
        ok = False
    return ok


def install_reaper(log=None, exit_on_signal=True):
    """Register reaping on atexit AND on SIGINT/SIGTERM. Idempotent.

    Three paths have to be covered because a tool can leave by any of them:
      * a normal return               -> atexit
      * Ctrl-C or `kill`              -> the signal handlers
      * `os._exit()`                  -> neither; call reap_child_processes() directly

    Any handler already installed is chained, not replaced -- a tool that installs its
    own SIGINT handling keeps it.
    """
    global _installed
    with _lock:
        if _installed:
            return
        _installed = True

    atexit.register(reap_child_processes, log)

    def make_handler(signum, previous):
        def handler(sig, frame):
            _quiet(log, "info", f"[_procs] signal {sig}, reaping children")
            reap_child_processes(log=log)
            if callable(previous) and previous not in (signal.SIG_DFL, signal.SIG_IGN):
                try:
                    previous(sig, frame)
                    return
                except Exception:
                    pass
            if exit_on_signal:
                # 128+signum is the shell's convention for "died on a signal", and it
                # keeps SIGINT distinguishable from SIGTERM in a test's exit code.
                os._exit(128 + sig)
        return handler

    for signum in (signal.SIGINT, signal.SIGTERM):
        try:
            previous = signal.getsignal(signum)
            signal.signal(signum, make_handler(signum, previous))
        except (ValueError, OSError):
            # Not the main thread, or the platform will not allow it. Not fatal.
            pass
