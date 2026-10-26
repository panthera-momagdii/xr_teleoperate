# RT6h test aid: for the launcher only, dump every thread's Python stack 45 s after start
# (a hung start-up shows where it waits) and on SIGUSR1. No effect on other processes.
try:
    with open("/proc/self/cmdline", "rb") as _f:
        _cmd = _f.read().split(b"\0")
    if any(a.endswith(b"teleop_hand_and_arm.py") for a in _cmd):
        import faulthandler as _fh, os as _os, signal as _sg
        _out = open(_os.path.join(_os.environ.get("FH_DIR", "/tmp"), f"fh_{_os.getpid()}.txt"), "w")
        _fh.register(_sg.SIGUSR1, file=_out, all_threads=True)
        _fh.dump_traceback_later(45, repeat=False, file=_out, exit=False)
except Exception:
    pass
