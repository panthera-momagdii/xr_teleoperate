#!/usr/bin/env python3
"""shared pieces of the P3 scripts (below_rest, press, grip): live needs the feature lock AND live_writes open in LOCKS.json
(opened only on Mohammed's P3 GO). In mock mode (CW_MOCK_PORTS set) CW_MOCK_P3=1 bypasses the P3 lock for offline tests only."""
import os
import subprocess
import cw


def check_live(feature):
    if cw.MOCK and os.environ.get("CW_MOCK_P3") == "1":
        return
    cw.refuse_if_locked(feature, True)
    cw.refuse_if_locked("live_writes", True)


def stream(argv, env_extra=None):
    """run a lib tool and print its output line by line (the press gate and the grip hold need to be visible while they wait)"""
    env = dict(os.environ, **cw.DDS_ENV, **(env_extra or {}))
    env.pop("G1RS_STOP_FLAG", None)
    env["CW_STOP_FILE"] = cw.STOP_FILE
    print("$ " + " ".join(argv), flush=True)
    p = subprocess.Popen(argv, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    out = []
    for line in p.stdout:
        out.append(line)
        print(line.rstrip(), flush=True)
    p.wait()
    return p.returncode, "".join(out)


def live_one_finger(a, run, stage, env, tool="g1rs_block.py"):
    import os as _os
    soc, _ = cw.battery()
    print("BATTERY", soc, "%")
    if soc is not None and soc <= 20:
        raise SystemExit("STOP: battery %s %% (at or below 20 %%)" % soc)
    st = cw.check_before_write(a.hand, run, getattr(a, "approve_hold_clear_7", False))
    other = "left" if a.hand == "right" else "right"
    w = cw.Watch(other, run)
    try:
        hs = cw.hold(a.hand, run, st)
        rc, out = stream([cw.PY, "-B", _os.path.join(cw.LIB, tool), a.hand, str(a.axis), stage, hs, run.path("block")]
                         + cw.hand_net(a.hand) + (["--no-body"] if cw.MOCK else []), env)
    finally:
        w.stop()
    soc2, _ = cw.battery()
    print("BATTERY end", soc2, "%")
    return "DONE" if "RESULT DONE" in out else "ABORTED"


def live_hand(a, run, tool, extra):
    """whole-hand tools (grip, release): battery, STATUS check, read-only watch of the other hand, hold, lib tool streamed"""
    import os as _os
    soc, _ = cw.battery()
    print("BATTERY", soc, "%")
    if soc is not None and soc <= 20:
        raise SystemExit("STOP: battery %s %% (at or below 20 %%)" % soc)
    st = cw.check_before_write(a.hand, run, getattr(a, "approve_hold_clear_7", False), getattr(a, "approve_hold_status_3", False))
    other = "left" if a.hand == "right" else "right"
    w = cw.Watch(other, run)
    try:
        hs = cw.hold(a.hand, run, st)
        rc, out = stream([cw.PY, "-B", _os.path.join(cw.LIB, tool), a.hand, hs, run.path("block")] + extra
                         + cw.hand_net(a.hand) + (["--no-body"] if cw.MOCK else []))
    finally:
        w.stop()
    soc2, _ = cw.battery()
    print("BATTERY end", soc2, "%")
    return "DONE" if "RESULT DONE" in out else "ABORTED"
