#!/usr/bin/env python3
"""calibration_ws common helpers (2026-10-13). Every script: dry-run by default (prints the planned writes, opens no connection to a
hand, the camera or DDS); --live is required for any write to a hand. Live runs reuse the proven tools in scripts/lib (copies of
g1rs_20261012b/tools: hold, block, wake, watch, read, burst guards, body monitor) unchanged except the block tool's b / s / w stages.
Each run gets runs/<script>_<YYYYmmdd_HHMMSS>[_dryrun]/ with config.json, console.log, the tool outputs and SHA256SUMS."""
import argparse
import hashlib
import io
import json
import os
import subprocess
import sys
import time

ROOT = "/home/unitree/calibration_ws"
LIB = os.path.join(ROOT, "scripts", "lib")
RUNS = os.path.join(ROOT, "runs")
LOCKS = os.path.join(ROOT, "LOCKS.json")
PY = "/home/unitree/miniconda3/envs/g1visit/bin/python"
DDS_ENV = {"CYCLONEDDS_URI": "file:///home/unitree/cyclonedds_ws/cyclonedds.xml"}
AXIS = {0: "little", 1: "ring", 2: "middle", 3: "index", 4: "thumb bend", 5: "thumb rotation"}
HANDS = ("right", "left")
# offline test mode: CW_MOCK_PORTS="right:PORT,left:PORT" points every tool at simulated hands on localhost (no DDS, no watch, no real hand)
MOCK = dict(x.split(":") for x in os.environ["CW_MOCK_PORTS"].split(",")) if os.environ.get("CW_MOCK_PORTS") else None
# operator stop: while this file exists every writing tool stops before its next write (mock tests use their own file)
STOP_FILE = os.environ["CW_MOCK_STOP_FILE"] if MOCK and os.environ.get("CW_MOCK_STOP_FILE") else os.path.join(ROOT, "STOP")


def hand_net(hand):
    return ["--host", "localhost", "--port", MOCK[hand]] if MOCK else []
sys.path.insert(0, LIB)
sys.path.insert(0, "/home/unitree/h0_tools")


class Tee(io.TextIOBase):
    def __init__(self, *streams):
        self.streams = streams

    def write(self, s):
        for st in self.streams:
            st.write(s)
            st.flush()
        return len(s)

    def flush(self):
        for st in self.streams:
            st.flush()


def parser(desc):
    p = argparse.ArgumentParser(description=desc, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--live", action="store_true", help="really write to the hand (default: dry-run, touches nothing)")
    p.add_argument("--note", default="", help="free text stored in config.json")
    return p


class Run:
    def __init__(self, name, args):
        self.live = bool(getattr(args, "live", False))
        stamp = time.strftime("%Y%m%d_%H%M%S")
        base = os.path.join(RUNS, "%s_%s" % (name, stamp))
        tail = ("_mock" if MOCK else "") if self.live else "_dryrun"
        self.dir, n = base + tail, 2
        while os.path.exists(self.dir):                    # two runs in the same second
            self.dir, n = "%s_%d%s" % (base, n, tail), n + 1
        os.makedirs(self.dir)
        self.log = open(os.path.join(self.dir, "console.log"), "w")
        sys.stdout = Tee(sys.__stdout__, self.log)
        sys.stderr = Tee(sys.__stderr__, self.log)
        cfg = {"script": name, "mode": "LIVE" if self.live else "DRY_RUN", "t_host": time.time(), "t_local": time.strftime("%Y-%m-%d %H:%M:%S %Z"),
               "args": vars(args), "argv": sys.argv, "lib_sha256": {f: sha(os.path.join(LIB, f)) for f in sorted(os.listdir(LIB)) if f.endswith(".py")},
               "script_sha256": sha(os.path.abspath(sys.argv[0])), "cw_sha256": sha(os.path.abspath(__file__))}
        json.dump(cfg, open(os.path.join(self.dir, "config.json"), "w"), indent=1, default=str)
        print("RUN %s  mode %s  dir %s" % (name, cfg["mode"], self.dir))

    def path(self, *p):
        return os.path.join(self.dir, *p)

    def finish(self, result):
        print("RESULT", result)
        sys.stdout.flush()
        sys.stdout, sys.stderr = sys.__stdout__, sys.__stderr__
        self.log.close()
        files = sorted(os.path.relpath(os.path.join(dp, f), self.dir) for dp, _, fs in os.walk(self.dir) for f in fs if f != "SHA256SUMS")
        with open(os.path.join(self.dir, "SHA256SUMS"), "w") as fh:
            for f in files:
                fh.write("%s  %s\n" % (sha(os.path.join(self.dir, f)), f))
        return result


def sha(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def locked(feature):
    try:
        lk = json.load(open(LOCKS))
    except OSError:
        return True, "LOCKS.json missing"
    v = lk.get(feature)
    return (v is not None and not str(v).startswith("open")), v


def refuse_if_locked(feature, live):
    lk, why = locked(feature)
    if MOCK and feature == "live_writes":
        return False                                   # simulated hands only
    if live and lk:
        raise SystemExit("LOCKED: %s (%s). Live refused; dry-run only." % (feature, why))
    return lk


# ---------------------------------------------------------------- plan printing (dry-run)
def w6(vals, note):
    return "fc 0x10  ANGLE_SET 1486 x6 = [%s]   %s" % (", ".join(str(v) for v in vals), note)


def w1(reg, axis, value, note):
    base = {"ANGLE_SET": 1486, "FORCE_SET": 1498, "SPEED_SET": 1522}[reg]
    return "fc 0x06  %s(%d %s) @%d = %s   %s" % (reg, axis, AXIS[axis], base + 2 * axis, value, note)


def hold_vec(axis_vals=None):
    """symbolic hold vector: H0..H5 = values taken at send time"""
    v = ["H%d" % k for k in range(6)]
    for k, x in (axis_vals or {}).items():
        v[k] = x
    return v


def print_plan(title, lines):
    print("PLANNED WRITES (%s): %d" % (title, sum(1 for l in lines if l.startswith("fc "))))
    for i, l in enumerate(lines):
        print("  %3d  %s" % (i + 1, l))


def hold_plan_lines(hand):
    return ["[read STATUS / ERROR on all six axes, log them; STATUS 2 or 255 needed (7 only with --approve-hold-clear-7)]",
            w6(["R0", "R1", "R2", "R3", "R4", "R5+15 (first hold after power-up) or stored thumb targets T4, T5 (later holds)"],
               "hold %s: own reading of each axis (3 agreeing reads)" % hand),
            "[log 5 s before and 10 s after; require STATUS 2 and ERROR 0 on all six axes before the next write]"]


# ---------------------------------------------------------------- live helpers
def battery(seconds=3.0):
    if MOCK:
        return None, {"mock": True}
    try:
        os.environ.update(DDS_ENV)
        import g1rs_body
        b = g1rs_body.BodyMonitor()
        b.start()
        time.sleep(seconds)
        soc, summ = b.battery(), b.summary()
        b.close()
        if soc is not None and soc <= 35:
            print("NOTICE battery %s %% (at or below 35 %%: plan the swap; not a stop)" % soc)
        return soc, summ
    except Exception as e:  # noqa: BLE001
        return None, {"error": repr(e)}


def read_state(hand, out_path, n=3):
    """read only: STATE n times + ANGLE_SET, SPEED_SET, FORCE_SET; lines appended to out_path"""
    from g1rs_read import Reader
    from h0_ro import STATE, CONFIG
    import h0_ro
    if MOCK:
        h0_ro.HANDS[hand] = dict(h0_ro.HANDS[hand], ip="localhost", port=int(MOCK[hand]))
    with open(out_path, "a") as f:
        r = Reader(hand, f)
        for i in range(n):
            s = r.sample(tuple(STATE), event="PRE_WRITE_STATE_%d" % i)
        c = r.sample(("ANGLE_SET", "SPEED_SET", "FORCE_SET"), table=CONFIG, event="PRE_WRITE_CONFIG")
        r.c.close()
    d = s["dec"]
    return {"status": d["status"], "error": d["error"], "angle_act": d["angle_act"], "current": d["current"], "temp": d["temp"],
            "force_act": d["force_act"], "angle_set": c["dec"]["angle_set"], "speed_set": c["dec"]["speed_set"], "force_set": c["dec"]["force_set"]}


def check_before_write(hand, run, approve7=False, approve3=False):
    if os.path.exists(STOP_FILE):
        raise SystemExit("STOP: %s exists (operator stop); remove it only on Mohammed's word" % STOP_FILE)
    lk, why = locked("hand_" + hand)                   # per-hand lock (absent = no lock)
    if lk and not MOCK:
        raise SystemExit("LOCKED: hand_%s (%s). No write to this hand." % (hand, why))
    st = read_state(hand, run.path("pre_write_state_%s.jsonl" % hand))
    print("PRE-WRITE %s: STATUS %s ERROR %s ANGLE %s ANGLE_SET %s SPEED %s" % (hand, st["status"], st["error"], st["angle_act"], st["angle_set"], st["speed_set"]))
    if any(st["error"]):
        raise SystemExit("STOP: ERROR nonzero on the %s hand before any write" % hand)
    if 7 in st["status"] and not approve7:
        raise SystemExit("STOP: STATUS 7 on the %s hand; a fresh hold clears it only with approval (--approve-hold-clear-7)" % hand)
    ok = (2, 255, 7, 3) if approve3 else (2, 255, 7)       # 3 (force reached, left by an aborted press or grip): release only, with approval
    bad = [s for s in st["status"] if s not in ok]
    if bad:
        raise SystemExit("STOP: STATUS %s on the %s hand (need 2, 255 after power-up, 7 with approval%s)" % (st["status"], hand, ", 3 with approval (release only)" if not approve3 else ""))
    if 3 in st["status"]:
        print("APPROVED: STATUS 3 on axes %s; the hold writes their own reading" % [k for k, s in enumerate(st["status"]) if s == 3])
    off = [k for k in range(6) if st["force_set"][k] != 2000 or st["speed_set"][k] != 1000]
    if off:                                            # power-up values (both hands, 13 Oct): FORCE_SET 2000, SPEED_SET 1000
        print("NOTE %s: FORCE_SET %s SPEED_SET %s not at power-up values on axes %s (left by an earlier run or abort)" % (hand, st["force_set"], st["speed_set"], off))
    st["first_hold"] = st["angle_set"] == [0, 0, 0, 0, 0, 1000]
    return st


def run_tool(argv, env_extra=None, log_name=None):
    env = dict(os.environ, **DDS_ENV, **(env_extra or {}))
    env.pop("G1RS_STOP_FLAG", None)
    env["CW_STOP_FILE"] = STOP_FILE
    print("$ " + " ".join(argv))
    p = subprocess.run(argv, env=env, capture_output=True, text=True)
    out = (p.stdout or "") + (p.stderr or "")
    print(out.rstrip()[-3000:])
    if log_name:
        open(log_name, "w").write(out)
    return p.returncode, out


def hold(hand, run, st, sub="hold"):
    out = run.path(sub)
    argv = [PY, "-B", os.path.join(LIB, "g1rs_hold.py"), hand, out, "--expect", "none"] + hand_net(hand)
    if not st["first_hold"]:
        argv += ["--thumb-keep", "%d,%d" % (st["angle_set"][4], st["angle_set"][5])]
    rc, txt = run_tool(argv, {"G1RS_POST_S": os.environ.get("CW_HOLD_POST_S", "10")})
    rc2, chk = run_tool([PY, "-B", os.path.join(LIB, "check_hold.py"), out])
    if "RESULT DONE" not in txt or not chk.strip().endswith("CLEAN"):
        raise SystemExit("STOP: hold on the %s hand not clean (STATUS 2 and ERROR 0 on all six needed)" % hand)
    return os.path.join(out, "summary.json")


class Watch:
    """read only watch of the idle hand at full rate; log only (STATUS 7 there is logged, not a stop: rule 7)"""

    def __init__(self, hand, run):
        self.p = None
        if MOCK:
            print("WATCH skipped (mock)")
            return
        self.dir = run.path("watch_%s" % hand)
        os.makedirs(self.dir, exist_ok=True)
        env = dict(os.environ, **DDS_ENV)
        env.pop("G1RS_STOP_FLAG", None)
        self.p = subprocess.Popen([PY, "-B", os.path.join(LIB, "g1rs_watch.py"), hand, self.dir, os.path.join(self.dir, "END")],
                                  stdout=open(os.path.join(self.dir, "stdout.txt"), "w"), stderr=subprocess.STDOUT, env=env)
        time.sleep(2.0)

    def stop(self):
        if self.p is None:
            return
        open(os.path.join(self.dir, "END"), "w").close()
        self.p.wait(timeout=15)
        os.remove(os.path.join(self.dir, "END"))
        print("WATCH", open(os.path.join(self.dir, "stdout.txt")).read().strip()[-300:])
