#!/usr/bin/env python3
"""g1rs_20261009 READ-ONLY hand reader (no write of any kind). Uses h0_ro.ROModbus: function 0x03 only, every (start, count)
checked against the h0 allowlist, all addresses even. Every line: t_mono_ns, t_host, seq, hand, raw words per group, decoded
values (packed ERROR/STATUS/TEMP lo-first: lower axis = low byte), last_cmd (always null here), event. No addresses in output.
  asfound <out_dir>                  left thumb first, then 5 full reads (STATE + ANGLE_SET/SPEED_SET/FORCE_SET) per hand
  rest <hand> <seconds> <out_dir>    STATE groups at full rate, CONFIG at start and end
3-read filter: ERROR nonzero, any TEMP byte >= 52, read loss > 1 s stop the run (event STOP_*); 1 or 2 reads over a limit
are logged as GLITCH events with the raw words."""
import json
import os
import sys
import time

sys.path.insert(0, "/home/unitree/h0_tools")
import h0_ro  # noqa: E402
from h0_ro import ROModbus, STATE, CONFIG, i16  # noqa: E402
import collections  # noqa: E402
STOP_FILE, STOP_FLAG = os.environ.get("G1RS_STOP_FILE"), os.environ.get("G1RS_STOP_FLAG")

CFG = ("ANGLE_SET", "SPEED_SET", "FORCE_SET")
PACKED = ("ERROR", "STATUS", "TEMP")


def lo(words):
    return [b for w in words for b in (w & 255, (w >> 8) & 255)]


def decode(raw):
    d = {}
    for g, w in raw.items():
        d[g.lower()] = lo(w) if g in PACKED else i16(w)
    return d


class Reader:
    def __init__(self, hand, out):
        h = h0_ro.HANDS[hand]
        self.hand, self.c = hand, ROModbus(h["ip"], h["port"], h["unit"], timeout=1.0)
        self.c.connect()
        self.f, self.seq = out, 0

    def sample(self, groups, table=STATE, event=None):
        raw = {g: self.c.read_group(table, g) for g in groups}
        rec = {"t_mono_ns": time.monotonic_ns(), "t_host": round(time.time(), 6), "seq": self.seq, "hand": self.hand,
               "raw": raw, "dec": decode(raw), "last_cmd": None, "event": event}
        self.f.write(json.dumps(rec, separators=(",", ":")) + "\n")
        self.seq += 1
        return rec

    def log(self, event, **kw):
        self.f.write(json.dumps({"t_mono_ns": time.monotonic_ns(), "t_host": round(time.time(), 6), "seq": self.seq,
                                 "hand": self.hand, "raw": None, "dec": None, "last_cmd": None, "event": event, **kw}) + "\n")
        self.seq += 1


def asfound(out_dir):
    f = open(os.path.join(out_dir, "state.jsonl"), "x")
    left = Reader("left", f)
    first = left.sample(("STATUS", "ERROR", "CURRENT", "ANGLE_ACT"), event="LEFT_THUMB_FIRST")
    d = first["dec"]
    print("LEFT THUMB FIRST: STATUS bend %d rot %d | ERROR bend %d rot %d | CURRENT bend %d rot %d | ANGLE bend %d rot %d" % (
        d["status"][4], d["status"][5], d["error"][4], d["error"][5], d["current"][4], d["current"][5], d["angle_act"][4], d["angle_act"][5]))
    right = Reader("right", f)
    for r in (left, right):
        for i in range(5):
            r.sample(tuple(STATE), event="ASFOUND_STATE_%d" % i)
            r.sample(CFG, table=CONFIG, event="ASFOUND_CONFIG_%d" % i)
        r.c.close()
    f.close()


def rest(hand, seconds, out_dir):
    f = open(os.path.join(out_dir, "state_%s.jsonl" % hand), "x")
    r = Reader(hand, f)
    r.sample(CFG, table=CONFIG, event="CONFIG_START")
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from g1rs_burst import BurstTracker
    bursts = BurstTracker(lambda name, **kw: r.log(name, **kw), hand)
    tq = [collections.deque() for _ in range(6)]
    t_end, last_ok, n_err = time.monotonic() + seconds, time.monotonic(), 0
    stop = None
    while time.monotonic() < t_end and not (STOP_FILE and os.path.exists(STOP_FILE)):
        try:
            rec = r.sample(tuple(STATE))
            last_ok = time.monotonic()
        except Exception as e:  # noqa: BLE001
            n_err += 1
            r.log("READ_ERROR", detail=repr(e))
            if time.monotonic() - last_ok > 1.0:
                stop = "STOP_READ_LOSS_GT_1S"
                break
            try:
                r.c.close()
                r.c.connect()
            except Exception:  # noqa: BLE001
                pass
            continue
        d = rec["dec"]
        bs = bursts.update(rec)
        if bs:
            stop = "STOP_" + bs[0]
            r.log("STOP_DETAIL", **bs[1])
        t = time.monotonic()
        for k in range(6):
            q = tq[k]
            q.append((t, d["temp"][k]))
            while t - q[0][0] > 2.0:
                q.popleft()
            if t - q[0][0] >= 1.0 and sorted(x[1] for x in q)[len(q) // 2] >= 52:
                stop = "STOP_TEMP_MEDIAN_GE_52"
        if stop:
            if STOP_FLAG:
                open(STOP_FLAG, "w").write("%s %s %s" % (hand, stop, time.strftime("%H:%M:%S")))
            break
    if stop:
        r.log(stop)
    r.sample(CFG, table=CONFIG, event="CONFIG_END" + ("_AFTER_" + stop if stop else ""))
    r.c.close()
    f.close()
    print(hand, "done", stop or "OK", "read errors", n_err, "bursts per axis", bursts.count)


if __name__ == "__main__":
    if sys.argv[1] == "asfound":
        asfound(sys.argv[2])
    elif sys.argv[1] == "rest":
        rest(sys.argv[2], float(sys.argv[3]), sys.argv[4])
