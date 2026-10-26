#!/usr/bin/env python3
"""g1rs_20261012 copy of the 9 Oct hold tool (g1rs_20261009/tools/g1rs_hold.py, sha 4beabf48). Changes: --expect none for later
holds in the same power-up (the rule alone: own reading, rotation + 15); an axis moving more than 5 counts during the hold is logged
as an event, not a stop (not on the 12 Oct ABORT list); temperature median >= 48 before the write pauses 5 min; commands carry a
test label. 9 Oct description:
HOLD IN PLACE. Writes exactly ONE frame: Modbus function 0x10 (write multiple registers) to ANGLE_SET,
start address 1486, quantity 6, 12 data bytes. Nothing else is ever written (no SPEED_SET, no FORCE_SET, no single-register write).
Values: axes 0..4 = that axis's own ANGLE_ACT from 3 consecutive identical full reads; axis 5 (thumb rotation) = reading + 15
(right) or the fixed 7 Oct left value 976 (must be reading + 12..18). Refused unless every value is within 3 counts of the
approved --expect vector, 0..1000, never 0xFFFF. Logs STATE at full rate from >= 5 s before the write to 30 s after.
Conditions (3-read filter; temperature = 2 s median): ERROR nonzero; STATUS outside 0,1,2 after the write; any axis more than
5 counts from its pre-write reading; temperature median >= 48 (PAUSE) / >= 52 (ABORT); read loss > 1 s. Before the write any
condition cancels the write; after it there is nothing left to write, the condition is logged and the run is marked.
Raw TEMP spikes (|raw - 2 s median| >= 4 C) and 1-2 read excursions are logged as events.
   g1rs_hold.py <right|left> <out_dir> --expect a,b,c,d,e,f [--host H --port P]   (host/port only for the mock)"""
import argparse
import collections
import json
import os
import struct
import sys
import time

sys.path.insert(0, "/home/unitree/h0_tools")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import h0_ro  # noqa: E402
from h0_ro import ROModbus, STATE, CONFIG  # noqa: E402
from g1rs_read import decode  # noqa: E402
from g1rs_burst import BurstTracker  # noqa: E402
STOP_FLAG = os.environ.get("G1RS_STOP_FLAG")
CW_STOP_FILE = os.environ.get("CW_STOP_FILE")          # calibration_ws operator stop: the hold is not sent while this file exists

REG, ADDR, QTY = "ANGLE_SET", 1486, 6
ROT_OFFSET = {"right": 15, "left": 15}
LEFT_ROT_FIXED = 976
PRE_S, POST_S = float(os.environ.get("G1RS_PRE_S", 5.0)), float(os.environ.get("G1RS_POST_S", 30.0))


def build_pdu(vals):
    if len(vals) != QTY or not all(isinstance(v, int) and 0 <= v <= 1000 for v in vals):
        raise SystemExit("REFUSED values %r" % (vals,))
    pdu = struct.pack(">BHHB6H", 0x10, ADDR, QTY, 2 * QTY, *vals)
    fc, a, n, nb = struct.unpack(">BHHB", pdu[:6])
    back = list(struct.unpack(">6H", pdu[6:]))
    if fc != 0x10 or a != ADDR or a % 2 or n != QTY or nb != 12 or back != list(vals) or 0xFFFF in back:
        raise SystemExit("REFUSED frame %s" % pdu.hex(" "))
    return pdu


class Hold:
    def __init__(self, hand, out, host=None, port=None):
        h = h0_ro.HANDS[hand]
        self.hand, self.out = hand, out
        self.c = ROModbus(host or h["ip"], port or h["port"], h["unit"], timeout=1.0)
        self.c.connect()
        self.st = open(os.path.join(out, "state.jsonl"), "x")
        self.cm = open(os.path.join(out, "commands.jsonl"), "x")
        self.seq, self.last_cmd, self.t_write = 0, None, None
        self.flags, self.events = [], []
        self.run = collections.defaultdict(list)
        self.tq = [collections.deque() for _ in range(6)]
        self.spike = [None] * 6
        self.ref, self.last_ok = None, time.monotonic()
        self.status_seq = [[] for _ in range(6)]
        self.moved_logged, self.pausing = False, False

    def line(self, f, rec):
        f.write(json.dumps(rec, separators=(",", ":")) + "\n")
        f.flush()

    def ev(self, name, **kw):
        rec = {"t_mono_ns": time.monotonic_ns(), "t_host": round(time.time(), 6), "seq": self.seq, "hand": self.hand, "raw": None,
               "dec": None, "last_cmd": self.last_cmd, "event": name, **kw}
        self.seq += 1
        self.line(self.st, rec)
        self.events.append({"event": name, **{k: v for k, v in kw.items() if k != "raw"}})

    def flag(self, name, **kw):
        if name not in [f["flag"] for f in self.flags]:
            self.flags.append({"flag": name, "t_rel_write_s": None if self.t_write is None else round(time.monotonic() - self.t_write, 3), **kw})
            self.ev("FLAG_" + name, **kw)

    def cond(self, name, bad, rec, **kw):
        r = self.run[name]
        if bad:
            r.append(rec["seq"])
            if len(r) == 3:
                self.flag(name, seqs=list(r), **kw)
        elif r:
            if len(r) < 3:
                self.ev("GLITCH", rule=name, seqs=list(r))
            r.clear()

    def sample(self, event=None):
        try:
            raw = {g: self.c.read_group(STATE, g) for g in STATE}
        except Exception as e:  # noqa: BLE001
            self.ev("READ_ERROR", detail=repr(e))
            if time.monotonic() - self.last_ok > 1.0:
                self.flag("READ_LOSS_GT_1S")
            try:
                self.c.close()
                self.c.connect()
            except Exception:  # noqa: BLE001
                pass
            return None
        t = time.monotonic()
        self.last_ok = t
        rec = {"t_mono_ns": time.monotonic_ns(), "t_host": round(time.time(), 6), "seq": self.seq, "hand": self.hand, "raw": raw,
               "dec": decode(raw), "last_cmd": self.last_cmd, "event": event}
        self.seq += 1
        self.line(self.st, rec)
        d = rec["dec"]
        if not hasattr(self, "bursts"):
            self.bursts = BurstTracker(lambda name, **kw: self.ev(name, **kw), self.hand)
        bs = self.bursts.update(rec)                  # ERROR stops at 6 reads; shorter bursts are GLITCH_BURST events
        if bs:
            self.flag(bs[0], **bs[1])
        if STOP_FLAG and os.path.exists(STOP_FLAG) and self.t_write is None:
            self.flag("STOP_FLAG_FROM_OTHER_HAND_WATCH")
        if CW_STOP_FILE and os.path.exists(CW_STOP_FILE) and self.t_write is None and not any(f["flag"] == "OPERATOR_STOP_FILE" for f in self.flags):
            self.flag("OPERATOR_STOP_FILE")
        if self.t_write is not None:
            self.cond("STATUS_NOT_0_1_2", any(s not in (0, 1, 2) for s in d["status"]), rec, status=d["status"])
            for k in range(6):
                if not self.status_seq[k] or self.status_seq[k][-1][1] != d["status"][k]:
                    self.status_seq[k].append((round(t - self.t_write, 3), d["status"][k]))
        if self.ref is not None:
            dev = [abs(a - b) for a, b in zip(d["angle_act"], self.ref)]
            if max(dev) > 5 and not self.moved_logged:
                self.moved_logged = True
                self.ev("AXIS_MOVED_GT5_DURING_HOLD", dev=dev)
        for k in range(6):
            q = self.tq[k]
            q.append((t, d["temp"][k]))
            while t - q[0][0] > 2.0:
                q.popleft()
            med = sorted(x[1] for x in q)[len(q) // 2]
            if abs(d["temp"][k] - med) >= 4:
                if self.spike[k] is None:
                    self.spike[k] = {"axis": k, "first_seq": rec["seq"], "median": med, "values": [], "raw_first": raw["TEMP"]}
                self.spike[k]["values"].append(d["temp"][k])
            elif self.spike[k] is not None:
                sp = self.spike[k]
                self.ev("TEMP_SPIKE", axis=k, first_seq=sp["first_seq"], n_reads=len(sp["values"]), values=sp["values"],
                        median=sp["median"], raw=sp["raw_first"])
                self.spike[k] = None
            if t - q[0][0] >= 1.0:
                if med >= 52:
                    self.flag("TEMP_MEDIAN_GE_52", axis=k, median=med)
                elif med >= 48 and not self.pausing:
                    self.ev("TEMP_MEDIAN_GE_48", axis=k, median=med, before_write=self.t_write is None)
                    if self.t_write is None:
                        self.pausing = True
                        t_end = time.monotonic() + 300
                        while time.monotonic() < t_end:
                            self.sample()
                        self.pausing = False
        return rec

    def agree3(self, timeout=3.0):
        win, t_end = collections.deque(maxlen=3), time.monotonic() + timeout
        while time.monotonic() < t_end:
            r = self.sample()
            if r is None:
                continue
            win.append(tuple(r["dec"]["angle_act"]))
            if len(win) == 3 and len(set(win)) == 1:
                return list(win[0]), r
        raise SystemExit("NO 3 AGREEING READS in %.1f s" % timeout)

    def send(self, vals, reason):
        if CW_STOP_FILE and os.path.exists(CW_STOP_FILE):
            raise SystemExit("OPERATOR_STOP_FILE: hold not sent")
        pdu = build_pdu(vals)
        m = self.c
        m.tid = (m.tid + 1) & 0xFFFF
        adu = struct.pack(">HHHB", m.tid, 0, len(pdu) + 1, m.unit) + pdu
        t_mono, t_host = time.monotonic_ns(), time.time()
        m.sock.sendall(adu)
        rtid, pid, length, _ = struct.unpack(">HHHB", m._recv_exact(7))
        body = m._recv_exact(length - 1)
        self.t_write = t_mono / 1e9
        rec = {"t_mono_ns": t_mono, "t_host": round(t_host, 6), "seq": self.seq, "hand": self.hand, "register": REG, "address": ADDR,
               "function": "0x10", "values": vals, "raw_words": ["%04X" % v for v in vals], "adu_hex": adu.hex(" "),
               "response_hex": body.hex(" "), "reason": reason, "test": "hold"}
        self.seq += 1
        self.line(self.cm, rec)
        self.last_cmd = {"register": REG, "address": ADDR, "values": vals, "t_mono_ns": t_mono}
        print("SENT ADU", adu.hex(" "), "| response PDU", body.hex(" "))
        if rtid != m.tid or pid != 0 or body != struct.pack(">BHH", 0x10, ADDR, QTY):
            self.flag("WRITE_RESPONSE_MISMATCH", response=body.hex(" "))
        return rec


def main():
    p = argparse.ArgumentParser()
    p.add_argument("hand", choices=["right", "left"])
    p.add_argument("out")
    p.add_argument("--expect", default="none")
    p.add_argument("--send-approved", help="six approved values: sent as they are if the fresh 3-read is within 3 counts of --source-reading")
    p.add_argument("--source-reading", help="the six readings the approved values came from")
    p.add_argument("--thumb-keep", help="later holds (same power-up): thumb bend, thumb rotation = their stored ANGLE_SET targets (resend, rule 4)")
    p.add_argument("--host")
    p.add_argument("--port", type=int)
    a = p.parse_args()
    expect = None if a.expect == "none" else [int(x) for x in a.expect.split(",")]
    os.makedirs(a.out, exist_ok=True)
    h = Hold(a.hand, a.out, a.host, a.port)
    cfg0 = decode({g: h.c.read_group(CONFIG, g) for g in ("ANGLE_SET", "SPEED_SET", "FORCE_SET")})
    h.ev("CONFIG_BEFORE", config=cfg0)
    t0 = time.monotonic()
    while time.monotonic() - t0 < PRE_S:
        h.sample()
    reading, last = h.agree3()
    h.ev("PRE_WRITE_STATUS", status=last["dec"]["status"], error=last["dec"]["error"], seq=last["seq"])   # 12 Oct b: STATUS read and logged before every hold
    print("pre-write STATUS", last["dec"]["status"], "ERROR", last["dec"]["error"])
    vals = reading[:5] + [reading[5] + ROT_OFFSET[a.hand]]
    if a.send_approved:
        appr, src = [int(x) for x in a.send_approved.split(",")], [int(x) for x in a.source_reading.split(",")]
        diff = [r - q for r, q in zip(reading, src)]
        h.ev("FRESH_READING_VS_SOURCE", fresh=reading, source=src, diff=diff, approved=appr)
        if max(abs(x) for x in diff) > 3:
            h.flag("FRESH_READING_DIFFERS_FROM_SOURCE_GT3", fresh=reading, source=src, diff=diff)
        vals = appr
    if a.thumb_keep:
        keep = [int(x) for x in a.thumb_keep.split(",")]
        stored = [decode({"ANGLE_SET": h.c.read_group(CONFIG, "ANGLE_SET")})["angle_set"][4:] for _ in range(3)]
        h.ev("THUMB_KEEP", keep=keep, stored_reads=stored)
        if not all(x == keep for x in stored):
            h.flag("THUMB_STORED_TARGETS_NOT_THE_KEPT_VALUES", keep=keep, stored_reads=stored)
        vals = reading[:4] + keep
    off = vals[5] - reading[5]
    ok = (not h.flags and (expect is None or all(abs(v - e) <= 3 for v, e in zip(vals, expect))) and 12 <= off <= 18)
    h.ev("HOLD_VALUES", reading=reading, values=vals, expect=expect, rot_offset=off, ok=ok)
    print("reading", reading, "-> hold", vals, "expect", expect, "rot offset", off, "OK" if ok else "REFUSED", "flags", h.flags)
    summary = {"hand": a.hand, "register": REG, "address": ADDR, "reading": reading, "values": vals, "expect": expect, "config_before": cfg0}
    if not ok:
        summary["result"] = "ABORTED_BEFORE_WRITE"
    else:
        h.ref = list(reading)
        h.send(vals, "hold in place after power-up: axes 0-4 own ANGLE_ACT (3 agreeing reads), thumb rotation %+d" % off)
        h.ev("CONFIG_AFTER_WRITE", config=decode({g: h.c.read_group(CONFIG, g) for g in ("ANGLE_SET", "SPEED_SET", "FORCE_SET")}))
        while time.monotonic() - h.t_write < POST_S:
            h.sample()
        cfg1 = decode({g: h.c.read_group(CONFIG, g) for g in ("ANGLE_SET", "SPEED_SET", "FORCE_SET")})
        h.ev("CONFIG_END", config=cfg1)
        end = h.sample("END")
        summary.update({"result": "DONE" if not h.flags else "DONE_WITH_FLAGS", "angle_set_readback": cfg1["angle_set"],
                        "angle_set_equals_sent": cfg1["angle_set"] == vals, "status_sequence_s_from_write": h.status_seq,
                        "angle_end": end["dec"]["angle_act"] if end else None, "status_end": end["dec"]["status"] if end else None,
                        "current_end": end["dec"]["current"] if end else None})
    summary.update({"flags": h.flags, "events": [e for e in h.events if e["event"] not in ("HOLD_VALUES",)][-200:]})
    json.dump(summary, open(os.path.join(a.out, "summary.json"), "w"), indent=1)
    print("RESULT", summary["result"], "flags", [f["flag"] for f in h.flags])
    h.c.close()


if __name__ == "__main__":
    main()
