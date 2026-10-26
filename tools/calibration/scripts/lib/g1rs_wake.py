#!/usr/bin/env python3
"""g1rs_20261012 Part B wake tests. Writes only:
  * SPEED_SET 500 on a non-thumb finger of the tested hand that is not already at 500 (function 0x06, one even address 1522 + 2 * axis
    per write), right after the fresh Part B hold and before the 10 s wait; never between wake writes; thumb speeds never.
  * Wake writes: ONE register, function 0x06, ANGLE_SET of one axis, address 1486 + 2 * axis, exactly as the test table:
      resend test: 3 writes of the value in that axis's ANGLE_SET register (3 agreeing reads); guard: it must equal the value the
                   hand's latest hold wrote for that axis, otherwise nothing is sent and the run stops (RESEND_GUARD_FAILURE);
      moving test: setup 650, then 950, 650, 950, 650, 950, 650, then return = the hold value of that finger (8 writes).
  Thumbs (axes 4, 5) get resends only. Never -1, never an odd address, never below the hold value of a finger.
Before each wake write all six axes of the tested hand are settled (STATUS 2, or ANGLE_ACT unchanged for >= 0.3 s). After each write
both hands are logged at full rate (STATE groups, one sampler thread per hand on its single connection) for >= 3 s and until all six
axes of the tested hand show STATUS 2. Motion of non-written axes is the result, not a fault.
Stop (no further write), 3-read filter except temperature (2 s median): ERROR nonzero (either hand); STATUS outside 0,1,2 (tested
hand, after its hold); a non-written finger more than 30 counts from where it was before the write; a thumb axis more than 10 counts
from its ANGLE_ACT at the latest hold while drawing current (driven move), or more than 20 counts with current 0 (sag; a thumb more
than 10 counts away with current 0 is logged as a THUMB_SAG event); no motion within 2 s of a wake write whose target is more than
20 counts from the finger; the written finger more than 10 counts short of target at the end of its window; temperature
median >= 52 (>= 48: pause 5 min, reads only); resend guard failure; read loss > 1 s; ANGLE_SET or SPEED_SET readback not what was
written (another writer); hold older than 30 s at the start (rule 6); battery <= 20 % (or 50 min after the session's first write if
unreadable); a second rt/lowcmd writer or nonzero kp, kd or tau (arm motion is an event only); stored ANGLE_SET reads that do not give
3 agreeing reads after 3 extra reads (each extra read logged as GLITCH); anything unexpected: no settle in 15 s before a write, no
STATUS 2 on all six axes within 15 s after it, a write response that is not the echo.
   g1rs_wake.py <hand> <tests e.g. R0,R1,R2,R3> <hold_summary.json> <out_dir> [--host H --port P --ohost H --oport P --no-body]"""
import argparse
import collections
import json
import os
import struct
import sys
import threading
import time

sys.path.insert(0, "/home/unitree/h0_tools")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import h0_ro  # noqa: E402
from h0_ro import ROModbus, STATE, CONFIG, i16  # noqa: E402
from g1rs_read import decode  # noqa: E402
from g1rs_burst import BurstTracker  # noqa: E402

A_ADDR, S_ADDR = 1486, 1522
TESTS = {"R0": ("right", 3, "resend"), "R1": ("right", 0, "move"), "R2": ("right", 4, "resend"), "R3": ("right", 5, "resend"),
         "L0": ("left", 3, "resend"), "L1": ("left", 0, "move"), "L2": ("left", 1, "move"), "L3": ("left", 2, "move"),
         "L4": ("left", 3, "move"), "L5": ("left", 4, "resend"), "L6": ("left", 5, "resend")}
MOVE_SEQ = [("setup", 650), ("moving", 950), ("moving", 650), ("moving", 950), ("moving", 650), ("moving", 950), ("moving", 650)]
OBS_MIN_S, SETTLE_MAX_S, WAIT_AFTER_SPEED_S = float(os.environ.get("G1RS_OBS_S", 3.0)), 15.0, float(os.environ.get("G1RS_WAIT_S", 10.0))
CLOSE_SOC = 20
FIRST_WRITE_EPOCH = float(os.environ.get("G1RS_FIRST_WRITE_EPOCH", 0)) or None


class Stop(Exception):
    def __init__(self, reason, **kw):
        super().__init__(reason)
        self.reason, self.detail = reason, kw


class Hand:
    """One connection per hand; the sampler thread and the writer share it under a lock."""

    def __init__(self, name, host, port, log, tested):
        h = h0_ro.HANDS[name]
        self.name, self.log, self.tested = name, log, tested
        self.c = ROModbus(host or h["ip"], port or h["port"], h["unit"], timeout=1.0)
        self.c.connect()
        self.lock = threading.Lock()
        self.recent = collections.deque(maxlen=4000)
        self.tq = [collections.deque() for _ in range(6)]
        self.spike = [None] * 6
        self.last_ok = time.monotonic()
        self.stop_reason = None
        self.runs = collections.defaultdict(list)
        self.limits = None          # set by the tester: callable(rec) -> list of (name, bad, detail)
        self.running = True
        self.th = threading.Thread(target=self.loop, daemon=True)

    def cfg(self, name):
        with self.lock:
            return i16(self.c.read_group(CONFIG, name))

    def loop(self):
        while self.running:
            try:
                with self.lock:
                    raw = {g: self.c.read_group(STATE, g) for g in STATE}
            except Exception as e:  # noqa: BLE001
                self.log.ev(self.name, "READ_ERROR", detail=repr(e))
                if time.monotonic() - self.last_ok > 1.0 and not self.stop_reason:
                    self.stop_reason = ("READ_LOSS_GT_1S", {})
                try:
                    with self.lock:
                        self.c.close()
                        self.c.connect()
                except Exception:  # noqa: BLE001
                    time.sleep(0.05)
                continue
            t = time.monotonic()
            self.last_ok = t
            rec = self.log.sample(self.name, raw)
            rec["t"] = t
            self.recent.append(rec)
            try:
                self.check(rec, t)
            except Stop as s:
                if not self.stop_reason:
                    self.stop_reason = (s.reason, s.detail)

    def cond(self, name, bad, rec, **kw):
        r = self.runs[name]
        if bad:
            r.append(rec["seq"])
            if len(r) >= 3:
                raise Stop(name, hand=self.name, seqs=list(r), **kw)
        elif r:
            self.log.ev(self.name, "GLITCH", rule=name, seqs=list(r))
            r.clear()

    def check(self, rec, t):
        d = rec["dec"]
        if not hasattr(self, "bursts"):
            self.bursts = BurstTracker(lambda name, **kw: self.log.ev(self.name, name, **kw), self.name)
        bs = self.bursts.update(rec)                  # ERROR stops at 6 reads; shorter bursts are GLITCH_BURST events
        if bs:
            raise Stop(bs[0], **bs[1])
        if self.tested:
            self.cond("STATUS_NOT_0_1_2", any(s not in (0, 1, 2) for s in d["status"]), rec, status=d["status"])
            if self.limits:
                for name, bad, detail in self.limits(d):
                    self.cond(name, bad, rec, **detail)
        for k in range(6):
            q = self.tq[k]
            q.append((t, d["temp"][k]))
            while t - q[0][0] > 2.0:
                q.popleft()
            med = sorted(x[1] for x in q)[len(q) // 2]
            if abs(d["temp"][k] - med) >= 4:
                if self.spike[k] is None:
                    self.spike[k] = {"first_seq": rec["seq"], "median": med, "values": [], "raw_first": rec["raw"]["TEMP"]}
                self.spike[k]["values"].append(d["temp"][k])
            elif self.spike[k] is not None:
                sp = self.spike[k]
                self.log.ev(self.name, "TEMP_SPIKE", axis=k, first_seq=sp["first_seq"], n_reads=len(sp["values"]), values=sp["values"],
                            median=sp["median"], raw=sp["raw_first"])
                self.spike[k] = None
            if t - q[0][0] >= 1.0 and med >= 52:
                raise Stop("TEMP_MEDIAN_GE_52", hand=self.name, axis=k, median=med)
            if t - q[0][0] >= 1.0 and med >= 48:
                self.log.temp48 = (self.name, k, med)


class Log:
    def __init__(self, out):
        self.st = open(os.path.join(out, "state.jsonl"), "x")
        self.cm = open(os.path.join(out, "commands.jsonl"), "x")
        self.lk = threading.Lock()
        self.seq, self.last_cmd, self.temp48 = 0, None, None

    def _w(self, f, rec):
        with self.lk:
            rec["seq"] = self.seq
            self.seq += 1
            f.write(json.dumps(rec, separators=(",", ":")) + "\n")
            f.flush()
        return rec

    def sample(self, hand, raw):
        return self._w(self.st, {"t_mono_ns": time.monotonic_ns(), "t_host": round(time.time(), 6), "seq": None, "hand": hand, "raw": raw,
                                 "dec": decode(raw), "last_cmd": self.last_cmd, "event": None})

    def ev(self, hand, name, **kw):
        return self._w(self.st, {"t_mono_ns": time.monotonic_ns(), "t_host": round(time.time(), 6), "seq": None, "hand": hand, "raw": None,
                                 "dec": None, "last_cmd": self.last_cmd, "event": name, **kw})

    def cmd(self, rec):
        rec = self._w(self.cm, rec)
        self.last_cmd = {k: rec[k] for k in ("t_mono_ns", "register", "address", "values", "test")}
        return rec


class Tester:
    def __init__(self, hand, hold, hold_reading, log, H, O, body):
        self.hand, self.hold, self.hold_reading, self.log, self.H, self.O, self.body = hand, hold, hold_reading, log, H, O, body
        self.before = None          # ANGLE_ACT of the tested hand just before the current write
        self.written = None
        self.writes = []
        self.sag = {4: False, 5: False}
        H.limits = self.limits

    def limits(self, d):
        a = d["angle_act"]
        out = []
        if self.before is not None:
            dev = [abs(a[k] - self.before[k]) if (k < 4 and k != self.written) else 0 for k in range(6)]
            out.append(("NON_WRITTEN_FINGER_GT30", max(dev) > 30, {"dev": dev}))
        cur = d["current"]
        for k in (4, 5):
            dev = abs(a[k] - self.hold_reading[k])
            out.append(("THUMB_DRIVEN_GT10_FROM_HOLD_READING", dev > 10 and cur[k] != 0, {"axis": k, "dev": dev, "current": cur[k], "angle": a[k]}))
            out.append(("THUMB_SAG_GT20_FROM_HOLD_READING", dev > 20, {"axis": k, "dev": dev, "current": cur[k], "angle": a[k]}))
            sag = dev > 10 and cur[k] == 0
            if sag and not self.sag[k]:
                self.log.ev(self.hand, "THUMB_SAG", axis=k, dev=dev, angle=a[k], hold_reading=self.hold_reading[k])
            elif self.sag[k] and dev <= 10:
                self.log.ev(self.hand, "THUMB_SAG_END", axis=k, dev=dev, angle=a[k])
            self.sag[k] = sag or (self.sag[k] and dev > 10)
        return out

    def poll(self):
        stop = os.environ.get("CW_STOP_FILE")             # calibration_ws operator stop file
        if stop and os.path.exists(stop):
            raise Stop("OPERATOR_STOP_FILE", file=stop)
        flag = os.environ.get("G1RS_STOP_FLAG")
        if flag and os.path.exists(flag):
            raise Stop("STOP_FLAG_FROM_OTHER_HAND_WATCH", flag=open(flag).read()[:300])
        if self.body is not None:
            for name, kw in self.body.pop_events():
                self.log.ev(self.hand, "BODY_" + name, **kw)      # arm motion, stale lowstate: events only
        for h in (self.H, self.O):
            if h is not None and h.stop_reason:
                raise Stop(h.stop_reason[0], **h.stop_reason[1])
        if self.log.temp48:
            hand, k, med = self.log.temp48
            self.log.temp48 = None
            self.log.ev(hand, "PAUSE_TEMP_48", axis=k, median=med)
            time.sleep(300)
            self.log.ev(hand, "PAUSE_END")

    def gate(self):
        soc = None if self.body is None else self.body.battery()
        if soc is not None and soc <= CLOSE_SOC:
            raise Stop("BATTERY_AT_OR_BELOW_20", soc=soc)
        if soc is None and FIRST_WRITE_EPOCH and time.time() - FIRST_WRITE_EPOCH > 3000:
            raise Stop("BATTERY_UNREADABLE_50_MIN_AFTER_FIRST_WRITE")
        if self.body is not None:
            b = self.body.check()                                 # only a second lowcmd writer or nonzero kp, kd, tau
            if b:
                raise Stop(b[0], detail=b[1])

    def wait_settled(self):
        """All six axes STATUS 2, or ANGLE_ACT unchanged for >= 0.3 s."""
        t_end = time.monotonic() + SETTLE_MAX_S
        while time.monotonic() < t_end:
            self.poll()
            rec = list(self.H.recent)
            if rec:
                now = rec[-1]["t"]
                win = [r for r in rec if now - r["t"] <= 0.3]
                if win and now - win[0]["t"] >= 0.28:
                    ok = all(rec[-1]["dec"]["status"][k] == 2 or len({r["dec"]["angle_act"][k] for r in win}) == 1 for k in range(6))
                    if ok:
                        return rec[-1]
            time.sleep(0.01)
        raise Stop("UNEXPECTED_NOT_SETTLED_15S")

    def raw_write(self, pdu, rec):
        stop = os.environ.get("CW_STOP_FILE")
        if stop and os.path.exists(stop):
            raise Stop("OPERATOR_STOP_FILE", file=stop)
        h = self.H
        with h.lock:
            m = h.c
            m.tid = (m.tid + 1) & 0xFFFF
            tid = m.tid
            adu = struct.pack(">HHHB", tid, 0, len(pdu) + 1, m.unit) + pdu
            t_mono, t_host = time.monotonic_ns(), time.time()
            try:
                m.sock.sendall(adu)
                rtid, pid, length, _ = struct.unpack(">HHHB", m._recv_exact(7))
                body = m._recv_exact(length - 1)
            except Exception as e:  # noqa: BLE001
                raise Stop("MODBUS_WRITE_FAILED", detail=repr(e))
        rec = self.log.cmd({"t_mono_ns": t_mono, "t_host": round(t_host, 6), "seq": None, "hand": self.hand, "function": "0x%02X" % pdu[0], **rec,
                            "adu_hex": adu.hex(" "), "response_hex": body.hex(" ")})
        if rtid != tid or pid != 0 or body != pdu:
            raise Stop("UNEXPECTED_WRITE_RESPONSE", response=body.hex(" "))
        return rec

    def speeds(self):
        sp = self.H.cfg("SPEED_SET")
        self.speed_before = list(sp)
        self.log.ev(self.hand, "SPEED_SET_BEFORE", speed_set=sp)
        for k in range(4):
            if sp[k] != 500:
                self.gate()
                addr = S_ADDR + 2 * k
                self.raw_write(struct.pack(">BHH", 0x06, addr, 500), {"register": "SPEED_SET", "address": addr, "values": [500], "raw_words": ["01F4"],
                                                                     "reason": "Part B speed 500 after the fresh hold", "test": "speed"})
                after = self.H.cfg("SPEED_SET")
                exp = list(sp)
                exp[k] = 500
                if after != exp:
                    raise Stop("SPEED_SET_READBACK_MISMATCH", before=sp, after=after, expected=exp)
                sp = after
        self.log.ev(self.hand, "SPEED_SET_AFTER", speed_set=sp)

    def angle_set_agreeing(self):
        r = [tuple(self.H.cfg("ANGLE_SET")) for _ in range(3)]
        extra = 0
        while not any(r.count(v) >= 3 for v in r):
            if extra == 3:
                raise Stop("ANGLE_SET_NO_3_AGREEING_READS", reads=[list(v) for v in r])
            extra += 1
            r.append(tuple(self.H.cfg("ANGLE_SET")))
            self.log.ev(self.hand, "GLITCH", rule="ANGLE_SET_READS_DISAGREE", extra_read=extra, reads=[list(v) for v in r])
        return list(max(set(r), key=r.count))

    def wake(self, test, axis, value, kind, n):
        self.gate()
        last = self.wait_settled()
        aset0 = self.angle_set_agreeing()
        if kind == "resend":
            if aset0[axis] != self.hold[axis]:
                raise Stop("RESEND_GUARD_FAILURE", axis=axis, angle_set=aset0[axis], hold=self.hold[axis])
            value = aset0[axis]
        if not (0 <= value <= 1000) or (axis < 4 and value < self.hold[axis]) or (axis >= 4 and kind != "resend"):
            raise Stop("REFUSED_VALUE", axis=axis, value=value)
        self.before, self.written = list(last["dec"]["angle_act"]), axis
        addr = A_ADDR + 2 * axis
        label = "%s w%d %s %d" % (test, n, kind, value)
        rec = self.raw_write(struct.pack(">BHH", 0x06, addr, value), {"register": "ANGLE_SET", "address": addr, "values": [value],
                                                                      "raw_words": ["%04X" % value], "reason": "Part B wake write (%s)" % kind,
                                                                      "test": label, "kind": kind, "axis": axis})
        t_w = rec["t_mono_ns"] / 1e9
        need_motion = kind in ("setup", "moving", "return") and abs(value - self.before[axis]) > 20
        aset1 = self.angle_set_agreeing()
        exp = list(aset0)
        exp[axis] = value
        if aset1 != exp:
            raise Stop("ANGLE_SET_READBACK_MISMATCH", before=aset0, after=aset1, expected=exp)
        start, direction, moved = self.before[axis], (value > self.before[axis]) - (value < self.before[axis]), not need_motion
        while True:
            self.poll()
            now = time.monotonic()
            rec2 = self.H.recent[-1] if self.H.recent else None
            if not moved and rec2 and rec2["t"] > t_w and (rec2["dec"]["angle_act"][axis] - start) * direction >= 3:
                moved = True
            if not moved and now - t_w > 2.0:
                raise Stop("UNEXPECTED_NO_MOTION_2S", axis=axis, target=value, start=start)
            if rec2 and rec2["t"] > t_w and now - t_w >= OBS_MIN_S and all(s == 2 for s in rec2["dec"]["status"]):
                break
            if now - t_w > SETTLE_MAX_S:
                raise Stop("UNEXPECTED_NOT_ALL_STATUS2_15S", status=rec2["dec"]["status"] if rec2 else None)
            time.sleep(0.01)
        a = rec2["dec"]["angle_act"][axis]
        self.writes.append({"test": test, "n": n, "kind": kind, "axis": axis, "address": addr, "value": value, "t_write_mono": t_w,
                            "t_window_end_mono": rec2["t"], "angle_end": a, "angle_set_before": aset0})
        if kind in ("setup", "moving", "return") and abs(a - value) > 10:
            raise Stop("SHORT_OF_TARGET_GT10", axis=axis, target=value, now=a)
        self.before = None

    def run(self, tests):
        for test in tests:
            hand, axis, kind = TESTS[test]
            if hand != self.hand:
                raise Stop("REFUSED_TEST_OTHER_HAND", test=test)
            self.log.ev(self.hand, "TEST_START", test=test, axis=axis, kind=kind)
            if kind == "resend":
                for n in range(3):
                    self.wake(test, axis, None, "resend", n + 1)
            else:
                seq = MOVE_SEQ + [("return", self.hold[axis])]
                for n, (k2, v) in enumerate(seq):
                    self.wake(test, axis, v, k2, n + 1)
            self.log.ev(self.hand, "TEST_END", test=test)


def wake_table(out):
    """Offline: per write, per axis: STATUS before / after, first STATUS change and first motion time, counts moved, peak current."""
    rows = [json.loads(l) for l in open(os.path.join(out, "state.jsonl"))]
    s = json.load(open(os.path.join(out, "summary.json")))
    hand = s["hand"]
    smp = [r for r in rows if r.get("dec") and r["hand"] == hand]
    oth = [r for r in rows if r.get("dec") and r["hand"] != hand]
    md, js = ["# WAKE_TABLE %s (%s hand)" % (os.path.basename(out.rstrip("/")), hand), "",
              "Per wake write and axis (0 little, 1 ring, 2 middle, 3 index, 4 thumb bend, 5 thumb rotation): STATUS before -> after; first "
              "STATUS change (s after the write); first motion >= 2 counts (s); max counts moved from the value before the write; peak |current| "
              "mA; woke = STATUS change or motion >= 2 counts or current. Window: write to all six STATUS 2 and >= 3 s.", ""], []
    by_test = collections.OrderedDict()
    for w in s["writes"]:
        by_test.setdefault(w["test"], []).append(w)
    for test, ws in by_test.items():
        md += ["## %s: axis %d, %s" % (test, ws[0]["axis"], ws[0]["kind"] if ws[0]["kind"] == "resend" else "setup, 6 moving, return"), "",
               "| write | value | " + " | ".join("ax%d" % k for k in range(6)) + " | woke | other hand max moved / current |", "|---|---|" + "---|" * 6 + "---|---|"]
        for w in ws:
            t0, t1 = w["t_write_mono"], w["t_window_end_mono"]
            pre = [r for r in smp if r["t_mono_ns"] / 1e9 < t0]
            win = [r for r in smp if t0 <= r["t_mono_ns"] / 1e9 <= t1]
            if not pre or not win:
                continue
            b = pre[-1]["dec"]
            cells, woke = [], []
            per = []
            for k in range(6):
                st_ch = next((r["t_mono_ns"] / 1e9 - t0 for r in win if r["dec"]["status"][k] != b["status"][k]), None)
                mv = next((r["t_mono_ns"] / 1e9 - t0 for r in win if abs(r["dec"]["angle_act"][k] - b["angle_act"][k]) >= 2), None)
                cnt = max(abs(r["dec"]["angle_act"][k] - b["angle_act"][k]) for r in win)
                cur = max(abs(r["dec"]["current"][k]) for r in win)
                st_after = win[-1]["dec"]["status"][k]
                seen = sorted({r["dec"]["status"][k] for r in win})
                w_ = st_ch is not None or mv is not None or cur > 0
                if w_:
                    woke.append(k)
                per.append({"axis": k, "status_before": b["status"][k], "status_after": st_after, "status_seen": seen,
                            "first_status_change_s": None if st_ch is None else round(st_ch, 3), "first_motion_s": None if mv is None else round(mv, 3),
                            "counts_moved": cnt, "peak_current": cur})
                cells.append("%d->%d %s %s %d %d" % (b["status"][k], st_after, "-" if st_ch is None else "%.3f" % st_ch, "-" if mv is None else "%.3f" % mv, cnt, cur))
            ow = [r for r in oth if t0 <= r["t_mono_ns"] / 1e9 <= t1]
            obp = [r for r in oth if r["t_mono_ns"] / 1e9 < t0]
            omv = max((abs(r["dec"]["angle_act"][k] - obp[-1]["dec"]["angle_act"][k]) for r in ow for k in range(6)), default=None) if obp else None
            ocur = max((abs(c) for r in ow for c in r["dec"]["current"]), default=None)
            md.append("| %d %s | %d | %s | %s | %s / %s |" % (w["n"], w["kind"], w["value"], " | ".join(cells), woke, omv, ocur))
            js.append({**{k: w[k] for k in ("test", "n", "kind", "axis", "value")}, "axes": per, "woke": woke, "other_hand_max_moved": omv, "other_hand_peak_current": ocur})
        md.append("")
    open(os.path.join(out, "WAKE_TABLE.md"), "w").write("\n".join(md) + "\n")
    json.dump(js, open(os.path.join(out, "wake_table.json"), "w"), indent=1)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("hand", choices=["right", "left"])
    p.add_argument("tests")
    p.add_argument("hold_summary")
    p.add_argument("out")
    p.add_argument("--host")
    p.add_argument("--port", type=int)
    p.add_argument("--ohost")
    p.add_argument("--oport", type=int)
    p.add_argument("--no-body", action="store_true")
    p.add_argument("--no-other", action="store_true")
    a = p.parse_args()
    tests = a.tests.split(",")
    hs = json.load(open(a.hold_summary))
    if hs["hand"] != a.hand or hs.get("result") != "DONE" or not hs.get("angle_set_equals_sent"):
        raise SystemExit("hold summary is not a clean hold of this hand")
    hold_t = json.loads(open(os.path.join(os.path.dirname(os.path.abspath(a.hold_summary)), "commands.jsonl")).readline())["t_mono_ns"] / 1e9
    os.makedirs(a.out, exist_ok=True)
    log = Log(a.out)
    body = None
    if not a.no_body:
        import g1rs_body
        body = g1rs_body.BodyMonitor()
        body.start()
        time.sleep(2.0)
        body.set_baseline()
    other = "left" if a.hand == "right" else "right"
    H = Hand(a.hand, a.host, a.port, log, True)
    O = None if a.no_other else Hand(other, a.ohost, a.oport, log, False)
    for h in (H, O):
        if h:
            h.th.start()
    t = Tester(a.hand, hs["values"], hs["reading"], log, H, O, body)
    result, reason, detail = "DONE", None, None
    try:
        age = time.monotonic() - hold_t
        log.ev(a.hand, "HOLD_AGE_AT_START", age_s=round(age, 3), hold=hs["values"], hold_reading=hs["reading"])
        if age > 30.0:
            raise Stop("HOLD_OLDER_THAN_30S", age_s=round(age, 3))
        if t.H.cfg("ANGLE_SET") != hs["values"]:
            raise Stop("ANGLE_SET_NOT_HOLD_AT_START")
        t.speeds()
        t_end = time.monotonic() + WAIT_AFTER_SPEED_S
        while time.monotonic() < t_end:
            t.poll()
            time.sleep(0.05)
        t.run(tests)
        time.sleep(1.0)
        t.poll()
        sp = t.H.cfg("SPEED_SET")                       # calibration_ws: start speeds back after the last wake write
        for k in range(4):
            if sp[k] != t.speed_before[k]:
                addr = S_ADDR + 2 * k
                t.raw_write(struct.pack(">BHH", 0x06, addr, t.speed_before[k]), {"register": "SPEED_SET", "address": addr, "values": [t.speed_before[k]],
                            "raw_words": ["%04X" % t.speed_before[k]], "reason": "start speed back after the wake tests", "test": "speed_restore"})
        log.ev(a.hand, "SPEED_RESTORED", speed_set=t.H.cfg("SPEED_SET"))
    except Stop as s:
        result, reason, detail = "ABORTED", s.reason, s.detail
        log.ev(a.hand, "STOP_" + s.reason, **{k: v for k, v in s.detail.items() if k != "raw"})
        time.sleep(2.0)
    except Exception as e:  # noqa: BLE001
        result, reason, detail = "ABORTED", "EXCEPTION", repr(e)
        log.ev(a.hand, "STOP_EXCEPTION", detail=repr(e))
    for h in (H, O):
        if h:
            h.running = False
            h.th.join(timeout=2)
    summ = {"hand": a.hand, "tests": tests, "hold": hs["values"], "hold_reading": hs["reading"], "result": result, "reason": reason, "detail": detail,
            "writes": t.writes, "speed_set_end": H.cfg("SPEED_SET"), "angle_set_end": H.cfg("ANGLE_SET"),
            "soc_end": None if body is None else body.battery()}
    json.dump(summ, open(os.path.join(a.out, "summary.json"), "w"), indent=1, default=str)
    try:
        wake_table(a.out)
    except Exception as e:  # noqa: BLE001
        print("wake table failed", repr(e))
    print("RESULT", result, reason or "", "writes", len(t.writes))
    if body is not None:
        body.close()


if __name__ == "__main__":
    main()
