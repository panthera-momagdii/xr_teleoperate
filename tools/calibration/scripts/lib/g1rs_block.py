#!/usr/bin/env python3
"""g1rs_20261012 copy of the 9 Oct finger timing block tool (g1rs_20261009/tools/g1rs_block.py, sha 300476e8), set for Part A:
stage "s" = SPEED_SET 500, hold -> 650, 6 x (950, 650) at 500, SPEED_SET 1000, 6 x (950, 650) at 1000, SPEED_SET 500, back to the
hold value (STATUS 2), speed left at 500. Stop list = Mohammed's ABORT list for Part A (no rules of the agent's own).
The 9 Oct description follows. Writes only:
  * ANGLE_SET: one six-value frame, function 0x10, address 1486, quantity 6; axes not under test = their hold values
    (the hold vector read back from ANGLE_SET at block start must equal the hold run's vector).
  * SPEED_SET of the finger under test only: function 0x06, one even address 1522 + 2 * axis, values 100 / 500 / 1000.
Never -1, never an odd address, never a thumb speed, never below the hold value (all targets >= hold value of that finger).
Stages (as h2_motion.full_block, rest = hold value): a speed 100: hold, 700, 850, 1000, hold, 3 x (1000, hold), dwell 2 s;
b speeds 100 / 500 / 1000: 650, then 3 x (950, 650), dwell 1.5 s; c speed 100: 800, steps +-5 / 10 / 20 and back (no follow
check), dwell 2 s; d 3 x (900 -> 800), 3 x (700 -> 800); e hold value at speed 100, STATUS 2, speed 1000.
Stop (no further write) on: ERROR nonzero, STATUS outside 0,1,2, non-commanded axis > 5 counts from block start, |current| of the
finger under test > 200 more than 2 s after it stopped (all 3 consecutive reads; non-test axis current is logged as NONTEST_CURRENT
events only, Mohammed 2026-10-09); temperature 2 s median >= 52 (>= 48: pause 5 min, reads
only); finger > 10 counts short of target after its dwell; no follow in 2 s; no settle in 12 s; read loss > 1 s; ANGLE_SET or
SPEED_SET readback not what was sent (another writer); battery <= 20 % or 22:00 PC2; arm or wrist motion (body monitor).
   g1rs_block.py <hand> <axis> <stages: abcde | cde> <hold_summary.json> <out_dir> [--host H --port P --no-body]"""
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
from h0_ro import ROModbus, STATE, CONFIG, i16  # noqa: E402
from g1rs_read import decode  # noqa: E402
from g1rs_burst import BurstTracker  # noqa: E402
STOP_FLAG = os.environ.get("G1RS_STOP_FLAG")

A_ADDR, S_ADDR = 1486, 1522
SPEEDS = (100, 500, 1000)
REPS = 6
# calibration_ws (2026-10-13): stage letters b (blip), s (steps), w (sweep), configured by the wrapper through the environment
CW_SPEEDS = [int(x) for x in os.environ.get("CW_SPEEDS", "500,1000").split(",")]
CW_REPS = int(os.environ.get("CW_REPS", 6))
CW_RANGE = [int(x) for x in os.environ.get("CW_RANGE", "650,950").split(",")]
CW_SWEEP_SPEED = int(os.environ.get("CW_SWEEP_SPEED", 100))
CW_BLIP = int(os.environ.get("CW_BLIP", 10))
# calibration_ws P3 stages (LOCKED by the wrappers until P3): r (below rest), p (press)
CW_FLOOR = int(os.environ.get("CW_FLOOR", 0))
CW_STEP = int(os.environ.get("CW_STEP", 25))
CW_PAUSE = float(os.environ.get("CW_PAUSE", 2.0))
CW_SPEED_P3 = int(os.environ.get("CW_SPEED_P3", 100))
CW_LEVELS = [int(x) for x in os.environ.get("CW_LEVELS", "200").split(",")]
CW_CLOSE_TO = int(os.environ.get("CW_CLOSE_TO", 0))
CW_GATE_TIMEOUT = float(os.environ.get("CW_GATE_TIMEOUT", 600))
CW_GATE = os.environ.get("CW_GATE", "first")
CW_FORCE_BELOW_REST = int(os.environ.get("CW_FORCE_BELOW_REST", 0))   # below_rest: FORCE_SET = rest + this (0 = leave FORCE_SET)
CW_THUMB_STEP = int(os.environ.get("CW_THUMB_STEP", 50))              # thumb stage t: one step and back
CW_FINGERS_OPEN = int(os.environ.get("CW_FINGERS_OPEN", 950))         # thumb rotation: fingers 0..3 opened to this first
CW_STOP_FILE = os.environ.get("CW_STOP_FILE")
CW_RECORD_TOP = int(os.environ.get("CW_RECORD_TOP", 950))            # record stage R: slow pass up to this and back
CW_BLIP_DWELL = float(os.environ.get("CW_BLIP_DWELL", 2.0))           # record stage R: blip held this long (video marker)
CW_F0_OVERRIDE = int(os.environ["CW_F0_OVERRIDE"]) if os.environ.get("CW_F0_OVERRIDE") else None   # press: rest force given (closing-pass reading)
CW_BLIP_GATE = os.environ.get("CW_BLIP_GATE") == "1"
CW_TARGETS = [int(x) for x in os.environ.get("CW_TARGETS", "").split(",") if x]    # position press (stage q): angle targets past contact
CW_SAFETY = int(os.environ.get("CW_SAFETY", 0))                                     # position press: limit = rest force + this (g)                 # press / below_rest: wait for the operator after the blip
CW_FLOOR_MARGIN = int(os.environ.get("CW_FLOOR_MARGIN", 15))         # record stage R: floor pass ends this far above the floor                         # operator stop: if this file exists no further write goes out                     # press: operator gate after level 1 only, or after every level
F_ADDR = 1498
HOLD_MAX_AGE_S = 30.0
FIRST_WRITE_EPOCH = float(os.environ.get("G1RS_FIRST_WRITE_EPOCH", 0)) or None
DW_A, DW_B, DW_CD = [float(os.environ.get("G1RS_DW", d)) for d in (2.0, 1.5, 2.0)]
FOLLOW_MIN, FOLLOW_S, SETTLE_S = 20, 2.0, 15.0
CLOSE_SOC = int(os.environ.get("G1RS_CLOSE_SOC", 20))
AXIS = {0: "little", 1: "ring", 2: "middle", 3: "index", 4: "thumb bend", 5: "thumb rotation"}
ROT_SETTLE = 15                  # thumb rotation settles about 15 counts below its target (12 Oct b, 13 Oct holds)


class Stop(Exception):
    def __init__(self, reason, **kw):
        super().__init__(reason)
        self.reason, self.detail = reason, kw


class Block:
    def __init__(self, hand, axis, hold, out, host=None, port=None, body=None):
        h = h0_ro.HANDS[hand]
        self.hand, self.axis, self.hold, self.body = hand, axis, list(hold), body
        self.c = ROModbus(host or h["ip"], port or h["port"], h["unit"], timeout=1.0)
        self.c.connect()
        self.st = open(os.path.join(out, "state.jsonl"), "x")
        self.cm = open(os.path.join(out, "commands.jsonl"), "x")
        self.seq, self.last_cmd, self.events, self.moves, self.dwells = 0, None, [], [], []
        self.runs = collections.defaultdict(list)
        self.tq = [collections.deque() for _ in range(6)]
        self.spike = [None] * 6
        self.pre = None
        self.t_change, self.ref_a = [time.monotonic()] * 6, None
        self.last_ok = time.monotonic()
        self.peak_current, self.max_dev, self.temp_max = [0] * 6, [0] * 6, [0] * 6
        self.vec, self.speed = list(hold), None
        self.first_frames = []
        self.ep = [None] * 6
        self.return_to = None
        self.hold_reading, self.sag = None, {4: False, 5: False}
        self.base, self.stopping = None, False          # base: six-value vector the test axis is written into (default: hold)

    # ------------------------------------------------------------- logging and reads
    def line(self, f, rec):
        f.write(json.dumps(rec, separators=(",", ":")) + "\n")
        f.flush()

    def ev(self, name, **kw):
        self.line(self.st, {"t_mono_ns": time.monotonic_ns(), "t_host": round(time.time(), 6), "seq": self.seq, "hand": self.hand,
                            "raw": None, "dec": None, "last_cmd": self.last_cmd, "event": name, **kw})
        self.seq += 1
        self.events.append({"event": name, **{k: v for k, v in kw.items() if k != "raw"}})

    def cond(self, name, bad, seq, **kw):
        r = self.runs[name]
        if bad:
            r.append(seq)
            if len(r) >= 3:
                raise Stop(name, seqs=list(r), **kw)
        elif r:
            self.ev("GLITCH", rule=name, seqs=list(r))
            r.clear()

    def cfg(self, name):
        try:
            return i16(self.c.read_group(CONFIG, name))
        except Exception as e:  # noqa: BLE001
            raise Stop("MODBUS_CONFIG_READ", detail=repr(e))

    def sample(self, event=None):
        try:
            raw = {g: self.c.read_group(STATE, g) for g in STATE}
        except Exception as e:  # noqa: BLE001
            self.ev("READ_ERROR", detail=repr(e))
            if time.monotonic() - self.last_ok > 1.0:
                raise Stop("READ_LOSS_GT_1S")
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
        a, cur = d["angle_act"], d["current"]
        self.peak_current = [max(p, abs(c)) for p, c in zip(self.peak_current, cur)]
        if self.ref_a is None:
            self.ref_a = list(a)
        for k in range(6):
            if abs(a[k] - self.ref_a[k]) > 1:
                self.ref_a[k], self.t_change[k] = a[k], t
        if not hasattr(self, "bursts"):
            self.bursts = BurstTracker(lambda name, **kw: self.ev(name, **kw), self.hand)
        bs = self.bursts.update(rec)                  # ERROR stops at 6 reads; shorter bursts are GLITCH_BURST events
        if bs:
            raise Stop(bs[0], **bs[1])
        fstop = getattr(self, "force_stop", None)                 # position press: safety limit on the commanded finger
        if fstop is not None:
            self.cond("SAFETY_FORCE_AT_OR_ABOVE_LIMIT", d["force_act"][self.axis] >= fstop, rec["seq"], force=d["force_act"][self.axis], limit=fstop)
        allow3 = getattr(self, "allow3", set())                   # press / grip: force reached (3) expected on the commanded fingers
        self.cond("STATUS_NOT_0_1_2", any(s not in (0, 1, 2) and not (s == 3 and k in allow3) for k, s in enumerate(d["status"])), rec["seq"], status=d["status"])
        k0 = self.axis
        for k in range(6):          # non-test axes: current is data only, one NONTEST_CURRENT event per episode
            if k == k0:
                continue
            ep = self.ep[k]
            if cur[k] != 0:
                if ep is None:
                    ep = self.ep[k] = {"axis": k, "first_seq": rec["seq"], "t_rel_last_cmd_s": None if self.last_cmd is None else
                                       round(t - self.last_cmd["t_mono_ns"] / 1e9, 3), "angle_start": a[k], "peak": 0, "n_reads": 0,
                                       "values": [], "zeros": 0}
                ep["peak"] = max(ep["peak"], abs(cur[k]))
                ep["n_reads"] += 1
                ep["zeros"] = 0
                if len(ep["values"]) < 40:
                    ep["values"].append(cur[k])
                ep["angle_end"] = a[k]
            elif ep is not None:
                ep["zeros"] += 1
                if ep["zeros"] >= 3:
                    ep["last_seq"] = rec["seq"] - 3
                    ep.pop("zeros")
                    self.ev("NONTEST_CURRENT", **ep)
                    self.ep[k] = None
        if self.pre is not None:
            dev = [abs(a[k] - self.pre[k]) if k != self.axis else 0 for k in range(6)]
            self.max_dev = [max(x, y) for x, y in zip(self.max_dev, dev)]
            fdev = [dev[k] if k < 4 else 0 for k in range(6)]          # fingers not under test: 5 counts (3 reads)
            self.cond("NON_COMMANDED_AXIS_GT5", max(fdev) > 5, rec["seq"], dev=fdev, angle=a)
            for k in (4, 5):                                           # thumbs: rule 6 against the hold reading (g1rs_20261012b)
                if k == self.axis:
                    continue                                           # the thumb under test is commanded
                tdev = abs(a[k] - self.hold_reading[k])
                self.cond("THUMB%d_DRIVEN_GT10_WITH_CURRENT" % k, tdev > 10 and cur[k] != 0, rec["seq"], axis=k, dev=tdev, current=cur[k])
                self.cond("THUMB%d_GT20_FROM_HOLD_READING" % k, tdev > 20, rec["seq"], axis=k, dev=tdev, current=cur[k])
                sag = tdev > 10 and cur[k] == 0
                if sag and not self.sag[k]:
                    self.ev("THUMB_SAG", axis=k, dev=tdev, angle=a[k], hold_reading=self.hold_reading[k])
                elif self.sag[k] and tdev <= 10:
                    self.ev("THUMB_SAG_END", axis=k, dev=tdev, angle=a[k])
                self.sag[k] = sag or (self.sag[k] and tdev > 10)
        for k in range(6):
            q = self.tq[k]
            q.append((t, d["temp"][k]))
            while t - q[0][0] > 2.0:
                q.popleft()
            med = sorted(x[1] for x in q)[len(q) // 2]
            if t - q[0][0] >= 1.0:
                self.temp_max[k] = max(self.temp_max[k], med)
            if abs(d["temp"][k] - med) >= 4:
                if self.spike[k] is None:
                    self.spike[k] = {"first_seq": rec["seq"], "median": med, "values": [], "raw_first": raw["TEMP"]}
                self.spike[k]["values"].append(d["temp"][k])
            elif self.spike[k] is not None:
                sp = self.spike[k]
                self.ev("TEMP_SPIKE", axis=k, first_seq=sp["first_seq"], n_reads=len(sp["values"]), values=sp["values"], median=sp["median"], raw=sp["raw_first"])
                self.spike[k] = None
            if t - q[0][0] >= 1.0 and med >= 52:
                raise Stop("TEMP_MEDIAN_GE_52", axis=k, median=med)
            if t - q[0][0] >= 1.0 and med >= 48 and not getattr(self, "pausing", False):
                self.pause(k, med)
        if self.body is not None:
            for name, kw in self.body.pop_events():
                self.ev("BODY_" + name, **kw)          # arm motion, stale lowstate: events only
            b = self.body.check()                      # only a second lowcmd writer or nonzero kp, kd, tau
            if b:
                raise Stop(b[0], detail=b[1])
        self.op_stop()
        return rec

    def op_stop(self):
        if CW_STOP_FILE and not self.stopping and os.path.exists(CW_STOP_FILE):
            raise Stop("OPERATOR_STOP_FILE", file=CW_STOP_FILE, t_seen=round(time.time(), 6))

    def off_target(self, value, target, tol=10):
        """thumb rotation: accepted from target - 25 to target + 10 (it settles about 15 below); other axes: within tol"""
        if self.axis == 5:
            return not (target - ROT_SETTLE - tol <= value <= target + tol)
        return abs(value - target) > tol

    def pause(self, k, med):
        self.pausing = True
        self.ev("PAUSE_TEMP_48", axis=k, median=med)
        t_end = time.monotonic() + 300
        while time.monotonic() < t_end:
            self.sample()
        self.pausing = False
        self.ev("PAUSE_END")

    def gate(self):
        soc = None if self.body is None else self.body.battery()
        if soc is not None and soc <= CLOSE_SOC:
            raise Stop("BATTERY_AT_OR_BELOW_20", soc=soc, close_soc=CLOSE_SOC)
        if STOP_FLAG and os.path.exists(STOP_FLAG):
            raise Stop("STOP_FLAG_FROM_OTHER_HAND_WATCH", flag=open(STOP_FLAG).read()[:300])
        if soc is None and FIRST_WRITE_EPOCH and time.time() - FIRST_WRITE_EPOCH > 3000:
            raise Stop("BATTERY_UNREADABLE_50_MIN_AFTER_FIRST_WRITE")
        return soc

    # ------------------------------------------------------------- writes
    def _send(self, pdu, rec):
        self.op_stop()                                 # checked right before every write
        m = self.c
        m.tid = (m.tid + 1) & 0xFFFF
        adu = struct.pack(">HHHB", m.tid, 0, len(pdu) + 1, m.unit) + pdu
        t_mono, t_host = time.monotonic_ns(), time.time()
        try:
            m.sock.sendall(adu)
            rtid, pid, length, _ = struct.unpack(">HHHB", m._recv_exact(7))
            body = m._recv_exact(length - 1)
        except Exception as e:  # noqa: BLE001
            raise Stop("MODBUS_WRITE", detail=repr(e))
        rec = {"t_mono_ns": t_mono, "t_host": round(t_host, 6), "seq": self.seq, "hand": self.hand, **rec, "adu_hex": adu.hex(" "),
               "response_hex": body.hex(" ")}
        self.seq += 1
        self.line(self.cm, rec)
        if len(self.first_frames) < 3:
            self.first_frames.append(adu.hex(" "))
        self.last_cmd = {"register": rec["register"], "address": rec["address"], "values": rec["values"], "t_mono_ns": t_mono}
        exp = pdu[:5] if pdu[0] == 0x10 else pdu
        if rtid != m.tid or pid != 0 or body != exp:
            raise Stop("WRITE_RESPONSE_MISMATCH", response=body.hex(" "))
        return t_mono / 1e9

    def angle(self, target, reason):
        base = self.base or self.hold
        vec = list(base)
        vec[self.axis] = int(target)
        floor = min(self.hold[self.axis], self.return_to) if self.return_to else self.hold[self.axis]
        if getattr(self, "floor_override", None) is not None:
            floor = self.floor_override                           # below_rest / press target floor (wrapper-checked)
        if not all(0 <= v <= 1000 for v in vec) or vec[self.axis] < floor:
            raise Stop("REFUSED_VECTOR", vec=vec)
        if any(vec[k] != base[k] for k in range(6) if k != self.axis):
            raise Stop("REFUSED_OTHER_AXES", vec=vec)
        pdu = struct.pack(">BHHB6H", 0x10, A_ADDR, 6, 12, *vec)
        t = self._send(pdu, {"register": "ANGLE_SET", "address": A_ADDR, "function": "0x10", "values": vec,
                             "raw_words": ["%04X" % v for v in vec], "reason": "Part A six-value write", "test": reason})
        for k in (self.axis, 4, 5):
            self.t_change[k] = time.monotonic()
        back = self.cfg("ANGLE_SET")
        if back != vec:
            raise Stop("ANGLE_SET_READBACK_MISMATCH", sent=vec, readback=back)
        self.vec = vec
        return t

    def set_speed(self, value, k=None):
        k = self.axis if k is None else k
        if value not in SPEEDS:
            raise Stop("REFUSED_SPEED", value=value)
        sp0 = self.cfg("SPEED_SET")
        addr = S_ADDR + 2 * k
        pdu = struct.pack(">BHH", 0x06, addr, value)
        self._send(pdu, {"register": "SPEED_SET", "address": addr, "function": "0x06", "values": [value], "raw_words": ["%04X" % value],
                         "reason": "speed of the finger under test", "test": "speed_%d" % value})
        sp1 = self.cfg("SPEED_SET")
        exp = list(sp0)
        exp[k] = value
        if sp1 != exp:
            raise Stop("SPEED_SET_READBACK_MISMATCH", before=sp0, after=sp1, expected=exp)
        if k == self.axis:
            self.speed = value
        self.sample("SPEED_SET %d axis %d" % (value, k))

    # ------------------------------------------------------------- motion
    def settle_reads(self, s, event=None):
        rows, t_end = [], time.monotonic() + s
        r = self.sample(event)
        rows.append(r)
        while time.monotonic() < t_end:
            rows.append(self.sample())
        return [r for r in rows if r]

    def set_force(self, value, k=None):
        k = self.axis if k is None else k
        if not (0 <= int(value) <= 3000):
            raise Stop("REFUSED_FORCE", value=value)
        f0 = self.cfg("FORCE_SET")
        addr = F_ADDR + 2 * k
        pdu = struct.pack(">BHH", 0x06, addr, int(value))
        self._send(pdu, {"register": "FORCE_SET", "address": addr, "function": "0x06", "values": [int(value)], "raw_words": ["%04X" % int(value)],
                         "reason": "force limit of the finger", "test": "force_%d" % int(value)})
        f1 = self.cfg("FORCE_SET")
        exp = list(f0)
        exp[k] = int(value)
        if f1 != exp:
            raise Stop("FORCE_SET_READBACK_MISMATCH", before=f0, after=f1, expected=exp)
        self.sample("FORCE_SET %d axis %d" % (int(value), k))

    def gate_wait(self, name):
        """operator decision between press levels: file CONTINUE or STOP in the block folder (reads continue meanwhile)"""
        d = os.path.dirname(self.st.name)
        print("WAITING_FOR_OPERATOR %s: create %s/CONTINUE or %s/STOP (timeout %.0f s)" % (name, d, d, CW_GATE_TIMEOUT), flush=True)
        self.ev("GATE_WAIT", gate=name)
        t_end = time.monotonic() + CW_GATE_TIMEOUT
        while time.monotonic() < t_end:
            self.sample()
            for f in ("CONTINUE", "STOP"):
                p = os.path.join(d, f)
                if os.path.exists(p):
                    txt = open(p).read().strip()
                    self.ev("GATE_" + f, gate=name, text=txt)
                    os.rename(p, p + "_used_%d" % self.seq)               # one decision per gate
                    print("GATE %s (%s)" % (f, txt), flush=True)
                    return f == "CONTINUE"
        self.ev("GATE_TIMEOUT", gate=name)
        print("GATE TIMEOUT after %.0f s: treated as STOP" % CW_GATE_TIMEOUT, flush=True)
        return False

    def move(self, target, dwell, tag, follow=True, allow_short=False):
        self.gate()
        start = self.sample()["dec"]["angle_act"][self.axis]
        t_cmd = self.angle(target, tag)
        direction = (target > start) - (target < start)
        need = follow and abs(target - start) > FOLLOW_MIN     # no-motion timeout only for moves of more than 20 counts
        followed, t_follow, win = not need, None, collections.deque()
        while True:
            r = self.sample()
            if r is None:
                continue
            t = time.monotonic()
            a = r["dec"]["angle_act"][self.axis]
            if not followed and (a - start) * direction >= 3:
                followed, t_follow = True, t - t_cmd
            if not followed and t - t_cmd > FOLLOW_S:
                raise Stop("UNEXPECTED_NO_MOTION_2S", start=start, target=target, now=a)
            win.append((t, a))
            while t - win[0][0] > 0.3:
                win.popleft()
            vals = [x[1] for x in win]
            if followed and t - win[0][0] >= 0.25 and max(vals) - min(vals) <= 1 and (abs(a - target) <= 3 or t - t_cmd > 1.0):
                break
            if t - t_cmd > SETTLE_S:
                raise Stop("UNEXPECTED_NO_SETTLE_15S", target=target, now=a)
        rec = {"axis": self.axis, "tag": tag, "speed": self.speed, "start": start, "target": target, "settled": a, "t_cmd_mono": round(t_cmd, 6),
               "t_follow_s": None if t_follow is None else round(t_follow, 4), "t_settle_s": round(t - t_cmd, 4)}
        self.moves.append(rec)
        rows = self.settle_reads(dwell, "DWELL %s" % tag)
        med = [sorted(c)[len(c) // 2] for c in zip(*[x["dec"]["angle_act"] for x in rows])]
        self.dwells.append({"tag": tag, "target": target, "n": len(rows), "angle_median": med,
                            "pos_median": [sorted(c)[len(c) // 2] for c in zip(*[x["dec"]["pos_act"] for x in rows])]})
        rec["status_end"] = rows[-1]["dec"]["status"][self.axis] if rows else None
        rec["force_max"] = max(x["dec"]["force_act"][self.axis] for x in rows) if rows else None
        if self.off_target(med[self.axis], target) and not allow_short:
            raise Stop("SHORT_OF_TARGET_GT10", target=target, median=med[self.axis])
        return rec

    def block_start(self):
        rows = self.settle_reads(1.0, "BLOCK_START axis=%d" % self.axis)
        self.pre = [sorted(c)[len(c) // 2] for c in zip(*[r["dec"]["angle_act"] for r in rows])]
        age = time.monotonic() - self.hold_t
        self.ev("HOLD_AGE_AT_BLOCK_START", age_s=round(age, 3))
        if age > HOLD_MAX_AGE_S:
            raise Stop("HOLD_OLDER_THAN_30S", age_s=round(age, 3))
        aset = self.cfg("ANGLE_SET")
        if aset != self.hold:
            raise Stop("ANGLE_SET_NOT_HOLD_AT_START", readback=aset, hold=self.hold)
        self.ev("BLOCK_START_INFO", pre=self.pre, angle_set=aset, speed_set=self.cfg("SPEED_SET"), soc=self.gate())

    def run(self, stages):
        hv = self.hold[self.axis]
        self.block_start()
        if set(stages) <= set("brpq") and ("r" in stages or "p" in stages or "q" in stages):
            sp0 = self.cfg("SPEED_SET")[self.axis]
            fs0 = self.cfg("FORCE_SET")[self.axis]
            rows = self.settle_reads(1.0, "F0_READ")
            f0 = sorted(r["dec"]["force_act"][self.axis] for r in rows)[len(rows) // 2]
            if CW_F0_OVERRIDE is not None and ("p" in stages or "q" in stages):
                self.ev("F0_OVERRIDE", measured=f0, used=CW_F0_OVERRIDE)
                print("REST FORCE: measured %d g at the hold, using %d g (--rest-force, closing-pass reading)" % (f0, CW_F0_OVERRIDE), flush=True)
                f0 = CW_F0_OVERRIDE
            self.ev("CW_PLAN_P3", stages=stages, floor=CW_FLOOR, step=CW_STEP, pause=CW_PAUSE, speed=CW_SPEED_P3, levels=CW_LEVELS,
                    close_to=CW_CLOSE_TO, start_speed=sp0, start_force_set=fs0, f0=f0)
            self.press_results = []
            self.set_speed(CW_SPEED_P3)
            if "b" in stages:
                bd = CW_BLIP_DWELL if CW_BLIP_GATE else 0.5
                self.move(hv + CW_BLIP, bd, "blip_out", follow=False)
                self.move(hv, bd, "blip_back", follow=False)
                if CW_BLIP_GATE and not self.gate_wait("after the blip: confirm which hand moved (CONTINUE = the expected hand moved)"):
                    print("STOPPED after the blip (operator STOP or gate timeout); no level run", flush=True)
                    stages = stages.replace("p", "").replace("r", "")
            if "r" in stages:
                self.floor_override = CW_FLOOR
                if CW_FORCE_BELOW_REST:
                    self.set_force(min(3000, f0 + CW_FORCE_BELOW_REST))   # a touch stops the finger (STATUS 3 = stop) instead of 2000 g
                    self.ev("BELOW_REST_FORCE_SET", f0=f0, above_rest=CW_FORCE_BELOW_REST, force_set=min(3000, f0 + CW_FORCE_BELOW_REST))
                v = hv - CW_STEP
                while v >= CW_FLOOR:
                    self.move(v, CW_PAUSE, "r_down_%d" % v)
                    v -= CW_STEP
                self.move(hv, 1.0, "r_back")
                if CW_FORCE_BELOW_REST and self.cfg("FORCE_SET")[self.axis] != fs0:
                    self.set_force(fs0)
            if "p" in stages:
                self.floor_override = CW_CLOSE_TO
                for i, dl in enumerate(CW_LEVELS):
                    lvl = min(3000, f0 + dl)
                    self.set_force(lvl)
                    self.allow3 = {self.axis}
                    r1 = self.move(CW_CLOSE_TO, CW_PAUSE, "p_close_L%d" % dl, allow_short=True)
                    self.move(hv, 1.0, "p_open_L%d" % dl)
                    self.allow3 = set()
                    res = {"level_above_rest_g": dl, "f0": f0, "force_set": lvl, "angle_reached": r1["settled"], "status_end": r1["status_end"],
                           "force_max": r1["force_max"], "force_max_above_rest": r1["force_max"] - f0,
                           "stopped_by": "force (3)" if r1["status_end"] == 3 else "target (2)" if r1["status_end"] == 2 else r1["status_end"]}
                    self.press_results.append(res)
                    self.ev("PRESS_RESULT", **res)
                    print("PRESS level +%d g: FORCE_SET %d, angle %s, status %s, force max %s (%+d g above rest)"
                          % (dl, lvl, r1["settled"], r1["status_end"], r1["force_max"], r1["force_max"] - f0), flush=True)
                    if i < len(CW_LEVELS) - 1 and (i == 0 or CW_GATE == "all"):
                        if not self.gate_wait("after level %d (+%d g): the scale reading decides the next level" % (i + 1, dl)):
                            print("PRESS no further levels after level %d (operator STOP or gate timeout)" % (i + 1), flush=True)
                            break
                self.set_force(fs0)
            if "q" in stages:                                     # position press (Mohammed, 16 Oct): targets past contact
                lim = max(0, min(3000, f0 + CW_SAFETY))
                if not CW_TARGETS or not CW_SAFETY or CW_TARGETS != sorted(CW_TARGETS, reverse=True) or CW_TARGETS[0] >= hv:
                    raise Stop("REFUSED_POSITION_PLAN", targets=CW_TARGETS, safety=CW_SAFETY, hold=hv)
                self.floor_override = CW_TARGETS[-1]
                self.set_force(lim)
                self.force_stop = lim                             # sample(): a reading at or above the limit (3 reads) stops the run
                self.press_results = []
                for i, tg in enumerate(CW_TARGETS):
                    r1 = self.move(tg, CW_PAUSE, "q_target_%d" % tg, allow_short=True)
                    self.move(hv, 1.0, "q_open_%d" % tg)
                    res = {"target": tg, "angle_reached": r1["settled"], "status_end": r1["status_end"], "force_max": r1["force_max"],
                           "force_max_above_rest": r1["force_max"] - f0, "limit": lim}
                    self.press_results.append(res)
                    self.ev("POSITION_PRESS_RESULT", **res)
                    print("POSITION level %d target %d: reached %s, status %s, force max %s (%+d g above rest; safety limit %d)"
                          % (i + 1, tg, r1["settled"], r1["status_end"], r1["force_max"], r1["force_max"] - f0, lim), flush=True)
                    if CW_GATE != "none" and i < len(CW_TARGETS) - 1 and (i == 0 or CW_GATE == "all"):
                        if not self.gate_wait("after level %d (target %d): the scale reading decides the next level" % (i + 1, tg)):
                            print("POSITION no further levels after level %d (operator STOP or gate timeout)" % (i + 1), flush=True)
                            break
                self.force_stop = None
                self.set_force(fs0)
            self.floor_override = None
            self.move(hv, 1.0, "e_rest")
            self.end_check(hv)
            if sp0 != self.speed and sp0 in SPEEDS:
                self.set_speed(sp0)
            return
        if stages == "t":
            self.thumb_step(hv)
            return
        if stages == "R":
            self.record(hv)
            return
        if stages == "TR":
            self.thumb_record(hv)
            return
        if set(stages) <= set("bsw"):
            sp0 = self.cfg("SPEED_SET")[self.axis]               # start speed of the finger, restored at the end
            speeds = CW_SPEEDS if "s" in stages else [CW_SWEEP_SPEED]
            if any(v not in SPEEDS for v in speeds + [CW_SWEEP_SPEED]):
                raise Stop("REFUSED_SPEED_LIST", speeds=speeds)
            lo, hi = CW_RANGE
            if lo < hv:
                self.floor_override = lo     # held above the range start (open pose, 16 Oct): range lo..hi is wrapper-checked
            self.ev("CW_PLAN", stages=stages, speeds=speeds, reps=CW_REPS, range=CW_RANGE, sweep_speed=CW_SWEEP_SPEED, blip=CW_BLIP, start_speed=sp0)
            self.set_speed(speeds[0])
            if "b" in stages:                                    # small blip to mark the start
                self.move(hv + CW_BLIP, 0.5, "blip_out", follow=False)
                self.move(hv, 0.5, "blip_back", follow=False)
            if "s" in stages:
                self.move(lo, DW_B, "b_pre_s%d" % speeds[0])
                for sp in speeds:
                    if self.speed != sp:
                        self.set_speed(sp)
                    for r in range(CW_REPS):
                        self.move(hi, DW_B, "b_up_s%d_rep%d" % (sp, r))
                        self.move(lo, DW_B, "b_down_s%d_rep%d" % (sp, r))
            if "w" in stages:
                if self.speed != CW_SWEEP_SPEED:
                    self.set_speed(CW_SWEEP_SPEED)
                self.move(lo, DW_CD, "w_lo")
                self.move(hi, DW_CD, "w_hi")
                self.move(lo, DW_CD, "w_lo_back")
            if self.speed != speeds[0]:
                self.set_speed(speeds[0])
            ret = self.return_to or hv
            self.move(ret, 1.0, "e_rest" if ret == hv else "e_return_%d" % ret)
            self.end_check(ret)
            if sp0 != self.speed and sp0 in SPEEDS:
                self.set_speed(sp0)                              # start pose: start speed back
            return
        self.set_speed(100)
        if "a" in stages:
            for tgt, tag in ((hv, "a_rest"), (700, "a_700"), (850, "a_850"), (1000, "a_1000"), (hv, "a_back_rest")):
                self.move(tgt, DW_A, tag)
            for r in range(3):
                self.move(1000, DW_A, "a_open_rep%d" % r)
                self.move(hv, DW_A, "a_rest_rep%d" % r)
        if "b" in stages:
            for sp in SPEEDS:
                if self.speed != sp:
                    self.set_speed(sp)
                self.move(650, DW_B, "b_pre_s%d" % sp)
                for r in range(3):
                    self.move(950, DW_B, "b_up_s%d_rep%d" % (sp, r))
                    self.move(650, DW_B, "b_down_s%d_rep%d" % (sp, r))
            self.set_speed(100)
        if "c" in stages:
            if "b" not in stages:
                self.move(hv, DW_A, "c_resume_rest")
            self.move(800, DW_CD, "c_ref")
            for sgn in (1, -1):
                for stp in (5, 10, 20):
                    self.move(800 + sgn * stp, DW_CD, "c_step_%+d" % (sgn * stp), follow=False)
                    self.move(800, DW_CD, "c_back_%+d" % (sgn * stp), follow=False)
        if "d" in stages:
            for src in (900, 700):
                for r in range(3):
                    self.move(src, 0.5, "d_src_%d_rep%d" % (src, r))
                    self.move(800, DW_CD, "d_800_from_%d_rep%d" % (src, r))
        if "e" in stages:
            if self.speed != 100:
                self.set_speed(100)
            self.move(hv, 1.0, "e_rest")
            t_end = time.monotonic() + 4.0
            st2 = False
            while time.monotonic() < t_end:
                r = self.sample()
                if r and r["dec"]["status"][self.axis] == 2:
                    st2 = True
                    break
            self.end_check(hv, wait=False, st2=st2)
            self.set_speed(1000)

    def write_base(self, vals, tag):
        """thumb rotation: fingers 0..3 to `vals` (opening from the hold, or back to the hold), thumbs at their hold values; waits until
        every finger that travels more than 20 counts has moved within 2 s and all four are still for 0.3 s (15 s limit, within 10)"""
        vals = [int(v) for v in vals]
        if vals[4:] != self.hold[4:] or not all(self.hold[k] <= vals[k] <= 1000 for k in range(4)):
            raise Stop("REFUSED_BASE_VECTOR", vec=vals)
        self.gate()
        start = self.sample()["dec"]["angle_act"]
        pdu = struct.pack(">BHHB6H", 0x10, A_ADDR, 6, 12, *vals)
        t_cmd = self._send(pdu, {"register": "ANGLE_SET", "address": A_ADDR, "function": "0x10", "values": vals,
                                 "raw_words": ["%04X" % v for v in vals], "reason": "fingers 0..3 for the thumb rotation step", "test": tag})
        back = self.cfg("ANGLE_SET")
        if back != vals:
            raise Stop("ANGLE_SET_READBACK_MISMATCH", sent=vals, readback=back)
        self.base = None if vals == self.hold else vals
        self.vec = vals
        moved = {k: abs(vals[k] - start[k]) <= FOLLOW_MIN for k in range(4)}
        win = collections.deque()
        while True:
            r = self.sample()
            if r is None:
                continue
            t = time.monotonic()
            a = r["dec"]["angle_act"]
            for k in range(4):
                if not moved[k] and abs(a[k] - start[k]) >= 3:
                    moved[k] = True
            if not all(moved.values()) and t - t_cmd > FOLLOW_S:
                raise Stop("UNEXPECTED_NO_MOTION_2S", start=start, target=vals, now=a)
            win.append((t, a[:4]))
            while t - win[0][0] > 0.3:
                win.popleft()
            if all(moved.values()) and t - win[0][0] >= 0.25 and all(max(c) - min(c) <= 1 for c in zip(*[x[1] for x in win])) and t - t_cmd > 1.0:
                break
            if t - t_cmd > SETTLE_S:
                raise Stop("UNEXPECTED_NO_SETTLE_15S", target=vals, now=a)
        if any(abs(a[k] - vals[k]) > 10 for k in range(4)):
            raise Stop("SHORT_OF_TARGET_GT10", target=vals, now=a)
        self.moves.append({"tag": tag, "start": start, "target": vals, "settled": a, "t_cmd_mono": round(t_cmd, 6), "t_settle_s": round(t - t_cmd, 4)})
        return a

    def record(self, hv):
        """stage R (one finger, one video clip). If CW_FLOOR < 0 (no measured floor): floor-finding pass first, FORCE_SET = rest +
        CW_FORCE_BELOW_REST, steps of CW_STEP down from the hold value, stops at the first STATUS 3 (force reached) or a step the finger
        cannot reach (more than 10 short, mechanical end) or at 0; that angle is the floor; back to the hold value. Then the clip:
        speed CW_SPEED_P3, blip +CW_BLIP held CW_BLIP_DWELL s and back, slow pass to CW_RECORD_TOP and back, FORCE_SET = rest +
        CW_FORCE_BELOW_REST, pass down to floor + CW_FLOOR_MARGIN (no touch at the mechanical end) and back; FORCE_SET and speed back
        to their start values. A touch in the clip's floor pass (STATUS 3) stops the run."""
        if self.axis > 3 or not (hv < CW_RECORD_TOP <= 1000) or not (CW_FLOOR < hv) or not CW_FORCE_BELOW_REST:
            raise Stop("REFUSED_RECORD_PLAN", axis=self.axis, top=CW_RECORD_TOP, floor=CW_FLOOR, force=CW_FORCE_BELOW_REST)
        sp0 = self.cfg("SPEED_SET")[self.axis]
        fs0 = self.cfg("FORCE_SET")[self.axis]
        rows = self.settle_reads(1.0, "F0_READ")
        f0 = sorted(r["dec"]["force_act"][self.axis] for r in rows)[len(rows) // 2]
        fset = min(3000, f0 + CW_FORCE_BELOW_REST)
        self.ev("CW_PLAN_RECORD", top=CW_RECORD_TOP, floor=CW_FLOOR, margin=CW_FLOOR_MARGIN, step=CW_STEP, blip=CW_BLIP, blip_dwell=CW_BLIP_DWELL,
                pause=CW_PAUSE, speed=CW_SPEED_P3, start_speed=sp0, start_force_set=fs0, f0=f0, force_set=fset)
        if sp0 != CW_SPEED_P3:
            self.set_speed(CW_SPEED_P3)
        floor = CW_FLOOR
        if floor < 0:                                          # floor-finding pass
            self.set_force(fset)
            self.allow3 = {self.axis}
            self.floor_override = 0
            v, floor, why = hv - CW_STEP, None, None
            while floor is None:
                tgt = max(v, 0)
                r1 = self.move(tgt, 0.5, "F_find_%d" % tgt, allow_short=True)
                if r1["status_end"] == 3:
                    floor, why = r1["settled"], "STATUS 3 (force limit)"
                elif r1["settled"] - tgt > 10:
                    floor, why = r1["settled"], "mechanical end (more than 10 short, STATUS %s)" % r1["status_end"]
                elif tgt == 0:
                    floor, why = r1["settled"], "reached 0"
                v -= CW_STEP
            self.floor_found = {"floor": floor, "by": why}
            self.ev("FLOOR_FOUND", floor=floor, by=why)
            print("FLOOR FOUND %d (%s)" % (floor, why), flush=True)
            self.move(hv, CW_PAUSE, "F_back")
            self.allow3 = set()
            self.floor_override = None
        target = floor + CW_FLOOR_MARGIN
        if not 0 <= target < hv:
            raise Stop("REFUSED_FLOOR_TARGET", floor=floor, target=target, hold=hv)
        self.record_floor_target = target
        self.move(hv + CW_BLIP, CW_BLIP_DWELL, "R_blip_out", follow=False)
        self.move(hv, CW_BLIP_DWELL, "R_blip_back", follow=False)
        self.move(CW_RECORD_TOP, CW_PAUSE, "R_up_%d" % CW_RECORD_TOP)
        self.move(hv, CW_PAUSE, "R_back_from_top")
        if self.cfg("FORCE_SET")[self.axis] != fset:
            self.set_force(fset)
        self.floor_override = target
        self.move(target, CW_PAUSE, "R_down_%d" % target)
        self.move(hv, CW_PAUSE, "R_back_from_floor")
        self.floor_override = None
        if self.cfg("FORCE_SET")[self.axis] != fs0:
            self.set_force(fs0)
        self.end_check(hv)
        if self.speed != sp0 and sp0 in SPEEDS:
            self.set_speed(sp0)

    def thumb_record(self, hv):
        """stage TR (thumb record clip, P3 only): fingers 0..3 to CW_FINGERS_OPEN first (speed CW_SPEED_P3), thumb blip +CW_BLIP held
        CW_BLIP_DWELL s; rotation (5) only: up to 1000 and back (top of its span); then FORCE_SET = rest + CW_FORCE_BELOW_REST and a
        floor-finding pass down from the hold value in CW_STEP steps (STATUS 3 allowed), stopping at the first STATUS 3, a step missed
        by more than 10 (rotation: against where it lands, 15 below its target), or 0; CW_PAUSE s at that end; back to the hold value
        (CW_PAUSE s); FORCE_SET back; fingers back to their hold values; speeds back."""
        k = self.axis
        if k not in (4, 5) or not CW_FORCE_BELOW_REST:
            raise Stop("REFUSED_THUMB_RECORD", axis=k)
        sp0 = self.cfg("SPEED_SET")
        fs0 = self.cfg("FORCE_SET")[k]
        rows = self.settle_reads(1.0, "F0_READ")
        f0 = sorted(r["dec"]["force_act"][k] for r in rows)[len(rows) // 2]
        fset = max(0, min(3000, f0 + CW_FORCE_BELOW_REST))
        self.ev("CW_PLAN_THUMB_RECORD", axis=k, hold=hv, fingers_open=CW_FINGERS_OPEN, step=CW_STEP, pause=CW_PAUSE, speed=CW_SPEED_P3,
                start_speed=sp0, start_force_set=fs0, f0=f0, force_set=fset)
        pre0 = list(self.pre)
        for f in range(4):
            if sp0[f] != CW_SPEED_P3:
                self.set_speed(CW_SPEED_P3, f)
        self.pre = None
        self.write_base([CW_FINGERS_OPEN] * 4 + self.hold[4:], "T_fingers_open_%d" % CW_FINGERS_OPEN)
        rows = self.settle_reads(1.0, "FINGERS_OPEN")
        self.pre = [sorted(c)[len(c) // 2] for c in zip(*[r["dec"]["angle_act"] for r in rows])]
        if sp0[k] != CW_SPEED_P3:
            self.set_speed(CW_SPEED_P3)
        self.floor_override = 0
        self.move(min(1000, hv + CW_BLIP), CW_BLIP_DWELL, "T_blip_out", follow=False)
        self.move(hv, CW_BLIP_DWELL, "T_blip_back", follow=False)
        if k == 5:
            self.move(1000, CW_PAUSE, "T_up_1000")
            self.move(hv, CW_PAUSE, "T_back_from_top")
        self.set_force(fset)
        self.allow3 = {k}
        off = ROT_SETTLE if k == 5 else 0
        v, floor, why = hv - CW_STEP, None, None
        while floor is None:
            tgt = max(v, 0)
            r1 = self.move(tgt, 0.5, "T_find_%d" % tgt, allow_short=True)
            missed = r1["settled"] - (tgt - off) > 10
            if r1["status_end"] == 3:
                floor, why = r1["settled"], "STATUS 3 (force limit)"
            elif tgt == 0 and r1["settled"] <= 10:
                floor, why = r1["settled"], "reached 0"
            elif missed:
                floor, why = r1["settled"], "step missed by more than 10 (STATUS %s)" % r1["status_end"]
            elif tgt == 0:
                floor, why = r1["settled"], "reached 0"
            v -= CW_STEP
        self.floor_found = {"floor": floor, "by": why}
        self.ev("FLOOR_FOUND", floor=floor, by=why)
        print("THUMB FLOOR FOUND axis %d: %d (%s)" % (k, floor, why), flush=True)
        self.settle_reads(max(0.0, CW_PAUSE - 0.5), "DWELL_AT_FLOOR")
        self.move(hv, CW_PAUSE, "T_back")
        self.allow3 = set()
        self.floor_override = None
        if self.cfg("FORCE_SET")[k] != fs0:
            self.set_force(fs0)
        self.pre = None
        self.write_base(list(self.hold), "T_fingers_back")
        self.settle_reads(0.5, "FINGERS_BACK")
        self.pre = pre0
        self.end_check(hv)
        for f in range(6):
            if self.cfg("SPEED_SET")[f] != sp0[f] and sp0[f] in SPEEDS:
                self.set_speed(sp0[f], f)

    def thumb_step(self, hv):
        """stage t (thumbs, P3 only): speed 100, one step of CW_THUMB_STEP counts and back. Bend (4): toward open (+). Rotation (5):
        toward the palm (-; + would pass 1000), with fingers 0..3 opened to CW_FINGERS_OPEN first and back to their hold values after.
        No blip: the step is the first thumb move. Speeds back to their start values."""
        k = self.axis
        if k not in (4, 5):
            raise Stop("REFUSED_THUMB_STAGE_AXIS", axis=k)
        tgt = hv + CW_THUMB_STEP if k == 4 else hv - CW_THUMB_STEP
        if not (0 <= tgt <= 1000) or not (0 < CW_THUMB_STEP <= 50):
            raise Stop("REFUSED_THUMB_TARGET", target=tgt, step=CW_THUMB_STEP)
        sp0 = self.cfg("SPEED_SET")
        self.ev("CW_PLAN_THUMB", axis=k, hold=hv, target=tgt, speed=CW_SPEED_P3, fingers_open=CW_FINGERS_OPEN if k == 5 else None, start_speed=sp0)
        self.floor_override = min(hv, tgt)
        pre0 = list(self.pre)
        if k == 5:
            for f in range(4):
                if sp0[f] != CW_SPEED_P3:
                    self.set_speed(CW_SPEED_P3, f)
            self.pre = None                                  # fingers are commanded while they open
            a = self.write_base([CW_FINGERS_OPEN] * 4 + self.hold[4:], "t_fingers_open_%d" % CW_FINGERS_OPEN)
            rows = self.settle_reads(1.0, "FINGERS_OPEN")
            self.pre = [sorted(c)[len(c) // 2] for c in zip(*[r["dec"]["angle_act"] for r in rows])]
        if sp0[k] != CW_SPEED_P3:
            self.set_speed(CW_SPEED_P3)
        self.move(tgt, CW_PAUSE, "t_step_%+d" % (tgt - hv))
        self.move(hv, CW_PAUSE, "t_back")
        if k == 5:
            self.pre = None
            self.write_base(list(self.hold), "t_fingers_back")
            self.settle_reads(0.5, "FINGERS_BACK")
            self.pre = pre0
        self.floor_override = None
        self.end_check(hv)
        for f in range(6):
            if self.cfg("SPEED_SET")[f] != sp0[f] and sp0[f] in SPEEDS:
                self.set_speed(sp0[f], f)

    def end_check(self, hv, wait=True, st2=None):
        if wait:
            t_end = time.monotonic() + 4.0
            st2 = False
            while time.monotonic() < t_end:
                r = self.sample()
                if r and r["dec"]["status"][self.axis] == 2:
                    st2 = True
                    break
        if True:
            last = self.sample("END_AT_HOLD")
            a = last["dec"]["angle_act"]
            dev = [abs(a[k] - self.pre[k]) for k in range(4) if k != self.axis]     # fingers only; thumbs by rule 6
            self.end = {"angle": a, "within_3_of_hold": abs(a[self.axis] - hv) <= 3, "status2": st2, "non_commanded_max_dev_end": max(dev),
                        "status": last["dec"]["status"], "error": last["dec"]["error"]}
            if max(dev) > 5:
                raise Stop("NON_COMMANDED_GT5_AT_END", dev=dev)
            if self.off_target(a[self.axis], hv):
                raise Stop("SHORT_OF_TARGET_GT10", target=hv, now=a[self.axis])


def main():
    p = argparse.ArgumentParser()
    p.add_argument("hand", choices=["right", "left"])
    p.add_argument("axis", type=int, choices=[0, 1, 2, 3, 4, 5])
    p.add_argument("stages")
    p.add_argument("hold_summary")
    p.add_argument("out")
    p.add_argument("--host")
    p.add_argument("--port", type=int)
    p.add_argument("--no-body", action="store_true")
    p.add_argument("--return-to", type=int, help="target of the final return write instead of the hold value (Mohammed, run06b: 549)")
    a = p.parse_args()
    if (a.axis >= 4) != (a.stages in ("t", "TR")) or (a.stages == "R" and a.axis > 3):
        raise SystemExit("thumb axes 4, 5 run stage t only, and stage t is for thumbs only")
    hs = json.load(open(a.hold_summary))
    if hs["hand"] != a.hand or hs.get("result") not in ("DONE",) or not hs.get("angle_set_equals_sent"):
        raise SystemExit("hold summary not a clean hold of this hand: %s %s" % (hs.get("hand"), hs.get("result")))
    os.makedirs(a.out, exist_ok=True)
    body = None
    if not a.no_body:
        import g1rs_body
        body = g1rs_body.BodyMonitor()
        body.start()
        time.sleep(2.0)
        body.set_baseline()
    b = Block(a.hand, a.axis, hs["values"], a.out, a.host, a.port, body)
    b.hold_t = json.loads(open(os.path.join(os.path.dirname(os.path.abspath(a.hold_summary)), "commands.jsonl")).readline())["t_mono_ns"] / 1e9
    result, reason, detail = "DONE", None, None
    try:
        b.return_to = a.return_to
        b.hold_reading = hs["reading"]
        b.run(a.stages)
    except Stop as s:
        result, reason, detail = "ABORTED", s.reason, s.detail
        b.stopping = True
        b.ev("STOP_" + s.reason, **s.detail)
        try:
            b.settle_reads(2.0, "AFTER_STOP_NO_WRITES")
        except Stop:
            pass
    except Exception as e:  # noqa: BLE001
        result, reason, detail = "ABORTED", "EXCEPTION", repr(e)
        b.ev("STOP_EXCEPTION", detail=repr(e))
    for ep in b.ep:
        if ep is not None:
            ep.pop("zeros", None)
            b.ev("NONTEST_CURRENT", open_at_end=True, **ep)
    s = {"hand": a.hand, "axis": a.axis, "finger": AXIS[a.axis], "stages": a.stages, "hold": b.hold, "result": result, "reason": reason, "detail": detail,
         "moves": b.moves, "dwells": b.dwells, "peak_current": b.peak_current, "max_non_commanded_dev": b.max_dev, "temp_median_max": b.temp_max,
         "end": getattr(b, "end", None), "floor_found": getattr(b, "floor_found", None),
         "record_floor_target": getattr(b, "record_floor_target", None), "press_results": getattr(b, "press_results", None), "first_frames": b.first_frames, "speed_end": b.cfg("SPEED_SET") if result == "DONE" else None,
         "events": [e for e in b.events if e["event"] != "BLOCK_START_INFO"][-300:], "soc_end": None if body is None else body.battery()}
    json.dump(s, open(os.path.join(a.out, "summary.json"), "w"), indent=1, default=str)
    print("RESULT", result, reason or "", "moves", len(b.moves), "first frames", b.first_frames[:2])
    if body is not None:
        body.close()


if __name__ == "__main__":
    main()
