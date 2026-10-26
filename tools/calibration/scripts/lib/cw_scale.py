#!/usr/bin/env python3
"""calibration_ws scale test tool (Mohammed, 18 Oct 2026). Runs right after a fresh hold on one hand (the wrapper scale_test.py does
the pre-flight and the hold). Built on the proven block tool (g1rs_block.Block: 135 Hz log of every sample with clock time, burst /
ERROR / STATUS / temperature / thumb rules, write echo and readback checks, STOP file before every write and on every read).

Scale mode, for each finger in --fingers (default index 3, then middle 2):
  open the finger to 950; contact search: FORCE_SET = rest force + --contact-g, close at speed 100 toward 650; the finger stops at
  the plate (contact = where it stops); reaching 650 means no contact: that finger is skipped; open to 950; then position targets
  contact - 10, 20, ... (--steps), each held --hold-s, open to 950 between. The safety limit (rest force + --safety-g) is set for
  the targets; a reading at or above it (3 reads) stops the run. Scale awake: a light tap (contact - 5 for 0.5 s) whenever more
  than --awake-s - 3 s passed since the last change on the scale, and one tap with the previous finger at the switch.
No-contact mode (--no-contact): each finger opens to 950, closes to 650, opens to 950 (inside 650..950 only).
Only the finger under test moves; the thumbs and the other fingers stay at their hold values. Temperature: a 2 s median of
48 C or more opens the finger and waits (up to 5 min) before the next press; 52 C stops. STATUS 7 on the hand stops.
Any stop (safety, temperature, STATUS, STOP file, Ctrl-C): open the finger(s) under test to 950 once, then no further write.
   cw_scale.py <hand> <hold_summary.json> <out_dir> [--no-contact] [--fingers 3,2] [--rest 3:1066,2:55] [--host --port --no-body]"""
import argparse
import json
import os
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import g1rs_block as B  # noqa: E402
from g1rs_block import Stop  # noqa: E402

NAME = {0: "little", 1: "ring", 2: "middle", 3: "index"}
FS_DEFAULT, SP_DEFAULT = 2000, 1000            # factory / power-up values (read on both hands)


class SoftStop(Exception):
    pass


class Scale(B.Block):
    def setup(self, a):
        self.a = a
        self.pausing = True                     # the block tool's in-read 5 min pause is replaced by the between-press check
        self.changes = []                       # clock times of a change on the scale (finger on / finger off)
        self.presses = []
        self.base = list(self.hold)
        self.tested = []
        self.soft = None                        # soft mode: (finger, limit) while a press is on

    def mark(self, kind, **kw):
        t = time.time()
        self.ev(kind, t_clock=round(t, 3), **kw)
        return t

    def sample(self, event=None):
        r = super().sample(event)
        if r is not None and self.soft is not None and r["dec"]["force_act"][self.soft[0]] >= self.soft[1]:
            k, lim = self.soft
            self.soft = None
            raise SoftStop("finger %d reading %d >= %d" % (k, r["dec"]["force_act"][k], lim))
        return r

    def scale_change(self, what):
        self.changes.append((time.time(), what))

    def on_k(self, k):
        """switch the commanded axis to finger k; the other fingers are checked against where they are now"""
        self.pre = None                     # no check against the old reference while the new one is read
        self.axis = k
        rows = self.settle_reads(0.3)
        self.pre = [sorted(c)[len(c) // 2] for c in zip(*[r["dec"]["angle_act"] for r in rows])]

    def go(self, k, target, dwell, tag, allow_short=False, follow=True):
        self.floor_override = min(getattr(self, "floor_override", None) or 1000, target, self.base[k])
        r = self.move(target, dwell, tag, follow=follow, allow_short=allow_short)
        return r

    def open950(self, k, tag, dwell=0.5):
        r = self.go(k, 950, dwell, tag)
        self.base[k] = 950
        return r

    def temp_check(self, k):
        med = max(sorted(x[1] for x in q)[len(q) // 2] for q in self.tq if q)
        if med >= 48:
            self.ev("PAUSE_TEMP_48", median=med)
            print("PAUSE: temperature median %d C; finger open, waiting below 48 C (max 5 min)" % med, flush=True)
            t_end = time.monotonic() + 300
            while time.monotonic() < t_end:
                self.sample()
                med = max(sorted(x[1] for x in q)[len(q) // 2] for q in self.tq if q)
                if med < 48:
                    break
            self.ev("PAUSE_END", median=med)
            if med >= 48:
                raise Stop("TEMP_STILL_GE_48_AFTER_5_MIN", median=med)

    def tap(self, k, contact, why):
        t0 = self.mark("TAP_START", finger=k, why=why)
        self.go(k, contact - 5, 0.5, "tap_%s" % why, allow_short=True)
        self.scale_change("tap on %s" % NAME[k])
        self.open950(k, "tap_open_%s" % why, dwell=0.2)
        self.scale_change("tap off %s" % NAME[k])
        self.mark("TAP_END", finger=k, s=round(time.time() - t0, 2))

    def awake(self, k, contact):
        if contact is not None and self.changes and time.time() - self.changes[-1][0] > self.a.awake_s - 3:
            self.tap(k, contact, "awake")

    def finger(self, k):
        a = self.a
        rest = a.rest[k]
        self.on_k(k)
        self.allow3 = {k}
        if self.cfg("SPEED_SET")[k] != a.speed:
            self.set_speed(a.speed, k)
        self.open950(k, "open_%s" % NAME[k])
        if a.no_contact:
            self.set_force(max(0, min(3000, rest + a.safety_g)), k)
            self.force_stop = max(0, min(3000, rest + a.safety_g))
            for tg in (650, 950):
                t0 = self.mark("PASS_START", finger=k, target=tg)
                self.go(k, tg, 1.0, "nc_%s_%d" % (NAME[k], tg))
                print("NO-CONTACT %s to %d: done in %.1f s" % (NAME[k], tg, time.time() - t0), flush=True)
            self.base[k] = 950
            self.force_stop = None
            return
        # contact search
        self.temp_check(k)
        self.set_force(max(0, min(3000, rest + a.contact_g)), k)
        t0 = self.mark("CONTACT_SEARCH_START", finger=k, limit=rest + a.contact_g)
        r = self.go(k, 650, 1.0, "contact_%s" % NAME[k], allow_short=True, follow=False)   # may already rest on the plate
        contact = r["settled"]
        if contact <= 650 + 10:
            self.mark("NO_CONTACT_SKIP", finger=k, reached=contact)
            print("CONTACT %s: none above 650 (reached %d): finger skipped" % (NAME[k], contact), flush=True)
            self.open950(k, "open_skip_%s" % NAME[k])
            return None
        self.scale_change("contact %s" % NAME[k])
        self.mark("CONTACT_FOUND", finger=k, contact=contact, force_max=r["force_max"], s=round(time.time() - t0, 2))
        print("CONTACT %s at %d (force max %s g, %.1f s)" % (NAME[k], contact, r["force_max"], time.time() - t0), flush=True)
        self.open950(k, "open_after_contact_%s" % NAME[k])
        self.scale_change("off %s" % NAME[k])
        # position targets
        lim = max(0, min(3000, rest + a.safety_g))
        if a.soft:                               # soft mode: register at the factory default, the software stop does the safety
            self.set_force(FS_DEFAULT, k)
            self.force_stop = None
        else:
            self.set_force(lim, k)
            self.force_stop = lim
        for i, d in enumerate(a.steps):
            tg = contact - d
            self.temp_check(k)
            self.awake(k, contact)
            t0 = self.mark("PRESS_START", finger=k, target=tg, depth=d)
            self.scale_change("press on %s %d" % (NAME[k], tg))
            if a.soft:
                self.soft = (k, lim)
                try:
                    r = self.go(k, tg, a.hold_s, "press_%s_%d" % (NAME[k], tg), allow_short=True)
                except SoftStop as e:
                    self.soft = None
                    self.mark("SOFT_STOP", finger=k, target=tg, detail=str(e))
                    print("SOFT STOP %s at target %d: %s; open to 950, skip its remaining presses" % (NAME[k], tg, e), flush=True)
                    self.open950(k, "soft_open_%s_%d" % (NAME[k], tg))
                    self.scale_change("soft stop off %s" % NAME[k])
                    self.presses.append({"finger": NAME[k], "target": tg, "depth": d, "soft_stop": str(e), "t_start": round(t0, 3)})
                    break
                self.soft = None
            else:
                r = self.go(k, tg, a.hold_s, "press_%s_%d" % (NAME[k], tg), allow_short=True)
            rows = [x for x in self.settle_reads(0.05)]
            t1 = self.mark("PRESS_END", finger=k, target=tg)
            self.open950(k, "open_%s_%d" % (NAME[k], tg))
            self.scale_change("press off %s %d" % (NAME[k], tg))
            res = {"finger": NAME[k], "target": tg, "depth": d, "reached": r["settled"], "status_end": r["status_end"],
                   "force_max": r["force_max"], "above_rest": None if r["force_max"] is None else r["force_max"] - rest,
                   "t_start": round(t0, 3), "t_end": round(t1, 3)}
            self.presses.append(res)
            self.ev("PRESS_RESULT", **res)
            print("PRESS %s %d/%d target %d (contact-%d): reached %s, force max %s g (%+d over rest), %s to %s" % (
                NAME[k], i + 1, len(a.steps), tg, d, r["settled"], r["force_max"], (r["force_max"] or 0) - rest,
                time.strftime("%H:%M:%S", time.localtime(t0)), time.strftime("%H:%M:%S", time.localtime(t1))), flush=True)
        self.force_stop = None
        return contact

    def run_all(self):
        a = self.a
        age = time.monotonic() - self.hold_t
        self.ev("HOLD_AGE_AT_START", age_s=round(age, 3))
        if age > a.max_hold_age:
            raise Stop("HOLD_OLDER_THAN_%dS" % a.max_hold_age, age_s=round(age, 3))
        if self.cfg("ANGLE_SET") != self.hold:
            raise Stop("ANGLE_SET_NOT_HOLD_AT_START")
        self.fs0 = self.cfg("FORCE_SET")
        self.sp0 = self.cfg("SPEED_SET")
        if a.soft:                               # set limits and speeds ourselves (no trust in leftovers); end at factory values
            for k in a.fingers:
                if self.fs0[k] != FS_DEFAULT:
                    self.set_force(FS_DEFAULT, k)
                if self.sp0[k] != a.speed:
                    self.set_speed(a.speed, k)
            self.fs0 = [FS_DEFAULT] * 6
            self.sp0 = [SP_DEFAULT] * 6
        print("START %s fingers %s, hold age %.1f s" % ("no-contact" if a.no_contact else "scale", [NAME[k] for k in a.fingers], age), flush=True)
        prev = None
        for k in a.fingers:
            if prev is not None and prev[1] is not None and not a.no_contact:
                self.on_k(prev[0])
                self.tap(prev[0], prev[1], "switch")       # keep the scale awake across the switch
            contact = self.finger(k)
            self.tested.append({"finger": NAME[k], "contact": contact})
            prev = (k, contact)
        self.finish_ok()

    def restore(self):
        for k in self.a.fingers:
            if self.cfg("FORCE_SET")[k] != self.fs0[k]:
                self.set_force(self.fs0[k], k)
            if self.cfg("SPEED_SET")[k] != self.sp0[k] and self.sp0[k] in B.SPEEDS:
                self.set_speed(self.sp0[k], k)

    def finish_ok(self):
        self.allow3 = set()
        self.restore()
        rows = self.settle_reads(1.0, "END_STILL")
        a = rows[-1]["dec"]["angle_act"]
        self.end = {"angle": a, "status": rows[-1]["dec"]["status"], "still_1s": all(max(c) - min(c) <= 1 for c in zip(*[r["dec"]["angle_act"] for r in rows]))}
        print("END open: %s status %s still %s" % (a, self.end["status"], self.end["still_1s"]), flush=True)

    def safe_open(self, why):
        """after a stop: one write that opens the finger(s) under test to 950 (others and thumbs at hold), then no further write"""
        self.stopping = True
        self.force_stop = None
        self.allow3 = {0, 1, 2, 3}
        vec = list(self.base)
        for k in self.a.fingers:
            vec[k] = 950
        try:
            import struct
            pdu = struct.pack(">BHHB6H", 0x10, B.A_ADDR, 6, 12, *vec)
            self._send(pdu, {"register": "ANGLE_SET", "address": B.A_ADDR, "function": "0x10", "values": vec,
                             "raw_words": ["%04X" % v for v in vec], "reason": "stop: open to 950", "test": "stop_open_%s" % why})
            self.ev("STOP_OPEN_SENT", vec=vec, why=why)
            print("STOP: opened to 950 (%s); no further write" % why, flush=True)
            self.settle_reads(2.0, "AFTER_STOP_OPEN")
        except Exception as e:  # noqa: BLE001
            self.ev("STOP_OPEN_FAILED", detail=repr(e))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("hand", choices=["right", "left"])
    p.add_argument("hold_summary")
    p.add_argument("out")
    p.add_argument("--no-contact", action="store_true")
    p.add_argument("--soft", action="store_true", help="presses with the register at the factory default and a software stop at rest + safety")
    p.add_argument("--fingers", default="3,2")
    p.add_argument("--rest", default="3:1066,2:55")
    p.add_argument("--contact-g", type=int, default=200)
    p.add_argument("--safety-g", type=int, default=700)
    p.add_argument("--steps", default="10,20,30,40,50,60")
    p.add_argument("--hold-s", type=float, default=8.0)
    p.add_argument("--awake-s", type=float, default=15.0)
    p.add_argument("--speed", type=int, default=100)
    p.add_argument("--max-hold-age", type=float, default=10.0)
    p.add_argument("--host")
    p.add_argument("--port", type=int)
    p.add_argument("--no-body", action="store_true")
    a = p.parse_args()
    a.fingers = [int(x) for x in a.fingers.split(",")]
    a.rest = {int(x.split(":")[0]): int(x.split(":")[1]) for x in a.rest.split(",")}
    a.steps = [int(x) for x in a.steps.split(",")]
    if not set(a.fingers) <= {0, 1, 2, 3} or not all(k in a.rest for k in a.fingers) or a.steps != sorted(a.steps) or max(a.steps) > 100:
        raise SystemExit("bad --fingers / --rest / --steps")
    hs = json.load(open(a.hold_summary))
    if hs["hand"] != a.hand or hs.get("result") != "DONE" or not hs.get("angle_set_equals_sent"):
        raise SystemExit("hold summary not a clean hold of this hand")
    os.makedirs(a.out, exist_ok=True)
    body = None
    if not a.no_body:
        import g1rs_body
        body = g1rs_body.BodyMonitor()
        body.start()
        time.sleep(1.0)
        body.set_baseline()
    s = Scale(a.hand, a.fingers[0], hs["values"], a.out, a.host, a.port, body)
    s.hold_reading = hs["reading"]
    s.hold_t = json.loads(open(os.path.join(os.path.dirname(os.path.abspath(a.hold_summary)), "commands.jsonl")).readline())["t_mono_ns"] / 1e9
    s.setup(a)
    signal.signal(signal.SIGINT, lambda *x: (_ for _ in ()).throw(Stop("CTRL_C")))
    result, reason, detail = "DONE", None, None
    try:
        s.run_all()
    except Stop as e:
        result, reason, detail = "ABORTED", e.reason, e.detail
        s.ev("STOP_" + e.reason, **e.detail)
        if e.reason not in ("MODBUS_WRITE", "READ_LOSS_GT_1S", "WRITE_RESPONSE_MISMATCH"):
            s.safe_open(e.reason)
    except Exception as e:  # noqa: BLE001
        result, reason, detail = "ABORTED", "EXCEPTION", repr(e)
        s.ev("STOP_EXCEPTION", detail=repr(e))
        s.safe_open("EXCEPTION")
    ch = [t for t, _ in s.changes]
    gaps = [round(b - a2, 2) for a2, b in zip(ch, ch[1:])]
    json.dump({"hand": a.hand, "mode": "no_contact" if a.no_contact else "scale", "result": result, "reason": reason, "detail": detail,
               "fingers": s.tested, "presses": s.presses, "scale_changes": s.changes, "max_gap_s": max(gaps) if gaps else None,
               "moves": s.moves, "end": getattr(s, "end", None), "events": s.events[-300:]},
              open(os.path.join(a.out, "summary.json"), "w"), indent=1, default=str)
    print("SCALE max gap between changes on the scale: %s s" % (max(gaps) if gaps else None), flush=True)
    print("RESULT", result, reason or "", flush=True)
    if body is not None:
        body.close()


if __name__ == "__main__":
    main()
