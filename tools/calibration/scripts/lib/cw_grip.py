#!/usr/bin/env python3
"""calibration_ws grip (P3, LOCKED by the wrapper until Mohammed's P3 GO). Built on the proven block tool (g1rs_block.Block: 135 Hz
log, burst / ERROR / temperature / thumb rules, write echo and readback checks). Fingers 0..3 close together, thumbs stay at their
hold values (rule 8). Writes: SPEED_SET and FORCE_SET of fingers 0..3 (single register, one even address each, read back), six-value
ANGLE_SET [C, C, C, C, H4, H5] to close and the hold vector to open, then FORCE_SET and SPEED_SET back to their start values.
FORCE_SET of finger k = its at-rest FORCE_ACT (1 s median at the hold) + --level grams (left middle: its own level from --levels).
Holds until the file RELEASE appears in the run folder or --max-hold-s passes. Stops on the block tool's stop list; STATUS 3 (force
reached) is allowed on the fingers while closed.
Release mode (--release-to N, no grip): after an aborted press / grip / below_rest, FORCE_SET of fingers 0..3 back to 2000, fingers
more than 20 counts below N opened to N (opening only), thumbs at hold, speeds back to 1000 (power-up values, both hands, 13 Oct).
   cw_grip.py <hand> <hold_summary.json> <out_dir> --levels 200,200,400,200 --close-to 400 [--max-hold-s 10] [--speed 100] [--host --port --no-body]
   cw_grip.py <hand> <hold_summary.json> <out_dir> --release-to 550 [--speed 100] [--host --port --no-body]"""
import argparse
import collections
import json
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import g1rs_block as B  # noqa: E402
from g1rs_block import Stop  # noqa: E402


class Grip(B.Block):
    def set_speed_k(self, k, value):
        if value not in B.SPEEDS:
            raise Stop("REFUSED_SPEED", value=value)
        sp0 = self.cfg("SPEED_SET")
        addr = B.S_ADDR + 2 * k
        self._send(struct.pack(">BHH", 0x06, addr, value), {"register": "SPEED_SET", "address": addr, "function": "0x06", "values": [value],
                                                             "raw_words": ["%04X" % value], "reason": "grip speed", "test": "speed_k%d_%d" % (k, value)})
        sp1 = self.cfg("SPEED_SET")
        exp = list(sp0)
        exp[k] = value
        if sp1 != exp:
            raise Stop("SPEED_SET_READBACK_MISMATCH", before=sp0, after=sp1, expected=exp)

    def write_vec(self, vals, reason, floor):
        vals = [int(v) for v in vals]
        if not all(0 <= v <= 1000 for v in vals) or any(vals[k] < floor for k in range(4)) or vals[4:] != self.hold[4:]:
            raise Stop("REFUSED_VECTOR", vec=vals)
        self.gate()                                  # battery at or below 20 %, stop flag, 50 min rule
        pdu = struct.pack(">BHHB6H", 0x10, B.A_ADDR, 6, 12, *vals)
        t = self._send(pdu, {"register": "ANGLE_SET", "address": B.A_ADDR, "function": "0x10", "values": vals, "raw_words": ["%04X" % v for v in vals],
                             "reason": "grip six-value write", "test": reason})
        for k in range(6):
            self.t_change[k] = time.monotonic()
        back = self.cfg("ANGLE_SET")
        if back != vals:
            raise Stop("ANGLE_SET_READBACK_MISMATCH", sent=vals, readback=back)
        self.vec = vals
        return t

    def wait_fingers(self, target, t_cmd, start, tag, allow_short):
        """every finger 0..3 that must travel more than 20 counts moves within 2 s; all four still for 0.3 s; 15 s limit"""
        moved = {k: abs(target[k] - start[k]) <= B.FOLLOW_MIN for k in range(4)}
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
            if not all(moved.values()) and t - t_cmd > B.FOLLOW_S:
                raise Stop("UNEXPECTED_NO_MOTION_2S", start=start, target=target, now=a)
            win.append((t, a[:4]))
            while t - win[0][0] > 0.3:
                win.popleft()
            still = t - win[0][0] >= 0.25 and all(max(c) - min(c) <= 1 for c in zip(*[x[1] for x in win]))
            if all(moved.values()) and still and t - t_cmd > 1.0:
                break
            if t - t_cmd > B.SETTLE_S:
                raise Stop("UNEXPECTED_NO_SETTLE_15S", target=target, now=a)
        if not allow_short and any(abs(a[k] - target[k]) > 10 for k in range(4)):
            raise Stop("SHORT_OF_TARGET_GT10", target=target, now=a)
        self.moves.append({"tag": tag, "start": start, "target": target, "settled": a, "t_cmd_mono": round(t_cmd, 6), "t_settle_s": round(t - t_cmd, 4),
                           "status": r["dec"]["status"], "force": r["dec"]["force_act"]})
        return r

    def grip(self, levels, close_to, max_hold, speed, gate_dir):
        rows = self.settle_reads(1.0, "BLOCK_START grip")
        self.pre = None                              # fingers are all commanded; thumbs follow rule 4 (hold reading)
        age = time.monotonic() - self.hold_t
        self.ev("HOLD_AGE_AT_BLOCK_START", age_s=round(age, 3))
        if age > B.HOLD_MAX_AGE_S:
            raise Stop("HOLD_OLDER_THAN_30S", age_s=round(age, 3))
        if self.cfg("ANGLE_SET") != self.hold:
            raise Stop("ANGLE_SET_NOT_HOLD_AT_START")
        f0 = [sorted(c)[len(c) // 2] for c in zip(*[r["dec"]["force_act"] for r in rows])]
        sp0, fs0 = self.cfg("SPEED_SET"), self.cfg("FORCE_SET")
        fset = [min(3000, f0[k] + levels[k]) for k in range(4)]
        self.ev("GRIP_PLAN", f0=f0, levels=levels, force_set=fset, close_to=close_to, start_speed=sp0, start_force_set=fs0)
        for k in range(4):
            if sp0[k] != speed:
                self.set_speed_k(k, speed)
        for k in range(4):
            self.set_force(fset[k], k)
        h = self.hold
        start = self.sample()["dec"]["angle_act"]
        self.write_vec([h[0], h[1], h[2], h[3] + B.CW_BLIP, h[4], h[5]], "blip_out", min(h[:4]))
        self.wait_fingers(h[:3] + [h[3] + B.CW_BLIP] + h[4:], time.monotonic(), start, "blip_out", False)
        start = self.sample()["dec"]["angle_act"]
        self.write_vec(h, "blip_back", min(h[:4]))
        self.wait_fingers(h, time.monotonic(), start, "blip_back", False)
        close = [close_to] * 4 + h[4:]
        self.allow3 = {0, 1, 2, 3}
        start = self.sample()["dec"]["angle_act"]
        t_cmd = self.write_vec(close, "grip_close", close_to)
        r = self.wait_fingers(close, t_cmd, start, "grip_close", True)
        self.grip_result = {"angles": r["dec"]["angle_act"][:4], "status": r["dec"]["status"][:4], "force": r["dec"]["force_act"][:4], "force_set": fset, "f0": f0}
        print("GRIP closed: angles %s status %s force %s (FORCE_SET %s)" % (self.grip_result["angles"], self.grip_result["status"], self.grip_result["force"], fset), flush=True)
        print("HOLDING until %s/RELEASE or %.0f s" % (gate_dir, max_hold), flush=True)
        t_end = time.monotonic() + max_hold
        while time.monotonic() < t_end and not os.path.exists(os.path.join(gate_dir, "RELEASE")):
            self.sample()
        self.ev("GRIP_RELEASE", by="file" if os.path.exists(os.path.join(gate_dir, "RELEASE")) else "max_hold_s")
        start = self.sample()["dec"]["angle_act"]
        t_cmd = self.write_vec(h, "grip_open", min(h[:4]))
        self.wait_fingers(h, t_cmd, start, "grip_open", False)
        self.allow3 = set()
        for k in range(4):
            if self.cfg("FORCE_SET")[k] != fs0[k]:
                self.set_force(fs0[k], k)
        for k in range(4):
            if self.cfg("SPEED_SET")[k] != sp0[k]:
                self.set_speed_k(k, sp0[k])
        last = self.sample("END_AT_HOLD")
        self.end = {"angle": last["dec"]["angle_act"], "status": last["dec"]["status"], "speed_set": self.cfg("SPEED_SET"), "force_set": self.cfg("FORCE_SET")}

    def release(self, to, speed):
        """after an aborted press / grip / below_rest (rule 6 rerun needs the finger back at rest): FORCE_SET of fingers 0..3 back to
        the power-up value 2000, fingers more than 20 counts below `to` open to `to` (opening only, away from the scale), thumbs at hold,
        speeds of fingers 0..3 back to the power-up value 1000"""
        self.settle_reads(1.0, "BLOCK_START release")
        self.pre = None
        age = time.monotonic() - self.hold_t
        self.ev("HOLD_AGE_AT_BLOCK_START", age_s=round(age, 3))
        if age > B.HOLD_MAX_AGE_S:
            raise Stop("HOLD_OLDER_THAN_30S", age_s=round(age, 3))
        if self.cfg("ANGLE_SET") != self.hold:
            raise Stop("ANGLE_SET_NOT_HOLD_AT_START")
        h = self.hold
        fs0, sp0 = self.cfg("FORCE_SET"), self.cfg("SPEED_SET")
        tgt = [to if h[k] < to - 20 else h[k] for k in range(4)] + h[4:]          # only fingers more than 20 counts below `to` move
        moving = [k for k in range(4) if tgt[k] != h[k]]
        self.ev("RELEASE_PLAN", hold=h, target=tgt, moving=moving, start_force_set=fs0, start_speed=sp0)
        for k in range(4):
            if fs0[k] != FS_POWERUP:
                self.set_force(FS_POWERUP, k)
        for k in moving:
            if sp0[k] != speed:
                self.set_speed_k(k, speed)
        if moving:
            start = self.sample()["dec"]["angle_act"]
            t_cmd = self.write_vec(tgt, "release_to_%d" % to, min(h[:4]))
            self.wait_fingers(tgt, t_cmd, start, "release", False)
        for k in range(4):
            if self.cfg("SPEED_SET")[k] != SP_POWERUP:
                self.set_speed_k(k, SP_POWERUP)
        last = self.sample("END_RELEASED")
        self.end = {"angle": last["dec"]["angle_act"], "status": last["dec"]["status"], "speed_set": self.cfg("SPEED_SET"), "force_set": self.cfg("FORCE_SET")}
        print("RELEASED: angles %s status %s FORCE_SET %s SPEED_SET %s" % (self.end["angle"], self.end["status"], self.end["force_set"], self.end["speed_set"]), flush=True)


    def pose(self, targets, speed, tag, final_speed=None):
        """scale-test pose (Mohammed, 16 Oct): fingers 0..3 to `targets` together at `speed` (open: from the hold up to e.g. 950;
        rest: from the open pose back down to the saved rest values), thumbs at their hold values; speeds back to their start values;
        every finger that travels more than 20 counts moves within 2 s, all still for 0.3 s, within 10 of the target."""
        self.settle_reads(1.0, "BLOCK_START pose")
        self.pre = None
        age = time.monotonic() - self.hold_t
        self.ev("HOLD_AGE_AT_BLOCK_START", age_s=round(age, 3))
        if age > B.HOLD_MAX_AGE_S:
            raise Stop("HOLD_OLDER_THAN_30S", age_s=round(age, 3))
        if self.cfg("ANGLE_SET") != self.hold:
            raise Stop("ANGLE_SET_NOT_HOLD_AT_START")
        h = self.hold
        sp0 = self.cfg("SPEED_SET")
        tgt = [int(x) for x in targets] + h[4:]
        self.ev("POSE_PLAN", tag=tag, hold=h, target=tgt, speed=speed, start_speed=sp0)
        for k in range(4):
            if sp0[k] != speed:
                self.set_speed_k(k, speed)
        start = self.sample()["dec"]["angle_act"]
        t_cmd = self.write_vec(tgt, tag, min(tgt[:4]))
        r = self.wait_fingers(tgt, t_cmd, start, tag, False)
        for k in range(4):
            back = final_speed if final_speed is not None else sp0[k]
            if self.cfg("SPEED_SET")[k] != back:
                self.set_speed_k(k, back)
        last = self.sample("END_POSE")
        self.end = {"angle": last["dec"]["angle_act"], "status": last["dec"]["status"], "speed_set": self.cfg("SPEED_SET"), "angle_set": self.cfg("ANGLE_SET")}
        print("POSE %s: angles %s status %s (target %s)" % (tag, self.end["angle"], self.end["status"], tgt), flush=True)


FS_POWERUP, SP_POWERUP = 2000, 1000              # read on both hands after power-up, 13 Oct 2026


def main():
    p = argparse.ArgumentParser()
    p.add_argument("hand", choices=["right", "left"])
    p.add_argument("hold_summary")
    p.add_argument("out")
    p.add_argument("--levels", help="grams above rest for fingers 0..3")
    p.add_argument("--pose-open", type=int, help="pose mode: fingers 0..3 from the hold up to this value (scale test)")
    p.add_argument("--pose-rest", help="pose mode: fingers 0..3 down to these 4 saved rest values (after the scale test)")
    p.add_argument("--release-to", type=int, help="release mode (no grip): open fingers 0..3 below this value to it, power-up FORCE_SET and speeds")
    p.add_argument("--close-to", type=int, default=400)
    p.add_argument("--max-hold-s", type=float, default=10.0)
    p.add_argument("--speed", type=int, default=100)
    p.add_argument("--host")
    p.add_argument("--port", type=int)
    p.add_argument("--no-body", action="store_true")
    a = p.parse_args()
    hs = json.load(open(a.hold_summary))
    if hs["hand"] != a.hand or hs.get("result") != "DONE" or not hs.get("angle_set_equals_sent"):
        raise SystemExit("hold summary not a clean hold of this hand")
    pose = None
    if a.pose_open is not None:
        if not 900 <= a.pose_open <= 1000 or any(v > a.pose_open for v in hs["values"][:4]):
            raise SystemExit("--pose-open 900..1000 and above every finger's hold value")
        pose, levels = ([a.pose_open] * 4, "pose_open_%d" % a.pose_open), None
    elif a.pose_rest is not None:
        r = [int(x) for x in a.pose_rest.split(",")]
        if len(r) != 4 or not all(450 <= x <= 650 for x in r):
            raise SystemExit("--pose-rest needs 4 values in 450..650")
        pose, levels = (r, "pose_rest"), None
    elif a.release_to is not None:
        levels = None
        if not 500 <= a.release_to <= 700:
            raise SystemExit("--release-to 500..700")
    else:
        levels = [int(x) for x in (a.levels or "").split(",") if x]
        if len(levels) != 4 or any(x < 0 or x > 3000 for x in levels):
            raise SystemExit("--levels needs 4 values in 0..3000")
    os.makedirs(a.out, exist_ok=True)
    body = None
    if not a.no_body:
        import g1rs_body
        body = g1rs_body.BodyMonitor()
        body.start()
        time.sleep(2.0)
        body.set_baseline()
    g = Grip(a.hand, 0, hs["values"], a.out, a.host, a.port, body)
    g.hold_reading = hs["reading"]
    g.hold_t = json.loads(open(os.path.join(os.path.dirname(os.path.abspath(a.hold_summary)), "commands.jsonl")).readline())["t_mono_ns"] / 1e9
    result, reason, detail = "DONE", None, None
    try:
        if pose is not None:
            g.pose(pose[0], a.speed, pose[1], final_speed=SP_POWERUP if pose[1] == "pose_rest" else None)
        elif a.release_to is not None:
            g.release(a.release_to, a.speed)
        else:
            g.grip(levels, a.close_to, a.max_hold_s, a.speed, a.out)
    except Stop as s:
        result, reason, detail = "ABORTED", s.reason, s.detail
        g.stopping = True
        g.ev("STOP_" + s.reason, **s.detail)
        try:
            g.settle_reads(2.0, "AFTER_STOP_NO_WRITES")
        except Stop:
            pass
    except Exception as e:  # noqa: BLE001
        result, reason, detail = "ABORTED", "EXCEPTION", repr(e)
        g.ev("STOP_EXCEPTION", detail=repr(e))
    json.dump({"hand": a.hand, "mode": pose[1] if pose else ("release" if a.release_to is not None else "grip"), "release_to": a.release_to, "result": result, "reason": reason,
               "detail": detail, "levels": levels, "close_to": a.close_to,
               "grip_result": getattr(g, "grip_result", None), "moves": g.moves, "end": getattr(g, "end", None), "first_frames": g.first_frames,
               "events": g.events[-200:]}, open(os.path.join(a.out, "summary.json"), "w"), indent=1, default=str)
    print("RESULT", result, reason or "")
    if body is not None:
        body.close()


if __name__ == "__main__":
    main()
