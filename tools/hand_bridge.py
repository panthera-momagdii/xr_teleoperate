#!/usr/bin/env python3
"""[panthera] RT7 hand bridge: DDS hand commands to the two Inspire RH56 hands over Modbus TCP,
written for the owner's rules of 23 Oct 2026. Replaces the vendor bridge (inspire_sdkpy
ModbusDataHandler writes CLEAR_ERROR on connect and one six-register write per message).

DDS (the launcher's --ee inspire_ftp controller, unchanged):
  in   rt/inspire_hand/ctrl/{l,r}   inspire_hand_ctrl, only angle_set (0..1000) is used
  out  rt/inspire_hand/state/{l,r}  inspire_hand_state: angle/pos/force/current/status/temp as read;
                                    err = 0 unless the 12 Oct ERROR rule (6 consecutive reads) fired
Modbus: one connection per hand, reads with the h0 read-only client (fc 0x03, allowlisted map),
writes built here and checked before they go out. Exactly two kinds of write exist:
  * ANGLE_SET, fc 0x10 at 1486, quantity 6, values 0..1000 (never 0xFFFF)
  * FORCE_SET of one axis, fc 0x06 at 1498 + 2 k, value 0..3000
Nothing else, and never CLEAR_ERROR (1004).

Per hand, in order:
  1. START   3 agreeing reads; SPEED_SET and FORCE_SET read and logged.
  2. OPEN    one ANGLE_SET of 1000 x6; wait until every axis reads 985 or more (thumb rotation
             settles about 15 below its target) and 3 reads agree, at most 5 s.
  3. FORCE   rest = mean FORCE_ACT over 20 reads at the open pose; FORCE_SET k = rest k + 200,
             left middle + 400, thumbs the same rule; clamped 0..3000; read back.
  4. FOLLOW  the latest DDS target is written as ONE six-value ANGLE_SET at most 30 times a
             second, and only when a value moved more than 10 counts from the last write.
  5. HOLD    latched, no more writes, the hand keeps its last target. Entered on a fault (below),
             when the command stream stops for more than XR_HAND_STALE_S (1 s: the launcher's [q]
             or exit), or on SIGTERM / SIGINT. Opening again needs a new bridge start (the
             owner's word). The other hand and the arms are not touched.
Faults (12 Oct filters, g1rs_burst.BurstTracker): ERROR nonzero on 6 consecutive reads; more than
10 bursts on one axis; a burst with an ANGLE_ACT or POS_ACT change; read loss over 1 s. Plus: a
thumb (bend or rotation) that moves more than 20 counts with no new command; hand temperature
2 s median 52 C or more (the g1rs_block limit). STATUS 7 is logged as an event, not a fault (12 Oct).

Usage: hand_bridge.py --out DIR [--hands left,right] [--mock-ports right:P,left:P] [--domain 0 --iface eth0]
"""
import argparse, collections, json, os, signal, socket, struct, sys, threading, time

sys.path.insert(0, "/home/unitree/h0_tools")
sys.path.insert(0, "/home/unitree/calibration_ws/scripts/lib")
import h0_ro                                    # noqa: E402  (read-only client and the hand table)
from h0_ro import ROModbus, STATE, CONFIG       # noqa: E402
from g1rs_burst import BurstTracker             # noqa: E402

ANGLE_SET, FORCE_SET = 1486, 1498
WRITE_HZ = float(os.environ.get("XR_HAND_WRITE_HZ", "30"))
DEADBAND = int(os.environ.get("XR_HAND_DEADBAND", "10"))
STALE_S = float(os.environ.get("XR_HAND_STALE_S", "1.0"))
FORCE_OVER = {"left": [200, 200, 400, 200, 200, 200], "right": [200] * 6}   # pinky ring middle index thumb-bend thumb-rot
THUMB_DRIFT = 20
TEMP_STOP_C = 52
GROUPS = ("ERROR", "STATUS", "TEMP", "CURRENT", "ANGLE_ACT", "POS_ACT", "FORCE_ACT")
PACKED = ("ERROR", "STATUS", "TEMP")


def i16(w):
    return [x - 0x10000 if x & 0x8000 else x for x in w]


def lo_bytes(words):
    return [b for w in words for b in (w & 255, (w >> 8) & 255)]


def decode(raw):
    return {g.lower(): (lo_bytes(w) if g in PACKED else i16(w)) for g, w in raw.items()}


class WriteRefused(RuntimeError):
    pass


def pdu_angle_set(vals):
    vals = [int(v) for v in vals]
    if len(vals) != 6 or any(v < 0 or v > 1000 for v in vals):
        raise WriteRefused(f"ANGLE_SET values {vals}")
    pdu = struct.pack(">BHHB6H", 0x10, ANGLE_SET, 6, 12, *vals)
    if 0xFFFF in struct.unpack(">6H", pdu[6:]):
        raise WriteRefused("0xFFFF in ANGLE_SET")
    return pdu


def pdu_force_set(axis, value):
    if axis not in range(6) or not (0 <= int(value) <= 3000):
        raise WriteRefused(f"FORCE_SET axis {axis} value {value}")
    return struct.pack(">BHH", 0x06, FORCE_SET + 2 * axis, int(value))


ALLOWED_WRITES = {(0x10, ANGLE_SET)} | {(0x06, FORCE_SET + 2 * k) for k in range(6)}


class HandLink:
    def __init__(self, name, host, port, unit, log):
        self.name, self.log = name, log
        self.c = ROModbus(host, port, unit, timeout=1.0)
        self.c.connect()
        self.lock = threading.Lock()

    def read_all(self):
        with self.lock:
            return {g: self.c.read_group(STATE, g) for g in GROUPS}

    def read_cfg(self, g):
        with self.lock:
            return i16(self.c.read_group(CONFIG, g))

    def write(self, pdu):
        fc, addr = pdu[0], struct.unpack(">H", pdu[1:3])[0]
        if (fc, addr) not in ALLOWED_WRITES:
            raise WriteRefused(f"fc 0x{fc:02X} at {addr}")
        with self.lock:
            m = self.c
            m.tid = (m.tid + 1) & 0xFFFF
            m.sock.sendall(struct.pack(">HHHB", m.tid, 0, len(pdu) + 1, m.unit) + pdu)
            rtid, pid, length, _ = struct.unpack(">HHHB", m._recv_exact(7))
            body = m._recv_exact(length - 1)
        expect = pdu if fc == 0x06 else struct.pack(">BHH", 0x10, ANGLE_SET, 6)
        if rtid != m.tid or body != expect:
            raise WriteRefused(f"write response mismatch: {body.hex()}")


class Hand:
    def __init__(self, name, link, out, pub, stop_all):
        self.name, self.link, self.out, self.pub = name, link, out, pub
        self.state, self.reason = "START", None
        self.target, self.t_target = None, 0.0
        self.last_written, self.t_write, self.n_writes, self.n_skipped = None, 0.0, 0, 0
        self.ref_thumb, self.t_thumb_cmd = None, 0.0
        self.temp_hist = collections.deque()
        self.err_latched = False
        self.last_ok = time.monotonic()
        self.seq = 0
        self.f_state = open(os.path.join(out, f"{name}_state.jsonl"), "a", buffering=1)
        self.f_ev = open(os.path.join(out, f"{name}_events.jsonl"), "a", buffering=1)
        self.bt = BurstTracker(self.ev, name)
        self.stop_all = stop_all
        self.status7 = 0

    def ev(self, name, **kw):
        rec = {"t_host": round(time.time(), 3), "hand": self.name, "event": name, **{k: v for k, v in kw.items() if k != "raw"}}
        self.f_ev.write(json.dumps(rec, separators=(",", ":")) + "\n")
        if name not in ("GLITCH_BURST",):
            print(f"[hand {self.name}] {name} {json.dumps({k: v for k, v in kw.items() if k != 'raw'})[:200]}", flush=True)

    def to_hold(self, why):
        if self.state != "HOLD":
            self.state, self.reason = "HOLD", why
            self.ev("HOLD", reason=why, last_written=self.last_written, writes=self.n_writes)
            print(f"[hand {self.name}] HOLD: {why}. No more writes; the hand keeps its last target "
                  f"{self.last_written}. Arms and the other hand go on.", flush=True)

    def sample(self):
        try:
            raw = self.link.read_all()
        except Exception as e:
            if time.monotonic() - self.last_ok > 1.0:
                self.to_hold(f"read loss over 1 s ({type(e).__name__})")
            time.sleep(0.05)
            try:
                self.link.c.reconnect()
            except Exception:
                pass
            return None
        self.last_ok = time.monotonic()
        rec = {"seq": self.seq, "t_mono_ns": time.monotonic_ns(), "raw": raw, "dec": decode(raw)}
        self.seq += 1
        d = rec["dec"]
        if self.seq % 10 == 0:
            self.f_state.write(json.dumps({"t": round(time.time(), 3), "state": self.state, "angle": d["angle_act"],
                                           "force": d["force_act"], "current": d["current"], "status": d["status"],
                                           "error": d["error"], "temp": d["temp"], "target": self.last_written},
                                          separators=(",", ":")) + "\n")
        stop = self.bt.update(rec)
        if stop is not None:
            if "ERROR" in str(stop[0]).upper():
                self.err_latched = True
            self.to_hold(f"12 Oct rule: {stop[0]} {stop[1]}"[:160])
        if 7 in d["status"]:
            self.status7 += 1
            if self.status7 in (1, 100, 1000):
                self.ev("STATUS_7", axes=[k for k in range(6) if d["status"][k] == 7], count=self.status7)
        now = time.monotonic()
        tv = [t for t in d["temp"] if 0 < t <= 100]
        if tv:
            self.temp_hist.append((now, max(tv)))
        while self.temp_hist and now - self.temp_hist[0][0] > 2.0:
            self.temp_hist.popleft()
        if len(self.temp_hist) >= 5:
            med = sorted(x[1] for x in self.temp_hist)[len(self.temp_hist) // 2]
            if med >= TEMP_STOP_C:
                self.to_hold(f"hand temperature 2 s median {med} C (limit {TEMP_STOP_C})")
        if self.state == "FOLLOW" and self.ref_thumb is not None and now - self.t_thumb_cmd > 2.0:
            for k in (4, 5):
                if abs(d["angle_act"][k] - self.ref_thumb[k - 4]) > THUMB_DRIFT:
                    self.to_hold(f"thumb {'bend' if k == 4 else 'rotation'} moved "
                                 f"{d['angle_act'][k] - self.ref_thumb[k - 4]:+d} counts with no new command")
        if self.state == "FOLLOW" and now - self.t_thumb_cmd > 2.0 and self.ref_thumb is None:
            self.ref_thumb = [d["angle_act"][4], d["angle_act"][5]]
        self.publish(d)
        return rec

    def publish(self, d):
        if self.pub is None:
            return
        try:
            from inspire_sdkpy import inspire_dds
            m = inspire_dds.inspire_hand_state(
                pos_act=list(d["pos_act"]), angle_act=list(d["angle_act"]), force_act=list(d["force_act"]),
                current=list(d["current"]), err=([1] * 6 if self.err_latched else [0] * 6),
                status=[min(255, max(0, s)) for s in d["status"]], temperature=[min(255, max(0, t)) for t in d["temp"]])
            self.pub.Write(m)
        except Exception as e:
            if self.seq % 500 == 1:
                print(f"[hand {self.name}] state publish failed: {e}", flush=True)

    def agree3(self, timeout=5.0, pred=None):
        win, t_end = collections.deque(maxlen=3), time.monotonic() + timeout
        while time.monotonic() < t_end and self.state != "HOLD":
            r = self.sample()
            if r is None:
                continue
            win.append(tuple(r["dec"]["angle_act"]))
            if len(win) == 3 and len(set(win)) == 1 and (pred is None or pred(list(win[0]))):
                return list(win[0])
        return None

    def startup(self):
        a0 = self.agree3()
        if a0 is None:
            self.to_hold("no 3 agreeing reads at start"); return
        speed, force = self.link.read_cfg("SPEED_SET"), self.link.read_cfg("FORCE_SET")
        self.ev("START", angle=a0, speed_set=speed, force_set=force)
        self.state = "OPEN"
        self.link.write(pdu_angle_set([1000] * 6))
        self.last_written, self.n_writes, self.t_thumb_cmd = [1000] * 6, 1, time.monotonic()
        self.ev("OPEN_SENT", values=[1000] * 6)
        a1 = self.agree3(timeout=5.0, pred=lambda a: min(a) >= 985)
        if a1 is None:
            self.to_hold("did not open to 985 or more in 5 s"); return
        fs = []
        t_end = time.monotonic() + 3.0
        while len(fs) < 20 and time.monotonic() < t_end and self.state != "HOLD":
            r = self.sample()
            if r is not None and not any(r["dec"]["error"]):
                fs.append(r["dec"]["force_act"])
        if len(fs) < 20:
            self.to_hold("no 20 clean reads for the rest force"); return
        rest = [round(sum(f[k] for f in fs) / len(fs)) for k in range(6)]
        lim = [min(3000, max(0, rest[k] + FORCE_OVER[self.name][k])) for k in range(6)]
        for k in range(6):
            self.link.write(pdu_force_set(k, lim[k]))
        back = self.link.read_cfg("FORCE_SET")
        self.ev("FORCE_SET", rest=rest, limit=lim, read_back=back)
        if back != lim:
            self.to_hold(f"FORCE_SET read back {back} is not {lim}"); return
        self.state = "FOLLOW"
        self.ref_thumb = None
        self.t_target = time.monotonic()
        self.ev("FOLLOW", open_angle=a1, force_limit=lim)

    def loop(self):
        try:
            self.startup()
        except WriteRefused as e:
            self.to_hold(f"write refused: {e}")
        except Exception as e:
            self.to_hold(f"start failed: {type(e).__name__}: {e}")
        period = 1.0 / WRITE_HZ
        while not self.stop_all.is_set():
            self.sample()
            if self.state != "FOLLOW":
                time.sleep(0.005)
                continue
            now = time.monotonic()
            if self.target is not None and now - self.t_target > STALE_S:
                self.to_hold(f"no hand command for {STALE_S:g} s (launcher stopped)")
                continue
            if self.target is None or now - self.t_write < period:
                continue
            tgt = [min(1000, max(0, int(round(v)))) for v in self.target]
            if max(abs(tgt[k] - self.last_written[k]) for k in range(6)) <= DEADBAND:
                self.n_skipped += 1
                continue
            try:
                self.link.write(pdu_angle_set(tgt))
            except Exception as e:
                self.to_hold(f"write failed: {type(e).__name__}: {e}")
                continue
            thumb_moved = max(abs(tgt[k] - self.last_written[k]) for k in (4, 5)) > 0
            self.last_written, self.t_write, self.n_writes = tgt, now, self.n_writes + 1
            if thumb_moved:
                self.t_thumb_cmd, self.ref_thumb = now, None
        self.to_hold("bridge stopping")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--hands", default="left,right")
    ap.add_argument("--mock-ports", default=None, help="right:PORT,left:PORT on localhost (fakes only)")
    ap.add_argument("--domain", type=int, default=0)
    ap.add_argument("--iface", default="eth0")
    ap.add_argument("--no-dds", action="store_true")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    mock = dict(x.split(":") for x in a.mock_ports.split(",")) if a.mock_ports else None
    stop_all = threading.Event()
    hands = {}
    pubs = {}
    if not a.no_dds:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
        from inspire_sdkpy import inspire_dds
        ChannelFactoryInitialize(a.domain, a.iface)
    for name in a.hands.split(","):
        h = h0_ro.HANDS[name]
        host, port = (".".join(str(v) for v in (127, 0, 0, 1)), int(mock[name])) if mock else (h["ip"], h["port"])
        pub = None
        if not a.no_dds:
            pub = ChannelPublisher(f"rt/inspire_hand/state/{name[0]}", inspire_dds.inspire_hand_state); pub.Init()
        hands[name] = Hand(name, HandLink(name, host, port, h["unit"], print), a.out, pub, stop_all)
    if not a.no_dds:
        subs = []
        for name, hd in hands.items():
            def cb(m, hd=hd):
                hd.target, hd.t_target = list(m.angle_set), time.monotonic()
            s = ChannelSubscriber(f"rt/inspire_hand/ctrl/{name[0]}", inspire_dds.inspire_hand_ctrl); s.Init(cb, 10); subs.append(s)

    def _stop(signum, frame):
        stop_all.set()
    for sg in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
        signal.signal(sg, _stop)
    print(f"[hand bridge] hands {list(hands)}: open, rest force + limit, then follow at {WRITE_HZ:g} writes/s per hand "
          f"(deadband {DEADBAND} counts); HOLD (no writes) on fault, stale stream ({STALE_S:g} s) or stop", flush=True)
    th = [threading.Thread(target=hd.loop, daemon=True) for hd in hands.values()]
    for t in th:
        t.start()
    while not stop_all.is_set():
        time.sleep(0.2)
    for t in th:
        t.join(timeout=2.0)
    for hd in hands.values():
        hd.ev("SUMMARY", state=hd.state, reason=hd.reason, writes=hd.n_writes, skipped=hd.n_skipped,
              bursts=list(hd.bt.count), status7=hd.status7)
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
