#!/usr/bin/env python3
"""[panthera] RT7 fake Inspire RH56 hand: a Modbus TCP server on localhost for the hand bridge's
tests. TEST FIXTURE: never talks to a real hand. Based on the simulated hand of the h2 / g1rs
offline tests (h2_20261007/tools/test_h2_offline.py): fc 0x03 reads, fc 0x06 single writes at
base + 2 k, fc 0x10 six-value writes at 1486; ANGLE_ACT moves toward ANGLE_SET at SPEED_SET;
thumb rotation settles 15 below its target.

Every write is logged to --writes (JSONL: t, fc, address, values), so a test can check that
nothing but ANGLE_SET x6 and FORCE_SET k was ever written (never 1004 CLEAR_ERROR, never 0xFFFF).

Faults are injected through --inject FILE (JSON, re-read every 0.1 s; an absent file = none):
  {"error_axis": k, "error_reads": n}   ERROR byte of axis k nonzero for the next n reads
  {"burst_axis": k, "burst_reads": n}   TEMP byte of axis k reads 0 for n reads (a glitch burst)
  {"drift_axis": k, "drift": c}         axis k drifts c counts away from its target, no command
  {"silent": true}                      stop answering (read loss)
  {"temp": t}                           every TEMP byte reads t C
Usage: fake_modbus_hand.py --name left --port-file F --writes W [--inject I] [--rest 23,23,-59,1004,230,-100]
"""
import argparse, json, os, socket, struct, threading, time


class Hand:
    def __init__(self, rest_force, start):
        self.pos = [float(x) for x in start]
        self.ang_set = [0, 0, 0, 0, 0, 1000]
        self.speed = [1000] * 6
        self.force_set = [2000] * 6
        self.target = [None] * 6
        self.status = [255] * 6
        self.cur = [0] * 6
        self.rest = list(rest_force)
        self.lock = threading.Lock()
        self.inj = {}
        self.err_left = 0
        self.burst_left = 0
        self.drift_done = set()
        threading.Thread(target=self.kin, daemon=True).start()

    def kin(self):
        while True:
            time.sleep(0.003)
            with self.lock:
                for k in range(6):
                    t = self.target[k]
                    if t is None:
                        continue
                    eff = t - (15 if k == 5 else 0)
                    d = eff - self.pos[k]
                    step = 4.0 * self.speed[k] * 0.003
                    if abs(d) > 0.5 and self.speed[k] > 0:
                        self.pos[k] += max(-step, min(step, d))
                        self.status[k], self.cur[k] = (0 if d > 0 else 1), (100 if k < 4 else 20)
                    else:
                        self.status[k], self.cur[k] = 2, 0

    def apply_inject(self, inj):
        if inj != self.inj:
            self.inj = dict(inj)
            self.err_left = int(inj.get("error_reads", 0))
            self.burst_left = int(inj.get("burst_reads", 0))
            if "drift_axis" in inj and inj["drift_axis"] not in self.drift_done:
                k = int(inj["drift_axis"])
                with self.lock:
                    if self.target[k] is not None:
                        self.target[k] = self.target[k] - int(inj.get("drift", 30))
                    else:
                        self.pos[k] -= int(inj.get("drift", 30))
                self.drift_done.add(k)

    @staticmethod
    def words(vals):
        return [(vals[2 * i] & 255) | ((vals[2 * i + 1] & 255) << 8) for i in range(3)]

    def read(self, a, n):
        with self.lock:
            if a == 1546:
                return [int(round(p)) for p in self.pos]
            if a == 1534:
                return [int(1900 - p * 1.8) for p in self.pos]
            if a == 1582:
                return [v & 0xFFFF for v in self.rest]
            if a == 1594:
                return [c & 0xFFFF for c in self.cur]
            if a == 1606:
                e = [0] * 6
                if self.err_left > 0:
                    e[int(self.inj.get("error_axis", 0))] = 4
                    self.err_left -= 1
                return self.words(e)
            if a == 1612:
                return self.words(self.status)
            if a == 1618:
                t = [int(self.inj.get("temp", 38))] * 6
                if self.burst_left > 0:
                    t[int(self.inj.get("burst_axis", 0))] = 0
                    self.burst_left -= 1
                return self.words(t)
            if a == 1486:
                return list(self.ang_set)
            if a == 1498:
                return list(self.force_set)
            if a == 1522:
                return list(self.speed)
            return [0] * n

    def w06(self, a, v):
        with self.lock:
            if 1486 <= a < 1498:
                k = (a - 1486) // 2
                self.ang_set[k] = v; self.target[k] = v
            elif 1498 <= a < 1510:
                self.force_set[(a - 1498) // 2] = v
            elif 1522 <= a < 1534:
                self.speed[(a - 1522) // 2] = v

    def w16(self, a, vals):
        with self.lock:
            if a == 1486:
                self.ang_set = list(vals)
                for k in range(6):
                    self.target[k] = vals[k]


def rx(c, n):
    b = b""
    while len(b) < n:
        x = c.recv(n - len(b))
        if not x:
            raise ConnectionError
        b += x
    return b


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", required=True)
    ap.add_argument("--port-file", required=True)
    ap.add_argument("--writes", required=True)
    ap.add_argument("--inject", default=None)
    ap.add_argument("--rest", default="23,23,-59,1004,230,-100")
    ap.add_argument("--start", default="932,929,949,951,454,972")
    a = ap.parse_args()
    hand = Hand([int(x) for x in a.rest.split(",")], [int(x) for x in a.start.split(",")])
    wlog = open(a.writes, "a", buffering=1)

    def inj_loop():
        while True:
            try:
                hand.apply_inject(json.load(open(a.inject)) if a.inject and os.path.exists(a.inject) else {})
            except Exception:
                pass
            time.sleep(0.1)
    threading.Thread(target=inj_loop, daemon=True).start()

    def handle(c):
        with c:
            while True:
                try:
                    tid, pid, ln, uid = struct.unpack(">HHHB", rx(c, 7))
                    pdu = rx(c, ln - 1)
                except ConnectionError:
                    return
                while hand.inj.get("silent"):
                    time.sleep(0.05)
                if pdu[0] == 3:
                    _, ad, n = struct.unpack(">BHH", pdu)
                    body = struct.pack(">BB", 3, 2 * n) + struct.pack(">%dH" % n, *hand.read(ad, n))
                elif pdu[0] == 6:
                    _, ad, v = struct.unpack(">BHH", pdu)
                    wlog.write(json.dumps({"t": time.time(), "fc": 6, "address": ad, "values": [v]}) + "\n")
                    hand.w06(ad, v)
                    body = pdu
                elif pdu[0] == 0x10:
                    _, ad, n, nb = struct.unpack(">BHHB", pdu[:6])
                    vals = list(struct.unpack(">%dH" % n, pdu[6:6 + nb]))
                    wlog.write(json.dumps({"t": time.time(), "fc": 16, "address": ad, "values": vals}) + "\n")
                    hand.w16(ad, vals)
                    body = struct.pack(">BHH", 0x10, ad, n)
                else:
                    wlog.write(json.dumps({"t": time.time(), "fc": pdu[0], "address": None, "values": []}) + "\n")
                    body = struct.pack(">BB", pdu[0] | 0x80, 1)
                c.sendall(struct.pack(">HHHB", tid, 0, len(body) + 1, uid) + body)

    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((".".join(str(v) for v in (127, 0, 0, 1)), 0))
    srv.listen(8)
    with open(a.port_file, "w") as f:
        f.write(str(srv.getsockname()[1]))
    while True:
        c, _ = srv.accept()
        threading.Thread(target=handle, args=(c,), daemon=True).start()


if __name__ == "__main__":
    main()
