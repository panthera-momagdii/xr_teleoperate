#!/usr/bin/env python3
"""A fake XR headset for offline tests: speaks vuer's websocket protocol to :8012.

No DDS, no robot. It connects to the launcher's (or quest_link_check's) vuer server
exactly as the Quest browser does -- a websocket on wss://<host>:<port>/ carrying
msgpack {"etype", "ts", "value"} events -- and sends CAMERA_MOVE and HAND_MOVE at a
fixed rate. The wrists trace a slow circle so the IK target, and therefore the
commanded joint vector, changes every frame.

What a test can do with it (RT4 link-loss guard):
    pause()          stop sending: what a stalled WAN link looks like to the launcher
    resume()         start again
    set_offset(xyz)  move both wrists (metres, OpenXR world) -- the "operator moved
                     while the link was down" case
    drop()           close the websocket; the client reconnects by itself (RT5b)

    python tools/fake_xr_client.py --seconds 20           # manual: stream for 20 s

The hand geometry is synthetic (identity joint rotations, fingers laid out ahead of
the wrist, a slow curl), good enough for retargeting to produce finite commands.
"""

import argparse
import asyncio
import math
import ssl
import threading
import time

import numpy as np

HEAD = np.array([0.0, 1.5, 0.0])
LEFT_WRIST = np.array([-0.22, 1.15, -0.32])
RIGHT_WRIST = np.array([0.22, 1.15, -0.32])


def _mat(xyz):
    """4x4, identity rotation, column-major flat list as WebXR sends it."""
    x, y, z = (float(v) for v in xyz)
    return [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, x, y, z, 1.0]


def _hand(wrist, side, curl):
    out = list(_mat(wrist))
    for j in range(1, 25):
        finger, knuckle = divmod(j - 1, 5)
        off = np.array([side * (finger - 2) * 0.018,
                        -0.012 * knuckle * curl,
                        -(0.03 + 0.022 * knuckle * (1.0 - 0.5 * curl))])
        out += _mat(wrist + off)
    return out


class FakeXR:
    def __init__(self, url="wss://127.0.0.1:8012/", rate=60.0, radius=0.04, period=4.0):
        self.url = url
        self.rate = float(rate)
        self.radius = float(radius)
        self.period = float(period)
        self.offset = np.zeros(3)
        self.sending = True
        self.connected = False
        self.connects = 0
        self._drop = False
        self.sent = 0
        self.errors = []
        self._stop = threading.Event()
        self._thread = None
        self._t0 = time.monotonic()

    # ---- control, from any thread
    def pause(self):
        self.sending = False

    def resume(self):
        self.sending = True

    def drop(self):
        self._drop = True

    def set_offset(self, xyz):
        self.offset = np.asarray(xyz, dtype=float).copy()

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5)

    # ---- the client
    def _frame(self, t):
        a = 2.0 * math.pi * t / self.period
        d = np.array([self.radius * math.cos(a), self.radius * math.sin(a), 0.0]) + self.offset
        curl = 0.5 + 0.5 * math.sin(a * 0.5)
        lw, rw = LEFT_WRIST + d, RIGHT_WRIST + d * np.array([-1.0, 1.0, 1.0])
        return {
            "CAMERA_MOVE": {"camera": {"matrix": _mat(HEAD)}},
            "HAND_MOVE": {"left": _hand(lw, -1.0, curl), "right": _hand(rw, 1.0, curl),
                          "leftState": {"pinch": False, "pinchValue": 3.0 + curl,
                                        "squeeze": False, "squeezeValue": 0.0},
                          "rightState": {"pinch": False, "pinchValue": 3.0 + curl,
                                         "squeeze": False, "squeezeValue": 0.0}},
        }

    def _run(self):
        try:
            asyncio.run(self._main())
        except Exception as exc:
            self.errors.append(repr(exc))

    async def _main(self):
        import aiohttp
        from msgpack import packb
        ctx = ssl.create_default_context()
        ctx.check_hostname = False
        ctx.verify_mode = ssl.CERT_NONE
        period = 1.0 / self.rate
        while not self._stop.is_set():
            try:
                async with aiohttp.ClientSession() as session:
                    async with session.ws_connect(self.url, ssl=ctx, heartbeat=None) as ws:
                        self.connected = True
                        self.connects += 1

                        async def drain():
                            async for _ in ws:
                                pass
                        reader = asyncio.ensure_future(drain())
                        try:
                            while not self._stop.is_set() and not ws.closed:
                                if self._drop:
                                    self._drop = False
                                    await ws.close()
                                    break
                                t = time.monotonic()
                                if self.sending:
                                    for etype, value in self._frame(t - self._t0).items():
                                        await ws.send_bytes(packb(
                                            {"etype": etype, "ts": int(time.time() * 1000),
                                             "value": value}, use_bin_type=True))
                                    self.sent += 1
                                await asyncio.sleep(max(0.0, period - (time.monotonic() - t)))
                        finally:
                            reader.cancel()
                self.connected = False
            except Exception as exc:
                self.connected = False
                if len(self.errors) < 20:
                    self.errors.append(repr(exc))
                await asyncio.sleep(0.5)        # server not up yet: retry


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default="wss://127.0.0.1:8012/")
    ap.add_argument("--rate", type=float, default=60.0)
    ap.add_argument("--seconds", type=float, default=20.0)
    args = ap.parse_args()
    xr = FakeXR(args.url, args.rate).start()
    t_end = time.monotonic() + args.seconds
    while time.monotonic() < t_end:
        time.sleep(1.0)
        print(f"connected={xr.connected} frames sent={xr.sent}", flush=True)
    xr.stop()
    if xr.errors:
        print("errors:", xr.errors[-3:])


if __name__ == "__main__":
    main()
