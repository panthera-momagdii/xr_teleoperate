#!/usr/bin/env python3
"""Count head-camera frames on the image server's ZMQ PUB for N seconds (loopback). Read only."""
import sys, time, zmq
port, secs = int(sys.argv[1]), float(sys.argv[2])
ctx = zmq.Context(); s = ctx.socket(zmq.SUB)
s.setsockopt(zmq.SUBSCRIBE, b""); s.setsockopt(zmq.RCVTIMEO, 1000); s.setsockopt(zmq.LINGER, 0)
s.connect(f"tcp://127.0.0.1:{port}")
t0 = time.monotonic(); n = 0; first = None; size = 0
while time.monotonic() - t0 < secs:
    try:
        m = s.recv()
    except zmq.Again:
        continue
    n += 1; size = len(m); first = first or time.monotonic()
span = time.monotonic() - (first or t0)
print(f"zmq {port}: {n} msgs in {span:.2f} s = {n/span if span > 0 else 0:.1f} msg/s, last msg {size} B")
s.close(); ctx.term()
