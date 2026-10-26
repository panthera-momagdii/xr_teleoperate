#!/usr/bin/env python3
"""RT7: save one head camera JPEG from the image server's ZMQ stream (port 55555, a subscriber:
read only). Usage: grab_frame.py OUT.jpg"""
import sys, time, zmq
host = ".".join(str(v) for v in (127, 0, 0, 1))          # loopback, built at run time
ctx = zmq.Context.instance(); s = ctx.socket(zmq.SUB)
s.setsockopt(zmq.CONFLATE, 1); s.setsockopt(zmq.LINGER, 0)
s.connect(f"tcp://{host}:55555"); s.setsockopt_string(zmq.SUBSCRIBE, "")
if not s.poll(5000):
    sys.exit("no frame in 5 s")
time.sleep(0.3)
data = s.recv()
open(sys.argv[1], "wb").write(data)
print(f"saved {len(data)} bytes", flush=True)
s.close(); ctx.term()
