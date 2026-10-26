#!/usr/bin/env python3
"""stdin -> stdout with a wall-clock stamp per line; IPv4 addresses masked (no addresses in files)."""
import re, sys, time
rx = re.compile(r"\b\d{1,3}(\.\d{1,3}){3}\b")
ansi = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]")
for line in sys.stdin:
    line = rx.sub("<ip>", ansi.sub("", line.rstrip("\r\n")))
    sys.stdout.write(time.strftime("%H:%M:%S.") + f"{int(time.time() * 1000) % 1000:03d} " + line + "\n")
    sys.stdout.flush()
