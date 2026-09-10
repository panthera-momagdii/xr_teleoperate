#!/usr/bin/env python3
"""Prove a TCP port is free before something tries to bind it -- and name the holder.

Read-only apart from a bind()/close() on the port under test. No DDS, no robot.

Why this exists
---------------
On 2026-09-09 `tools/quest_link_check.py` was still holding 8012 when the launcher
started. Vuer's own bind failed inside its aiohttp startup thread, which the launcher
never notices: it carried on with a dead XR path, and pressing `r` moved the arms to
televuer's fallback pose (`CONST_LEFT_ARM_POSE`/`CONST_RIGHT_ARM_POSE`,
teleop/televuer/src/televuer/tv_wrapper.py:161-169) instead of following the operator.

An unfollowable robot moving on its own is the worst failure mode in this system, so
the launcher now refuses to start at all when the port is busy, and says which process
to kill.

Usage:
    python tools/port_guard.py                 # check 8012
    python tools/port_guard.py --port 8013
    python tools/port_guard.py --wait 5        # wait up to 5 s for it to free up
"""

import argparse
import errno
import os
import socket
import sys
import time

DEFAULT_PORT = 8012


def port_is_free(port, host="0.0.0.0"):
    """True if `host:port` can be bound right now.

    Deliberately WITHOUT SO_REUSEADDR: the question is "would vuer's bind succeed",
    and vuer does not set it either. With SO_REUSEADDR this would happily bind
    alongside a listener on 127.0.0.1:<port> and report a free port that is not.

    The socket is closed immediately, so this never holds the port itself -- which is
    the whole bug it exists to prevent.
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind((host, port))
        return True
    except OSError as exc:
        if exc.errno in (errno.EADDRINUSE, errno.EACCES):
            return False
        raise
    finally:
        s.close()


def _proc_name(pid):
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as fh:
            raw = fh.read()
        cmd = raw.replace(b"\0", b" ").decode("utf-8", "replace").strip()
        return cmd or f"<pid {pid}, no cmdline>"
    except OSError:
        return f"<pid {pid}, gone>"


def _inode_to_pid(target_inode):
    """Map a socket inode to the pid holding it by walking /proc/*/fd.

    Only sees processes this user owns, which is the normal case here and needs no
    sudo. Returns None when the holder belongs to someone else.
    """
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        fd_dir = f"/proc/{entry}/fd"
        try:
            fds = os.listdir(fd_dir)
        except OSError:
            continue                      # not ours, or exited mid-scan
        for fd in fds:
            try:
                link = os.readlink(f"{fd_dir}/{fd}")
            except OSError:
                continue
            if link == f"socket:[{target_inode}]":
                return int(entry)
    return None


def holder_of(port):
    """Return (pid, cmdline) of the process LISTENing on `port`, or (None, None).

    Parses /proc/net/tcp{,6} rather than shelling out to `ss`, so it works with no
    iproute2 and no sudo. State 0A is TCP_LISTEN.
    """
    for proc_file, addr_len in (("/proc/net/tcp", 8), ("/proc/net/tcp6", 32)):
        try:
            with open(proc_file) as fh:
                lines = fh.read().splitlines()[1:]
        except OSError:
            continue
        for line in lines:
            parts = line.split()
            if len(parts) < 10 or parts[3] != "0A":
                continue
            local = parts[1]
            if ":" not in local:
                continue
            _, port_hex = local.rsplit(":", 1)
            if int(port_hex, 16) != port:
                continue
            pid = _inode_to_pid(parts[9])
            if pid is not None:
                return pid, _proc_name(pid)
            return None, "<held by another user; try `sudo ss -ltnp`>"
    return None, None


def describe_busy(port):
    """A multi-line explanation of who holds `port` and what to do about it."""
    pid, cmd = holder_of(port)
    lines = [f"port {port} is already in use on 0.0.0.0"]
    if pid is not None:
        lines.append(f"    held by PID {pid}: {cmd}")
        lines.append(f"    free it with:  kill {pid}    (then `kill -9 {pid}` if it lingers)")
    elif cmd:
        lines.append(f"    holder: {cmd}")
    else:
        lines.append("    could not identify the holder from /proc; try `ss -ltnp`")
    return "\n".join(lines)


def wait_until_free(port, timeout, poll=0.05):
    """Block until `port` is free or `timeout` seconds elapse. Returns True if free."""
    deadline = time.monotonic() + timeout
    while True:
        if port_is_free(port):
            return True
        if time.monotonic() >= deadline:
            return False
        time.sleep(poll)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--wait", type=float, default=0.0,
                    help="seconds to wait for the port to become free (default 0)")
    args = ap.parse_args()

    free = (wait_until_free(args.port, args.wait) if args.wait > 0
            else port_is_free(args.port))
    if free:
        print(f"port {args.port} is free")
        return 0
    print(describe_busy(args.port), file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
