#!/usr/bin/env python3
"""Prove the Quest can reach this laptop and that hand tracking arrives. No DDS.

This is the first thing to run on site, before the robot is powered. It brings up the
same TeleVuerWrapper the launcher does, shows a moving marker in the headset so the
operator can confirm the image path visually, and then streams the head pose, both
wrist poses, motion_data_ready and the per-hand pinch value.

    python tools/quest_link_check.py                 # then open the printed URL on the Quest

It touches no DDS at all: the gate check greps this file for the Unitree SDK package
name and must find nothing. Nothing here can move a robot.

Certificates come from televuer's own resolution order -- XR_TELEOP_CERT/XR_TELEOP_KEY,
then ~/.config/xr_teleoperate/{cert,key}.pem -- so this tool and the launcher cannot
disagree about which certificate the headset is being asked to trust.

Exit codes: 0 connected and hands tracked, 2 nothing ever connected,
            3 connected but motion_data_ready never became true (hands never tracked).
"""

import argparse
import os
import socket
import sys
import time

import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(REPO_ROOT)

from televuer import TeleVuerWrapper  # noqa: E402

EXIT_OK, EXIT_NO_CONNECT, EXIT_NO_HANDS = 0, 2, 3

IMG_H, IMG_W = 480, 640
VUER_PORT = 8012


def local_ips():
    """Every non-loopback IPv4 address of this host, best effort."""
    found = []
    try:
        import subprocess
        out = subprocess.run(["ip", "-4", "-o", "addr"], capture_output=True, text=True).stdout
        for line in out.splitlines():
            parts = line.split()
            if len(parts) >= 4 and parts[1] != "lo":
                found.append(parts[3].split("/")[0])
    except Exception:
        pass
    if not found:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            found.append(s.getsockname()[0])
            s.close()
        except Exception:
            pass
    return [ip for ip in dict.fromkeys(found) if not ip.startswith("127.")]


def established_peers(port=VUER_PORT):
    """Peers with an ESTABLISHED TCP connection to our port. Diagnostics only.

    Deliberately NOT the connection criterion: a `curl -k https://<ip>:8012` shows up
    here, and that is a reachability test, not a headset. But when the operator can load
    the page and the session still never starts, this is the line that says whether
    anything reached the port at all.
    """
    peers = []
    try:
        with open("/proc/net/tcp") as fh:
            next(fh)
            for line in fh:
                f = line.split()
                local, remote, state = f[1], f[2], f[3]
                if state != "01":                      # 01 = ESTABLISHED
                    continue
                if int(local.split(":")[1], 16) != port:
                    continue
                hexip = remote.split(":")[0]
                ip = ".".join(str(int(hexip[i:i + 2], 16)) for i in (6, 4, 2, 0))
                peers.append(ip)
    except Exception:
        pass
    return peers


def marker_image(t):
    """480x640 BGR with a marker that moves, so a frozen image is obvious in-headset."""
    img = np.zeros((IMG_H, IMG_W, 3), dtype=np.uint8)
    img[:] = (40, 40, 40)
    img[::80, :] = (70, 70, 70)
    img[:, ::80] = (70, 70, 70)
    cx = int((IMG_W / 2) + (IMG_W / 2 - 60) * np.sin(t * 1.2))
    cy = int((IMG_H / 2) + (IMG_H / 2 - 60) * np.sin(t * 0.8))
    y, x = np.ogrid[:IMG_H, :IMG_W]
    img[((x - cx) ** 2 + (y - cy) ** 2) <= 40 ** 2] = (0, 200, 255)
    bar = int((t * 60) % IMG_W)
    img[0:14, max(0, bar - 40):bar] = (0, 255, 0)
    return img


def fmt_vec(v):
    try:
        a = np.asarray(v, dtype=float).reshape(-1)
        if a.size >= 16:                                # a 4x4 pose: take the translation
            a = np.asarray(v, dtype=float).reshape(4, 4)[:3, 3]
        return "[" + " ".join(f"{x:+7.3f}" for x in a[:3]) + "]"
    except Exception:
        return str(v)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--timeout", type=float, default=120.0,
                        help="how long to wait for the headset to connect")
    parser.add_argument("--seconds", type=float, default=30.0,
                        help="how long to stream once connected")
    parser.add_argument("--rate", type=float, default=5.0, help="print rate in Hz")
    parser.add_argument("--display-mode", default="immersive",
                        choices=["immersive", "ego", "pass-through"])
    args = parser.parse_args()

    # Line buffering: this tool spends most of its life waiting, and an operator staring
    # at a redirected log needs the countdown as it happens, not in one burst at exit.
    # It also means a Ctrl-C cannot take the last few lines down with the buffer.
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:
        pass

    cert = os.getenv("XR_TELEOP_CERT")
    key = os.getenv("XR_TELEOP_KEY")
    default_dir = os.path.expanduser("~/.config/xr_teleoperate")
    print("=== Quest link check (no DDS; nothing here can move a robot) ===")
    if cert and key:
        print(f"certificate: XR_TELEOP_CERT={cert}\n             XR_TELEOP_KEY={key}")
    else:
        print(f"certificate: {default_dir}/cert.pem + key.pem (televuer's default)")
        for name in ("cert.pem", "key.pem"):
            path = os.path.join(default_dir, name)
            print(f"             {name}: {'present' if os.path.exists(path) else 'MISSING'}")

    ips = local_ips()
    if not ips:
        print("\nNo non-loopback IPv4 address found. Is this laptop on a network?")
    print("\nOpen ONE of these on the Quest (accept the certificate warning):")
    for ip in ips:
        print(f"    https://{ip}:{VUER_PORT}/?ws=wss://{ip}:{VUER_PORT}")
    print()

    wrapper = TeleVuerWrapper(
        use_hand_tracking=True,
        binocular=False,
        img_shape=(IMG_H, IMG_W),
        display_mode=args.display_mode,
        # zmq=True is what enables televuer's own render loop, the path render_to_xr()
        # feeds. immersive mode refuses to start with both zmq and webrtc off. There is
        # no ZMQ *network* peer here: the image is produced in this process.
        zmq=True,
        webrtc=False,
        cert_file=cert,
        key_file=key,
        arm_reference_mode="head_yaw",
    )

    rc = EXIT_NO_CONNECT
    started = time.monotonic()
    period = 1.0 / args.rate
    try:
        baseline = None
        connected_at = None
        hands_ever = False
        last_note = 0.0
        peers_ever = set()      # a websocket is persistent, but a page load is not:
                                # sampling only at the timeout would miss it entirely

        while True:
            now = time.monotonic()
            wrapper.render_to_xr(marker_image(now - started))
            data = wrapper.get_tele_data()
            head = np.asarray(data.head_pose, dtype=float)
            if baseline is None:
                baseline = head.copy()

            # Connection criterion: actual XR data. A curl cannot produce a head pose.
            moved = bool(np.any(np.abs(head - baseline) > 1e-6))
            if connected_at is None and (moved or data.motion_data_ready):
                connected_at = now
                print(f"websocket is connected  (after {now - started:.1f} s; "
                      f"TCP peers on {VUER_PORT}: {established_peers() or 'none visible'})")
                print(f"\n  {'t':>6}  {'head':<26} {'L wrist':<26} {'R wrist':<26} "
                      f"{'ready':<6} {'pinch L':>8} {'pinch R':>8}")

            if connected_at is None:
                peers_ever.update(established_peers())
                if now - started >= args.timeout:
                    peers = established_peers()
                    print(f"\nTIMEOUT after {args.timeout:g} s: no XR data arrived.")
                    print(f"  ESTABLISHED TCP peers on port {VUER_PORT} right now: "
                          f"{peers or 'none'}")
                    print(f"  peers seen at any point during the wait: "
                          f"{sorted(peers_ever) or 'none'}")
                    if peers_ever:
                        print("  Something reached the port but never sent XR data -- the page "
                              "loaded but the session did not start. Usually the certificate "
                              "was not accepted, or the headset opened http:// not https://.")
                    else:
                        print("  Nothing reached the port at all -- wrong IP, a firewall, or "
                              "the headset is on a different network.")
                    rc = EXIT_NO_CONNECT
                    break
                if now - last_note >= 5.0:
                    last_note = now
                    print(f"  waiting... {args.timeout - (now - started):5.0f} s left"
                          f"   TCP peers on {VUER_PORT} now: "
                          f"{established_peers() or 'none'}"
                          f"   ever: {sorted(peers_ever) or 'none'}")
                time.sleep(0.1)
                continue

            hands_ever = hands_ever or bool(data.motion_data_ready)
            print(f"  {now - connected_at:6.1f}  {fmt_vec(data.head_pose):<26} "
                  f"{fmt_vec(data.left_wrist_pose):<26} {fmt_vec(data.right_wrist_pose):<26} "
                  f"{str(bool(data.motion_data_ready)):<6} "
                  f"{float(data.left_hand_pinchValue):>8.3f} "
                  f"{float(data.right_hand_pinchValue):>8.3f}")

            if now - connected_at >= args.seconds:
                if hands_ever:
                    print(f"\nOK: connected and motion_data_ready went true. "
                          f"Streamed {args.seconds:g} s.")
                    rc = EXIT_OK
                else:
                    print(f"\nCONNECTED BUT NO HAND TRACKING: motion_data_ready never became "
                          f"true in {args.seconds:g} s. The headset is talking to us, but it "
                          f"is not reporting hands -- check that hand tracking is enabled on "
                          f"the Quest and that the hands are in view.")
                    rc = EXIT_NO_HANDS
                break
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nCtrl-C -- closing the wrapper.")
        rc = EXIT_OK
    finally:
        try:
            wrapper.close()
            print("televuer wrapper closed.")
        except Exception as exc:
            print(f"wrapper.close() raised: {exc}")
    return rc


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
