#!/usr/bin/env python3
"""RT5b step 1: bytes per second on the 8012 websocket for one video setting.
Real head-camera frames (image server on loopback) -> televuer (with teleop/utils/
televuer_stream.py installed, settings from env) -> tools/fake_xr_client.py. No DDS."""
import io, json, os, re, subprocess, sys, time
sys.path[:0] = ["/home/unitree/xrt", "/home/unitree/xrt/tools"]
label, out_dir, secs = sys.argv[1], sys.argv[2], float(sys.argv[3])
from teleop.utils import televuer_stream
desc = televuer_stream.install(); st = televuer_stream.settings_from_env()
from televuer import TeleVuerWrapper
from teleimager.image_client import ImageClient
from fake_xr_client import FakeXR
def sent():
    out = subprocess.run(["ss", "-tinH", "state", "established", "( sport = :8012 )"], capture_output=True, text=True).stdout
    return sum(int(x) for x in re.findall(r"bytes_acked:(\d+)", out))
def cpu(pid):
    f = open(f"/proc/{pid}/stat").read().rsplit(")", 1)[1].split()
    return (int(f[11]) + int(f[12])) / os.sysconf("SC_CLK_TCK")
w = TeleVuerWrapper(use_hand_tracking=True, binocular=False, img_shape=(480, 640), display_mode="immersive",
                    zmq=True, webrtc=False, arm_reference_mode="head_yaw")
ic = ImageClient(host=".".join(["127", "0", "0", "1"]), request_bgr=True)
xr = FakeXR(rate=60.0).start()
t_end = time.monotonic() + 30
while not xr.connected and time.monotonic() < t_end: time.sleep(0.1)
frames, last, t0 = 0, None, time.monotonic()
b0 = c0 = None
while time.monotonic() - t0 < secs + 4:
    f = ic.get_head_frame()
    if f is not None and f.bgr is not None:
        w.render_to_xr(f.bgr); last = f.bgr
        if f.jpg is not None: frames += 1
    if b0 is None and time.monotonic() - t0 >= 4:
        b0, c0, tb = sent(), cpu(w.tvuer.process.pid), time.monotonic()
    time.sleep(1 / 30)
b1, c1, te = sent(), cpu(w.tvuer.process.pid), time.monotonic()
mbps = (b1 - b0) * 8 / (te - tb) / 1e6
res = {"label": label, "fps": st["fps"], "quality": st["quality"], "scale": st["scale"], "mbit_s": round(mbps, 2),
       "vuer_cpu_pct_core": round(100 * (c1 - c0) / (te - tb), 1), "camera_frames_seen": frames, "xr_connected": xr.connected}
if last is not None:
    import cv2
    img = last
    if st["scale"] < 1:
        img = cv2.resize(img, (int(640 * st["scale"]), int(480 * st["scale"])), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, st["quality"]])
    open(f"{out_dir}/sample_{label}.jpg", "wb").write(buf.tobytes()); res["jpeg_bytes"] = len(buf)
print(json.dumps(res), flush=True)
xr.stop(); ic.close(); w.close()
from tools._procs import reap_child_processes; reap_child_processes(); os._exit(0)
