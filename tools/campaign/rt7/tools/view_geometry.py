#!/usr/bin/env python3
"""RT7: what the head camera (URDF d435_link, 47.6 deg down) sees, in the robot's torso frame.
640x480 colour from a 69 x 42 deg 16:9 sensor: 4:3 crop, about 54 x 42 deg. CPU only."""
import os, pickle, numpy as np, pinocchio as pin
m = pickle.load(open(os.path.expanduser("~/xrt/teleop/g1_29_model_cache.pkl"), "rb"))["robot_model"]
d = m.createData(); q = pin.neutral(m); pin.framesForwardKinematics(m, d, q)
F = lambda n: d.oMf[m.getFrameId(n)]
cam = F("d435_link"); tor = F("torso_link"); sh = F("left_shoulder_pitch_link")
VH, HH = np.radians(21.0), np.radians(27.0)          # half FOV vertical, horizontal
R = cam.rotation; ax = R[:, 0]                          # camera x = optical axis (ROS link frame)
pitch = np.degrees(np.arcsin(-ax[2]))
print(f"camera {cam.translation.round(3)} m in the base frame, axis {pitch:.1f} deg below horizontal; "
      f"left shoulder pitch joint {sh.translation.round(3)} m; camera {cam.translation[2] - sh.translation[2]:+.3f} m above it")
top, bot = np.radians(pitch) - VH, np.radians(pitch) + VH
print(f"vertical view: {np.degrees(top):.1f} to {np.degrees(bot):.1f} deg below horizontal; horizontal +-{np.degrees(HH):.0f} deg")
print("ahead of the camera (m) | top edge of view, below the shoulder joint (m) | bottom edge below the shoulder (m) | half width (m)")
for x in (0.2, 0.3, 0.4, 0.5, 0.6):
    zc = cam.translation[2] - sh.translation[2]
    zt, zb = zc - x * np.tan(top), zc - x * np.tan(bot)
    r = np.hypot(x / np.cos(np.radians(pitch)), 0)      # rough range along the axis for width
    print(f"  {x:.1f} | {-zt:+.2f} | {-zb:+.2f} | {x / np.cos(np.radians(pitch)) * np.tan(HH):.2f}")
import sys; sys.stdout.flush(); os._exit(0)
