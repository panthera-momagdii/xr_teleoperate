#!/usr/bin/env python3
"""RT7: the launcher's arm start gate (teleop/robot_control/start_gate.py, pure check) on a
pose read by rt6/tools/arm_pose_read.py. CPU only, nothing on DDS.
Usage: gate_check.py ARM_POSE_JSON"""
import json, os, sys
sys.path.insert(0, os.path.expanduser("~/xrt"))
import numpy as np
from teleop.robot_control import start_gate as sg
from teleop.robot_control.start_pose import load_pose
q = np.array(json.load(open(sys.argv[1]))["arm_q_mean"])
lo, hi = sg.urdf_arm_limits()
last = os.path.expanduser("~/panthera/remote-teleop-20261012/rt6/last_release_pose.yaml")
low = load_pose(os.path.expanduser("~/xrt/poses/low.yaml"))
refuse, notes = sg.check(q, lo, hi, sg.DEFAULT_MARGIN, sg.DEFAULT_MOVED_TOL, last, q_start=low, rest_pose_set=True)
d = np.minimum(q - lo, hi - q)
k = int(np.argmin(d))
print(f"start gate: {'REFUSE: ' + refuse if refuse else 'PASS'}; closest joint {sg.ARM_JOINT_NAMES[k] if hasattr(sg, 'ARM_JOINT_NAMES') else k} "
      f"{d[k]:.3f} rad from its limit (margin {sg.DEFAULT_MARGIN} rad)")
for n in notes:
    print("  notice:", n)
