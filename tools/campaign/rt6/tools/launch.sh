#!/bin/bash
# RT6 launcher: DOMAIN 0, the real robot. Arms only: no --sim, no --ee, no --motion.
# Starting it moves the arms from where they are to q = 0 at XR_ARM_VEL_LIMIT (arm controller start).
C=~/panthera/remote-teleop-20261012; O=${RT6_OUT:-$C/rt6/out/session2}; X=~/miniconda3/envs/xrt/bin/python
E="PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1"
KN="XR_ARM_VEL_LIMIT=${RT6_VEL:-3} XR_STALE_S=0.5 XR_REENGAGE_T=2.0 XR_DISPLAY_FPS=15 XR_JPEG_QUALITY=60 XR_DISPLAY_SCALE=0.75 XR_START_POSE=${RT6_START_POSE:-/home/unitree/xrt/poses/low.yaml} XR_LAST_POSE_FILE=$C/rt6/last_release_pose.yaml ${RT6_EXTRA_ENV:-}"
LO=$(printf '%d.%d.%d.%d' 127 0 0 1)   # loopback, built at run time (no addresses in files)
# RT7: release at once (zero torque) on chatter: the arm watcher writes this file, the arm
# controller polls it. A file left from an earlier run is removed here (and the controller
# ignores one older than itself).
REL=${RT7_RELEASE_FILE:-$C/rt7/release_request}; rm -f "$REL"; KN="$KN XR_RELEASE_FILE=$REL"
LCMD="cd ~/xrt/teleop && env $E $KN $X teleop_hand_and_arm.py --arm G1_29 --network-interface eth0 --img-server-ip $LO ${RT6_ARGS:-}; echo \"[rt6] launcher exited with \$?\"; sleep 120"
echo "launch $(date +%T) env: $KN" >> $O/launch.txt
tmux new -d -s rt6 -x 200 -y 50 "$LCMD"; sleep 1; tmux has-session -t rt6 || { echo "tmux rt6 did not start"; exit 1; }
tmux pipe-pane -o -t rt6 "$X -u $C/rt5/tools/stamp.py >> $O/launcher_pane.log"
timeout 120 bash -c "until grep -q -E 'attach check|launcher exited|REFUSING' $O/launcher_pane.log 2>/dev/null; do sleep 0.5; done"
grep -E '\[start-pose\]|\[wrist\]|\[link\] guard|\[video\]|Enter debug mode|Lock OK|Press \[r\]|attach check|REFUSING|launcher exited' $O/launcher_pane.log | cut -c1-130
