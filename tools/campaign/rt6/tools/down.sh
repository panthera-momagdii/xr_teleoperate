#!/bin/bash
# RT6 teardown: q through tmux first (go home, reap), then image server and watchers; config restored.
C=~/panthera/remote-teleop-20261012; O=${RT6_OUT:-$C/rt6/out/session2}
if tmux has-session -t rt6 2>/dev/null && ps -eo args= | grep -q '[t]eleop_hand_and_arm.py --arm'; then
  echo "q sent by agent $(date +%T)" >> $O/teardown.txt; tmux send-keys -t rt6 q
  timeout 15 bash -c "until grep -q 'launcher exited with' $O/launcher_pane.log; do sleep 0.2; done"
fi
tmux kill-session -t rt6 2>/dev/null
ps -eo pid=,args= | awk '$2 ~ /g1visit\/bin\/python$/ && $0 ~ /teleimager.image_server/ {print $1}' | xargs -r kill -INT
timeout 15 bash -c 'while ss -ltn | grep -q ":55555 "; do sleep 0.5; done'
ps -eo pid=,args= | awk '$2 ~ /xrt\/bin\/python$/ && ($0 ~ /session_watch\.py/ || $0 ~ /d0_watch\.py/ || $0 ~ /arm_watch\.py/) {print $1}' | xargs -r kill -INT
ps -eo pid=,comm= | awk '$2=="tegrastats"{print $1}' | xargs -r kill -TERM
sleep 3
ps -eo pid=,args= | awk '$2 ~ /xrt\/bin\/python$/ && ($0 ~ /session_watch\.py|d0_watch\.py|arm_watch\.py|teleop_hand_and_arm/) {print $1}' | xargs -r kill -KILL
tmux kill-session -t rt6img 2>/dev/null; sleep 1
[ -f $O/cam_config_server.yaml.before_rt6 ] && cp -a $O/cam_config_server.yaml.before_rt6 ~/teleimager/cam_config_server.yaml
echo "left: $(ps -eo pid=,args= | grep -E 'teleop_hand|image_server|_watch|tegrastats' | grep -v -E 'grep|bash -c' | wc -l) procs"
tmux ls 2>&1 | head -1; ss -ltn | grep -E ':(8012|60000|60001|55555) ' || echo "ports free"
echo "config sha $(sha256sum ~/teleimager/cam_config_server.yaml | cut -c1-16) (before: $(cat $O/cam_config_sha_before.txt 2>/dev/null))"
