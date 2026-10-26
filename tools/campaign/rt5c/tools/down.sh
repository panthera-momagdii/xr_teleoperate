#!/bin/bash
# Teardown for rt5c/tools/up.sh. q through tmux first (orderly: go home, reap children), then the rest.
C=~/panthera/remote-teleop-20261012; O=${OUT_DIR:-$C/rt5c/out/session}
if tmux has-session -t rt5 2>/dev/null; then
  echo "q sent by agent $(date +%T)" >> $O/teardown.txt; tmux send-keys -t rt5 q
  timeout 10 bash -c "until grep -q 'launcher exited with' $O/launcher_pane.log; do sleep 0.2; done"
fi
tmux kill-session -t rt5 2>/dev/null
ps -eo pid=,args= | awk '$2 ~ /g1visit\/bin\/python$/ && $0 ~ /teleimager.image_server/ {print $1}' | xargs -r kill -INT
ps -eo pid=,args= | awk '$2 ~ /xrt\/bin\/python$/ && $0 ~ /tools\/fake_(lowstate|inspire_state)\.py/ {print $1}' | xargs -r kill -TERM
timeout 15 bash -c 'while ss -ltn | grep -q ":55555 "; do sleep 0.5; done'
ps -eo pid=,args= | awk '$2 ~ /xrt\/bin\/python$/ && ($0 ~ /session_watch\.py/ || $0 ~ /d0_watch\.py/) {print $1}' | xargs -r kill -INT
ps -eo pid=,comm= | awk '$2=="tegrastats"{print $1}' | xargs -r kill -TERM
sleep 3
ps -eo pid=,args= | awk '$2 ~ /xrt\/bin\/python$/ && ($0 ~ /session_watch\.py|d0_watch\.py|fake_|teleop_hand_and_arm/) {print $1}' | xargs -r kill -KILL
tmux kill-session -t rt5img 2>/dev/null; tmux kill-session -t rt5fakes 2>/dev/null; sleep 1
[ -f $O/cam_config_server.yaml.before_rt5c ] && cp -a $O/cam_config_server.yaml.before_rt5c ~/teleimager/cam_config_server.yaml
echo "left: $(ps -eo pid=,args= | grep -E 'teleop_hand|image_server|fake_|session_watch|d0_watch|tegrastats' | grep -v -E 'grep|bash -c' | wc -l) procs"
tmux ls 2>&1 | head -1; ss -ltn | grep -E ':(8012|60000|60001|55555) ' || echo "ports free"
echo "config sha $(sha256sum ~/teleimager/cam_config_server.yaml | cut -c1-16)"
