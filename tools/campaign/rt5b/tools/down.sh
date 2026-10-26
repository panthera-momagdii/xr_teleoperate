#!/bin/bash
# RT5b teardown. q through tmux; if the launcher has not exited in 15 s, record its threads, then SIGHUP via kill-session.
C=~/panthera/remote-teleop-20261012; O=$C/rt5b/out/${RECHECK_DIR:-recheck2}
L=$(ps -eo pid=,args= | awk '$2 ~ /xrt\/bin\/python$/ && $0 ~ /teleop_hand_and_arm.py/ {print $1}' | head -1)
echo "q sent by agent $(date +%T), launcher pid $L" >> $O/teardown.txt
tmux send-keys -t rt5 q
for i in $(seq 1 30); do [ -n "$L" ] && [ -d /proc/$L ] || break; sleep 0.5; done
if [ -n "$L" ] && [ -d /proc/$L ]; then
  echo "launcher still alive 15 s after q; threads:" >> $O/teardown.txt
  for t in /proc/$L/task/*; do echo "  $(basename $t) $(cat $t/comm) $(cat $t/wchan) $(awk '{print $3}' $t/stat)" >> $O/teardown.txt; done
  ps -eo pid=,ppid=,args= | awk -v p=$L '$2==p' | cut -c1-120 >> $O/teardown.txt
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
cp -a $C/rt5b/out/cam_config_server.yaml.before_rt5b ~/teleimager/cam_config_server.yaml
echo "--- left:"; ps -eo pid=,args= | grep -E 'teleop_hand|image_server|fake_|session_watch|d0_watch|tegrastats' | grep -v -E 'grep|bash -c'
tmux ls 2>&1 | head -1; ss -ltn | grep -E ':(8012|60000|60001|55555) ' || echo "ports free"; lsmod | grep '^uvcvideo'
echo "config sha $(sha256sum ~/teleimager/cam_config_server.yaml | cut -c1-16)"
cat $O/teardown.txt
