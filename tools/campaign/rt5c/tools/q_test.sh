#!/bin/bash
LO=$(printf '%d.%d.%d.%d' 127 0 0 1)   # loopback, built at run time (no addresses in files)
# RT5c step 2: [q] in tmux on fakes, N runs. Domain 1 / lo, --sim. Readers only on domain 1 for evidence.
C=~/panthera/remote-teleop-20261012; O=$C/rt5c/out/qtest; mkdir -p $O; X=~/miniconda3/envs/xrt/bin/python; G=~/miniconda3/envs/g1visit/bin/python
E="PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1"; N=${1:-3}
tmux new -d -s rt5fakes -n lowstate "cd ~/xrt && env $E $X tools/fake_lowstate.py --domain 1 --iface lo --rate 1000 --follow-cmd --q-arms 0.3 2>&1 | $X -u $C/rt5/tools/stamp.py >> $O/fake_lowstate.log"
tmux new-window -t rt5fakes -n inspire "cd ~/xrt && env $E $X tools/fake_inspire_state.py --domain 1 --iface lo 2>&1 | $X -u $C/rt5/tools/stamp.py >> $O/fake_inspire.log"
cd $C/rt5 && env $E nohup setsid $X -B tools/session_watch.py $O/session_watch.csv </dev/null > $O/session_watch.err 2>&1 &
sleep 4
for i in $(seq 1 $N); do
  P=$O/pane_$i.log; : > $P
  tmux new -d -s rt5 -x 200 -y 50 "cd ~/xrt/teleop && env $E XR_ARM_VEL_LIMIT=3 XR_STALE_S=0.5 XR_REENGAGE_T=2.0 XR_DISPLAY_FPS=15 XR_JPEG_QUALITY=60 XR_DISPLAY_SCALE=0.75 $X teleop_hand_and_arm.py --sim --network-interface lo --ee inspire_ftp --img-server-ip $LO; echo \"[rt5] launcher exited with \$?\"; sleep 30"
  tmux pipe-pane -o -t rt5 "$X -u $C/rt5/tools/stamp.py >> $P"
  timeout 120 bash -c "until grep -q 'attach check' $P; do sleep 0.5; done"
  sleep 3; tmux send-keys -t rt5 r; sleep 4
  t0=$(date +%s.%N); tmux send-keys -t rt5 q
  timeout 30 bash -c "until grep -q 'launcher exited with' $P; do sleep 0.1; done"; t1=$(date +%s.%N)
  rc=$(grep -o 'launcher exited with [0-9-]*' $P | awk '{print $4}')
  home=$(grep -c 'both arms have reached the home position' $P); kb=$(grep -c 'keyboard listener did not stop' $P)
  sleep 1
  left=$(ps -eo pid=,args= | awk '$2 ~ /xrt\/bin\/python$/ && $0 ~ /teleop_hand_and_arm.py/' | wc -l)
  port=$(ss -ltn | grep -c ':8012 ')
  lastq=$(tail -3 $O/session_watch.csv | awk -F, '$2>0{print $3}' | tail -1)
  echo "run $i: q->exit $(echo "$t1 - $t0" | bc | cut -c1-5) s, exit=$rc, home_line=$home, kb_timeout_notice=$kb, launcher_procs_left=$left, 8012_listeners=$port" | tee -a $O/summary.txt
  tmux kill-session -t rt5 2>/dev/null; sleep 2
done
awk -F, 'NR>1 && $2>0 {print $1, $3}' $O/session_watch.csv | awk '{print $2}' | sort -u | tr '\n' ' ' > $O/arm_q_values_seen.txt
ps -eo pid=,args= | awk '$2 ~ /xrt\/bin\/python$/ && ($0 ~ /session_watch\.py/ || $0 ~ /fake_(lowstate|inspire_state)\.py/) {print $1}' | xargs -r kill -TERM
sleep 2; tmux kill-session -t rt5fakes 2>/dev/null
ps -eo pid=,args= | awk '$2 ~ /xrt\/bin\/python$/ && ($0 ~ /session_watch\.py|fake_|teleop_hand_and_arm/) {print $1}' | xargs -r kill -KILL
echo "--- left: $(ps -eo pid=,args= | grep -E 'teleop_hand|fake_|session_watch' | grep -v -E 'grep|bash -c' | wc -l) procs; tmux: $(tmux ls 2>&1 | head -1)" | tee -a $O/summary.txt
echo "arm max|q| values seen on rt/lowcmd: $(cat $O/arm_q_values_seen.txt)" | tee -a $O/summary.txt
