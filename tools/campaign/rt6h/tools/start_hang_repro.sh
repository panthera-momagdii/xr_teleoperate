#!/bin/bash
# RT6h: repeated launcher starts on fakes (domain 1, lo) to catch the start-up hang after
# "Initialize G1_29_ArmController...". On a hang: gdb thread backtraces of the launcher.
# Usage: start_hang_repro.sh OUTDIR N [ENV=VAL ...]
O=${1:?outdir}; N=${2:-10}; shift 2; mkdir -p $O
X=~/miniconda3/envs/xrt/bin/python
cd ~/xrt
for i in $(seq 1 $N); do
  pane=$O/pane_$i.log; : > $pane
  setsid $X tools/fake_lowstate.py --domain 1 --iface lo --rate 500 --follow-cmd --q-arms 0.0 </dev/null >/dev/null 2>&1 &
  FX=$!
  sleep 2
  tmux new -d -s hrep -x 200 -y 50 "cd ~/xrt/teleop && env PYTHONUNBUFFERED=1 PYTHONNOUSERSITE=1 FH_OUT=$O/fh_$i.txt $* $X $(dirname $0)/fh_wrap.py teleop_hand_and_arm.py --sim --network-interface lo; echo '[hrep] exited'; sleep 5"
  tmux pipe-pane -o -t hrep "cat >> $pane"
  ok=0
  for k in $(seq 1 90); do
    grep -q "Press \[r\]" $pane && { ok=1; break; }
    sleep 0.5
  done
  if [ $ok = 1 ]; then
    echo "$(date +%T) run $i: prompt" >> $O/result.txt
  else
    P=$(pgrep -f "^$X [^ ]*fh_wrap.py teleop_hand_and_arm.py --sim" | head -1)
    echo "$(date +%T) run $i: NO PROMPT, launcher pid $P" >> $O/result.txt
    [ -n "$P" ] && kill -USR1 $P && sleep 2
  fi
  tmux send-keys -t hrep q
  sleep 6
  tmux kill-session -t hrep 2>/dev/null
  kill -TERM -- -$FX 2>/dev/null; sleep 1; kill -KILL -- -$FX 2>/dev/null
  for p in $(ps -eo pid,ppid,args | awk -v X="$X" '$2==1 && $3==X {print $1}'); do kill -9 $p 2>/dev/null; done
  sleep 2
done
echo "$(date +%T) repro done" >> $O/result.txt
