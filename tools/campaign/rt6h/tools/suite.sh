#!/bin/bash
# RT6h regression suite: every overnight test in the fork, one after another. DOMAIN 1 /
# LOOPBACK and CPU only. Never `tmux kill-server`: only the tests' own sessions are cleaned.
# Usage: suite.sh OUTDIR [test names...]
O=${1:?outdir}; shift; mkdir -p $O
X=~/miniconda3/envs/xrt/bin/python
T=${*:-"g1_failfast g2_hygiene g3_wrist_offset g4_start_pose g5_inspire_slew g6_record g7_link_guard g8_stream_reconnect g9_lower_release g10_torque_cap g11_lower_physics g12_start_gate g13_wrist_start g14_slide_margin g15_cap_bounds_rate"}
: > $O/summary.txt
cd ~/xrt
for t in $T; do
  f=$(ls tools/overnight/test_${t}*.py | head -1)
  echo "$(date +%T) start $t" >> $O/progress.txt
  PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 ${SUITE_ENV:-} timeout 1800 $X -B $f </dev/null > $O/$t.log 2>&1
  rc=$?
  res=$(grep -E "^[0-9]+/[0-9]+ passed" $O/$t.log | tail -1)
  echo "$t exit $rc $res" >> $O/summary.txt
  echo "$(date +%T) done $t exit $rc $res" >> $O/progress.txt
  for s in g7 g8 g9 g10 g11 g12 g13 g14 g15 rt5c; do tmux kill-session -t $s 2>/dev/null; done
  # orphaned logging_mp listeners of the xrt interpreter (PPID 1, idle); interpreter-anchored
  for p in $(ps -eo pid,ppid,args | awk -v X="$X" '$2==1 && $3==X {print $1}'); do kill -9 $p 2>/dev/null; done
  sleep 3
done
echo "$(date +%T) suite done" >> $O/progress.txt
