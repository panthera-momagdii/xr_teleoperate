#!/bin/bash
# RT4 step 4: G1..G6 as they are, then G7. Domain 1, loopback only.
OUT=~/panthera/remote-teleop-20261012/rt5b/out/tests
mkdir -p $OUT; : > $OUT/summary.txt
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
PY=~/miniconda3/envs/xrt/bin/python
cd ~/xrt
PAT='fake_lowstate|fake_inspire_state|fake_hand_state|teleop_hand_and_arm|domain0_census|quest_link_check|port_guard|/tmp/tmp[a-z0-9_]+\.py|test_dex5_failclosed'
for t in g1_failfast g2_hygiene g3_wrist_offset g4_start_pose g5_inspire_slew g6_record g7_link_guard g8_stream_reconnect; do
  s=$(date +%s)
  timeout -k 10 1500 $PY tools/overnight/test_$t.py </dev/null > $OUT/test_$t.log 2>&1
  rc=$?
  sleep 2
  left=$(pgrep -af "$PAT" | grep -v -E 'pgrep|run_tests' | wc -l)
  echo "$(date +%T) test_$t rc=$rc $(( $(date +%s) - s ))s $(tail -1 $OUT/test_$t.log) leftover_procs=$left" | tee -a $OUT/summary.txt
  [ "$left" != "0" ] && pgrep -af "$PAT" | grep -v -E 'pgrep|run_tests' >> $OUT/summary.txt
done
echo DONE >> $OUT/summary.txt
