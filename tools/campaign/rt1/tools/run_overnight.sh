#!/bin/bash
# RT1 step 7: run the fork's offline tests as they are. Domain 1, loopback only.
OUT=~/panthera/remote-teleop-20261012/rt1/out/tests
mkdir -p $OUT
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
PY=~/miniconda3/envs/xrt/bin/python
cd ~/xrt
for t in g1_failfast g2_hygiene g3_wrist_offset g4_start_pose g5_inspire_slew g6_record; do
  s=$(date +%s)
  timeout -k 10 900 $PY tools/overnight/test_$t.py </dev/null > $OUT/test_$t.log 2>&1
  rc=$?
  e=$(( $(date +%s) - s ))
  left=$(pgrep -af 'fake_lowstate|fake_inspire_state|fake_hand_state|teleop_hand_and_arm|domain0_census|quest_link_check|port_guard' | grep -v -E 'pgrep|run_overnight' | wc -l)
  echo "$(date +%T) test_$t rc=$rc ${e}s leftover_procs=$left" | tee -a $OUT/summary.txt
  if [ "$left" != "0" ]; then pgrep -af 'fake_lowstate|fake_inspire_state|fake_hand_state|teleop_hand_and_arm|domain0_census|quest_link_check|port_guard' | grep -v -E 'pgrep|run_overnight' >> $OUT/summary.txt; fi
done
echo DONE >> $OUT/summary.txt
