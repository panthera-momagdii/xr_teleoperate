#!/bin/bash
# RT1 step 8: one IK run = ik_smoke --steps 2000, then ik_timing on the same targets.
# Usage: run_ik.sh <label>. cwd teleop/robot_control, model cache removed before each tool.
L=$1
C=~/panthera/remote-teleop-20261012
O=$C/rt1/out/ik
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
PY=~/miniconda3/envs/xrt/bin/python
cd ~/xrt/teleop/robot_control || exit 1
echo "$(date +%T) [$L] start loadavg $(cat /proc/loadavg)" | tee -a $O/ik_runs.txt
tegrastats --interval 1000 --logfile $O/tegrastats_$L.log &
TG=$!
rm -f g1_29_model_cache.pkl
timeout -k 10 900 $PY ../../tools/ik_smoke.py --steps 2000 --label rt1_$L </dev/null > $O/ik_smoke_$L.log 2>&1
echo "$(date +%T) [$L] ik_smoke rc=$? loadavg $(cat /proc/loadavg)" | tee -a $O/ik_runs.txt
rm -f g1_29_model_cache.pkl
timeout -k 10 900 $PY $C/rt1/tools/ik_timing.py --steps 2000 --label $L --out $O/ik_timing_$L.json </dev/null > $O/ik_timing_$L.log 2>&1
echo "$(date +%T) [$L] ik_timing rc=$? loadavg $(cat /proc/loadavg)" | tee -a $O/ik_runs.txt
rm -f g1_29_model_cache.pkl
kill $TG; wait $TG 2>/dev/null
pkill -KILL -f 'tools/ik_smoke.py' 2>/dev/null; pkill -KILL -f 'rt1/tools/ik_timing.py' 2>/dev/null
echo "$(date +%T) [$L] done" | tee -a $O/ik_runs.txt
