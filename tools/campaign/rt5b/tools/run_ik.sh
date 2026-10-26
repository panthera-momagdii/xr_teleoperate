#!/bin/bash
# RT5b step 2 runner. Usage: run_ik.sh "<video env>"  (chosen setting from step 1)
C=~/panthera/remote-teleop-20261012; O=$C/rt5b/out; X=~/miniconda3/envs/xrt/bin/python; G=~/miniconda3/envs/g1visit/bin/python
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
CFG=~/teleimager/cam_config_server.yaml
VID="$1"
imgsrv() { sed -i -E "0,/^  fps: [0-9]+/s//  fps: $1/" $CFG
  tmux new -d -s rt5bimg "cd ~/teleimager && env PATH=$C/rt1/nosudo:\$PATH $G -m teleimager.image_server 2>&1 | $X -u $C/rt5/tools/stamp.py >> $O/image_server_ik.log"
  timeout 30 bash -c 'until ss -ltn | grep -q ":55555 "; do sleep 0.5; done'; sleep 3; }
stop_imgsrv() { for p in $(ps -eo pid=,args= | awk '$2 ~ /g1visit\/bin\/python$/ && $0 ~ /teleimager.image_server/ {print $1}'); do kill -INT $p; done
  timeout 15 bash -c 'while ss -ltn | grep -q ":55555 "; do sleep 0.5; done'; tmux kill-session -t rt5bimg 2>/dev/null; sleep 1; }
T1="OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1"
run() { env $2 timeout 150 $X $C/rt5b/tools/ik_bench.py $1 20 $3 </dev/null 2>>$O/ik.err | grep '^{' | tee -a $O/ik.jsonl; sleep 2; }
imgsrv 15
run G_follow_video "RT5B_FOLLOW=1 $VID"
run H_follow_video_gil5ms "RT5B_FOLLOW=1 XR_GIL_SWITCH_S=0.005 $VID"
stop_imgsrv
echo IK_DONE >> $O/ik.jsonl
