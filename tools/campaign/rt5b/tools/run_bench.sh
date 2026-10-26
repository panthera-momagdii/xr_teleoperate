#!/bin/bash
# RT5b step 1 runner. Image server in tmux rt5bimg (nosudo stub); one bench per setting.
C=~/panthera/remote-teleop-20261012; O=$C/rt5b/out; X=~/miniconda3/envs/xrt/bin/python; G=~/miniconda3/envs/g1visit/bin/python
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
CFG=~/teleimager/cam_config_server.yaml
imgsrv() {   # $1 = camera fps
  sed -i -E "0,/^  fps: [0-9]+/s//  fps: $1/" $CFG
  tmux new -d -s rt5bimg "cd ~/teleimager && env PATH=$C/rt1/nosudo:\$PATH $G -m teleimager.image_server 2>&1 | $X -u $C/rt5/tools/stamp.py >> $O/image_server_bench.log"
  timeout 30 bash -c 'until ss -ltn | grep -q ":55555 "; do sleep 0.5; done'; sleep 3
}
stop_imgsrv() {
  for p in $(ps -eo pid=,args= | awk '$2 ~ /g1visit\/bin\/python$/ && $0 ~ /teleimager.image_server/ {print $1}'); do kill -INT $p; done
  timeout 15 bash -c 'while ss -ltn | grep -q ":55555 "; do sleep 0.5; done'; tmux kill-session -t rt5bimg 2>/dev/null; sleep 1
}
bench() {   # label fps quality scale
  env XR_DISPLAY_FPS=$2 XR_JPEG_QUALITY=$3 XR_DISPLAY_SCALE=$4 timeout 90 $X $C/rt5b/tools/video_bench.py $1 $O 15 </dev/null 2>>$O/bench.err | grep '^{' | tee -a $O/bench.jsonl
}
sed -i '41s/enable_webrtc: true/enable_webrtc: false/' $CFG
imgsrv 30
bench cam30_d30_q80_s1   30 80 1.0
stop_imgsrv
imgsrv 15
bench cam15_d15_q80_s1   15 80 1.0
bench cam15_d15_q60_s1   15 60 1.0
bench cam15_d15_q50_s1   15 50 1.0
bench cam15_d15_q70_s075 15 70 0.75
bench cam15_d15_q60_s075 15 60 0.75
bench cam15_d15_q80_s05  15 80 0.5
stop_imgsrv
echo BENCH_DONE >> $O/bench.jsonl
