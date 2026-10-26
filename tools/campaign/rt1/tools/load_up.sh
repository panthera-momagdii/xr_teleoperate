#!/bin/bash
# RT1 step 8B load: image server (ZMQ + WebRTC, no viewer) + fakes on domain 1 / lo + recorder.
C=~/panthera/remote-teleop-20261012
L=$C/logs
P=$C/rt1/out/load_pids.txt
: > $P
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
X=~/miniconda3/envs/xrt/bin/python
G=~/miniconda3/envs/g1visit/bin/python
cd ~/teleimager
setsid env PATH=$C/rt1/nosudo:$PATH $G -m teleimager.image_server </dev/null > $L/rt1_image_server.log 2>&1 &
echo "imgsrv $!" >> $P
cd ~/xrt
setsid $X tools/fake_lowstate.py --domain 1 --iface lo --rate 1000 </dev/null > $L/rt1_fake_lowstate.out 2>&1 &
echo "fake_lowstate $!" >> $P
setsid $X tools/fake_inspire_state.py --domain 1 --iface lo </dev/null > $L/rt1_fake_inspire.out 2>&1 &
echo "fake_inspire $!" >> $P
setsid $X tools/record_lowstate.py --label rt1_B --domain 1 --iface lo --out-dir $C/rt1/out/record </dev/null > $L/rt1_record_lowstate.out 2>&1 &
echo "record_lowstate $!" >> $P
cat $P
