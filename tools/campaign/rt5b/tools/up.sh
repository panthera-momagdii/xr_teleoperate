#!/bin/bash
LO=$(printf '%d.%d.%d.%d' 127 0 0 1)   # loopback, built at run time (no addresses in files)
# RT5b step 6 bring-up: RT5 setup with the RT5b settings. Domain 1 / lo, --sim.
C=~/panthera/remote-teleop-20261012; O=$C/rt5b/out/${RECHECK_DIR:-recheck}; mkdir -p $O; X=~/miniconda3/envs/xrt/bin/python; G=~/miniconda3/envs/g1visit/bin/python
E="PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1"; CFG=~/teleimager/cam_config_server.yaml
sed -i -E "0,/^  fps: [0-9]+/s//  fps: 15/" $CFG
sed -i '41s/enable_webrtc: true/enable_webrtc: false/' $CFG
cd $C/rt5 && env $E nohup setsid $X -I -B tools/d0_watch.py eth0 $O/d0_watch.csv 10 </dev/null > $O/d0_watch.err 2>&1 &
tmux new -d -s rt5fakes -n lowstate "cd ~/xrt && env $E $X tools/fake_lowstate.py --domain 1 --iface lo --rate 1000 --follow-cmd 2>&1 | $X -u $C/rt5/tools/stamp.py >> $O/fake_lowstate.log"
tmux new-window -t rt5fakes -n inspire "cd ~/xrt && env $E $X tools/fake_inspire_state.py --domain 1 --iface lo 2>&1 | $X -u $C/rt5/tools/stamp.py >> $O/fake_inspire.log"
tmux new -d -s rt5img "cd ~/teleimager && env $E PATH=$C/rt1/nosudo:\$PATH $G -m teleimager.image_server 2>&1 | $X -u $C/rt5/tools/stamp.py >> $O/image_server.log"
timeout 30 bash -c 'until ss -ltn | grep -q ":55555 "; do sleep 0.5; done'; sleep 2
cd $C/rt5 && env $E nohup setsid $X -B tools/session_watch.py $O/session_watch.csv </dev/null > $O/session_watch.err 2>&1 &
nohup setsid tegrastats --interval 1000 --logfile $O/tegrastats.log </dev/null >/dev/null 2>&1 &
tmux new -d -s rt5 -x 200 -y 50 "cd ~/xrt/teleop && env $E XR_ARM_VEL_LIMIT=3 XR_STALE_S=0.5 XR_REENGAGE_T=2.0 XR_DISPLAY_FPS=15 XR_JPEG_QUALITY=60 XR_DISPLAY_SCALE=0.75 $X teleop_hand_and_arm.py --sim --network-interface lo --ee inspire_ftp --img-server-ip $LO; echo \"[rt5] launcher exited with \$?\"; sleep 60"
tmux pipe-pane -o -t rt5 "$X -u $C/rt5/tools/stamp.py >> $O/launcher_pane.log"
timeout 120 bash -c "until grep -q 'Press \[r\]' $O/launcher_pane.log; do sleep 1; done"
grep -E 'attach check|\[video\]|\[link\] guard' $O/launcher_pane.log | head -3
tmux ls
