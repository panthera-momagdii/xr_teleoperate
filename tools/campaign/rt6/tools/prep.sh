#!/bin/bash
# RT6 prep: everything that cannot move the robot. Camera config, image server (nosudo stub),
# watchers. Does NOT start the launcher (that is launch.sh).
C=~/panthera/remote-teleop-20261012; O=${RT6_OUT:-$C/rt6/out/session2}; mkdir -p $O; X=~/miniconda3/envs/xrt/bin/python; G=~/miniconda3/envs/g1visit/bin/python
E="PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1"; CFG=~/teleimager/cam_config_server.yaml
[ -f $O/cam_config_server.yaml.before_rt6 ] || { cp -a $CFG $O/cam_config_server.yaml.before_rt6; sha256sum $CFG | cut -c1-16 > $O/cam_config_sha_before.txt; }
sed -i -E "0,/^  fps: [0-9]+/s//  fps: 15/" $CFG
sed -i '41s/enable_webrtc: true/enable_webrtc: false/' $CFG
for try in 1 2; do
  tmux kill-session -t rt6img 2>/dev/null
  tmux new -d -s rt6img "cd ~/teleimager && env $E PATH=$C/rt1/nosudo:\$PATH $G -m teleimager.image_server 2>&1 | $X -u $C/rt5/tools/stamp.py >> $O/image_server.log"
  timeout 30 bash -c 'until ss -ltn | grep -q ":55555 " && ss -ltn | grep -q ":60000 "; do sleep 0.5; done' && break
  echo "image server not up (try $try), retrying"; sleep 3
done
(cd $C/rt5 && env $E nohup setsid $X -I -B tools/d0_watch.py eth0 $O/d0_watch.csv 10 </dev/null > $O/d0_watch.err 2>&1 &)
# RT7: housing [0] warn 75 / stop 80 C, winding [1] warn 110 / stop 120 C (owner, 23 Oct). The
# old single value (RT6_WARN_C / RT6_STOP_C) is gone; the watcher refuses stop values above 80 / 120.
if [ -n "${RT6_WARN_C:-}${RT6_STOP_C:-}" ]; then echo "REFUSED: RT6_WARN_C / RT6_STOP_C are gone; use RT7_HOUSING_* / RT7_WINDING_*"; exit 1; fi
(cd $C/rt6 && env $E ARM_WATCH_HOUSING_WARN_C=${RT7_HOUSING_WARN_C:-75} ARM_WATCH_HOUSING_STOP_C=${RT7_HOUSING_STOP_C:-80} ARM_WATCH_WINDING_WARN_C=${RT7_WINDING_WARN_C:-110} ARM_WATCH_WINDING_STOP_C=${RT7_WINDING_STOP_C:-120} ARM_WATCH_RELEASE_FILE=${RT7_RELEASE_FILE:-$C/rt7/release_request} nohup setsid $X -I -B tools/arm_watch.py $O/arm_watch.csv rt6 </dev/null > $O/arm_watch.out 2>&1 &)
(cd $C/rt5 && env $E nohup setsid $X -B tools/session_watch.py $O/session_watch.csv </dev/null > $O/session_watch.err 2>&1 &)
(nohup setsid tegrastats --interval 1000 --logfile $O/tegrastats.log </dev/null >/dev/null 2>&1 &)
sleep 4; tmux ls; ss -ltn | grep -E ':(8012|60000|55555) ' | awk '{print $4}' | sed 's/.*://' | tr '\n' ' '; echo; tail -1 $O/arm_watch.csv
