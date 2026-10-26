#!/bin/bash
# GO P2 (Mohammed): commands 5 to 10 (1 to 4 done at 22:55 to 22:57; command 5 was refused by a lock-check bug) in order; stop at the first command that does not end with RESULT DONE; battery notice at 35 %.
cd /home/unitree/calibration_ws
P=/home/unitree/miniconda3/envs/g1visit/bin/python
export CYCLONEDDS_URI=file:///home/unitree/cyclonedds_ws/cyclonedds.xml
NOTICE=0
batt () { $P -B -c "
import sys,time; sys.path.insert(0,'scripts/lib'); import g1rs_body
b=g1rs_body.BodyMonitor(); b.start(); time.sleep(2.5); print(b.battery()); b.close()" 2>/dev/null | tail -1; }
i=4
while IFS= read -r cmd; do
  i=$((i+1))
  B=$(batt); echo "$(date +%T) battery $B % before command $i"
  if [ "$NOTICE" = "0" ] && [ -n "$B" ] && [ "$B" -le 35 ]; then echo "NOTICE: battery passed 35 % ($B %)"; NOTICE=1; fi
  echo "$(date +%T) CMD $i: $cmd"
  OUT=$($P -B scripts/$cmd 2>&1); echo "$OUT" | grep -E "^RUN |^RESULT|STOP|LOCKED|Traceback" | tail -4
  R=$(echo "$OUT" | grep "^RESULT" | tail -1)
  [ "$R" = "RESULT DONE" ] || { echo "CHAIN STOPPED at command $i ($R)"; break; }
done <<'CMDS'
sweep.py --hand right --axis 3 --speed 100 --live
steps.py --hand right --axis 3 --speeds 100,500,1000 --reps 6 --live
sweep.py --hand left --axis 3 --speed 100 --live
steps.py --hand left --axis 3 --speeds 100,500,1000 --reps 6 --live
rest_log.py --minutes 5 --live
force_zero.py --seconds 30 --live
CMDS
$P - <<'PYEOF'
import json
lk = json.load(open("LOCKS.json"))
lk["live_writes"] = "locked again after P2 command 10 (Mohammed): until GO P3"
json.dump(lk, open("LOCKS.json", "w"), indent=1)
print("live_writes relocked")
PYEOF
echo "$(date +%T) battery $(batt) % at the end"
echo P2_END
