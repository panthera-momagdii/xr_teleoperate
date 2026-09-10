#!/usr/bin/env bash
# Fail-closed suite driver. DOMAIN 1, LOOPBACK ONLY -- never touches the robot.
# Usage: tools/overnight/run_failclosed.sh [timeout_s]
# cwd must be the repo root; the test itself needs cwd=teleop/.
set -u
TMO="${1:-4}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT/teleop" || exit 1
export HAND_STATE_TIMEOUT_S="$TMO"
PY=/home/mohammed/miniforge3/envs/tv/bin/python
DOM="--domain 1 --iface lo"
fails=0

run () {   # run <label> <expected_rc> <cmd...>
  local label="$1"; shift
  local want="$1"; shift
  echo "--- $label ---"
  timeout 120 "$@" 2>&1 | sed 's/^/    /'
  local rc=${PIPESTATUS[0]}
  if [ "$rc" = "$want" ]; then echo "    EXIT=$rc (expected $want)  OK"
  else echo "    EXIT=$rc (expected $want)  *** MISMATCH ***"; fails=$((fails+1)); fi
  echo
}

echo "##### Dex5 lane (HAND_MODEL=dex5, fixture fake_hand_state.py) #####"
export HAND_MODEL=dex5
run "dex5/preflight/silent (stale-assert case: 1 today, 0 after G5)" 0 \
    $PY ../tools/test_dex5_failclosed.py --target preflight --case silent $DOM

timeout 60 $PY ../tools/fake_hand_state.py $DOM --n 20 --seconds 25 \
    > "$ROOT/logs/overnight/G0_fake_dex5.log" 2>&1 &
FAKE=$!; sleep 3
run "dex5/preflight/dex5 (20 motors, must pass)" 0 \
    $PY ../tools/test_dex5_failclosed.py --target preflight --case dex5 $DOM
run "dex5/readers (preflight must leave no reader)" 0 \
    $PY ../tools/test_dex5_failclosed.py --target readers --case dex5 $DOM
kill $FAKE 2>/dev/null; wait $FAKE 2>/dev/null

timeout 60 $PY ../tools/fake_hand_state.py $DOM --n 7 --seconds 15 \
    > "$ROOT/logs/overnight/G0_fake_dex3.log" 2>&1 &
FAKE=$!; sleep 3
run "dex5/preflight/dex3 (7 motors, must refuse)" 0 \
    $PY ../tools/test_dex5_failclosed.py --target preflight --case dex3 $DOM
kill $FAKE 2>/dev/null; wait $FAKE 2>/dev/null

echo "##### Inspire lane (HAND_MODEL=inspire_ftp, fixture fake_inspire_state.py) #####"
export HAND_MODEL=inspire_ftp
run "inspire/preflight/silent (stale-assert case: 1 today, 0 after G5)" 0 \
    $PY ../tools/test_dex5_failclosed.py --target preflight --case silent $DOM

timeout 60 $PY ../tools/fake_inspire_state.py $DOM --seconds 20 \
    > "$ROOT/logs/overnight/G0_fake_inspire.log" 2>&1 &
FAKE=$!; sleep 3
run "inspire/preflight/dex5 (6 motors, must pass)" 0 \
    $PY ../tools/test_dex5_failclosed.py --target preflight --case dex5 $DOM
kill $FAKE 2>/dev/null; wait $FAKE 2>/dev/null

echo "##### summary: $fails mismatch(es) #####"
exit $((fails > 0))
