# calibration_ws: one command per onsite hand test (PC2, built 2026-10-13, session g1rs_20261013_prep)
Every script is DRY-RUN by default: it prints its planned writes and opens no connection to a hand, the camera or DDS.
`--live` is required to run it for real (for the read-only scripts `--live` only means "really read"). Run from this folder with
`P=/home/unitree/miniconda3/envs/g1visit/bin/python`. Each run writes runs/<script>_<date_time>[_dryrun|_mock]/ with config.json
(arguments, tool hashes), console.log, the tool logs (135 Hz state.jsonl, commands.jsonl with every write and its frame) and SHA256SUMS.
Locks: LOCKS.json (live hand writes: locked again after P2 until the P3 GO; below_rest, press, grip, release, thumb motion, wide sweep
range until P3; 1009 re-zero and arm live never). Live hand runs: STATUS / ERROR read, fresh hold right before the block, STATUS 2 / ERROR 0 check, blip, block,
back to the start pose (hold values, start speeds), idle hand watched read only. Proven tools in scripts/lib (copies of
g1rs_20261012b/tools; diffs: DIFF_lib_block_vs_12b.txt, DIFF_lib_block_P3_vs_P2.txt, DIFF_lib_wake_vs_12b.txt, DIFF_lib_hold_vs_12b.txt; P2 block tool kept
in provenance/).
OPERATOR STOP: `touch /home/unitree/calibration_ws/STOP`: every writing tool (hold, block, wake, grip, release) checks it on every read
and right before every write, writes nothing more and ends ABORTED; new live runs refuse while it exists. Remove only on Mohammed's word.
Offline tests (simulated hands, no robot): tests/mock_live_test.py (sweep, steps, wake_test), tests/mock_p3_test.py (below_rest, press,
grip, release, ERROR abort), tests/mock_p3b_test.py (below_rest FORCE_SET, thumb steps, STOP file); results in tests/results_*.txt. Press gate: create CONTINUE or STOP (text = scale reading) in runs/press_*/block/.

| script | command | moves | saves |
|---|---|---|---|
| preflight | `$P scripts/preflight.py [--hold none\|right\|left\|both] [--seconds 10] --live` | nothing; with --hold a hold (no motion) | as-found reads, 10 s rest stats (STATUS, ERROR, temp, force), hold logs |
| sweep | `$P scripts/sweep.py --hand right --axis 3 [--range 650,950] [--speed 100] --live` | 1 finger: blip, 650 -> 950 -> 650 slowly, back | block log 135 Hz, writes, analysis |
| sweep --record | `$P scripts/sweep.py --record --hand left --axis 3 [--floor N] [--top 950] [--dwell 2] --live` | 1 finger = 1 video clip: (no measured floor: floor-finding pass, 25-count steps, stops at the first STATUS 3, a step it cannot reach, or 0), blip held 2 s, 950 and back, FORCE_SET rest + 400 g, floor + 15 and back, speed 100 (LOCKED: sweep_wide_range; measured floor: left index 35) | block log, writes |
| steps | `$P scripts/steps.py --hand right --axis 3 [--speeds 100,500,1000] [--reps 6] --live` | 1 finger: blip, 650 / 950 steps at each speed, back | block log, writes, t90 / delay / settling table |
| steps (thumbs) | `$P scripts/steps.py --hand right --axis 4\|5 [--thumb-step 50] [--pause 2] --live` | bend: +50 toward open and back; rotation: fingers to 950, -50 toward the palm and back, fingers back; speed 100 (LOCKED: thumb_motion) | block log, writes, moves |
| wake_test | `$P scripts/wake_test.py --hand right [--tests R0,R1,R2,R3] --live` | small: blip, single-register writes, moving tests 650 / 950 | wake log both hands, WAKE_TABLE.md per test |
| below_rest | `$P scripts/below_rest.py --hand right --axis 3 [--floor 0] [--step 25] [--pause 2] [--speed 100] --live` | 1 finger: blip, FORCE_SET rest + 400 g (left middle + 500), steps from the hold value down to the floor, back, FORCE_SET back (a touch = STATUS 3 = stop) (LOCKED until P3) | block log, writes, dwell medians per step |
| rest_log | `$P scripts/rest_log.py [--hands right,left] [--minutes 5] [--hold] --live` | none (with --hold: a hold) | positions of both hands at full rate, rest stats |
| tactile_map | `$P scripts/tactile_map.py [--hands right,left] [--baseline-s 60] [--cue --dwell 5] --live` | none | 17 touch arrays per sample; with --cue a CUE time mark per pad |
| force_zero | `$P scripts/force_zero.py [--hands right,left] [--seconds 30] --live` | none (--rezero LOCKED) | FORCE_ACT mean / sd per axis |
| powerup_read | `$P scripts/powerup_read.py [--last-close PATH] --live` | none | all register groups after power-up, angles against the last close |
| press | `$P scripts/press.py --hand right --axis 3 [--levels 200,300,400,500] [--close-to 300] [--gate first\|all] --live` | 1 finger closes toward 300, FORCE_SET = rest + level (each <= 500 g; left middle 400,450,500), operator gate after level 1 (LOCKED until P3) | block log, writes, PRESS_RESULT per level |
| press (scale pose) | `$P scripts/press.py --hand left --pose open --live`, then presses with `--blip-gate --gate all --pause 8 --close-to 200`, then `--pose rest --live` after the scale is removed | fingers 0..3 to 950 and stay; each press starts and opens back at 950; rest values saved in state/scale_pose_<hand>.json (LOCKED: press) | logs, writes, saved rest values |
| scale_test | `$P scripts/scale_test.py [--no-contact-check] --live` | LEFT hand: pre-flight, hold, index then middle: contact search, targets contact-10..-60 (8 s each), open to 950 between; scale kept awake; ends open at 950; packs the run (LOCKED: scale_test, live_writes) | block log 135 Hz with clock time, press marks, tar.gz + sha256 |
| grip | `$P scripts/grip.py --hand right [--level 200] [--close-to 400] [--max-hold-s 10] --live` | fingers 0..3 close (FORCE_SET rest + 200 g, left middle + 400 g), thumbs still, hold, open (LOCKED until P3) | grip log, writes, angles / status / force when closed |
| release | `$P scripts/release.py --hand right [--to 550] [--approve-hold-status-3] --live` | after an aborted press / grip / below_rest: FORCE_SET 2000, fingers below 530 open to 550, speeds 1000 (LOCKED until P3) | log, writes |
| headcam | `$P scripts/headcam.py [--seconds 10] [--every 3] --live` | none | color + depth frames, frames.jsonl time stamps, stream_info.json (intrinsics, resolution) |
| arm_hold, arm_pose, arm_steps | `$P scripts/arm_hold.py` etc. | none: DRY-RUN ONLY, publish nothing | plan |
| pack | `$P scripts/pack.py <session_dir> [--name NAME] --live` | none | /home/unitree/g1rs_<name>.tar.gz + .sha256, SHA256SUMS inside |
