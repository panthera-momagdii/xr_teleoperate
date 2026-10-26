# `teleop_hand_and_arm.py` exit codes, and the correction to SPARK_HOST.md §3

Added by the **G1** patch, 2026-09-09.

## The correction

`~/panthera/teleop_host/SPARK_HOST.md` §3 line 145 still reads:

```bash
XR_ARM_VEL_LIMIT=5 python teleop_hand_and_arm.py \
    --arm G1_29 --ee dex1 \                       # <-- WRONG
    --network-interface "$NIC" \
    --img-server-ip <pc2_lan_ip>
```

**`--ee dex1` is wrong and it cost a session.** This robot has no Dex1 gripper, so
`Dex1_1_Gripper_Controller` sat in an unbounded `while not self.gripper_sub_ready`
loop ([robot_hand_unitree.py:589-591](../teleop/robot_control/robot_hand_unitree.py#L589-L591))
printing `Waiting to subscribe dds...` at 100 Hz and never returned.

The arms-only command has **no `--ee` at all**:

```bash
XR_ARM_VEL_LIMIT=5 python teleop_hand_and_arm.py \
    --arm G1_29 \
    --network-interface "$NIC" \
    --img-server-ip <pc2_lan_ip>
```

`--ee` is optional; omitting it is a first-class mode and is exactly what worked three
times on 2026-09-09 (hanging robot, development mode, velocity limit 3 and 5 rad/s,
`r` starts, `q` goes home).

> SPARK_HOST.md lives outside this repository, so this correction could not be shipped
> as part of a patch. Either apply the two-line edit there by hand, or treat
> `RUNBOOK_PARTB.md` (G7) as superseding §3 — that is the intent.

## Exit codes

| code | meaning | what to do |
|---|---|---|
| `0` | clean exit, `q` pressed | — |
| `1` | an exception escaped to the top level | read the traceback |
| `2` | bad arguments (argparse's own code) | read the usage line |
| `3` | **an `--ee` was given and its state topic never arrived** within `XR_HAND_WAIT_S` (default 10 s) | the end effector is not fitted or not powered, or its bridge is down, or `--network-interface` names the wrong NIC. **For arms-only work, drop `--ee`.** |
| `4` | **the XR port (8012) is already held** by another process | the refusal names the PID and its full command line. `kill <pid>`. Usually a `quest_link_check.py` left running, or an orphaned launcher from a previous run. |
| `5` | **SIGHUP: the operator's terminal went away** (RT4) | orderly stop, same as `q`: go home at `XR_ARM_VEL_LIMIT`, children reaped. The shutdown log is `logs/launcher_hup_<pid>.log`. Run the launcher in tmux on PC2 and an SSH drop never sends this. |
| `6` | **SIGTERM** (RT4) | the same orderly stop as `q`. |
| `7` | **start gate (RT6f): an arm joint is within `XR_GATE_LIMIT_MARGIN` (default 0.05 rad) of a URDF limit** | the refusal names each joint, its angle, the limit and the distance. Checked right after DDS init, before debug mode and before the arm controller, so nothing was commanded and the pose was not recorded as the rest pose. The arms were probably moved by hand: someone at the robot moves them clear, then start again. |

These are stable; scripts may depend on them.

**Start gate warning (RT6f, not an exit):** with `XR_LAST_POSE_FILE` set, the launcher saves the
measured arm pose after every completed lowering, and at the next start warns `[gate] WARNING:
arms were moved since the last session` for each joint more than `XR_GATE_MOVED_TOL` (default
0.2 rad) away from it. It still starts.

### Why exit 3 exists

Five of the seven `--ee` families build a controller that waits for its state topic in
an **unbounded** loop, so a missing end effector hung the launcher forever instead of
refusing. `hand_config.EE_STATE_TOPICS` lists all five with file and line. The launcher
now runs a bounded check for **every** `--ee` family before
`MotionSwitcher().Enter_Debug_Mode()` — the same placement, and for the same reason, as
the existing `hand_config.preflight()`: once debug mode is entered, refusing costs a
go-home with the arms released.

### Why exit 4 exists

Vuer binds 8012 inside its own aiohttp startup thread and the launcher never learns
that the bind failed. On 2026-09-09 `quest_link_check.py` was still holding the port;
the launcher ran on with a dead XR path, and pressing `r` moved the arms to televuer's
fallback pose (`CONST_LEFT_ARM_POSE`/`CONST_RIGHT_ARM_POSE`,
[tv_wrapper.py:161-169](../teleop/televuer/src/televuer/tv_wrapper.py#L161-L169))
instead of following the operator. A robot moving on its own, unfollowable, is the
worst failure mode in this system, so the launcher now refuses to start at all.

`XR_VUER_PORT` overrides the port that is checked, for the rare case where vuer has
been pointed somewhere else.

## The startup line you should read every time

```
[cert] /home/mohammed/.config/xr_teleoperate/cert.pem (via ~/.config/xr_teleoperate) sha256=9e197f549d265750... SAN=[localhost, panthera.local, ...]
[xr] port 8012 is free
```

If that sha256 differs from the last session, **the Quest's stored certificate
exception no longer matches** and the headset will refuse to connect until you accept
the new certificate once. That is exactly what happened on 2026-09-09; see
`logs/overnight/cert_evidence/FINDING.md`.

Standalone equivalents:

```bash
python tools/cert_info.py      # resolved path, sha256, subject, expiry, SAN list
python tools/port_guard.py     # is 8012 free? if not, which PID holds it?
```

## A note on reading the launcher's log

`logging_mp` renders through `rich`, which word-wraps every message into a narrow
column, **breaks long tokens across lines** (`rt/dex1/left/stat e`,
`/home/.../cert.pe m`) and **injects the source-location gutter into the middle of the
first line**. Do not grep the log for a full sentence, and do not trust a topic name
copied out of it.

Everything an operator has to act on — the certificate line, the port refusal, the
end-effector refusal, and the `[r]/[s]/[q]` prompt — is therefore *also* printed
verbatim to stderr by `notice()`, unwrapped. Those are the lines to read and the lines
a script should match.

## RT4: link loss, re-engage, and what ends a session (added 18 Oct 2026)

### Link-loss guard (`teleop/robot_control/link_guard.py`)

| knob | default | meaning |
|---|---|---|
| `XR_STALE_S` | `0.5` | no new wrist pose for this long means HOLD |
| `XR_REENGAGE_T` | `2.0` | blend time from the hold pose back to the operator |

1. No new wrist pose for `XR_STALE_S`: the arms hold the last command, the hands hold
   their last target. The launcher prints `[link] HOLD: ...`.
2. The stream comes back: nothing moves. The launcher prints `[link] XR stream is back.
   Still holding: press [r] to re-engage.`
3. `[r]` with a live stream: blend from the hold pose to the operator over
   `XR_REENGAGE_T`, every step inside `XR_ARM_VEL_LIMIT / --frequency`, feed-forward
   torque zero during the blend. Then `[link] re-engaged`.
4. `[r]` with a stale stream: refused, still holding.
5. `[r]` pressed with no live stream at all: HOLD at the arms' measured position, not
   at televuer's fallback pose.

"Fresh" means the raw wrist bytes changed since the last loop. Real hand tracking never
repeats bit for bit; a stalled websocket does.

Why 0.5 s: RT2 measured PC2 to Giza at median 177 ms, p95 187 ms, max 259 ms, jitter
5.24 ms. 0.5 s is about twice the worst RTT seen, and 15 frames at 30 Hz.

### What ends a session

| event | what happens | exit |
|---|---|---|
| `q` | go home, children reaped | 0 |
| Ctrl-C / SIGINT | children reaped, then go home (a stop flag, not KeyboardInterrupt: CasADi swallows that inside IPOPT) | 0 |
| SIGTERM | children reaped, then go home | 6 |
| SSH dies (SIGHUP) | output moves to `logs/launcher_hup_<pid>.log`, go home, children reaped | 5 |
| launcher SIGKILLed | no go-home (nothing can run); every child dies at once by `PR_SET_PDEATHSIG` | -9 |

All five are tested on fakes by `tools/overnight/test_g7_link_guard.py`: after each one,
nothing publishes on `rt/lowcmd` or `rt/inspire_hand/ctrl/*`, no process of the
launcher's session is left, and 8012 is bindable.

Recommended for remote work: run the launcher in tmux on PC2. Then an SSH drop does not
reach the launcher at all; if the headset also drops, the guard holds, and the operator
re-attaches (`tmux attach`) and presses `r` or `q`.
