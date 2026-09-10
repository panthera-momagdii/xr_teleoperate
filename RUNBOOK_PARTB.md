# RUNBOOK — Part B, the G1 on the wire

The order that actually worked on **2026-09-09**, with the new env knobs and exit codes
folded in. This supersedes `~/panthera/teleop_host/SPARK_HOST.md` §3, whose arms-only
command is wrong (it still says `--ee dex1`; see `docs/launcher_exit_codes.md`).

Host `gx10-e524` (the Spark). Robot LAN NIC **`enP7s7`**, DDS **domain 0**,
PC2 `192.168.123.164`, PC1 `192.168.123.161`. Headset reaches this box on the **WiFi**
address, `192.168.8.7` (DHCP — check with `ip -br a show wlP9s9`).

---

## 0. Every session, from a fresh shell

```bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate tv
export NIC=enP7s7
cd ~/panthera/xr_teleoperate
```

## 1. Before the robot is powered

```bash
python tools/port_guard.py            # 8012 must be FREE
python tools/cert_info.py             # note the sha256
```

**Write the sha256 down and compare it with the last session.** If it changed, the
Quest's stored certificate exception no longer matches and the headset will refuse to
connect until you accept the new one. That cost a session on Sep 9; see
`logs/overnight/cert_evidence/FINDING.md`.

If `port_guard` reports a holder, kill the PID it names. It is usually an orphaned
`quest_link_check.py` or a launcher from a previous run.

Clean up last session's orphans while you are here:

```bash
pgrep -af 'domain0_census|quest_link_check|teleop_hand_and_arm' | grep -v grep
# kill -9 <pids>
```

## 2. PC2 first

In this order:

1. **D455 UNPLUGGED at PC2 boot.** Boot PC2 with the camera disconnected, then plug it
   in once PC2 is up. Plugged in at boot it does not enumerate reliably.
2. Start the **Modbus↔DDS bridge** (`docs/next_visit_pc2.md`). Without it the Inspire
   state topics are silent and the launcher refuses with **exit 3** — correctly.
3. Start the **image server**.
4. Start **tempwatch**.

Confirm from this box before going further:

```bash
python tools/domain0_census.py --domain 0 --iface "$NIC" --seconds 20
```

Expect `rt/lowstate` at ~500 Hz and, if the bridge is up,
`rt/inspire_hand/state/{l,r}`.

## 3. Port check, then the launcher — launcher FIRST, headset second

This order matters. The launcher must own 8012 before the headset connects.

**Arms only — do this first, every session. NO `--ee`:**

```bash
cd ~/panthera/xr_teleoperate/teleop
XR_ARM_VEL_LIMIT=5 \
python teleop_hand_and_arm.py \
    --arm G1_29 \
    --network-interface "$NIC" \
    --img-server-ip 192.168.123.164
```

> `--ee` is **optional**, and omitting it is a first-class mode. This is the command
> that worked three times on Sep 9 (hanging robot, development mode, velocity limit 3
> and 5 rad/s). Passing `--ee dex1` on this robot used to hang forever; it now exits 3.

**Arms + Inspire RH56E2-T1 hands** (the hands actually fitted), once the bridge is up:

```bash
XR_ARM_VEL_LIMIT=5 \
python teleop_hand_and_arm.py \
    --arm G1_29 --ee inspire_ftp \
    --network-interface "$NIC" \
    --img-server-ip 192.168.123.164
```

Read the first three lines. They are printed unwrapped, on stderr:

```
[cert] /home/mohammed/.config/xr_teleoperate/cert.pem (via ~/.config/xr_teleoperate) sha256=... SAN=[...]
[xr] port 8012 is free
[wrist] mapping is 1:1 and unshifted (...)
[start-pose] disabled (set XR_START_POSE=<yaml> to enable)
```

Then wait for:

```
🟢  Press [r] to start syncing the robot with your movements.
```

**Do not press anything yet.**

## 4. Headset

1. On the Quest, open the browser and reload
   **`https://192.168.8.7:8012/?ws=wss://192.168.8.7:8012`**
2. Accept the certificate warning. If it will not offer one, clear the site data for
   `192.168.8.7` and reload — that is the stale-exception case from §1.
3. Wait until hand tracking is live (you can see your hands rendered).

## 5. Run

| key | effect |
|---|---|
| `r` | start following. **The arms move on this keypress.** |
| `s` | start / save recording (only with `--record`) |
| `q` | stop, go home, exit |

**e-stop: `L2 + B` on the controller.** Know where it is before you press `r`.

Stop immediately if:

* an arm moves when you are not moving;
* an arm keeps going after you stop;
* the robot's own tracking of your hands lags visibly;
* any joint reaches a limit and stays there;
* a temperature warning appears from tempwatch.

## 6. The knobs, and what they are for

All default to today's behaviour, so a session with none of them set is unchanged.

| env | default | use it when |
|---|---|---|
| `XR_ARM_VEL_LIMIT` | `30.0` rad/s | **always set this to 5 or lower** until the mapping is trusted |
| `XR_WRIST_Z_OFFSET` | `0.0` m | **vertical tracking clips — start at `0.20`** (see below) |
| `XR_WRIST_X_OFFSET` | `0.0` m | the robot reaches too near / too far forward |
| `XR_WRIST_Z_SCALE` | `1.0` | you run out of vertical travel before the robot does |
| `XR_WRIST_XY_SCALE` | `1.0` | same, horizontally |
| `XR_START_POSE` | unset | you want `r` to go to a known pose first |
| `XR_START_T` | `3.0` s | how long the approach to that pose takes |
| `XR_BLEND_T` | `2.0` s | how long the blend into your motion takes |
| `XR_HAND_SLEW` | `1500` units/s | the fingers move too fast / too slow |
| `XR_HAND_WAIT_S` | `10` s | the bridge is slow to come up |
| `XR_VUER_PORT` | `8012` | 8012 is taken by something you cannot kill |

### The vertical fix — the one number from last night

At `x = 0.30 m` in front of it, the G1 wrist can reach **`z = +0.05 … +0.55 m`**
relative to the IK base. An operator standing with hands at belly height produces a
target at **`z = −0.05`** — *below the reachable band entirely*. That is the vertical
clipping, and it is why holding your hands up awkwardly high made it work.

```bash
XR_ARM_VEL_LIMIT=5 XR_WRIST_Z_OFFSET=0.20 python teleop_hand_and_arm.py ...
```

`0.20` puts the chest/ready posture at the centre of the band. Adjust by feel. Leave
`XR_WRIST_Z_SCALE` at 1.0 to begin with — the band is 0.50 m tall and your comfortable
vertical travel is about the same. Full reasoning and figures: `docs/xr_frames.md`,
`logs/overnight/reach_map_side_x0p30.png`.

### A fixed start pose

```bash
XR_START_POSE=poses/ready.yaml XR_ARM_VEL_LIMIT=5 python teleop_hand_and_arm.py ...
```

`r` then moves to `poses/ready.yaml` over 3 s at the velocity limit, blends into your
motion over 2 s, and follows normally after that. **`poses/ready.yaml` is a derived
guess.** Capture the real one off the robot instead:

```bash
# put the arms where you want them, hold them still
python tools/capture_pose.py --domain 0 --iface "$NIC" --out poses/tray.yaml
```

That tool is read-only — it installs a guard that makes constructing a DDS writer raise,
so it cannot command anything even by accident.

## 7. Exit codes

| code | meaning | what to do |
|---|---|---|
| `0` | clean exit, `q` pressed | — |
| `1` | an exception escaped | read the traceback |
| `2` | bad arguments, or a bad pose/knob value | read the message; it names the joint or the env var |
| `3` | **an `--ee` was given and its state never arrived** in `XR_HAND_WAIT_S` | the hand is not fitted/powered, or the bridge is down, or the NIC is wrong. **For arms-only, drop `--ee`.** |
| `4` | **8012 is already held** | the message names the PID. `kill` it. |

## 8. Recording

```bash
... --record --task-dir ./utils/data/ --task-name pick_cube
```

Works with **and without** `--ee`. The episode records which it was, in
`info.source.end_effector.present`. Layout and shapes: `docs/episode_format.md`.

`s` starts, `s` again saves. **Press `s` to save before `q`** — killing the launcher
mid-episode leaves an unclosed JSON array.

## 9. Shutting down

1. `q` — the arms go home. Watch them; `ctrl_dual_arm_go_home` gives up silently after
   5 s if they do not arrive.
2. Check nothing is left behind:
   ```bash
   pgrep -af 'teleop_hand_and_arm|quest_link_check|domain0_census'
   python tools/port_guard.py
   ```
3. **The robot stays in debug mode.** `Exit_Debug_Mode()` is commented out in the
   launcher's `finally` (upstream's choice, `teleop_hand_and_arm.py`). Power-cycle or
   re-enter normal mode deliberately.

## 10. If the headset will not connect

In this order:

1. `python tools/cert_info.py` — did the sha256 change since it last worked?
2. `python tools/port_guard.py` — is the launcher actually holding 8012?
3. Is the URL the **WiFi** address? `ip -br a show wlP9s9`. The wired and Tailscale
   addresses are not reachable from the headset.
4. `python tools/quest_link_check.py --port 8013` — this now runs *alongside* the
   launcher instead of fighting it for 8012. Exit 0 = connected and hands tracked,
   2 = nothing connected, 3 = connected but no hand tracking (usually the certificate),
   4 = the port was busy.
