# Next visit — get the Inspire hands talking to PC2

The hands are reachable on the robot LAN and **nothing on PC2 can speak to them**. That is
the whole gap. It is a bounded software gap, not a hardware fault: both hands answer ping
in under half a millisecond and both show a green power LED.

Read [`inspire_rh56e2.md`](inspire_rh56e2.md) first — it is the sourced account of what
these hands are and what drives them. This file is the plan.

## 0. Before anything

```bash
python tools/domain0_census.py --domain 0 --iface <nic> --seconds 30
```

**A STOP verdict means there is no Part B.** Something else is driving the robot, or is
about to. The census was wrong about this on 2026-08-28 and produced a false STOP; that is
fixed (g14e), and it now passes on a robot where PC1 declares an idle `rt/arm_sdk`
endpoint, which is this robot's normal shape.

## 1. The bridge — install on PC2, from our fork

The fork exists and the patch is done: **`panthera-momagdii/inspire_hand_ws`**, branch
`panthera/pc2-headless`, based on `fc75490` (verified with `git ls-remote` to be the tip of
`NaCl-1374/inspire_hand_ws` master). Do **not** install `NaCl-1374/inspire_hand_ws`
directly.

> At `fc75490`, `inspire_sdkpy/__init__.py` eagerly imported `ModbusDataHandler` **and**
> `qt_tabs`, so importing the IDL dragged in `unitree_sdk2py`, `pyqtgraph`, `PyQt5` and
> `colorcet`. There was no import path around it — importing a submodule still runs the
> package `__init__`. PC2 is a headless aarch64 Jetson; a Qt stack there is a long build
> and a pointless dependency.

Patched: those four names now resolve through a PEP 562 `__getattr__` on first attribute
access, and `setup.py` moves `PyQt5`/`pyqtgraph`/`colorcet` into `extras_require["gui"]`.
`__all__` is unchanged and, with Qt present, the resolved package namespace is identical to
pristine `fc75490`. See `README_PANTHERA.md` in the fork.

`unitree_sdk2py` is a **prerequisite, not a dependency** — the bridge imports it to publish,
but it is not on PyPI so it cannot go in `install_requires`. It should already be present on
PC2 for `xr_teleoperate`. Check before anything else:

```bash
python -c "import unitree_sdk2py, cyclonedds; print('prereqs ok')"
```

Then:

```bash
# on PC2, in the conda env (Python 3.10, conda-forge only -- the Anaconda default
# channels demand a ToS acceptance, which is a licensing decision, not a technical one)
git clone https://github.com/panthera-momagdii/inspire_hand_ws.git ~/panthera/inspire_hand_ws
cd ~/panthera/inspire_hand_ws && git checkout panthera/pc2-headless

python -m pip install pymodbus==3.6.9
python -m pip install inspire_hand_sdk --no-deps      # NOT -e -- see below
python -c "from inspire_sdkpy import inspire_dds, inspire_hand_defaut; print('ok')"
```

Two flags, both deliberate:

* **`--no-deps` is required, not tidiness.** Upstream pins `cyclonedds==0.10.2`; without
  `--no-deps` pip will try to move PC2's cyclonedds — the one `unitree_sdk2py` is already
  using — onto that version. The pin was left alone on purpose; it is not testable from an
  x86_64 laptop.
* **No `-e`.** The fork is pinned and changes rarely, so an editable install buys nothing,
  and pip's *default* editable mode actively breaks on this project. `inspire_hand_sdk` has
  a `setup.py` and no `pyproject.toml`, so `pip install -e` uses setuptools' **strict** mode,
  which does not put the source directory in the `.pth` file — it writes a
  `build/__editable__.inspire_sdkpy-1.0.0-py3-none-any/` tree inside the project and points
  the `.pth` at *that*. `build` is gitignored, so a `git clean` or the next reinstall round
  removes it silently and the `.pth` dangles. The metadata still lives in site-packages, so
  you get `pip show inspire_sdkpy` succeeding while `import inspire_sdkpy` raises
  `ModuleNotFoundError` — which cost a debugging session on the laptop on 2026-09-01. A
  plain install copies the package into site-packages and has none of this. Re-run the same
  command after a `git pull`. If you ever do need `-e`, pass
  `--config-settings editable_mode=compat`.

Verified on the laptop in a bare Python 3.10 venv holding only `pymodbus==3.6.9`,
`cyclonedds` and `numpy`, with `PyQt5`, `pyqtgraph`, `colorcet` and `unitree_sdk2py` all
confirmed absent: the import above succeeds, the DDS types round-trip through CDR, and
`angle_act` is 6-wide per hand — which is the 12-wide `dual_hand_state_array` after
`concatenate(left, right)`.

**Two upstream defects were found and deliberately not patched** (fixing them would change
bridge behaviour):

* `network='<nic>'` **is silently ignored.** The branch in `inspire_sdk.py:77-81` and
  `inspire_sdk_double.py:73-77` is inverted, so both paths reach cyclonedds'
  auto-determine. Set the NIC yourself instead, before constructing the handler, and pass
  `initDDS=False`:

  ```python
  from unitree_sdk2py.core.channel import ChannelFactoryInitialize
  ChannelFactoryInitialize(0, "<robot-lan-nic>")   # once per process
  handler = inspire_sdk.ModbusDataHandler(ip="192.168.123.210", LR="l",
                                          device_id=1, initDDS=False)
  ```

  `CYCLONEDDS_URI` is **not** a workaround — the SDK builds `Domain(id, config)` with an
  inline XML config, which takes precedence.

* **A DDS init failure is swallowed** and `__init__` returns before `self.pub` exists, so
  the constructor looks like it succeeded and the first `read()` raises
  `AttributeError: ... has no attribute 'pub'`. If you see that, scroll up for
  `Error during ChannelFactory initialization:`.

No root is needed for any of this. The bridge talks TCP to the hands and DDS to the
domain; it needs no kernel modules and no device nodes.

**Check first whether Unitree ships its own build.** Their documentation says they provide
an example bridge, but their doc centre is JavaScript-rendered and could not be read from
the laptop. Prefer theirs if it exists.

## 2. Configure it — and resolve left/right by reading, not guessing

The bridge is `inspire_sdk.ModbusDataHandler(ip=..., LR='l'|'r', device_id=1)`, driven by
`example/Headless_driver_{l,r}.py`. The `*_485_*` variants are for the serial hands and
are not ours.

| setting | value | note |
|---|---|---|
| hand addresses | `192.168.123.210`, `192.168.123.211` | **which is which is UNKNOWN** |
| TCP port | `6000` | not 502. The factory default address is `192.168.11.210`; ours have been readdressed |
| `device_id` | `1` | as in the examples |
| DDS domain | `0` | the robot's domain |
| DDS interface | PC2's robot-LAN NIC | the same one the census uses |

### The left/right assignment

**Read it. Do not infer it.** Not from `.210 < .211`, not from the MAC ordering (the MACs
are just the IPs in hex — there is no vendor OUI to reason from).

There is a hint and it is only a hint: the upstream repo's own `Headless_driver_l.py`
hardcodes `ip='192.168.123.211', LR='l'` — someone else's G1, on the same subnet as ours,
with `.211` as **left**. That is a lead worth checking first, not an answer.

How to resolve it without moving anything:

1. Bring the bridge up against **one** address only, with a chosen `LR`.
2. `python tools/inspire_probe.py --domain 0 --iface <nic> --seconds 30`
3. Have someone hold one finger of one physical hand closed. `angle_act` for that DOF
   drops (0 = fully bent, 1000 = fully open) on whichever side is publishing.
4. That tells you which physical hand that address is. Write it into
   `docs/g1_contract.yaml` under `hand_transport.hosts`.

Reading is a session on the hand but moves nothing. Assigning left/right wrongly and then
commanding is how a hand gets driven against the wrong limits.

## 3. Probe before commanding

```bash
python tools/inspire_probe.py --domain 0 --iface <nic> --seconds 60
```

Exit 3 means a side was silent. Fill into `docs/g1_contract.yaml`:

- `angle_act` width per side (expect 6) and the resting values
- `err` and `status` per DOF — **any non-zero `err` and the pre-flight will refuse**
- `temperature` per DOF at idle — this is the first real number for the 45 °C limit
- `force_act` and `current` at idle
- the publish rate the bridge achieves

Then re-run the census; `rt/inspire_hand/state/{l,r}` are now on its watched list.

## 4. Then, and only then, the launcher

```bash
XR_ARM_VEL_LIMIT=5 python teleop/teleop_hand_and_arm.py \
    --arm G1_29 --ee inspire_ftp --network-interface <nic> \
    --img-server-ip 192.168.123.164
```

The pre-flight runs before `Enter_Debug_Mode()` and refuses — exit 1, having moved
nothing — if either side is silent, the DOF count is wrong, any `err` is non-zero, or any
DOF is over `HAND_TEMP_LIMIT_C`. **This is the live proof of g14c, which is only
stub-verified offline.** No `--motion`, ever. No `--sim` on the robot LAN.

## 5. Open list — things only the robot can answer

| question | how to answer it | where it goes |
|---|---|---|
| which of `.210`/`.211` is left | §2 above | `g1_contract.yaml` `hand_transport.hosts` |
| left hand SN — 2 characters unreadable | read the left wrist ring directly | `hand_model.serial_numbers.left` |
| **`ticks/sample` on `rt/lowstate`** | the census prints it now. `~1` means `tick` is a per-publish counter and the 2026-08-24 "999.6 Hz" was a real rate; anything else means it is a clock and that figure was ticks, not publishes | `g1_contract.yaml` `domain0.lowstate` |
| **arm motor temperature limit from Unitree** | ask Unitree. No numeric limit appears in any public documentation; issue #129 reports a shoulder-pitch overheat with our hand family and quotes no temperature | `panthera_g1_teleop_offline.md` §4a, replacing our 60/75 °C |
| does `T1` tactile reach DDS | the bridge publishes `rt/inspire_hand/touch/{l,r}`; subscribe and look | `g1_contract.yaml` `hand_model.tactile` |
| is E2 really DFTP | ask Inspire; distributors say yes, no vendor document does | `inspire_rh56e2.md` |
| idle hand temperature | §3 | the 45 °C limit's justification |

## 6. Restore list — our containers stay down until after Part B

From `p1_restore_list.txt`. **All four stay stopped** for the census and for B2; PC2 load
was 4.26 with them up and 0.54 with them down, and the burn was ours (`ferox_vision` on the
GPU, `panthera_g1_driver`/`realsense` on CPU).

- `panthera_g1_driver` → `docker start panthera_g1_driver`
- the other three → `docker compose up -d` in their directories

**Never run `setup_uvc.sh` on PC2.** It unloads `uvcvideo`, which the stock `videohub_pc4`
streams depend on. The image server runs on the UVC fallback because `pyrealsense2` needs
GLIBC 2.32 and PC2 has 2.31; it was measured at 29.1 fps 640×480 on `/dev/video5`.
