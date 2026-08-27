# Inspire RH56E2-T1 — what drives it, and what is still unknown

Written on the laptop, 2026-08-27, from vendor documentation and source. **Nothing here was
measured on our hands.** Every claim carries its source; the ones that matter for safety are
marked as inference where they are inference.

## The one-line answer

> **E2 is driven by the `inspire_sdkpy` Modbus-TCP↔DDS bridge at `fc75490`, same register map
> as the hand Unitree calls "FTP": yes — but distributor-asserted, not vendor-confirmed.**

The register map is confirmed by construction: the bridge's own tactile register addresses are
byte-for-byte the addresses printed in the **RH56DFTP** manual (below). What is *not* confirmed
by any Inspire or Unitree document is the sentence "RH56E2 is the same hand as RH56DFTP" — that
comes from distributors. Treat the map as correct and verify on contact, at read-only.

## Identity

| item | value | source |
|---|---|---|
| model | Inspire **RH56E2-T1** | labels read at the wrist ring, 2026-08-27 |
| right label / SN | `RH56E2-2R-T1` / `411AA1A04140021` | direct reading |
| left label / SN | `RH56E2-2L-T1` / `401AA1A0416??01` | photo, **2 characters unconfirmed** |
| **T1** | **resistive tactile, 17 sensors, 0–30 N** | Knox Labs RH56E2 spec sheet |
| T2 (not ours) | capacitive, 5 sensors, 0–20 N | same |
| mass | **790 g ± 10 g** per hand | same |
| DOF / joints | 6 actuators, 12 joints | RH56DFTP manual §1.2 |
| interfaces | RS485 / CAN / **Modbus TCP** on one unit | manual §1.2, spec sheet |
| "also sold as" | **RH56DFTP** | Knox Labs spec table; US Robot Store lists "RH56E2 (RH56DFTP for Unitree G1, H2)" |

`T1` was a guess in the contract ("most likely tactile"). It is now sourced: T1 and T2 are the
**tactile technology options**, and T1 is the 17-sensor resistive one. That matches the manual's
"number of tactile sensors: 5–17" range and Inspire's E2 page ("17 tactile sensors in a single hand").

### How solid is "E2 == DFTP"?

| evidence | strength |
|---|---|
| Knox Labs spec table: RH56E2 "also sold as RH56DFTP" | distributor |
| US Robot Store product title: "RH56E2 (RH56DFTP for Unitree G1, H2 Humanoid Robot)" | distributor |
| Generation Robots hosts the RH56E2 manual as `user-manual-RH56E2.pdf` — the PDF's own title page reads **"THE DEXTEROUS HAND RH56DFTP USER MANUAL"**, and the string "RH56E2" appears **nowhere** in its 27 pages | distributor filing + vendor document |
| `inspire_sdkpy`'s register constants == the RH56DFTP manual's register map | **direct, verifiable** |
| an Inspire or Unitree document stating the equivalence | **none found** |

The last row is why this is "yes, distributor-asserted" and not "yes". The practical risk is low —
the register map is what the bridge actually uses, and that is verified — but do not write
"RH56F-TP" or "RH56DFTP" into the contract as *our* model name. Ours is what the label says.

## The software path

Two different Inspire paths exist in xr_teleoperate. Ours is the second.

| | `--ee inspire_dfx` | `--ee inspire_ftp` ← **ours** |
|---|---|---|
| transport | RS-485 serial | **Modbus TCP over Ethernet** |
| DDS topics | `rt/inspire/cmd`, `rt/inspire/state` | `rt/inspire_hand/{ctrl,state,touch}/{l,r}` |
| DDS types | `unitree_go::MotorCmds_` / `MotorStates_` | `inspire.inspire_hand_{ctrl,state,touch}` |
| served by | `unitreerobotics/dfx_inspire_service` | the **`inspire_sdkpy` bridge** |
| applies to us? | no — PC2 has no serial path at all | yes |

### The bridge

* repo: `https://github.com/NaCl-1374/inspire_hand_ws`
* pinned SHA: **`fc754900caaaa82c9b59fb12c1b79ebfd1c1a0e7`**
* package: `inspire_hand_sdk/inspire_sdkpy` (`pip install -e .`) — **not on PyPI**
* the bridge program: `example/Headless_driver_{l,r}.py` (Ethernet) — the `*_485_*` variants are
  for the serial hands and are not ours. C++ equivalent: `cpp_example/Hand_dds_example.cpp`.
* the bridge object: `inspire_sdk.ModbusDataHandler(ip=..., LR='l'|'r', device_id=1)`
* factory default hand address: `192.168.11.210`, **TCP port 6000** (manual §2). Note **6000**,
  not Modbus's usual 502 — the PC2 report's "no TCP connect to port 502" was guarding the wrong port.

> **This repo is a third party's workspace, not Unitree's.** Unitree's own documentation
> ("宇树提供将 ModbusTCP 收发的数据转为 DDS 消息的示例程序" — Unitree provides example programs
> converting ModbusTCP traffic to DDS messages) is behind a JavaScript-rendered doc centre that
> could not be read from here. Before installing this on PC2, check whether Unitree ships its own
> build; prefer theirs if it exists.

### Dependency trap

`inspire_sdkpy/__init__.py` eagerly imports `ModbusDataHandler` **and** `qt_tabs`, so simply
reaching the IDL pulls in `pymodbus`, `pyqtgraph`, `PyQt5` and `colorcet`. There is no import path
that avoids it — importing a submodule still executes the package `__init__`.

Consequences:
* the **teleop host** needs `pymodbus` and a Qt stack installed even though it never speaks Modbus
  and never opens a window — `Inspire_Controller_FTP` does `from inspire_sdkpy import inspire_dds`;
* **PC2** is a headless aarch64 Jetson. Budget time for PyQt5 there, or patch the `__init__`.

Installed here: `inspire_sdkpy 1.0.0` (editable, `--no-deps`), `pymodbus 3.6.9`, `PyQt5 5.15.11`,
`pyqtgraph 0.14.0`, `colorcet 3.2.1`, `pyserial 3.5`.

## The IDL, verified by import

`inspire.inspire_hand_ctrl` — 5 fields
| field | type |
|---|---|
| `pos_set`, `angle_set`, `force_set`, `speed_set` | `sequence[int16, 6]` |
| `mode` | `int8` |

`inspire.inspire_hand_state` — 7 fields, **all 6 wide**
| field | type | note |
|---|---|---|
| `pos_act` | `sequence[int16, 6]` | actuator position |
| `angle_act` | `sequence[int16, 6]` | **the only field xr_teleoperate reads** |
| `force_act` | `sequence[int16, 6]` | |
| `current` | `sequence[int16, 6]` | |
| `err` | `sequence[uint8, 6]` | |
| `status` | `sequence[uint8, 6]` | |
| `temperature` | `sequence[uint8, 6]` | |

`inspire.inspire_hand_touch` — 17 regions: per finger `tip[9]` + `top[96]` + `palm[80]`, the thumb
additionally `middle[9]`, plus `palm_touch[112]`. **17 regions ↔ T1's 17 sensors.**

### Two things the state message gives us that upstream throws away

`Inspire_Controller_FTP` reads `angle_act` only. `force_act`, `current`, `err`, `status` and
`temperature` all arrive on the same message and are discarded — including the error and
temperature fields that any fail-closed check would want. `inspire_hand_touch` is a **separate
topic** that the controller does not subscribe to at all, so **T1 tactile data does not reach
xr_teleoperate today** even when the bridge publishes it.

## Register map (RH56DFTP manual V1.0.0, Dec 2024, ID PRJ-02-TS-U-010)

| address | name | width | access |
|---|---|---|---|
| 1004 | `CLEAR_ERROR` | 1 byte | W/R |
| 1032 | `DEFAULT_SPEED_SET(m)` | 6×int16 | W/R |
| 1044 | `DEFAULT_FORCE_SET(m)` | 6×int16 | W/R |
| 1486 | `ANGLE_SET(m)` | 6×int16 | W/R |
| 1498 | `FORCE_SET(m)` | 6×int16 | W/R |
| 1522 | `SPEED_SET(m)` | 6×int16 | W/R |
| 1534 | `POS_ACT(m)` | 6×int16 | R |
| **1546** | **`ANGLE_ACT(m)`** | 6×int16 | R |
| 1582 | `FORCE_ACT(m)` | 6×int16 | R |
| 1606 | `ERROR(m)` | 6 bytes | R |
| 1612 | `STATUS(m)` | 6 bytes | R |
| 1618 | `TEMP(m)` | 6 bytes | R |
| 3000 / 3370 / 3740 / 4110 / 4480 / 4900 | `FINGER{ONE..FIV,PALM}_TOUCH` | 370 bytes each | R |

The tactile block is the proof of the shared map: `inspire_hand_defaut.py` lists
`3000, 3370, 3740, 4110, 4480` for `fingerone..fingerfive_tip_touch` — the manual's addresses exactly.

Ranges: angle/speed `0–1000` (`-1` = "leave alone"), force `0–3000`.
**Angle semantics: `1000` = fully open, `0` = fully bent.** Manual §2.6: setting `ANGLE_SET(3)` to 0
bends the index finger; `ANGLE_ACT(3)` of 1000 is fully open.

## DOF order — verified end to end

The manual's DOF index `m`, the DDS array index, and xr_teleoperate's enum all agree:

| m | manual | `Inspire_*_Hand_JointIndex` |
|---|---|---|
| 0 | little finger | `kRightHandPinky = 0` |
| 1 | ring finger | `kRightHandRing = 1` |
| 2 | middle finger | `kRightHandMiddle = 2` |
| 3 | index finger | `kRightHandIndex = 3` |
| 4 | thumb bending | `kRightHandThumbBend = 4` |
| 5 | thumb rotation | `kRightHandThumbRotation = 5` |

(The left enum is the same order offset by 6, because the DFX path packs both hands into one
12-element message.)

The retargeting config is in the **opposite** order — `inspire_hand.yml` lists
`thumb_yaw, thumb_pitch, index, middle, ring, pinky` over 12 URDF joints — and
`{left,right}_dex_retargeting_to_hardware = [4, 6, 2, 0, 9, 8]` reorders it to
`pinky, ring, middle, index, thumb_pitch, thumb_yaw`. That is the manual's order exactly.
**Upstream's Inspire index mapping is correct.** Both hands use the same permutation.

Assets present: `assets/inspire_hand/{inspire_hand.yml, inspire_hand_left.urdf,
inspire_hand_right.urdf, meshes/}` (26 meshes). Retargeting type DexPilot, one URDF per side.

## Still unknown — for the next visit, not for the laptop

1. **Which of `192.168.123.210` / `.211` is left and which is right.** The bridge example in the
   repo hardcodes `ip='192.168.123.211', LR='l'` — that is *someone else's* G1, and is a lead, not
   an answer. Read it and compare with the physical hand before commanding anything.
2. Whether the hands still listen on the factory port **6000** and what their configured addresses
   imply about `device_id`.
3. Whether Unitree ships its own build of this bridge (prefer it if so).
4. Whether the bridge on PC2 can publish `touch` at a useful rate, and whether we want it — nothing
   consumes it today.
5. The E2-vs-DFTP equivalence, confirmed by Inspire rather than by a reseller.

## Sources

* RH56DFTP User Manual V1.0.0, Beijing Inspire-Robots, Dec 2024, ID `PRJ-02-TS-U-010`, 27 pp —
  hosted as `https://static.generation-robots.com/media/user-manual-RH56E2.pdf` and as
  `https://en.inspire-robots.com/wp-content/uploads/2025/01/INSPIRE-ROBOTS-The-Dexterous-Hand-RH56DFTP-User-Manual-V1.0.0.pdf`
* Inspire RH56E2 product page — `https://en.inspire-robots.com/product/rh56e2`
* Knox Labs RH56E2 spec sheet (T1/T2, 790 g, "also sold as RH56DFTP") —
  `https://www.knoxlabs.com/products/inspire-robots-rh56e2-dexterous-hand`
* US Robot Store, "RH56E2 (RH56DFTP for Unitree G1, H2 Humanoid Robot)" —
  `https://www.usrobotstore.com/products/inspire-robots-5-finger-robotic-dexterous-hand-rh56e2`
* `inspire_sdkpy` / bridge — `https://github.com/NaCl-1374/inspire_hand_ws` @ `fc75490`, incl.
  `hand_ftp.md`
* `unitreerobotics/dfx_inspire_service` @ `d6c4eae` — the *serial* path, for contrast
* xr_teleoperate issue #48 (RH56DFTP support request, origin of the `inspire_ftp` path) —
  `https://github.com/unitreerobotics/xr_teleoperate/issues/48`
