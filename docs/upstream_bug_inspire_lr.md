# Upstream report: `Headless_driver_l.py` asserts a left/right mapping it cannot know

**Status: DRAFT — do not send yet.** The argument below is complete and correct, but
the one piece of hard evidence that would make it a bug report rather than a complaint
does not exist yet. §5 is the ten-minute procedure to get it. Fill in §4 on the robot,
then send.

* Repository: `NaCl-1374/inspire_hand_ws` (the `inspire_sdkpy` Modbus-TCP↔DDS bridge)
* Files: `inspire_hand_sdk/example/Headless_driver_l.py`,
  `inspire_hand_sdk/example/Headless_driver_r.py`
* Version seen: `@ fc75490`

---

## 1. What the code says

`Headless_driver_l.py:11`

```python
handler = inspire_sdk.ModbusDataHandler(ip='192.168.123.211', LR='l', device_id=1)
```

`Headless_driver_r.py:11`

```python
handler = inspire_sdk.ModbusDataHandler(ip='192.168.123.210', LR='r', device_id=1)
```

Both files have the vendor default commented out directly above:

```python
# handler = inspire_sdk.ModbusDataHandler(ip=inspire_hand_defaut.defaut_ip, LR='r', device_id=1)
```

So these two lines are **site-specific edits that were committed**, and they encode a
claim: on the author's robot, `.211` is the left hand and `.210` is the right.

## 2. Why that is a problem, not a preference

The `LR` flag is not cosmetic. It selects which DDS topics the bridge publishes and
subscribes (`rt/inspire_hand/state/l` vs `/r`, `ctrl/l` vs `ctrl/r`), and it selects
which hand's URDF and retargeting the consumer will apply. If the address↔side mapping
differs on another robot — and nothing in the protocol pins it — then:

* the left hand publishes on the right hand's topic and vice versa;
* a teleoperation client retargets the operator's **left** hand and sends the result to
  the **right** hand;
* the two RH56 URDFs are mirror images, so the joint limits applied are the *other*
  hand's. Commands are not merely swapped, they are outside the travel of the hand
  receiving them.

Nothing detects it. The state stream is well-formed and the angles are plausible; the
only symptom is a hand doing the other hand's job, at limits that are not its own.

## 3. Why the mapping cannot be inferred

There is no ordering fact to lean on. Measured on our robot:

| | `192.168.123.210` | `192.168.123.211` |
|---|---|---|
| ping | 2/2, rtt avg 0.350 ms | 2/2, rtt avg 0.248 ms |
| MAC | `de:08:c0:a8:7b:d2` | `de:08:c0:a8:7b:d3` |

`c0:a8:7b:d2` is `192.168.123.210` written in hex, and `de:` is a locally administered
prefix — the Ethernet module derives its MAC from its configured IP. **There is no
vendor OUI, and the MAC carries no information the IP does not.** Nor does the RH56
report a side over Modbus. The addresses are configurable, so `.210 < .211` says only
that someone assigned them in that order.

The mapping is therefore a property of **how a particular robot was wired**, and the
only way to establish it is to observe a hand.

## 4. Evidence from our robot

> **NOT YET COLLECTED.** Our own documentation
> (`docs/g1_contract.yaml: hand_transport.hosts.left_or_right_unassigned`,
> `docs/inspire_rh56e2.md` "Still unknown", item 1) records this as deliberately
> unresolved: *"Which address is which hand is UNKNOWN and stays unknown until it is
> read, not guessed. Do not infer it from .210 < .211 or from the MAC ordering."*
>
> We will not send a report claiming a mapping we have not measured — that would be the
> same mistake this report is about. Run §5 first and replace this section with the
> result.

Fill in:

```
Date:            ____________
Address probed:  192.168.123.___
Finger closed:   ____________  (physically, on the ____ hand)
angle_act before: [ _ , _ , _ , _ , _ , _ ]
angle_act after:  [ _ , _ , _ , _ , _ , _ ]
Conclusion:      192.168.123.___ is the ______ hand on this robot.
Does it match Headless_driver_l.py's .211 = left?   yes / no
```

## 5. How to collect it, without commanding anything

Reading is a Modbus session on the hand; it moves nothing. **Do not send a command
until the mapping is known** — that is exactly the failure this report describes.

```bash
# on PC2, with the bridge running for ONE address at a time
python tools/inspire_probe.py --domain 0 --iface enP7s7 --seconds 30
```

Then, physically: hold one finger of **one identified hand** closed and watch which
address's `angle_act` drops on that DOF. DOF order is the manual's:
`0 little, 1 ring, 2 middle, 3 index, 4 thumb bend, 5 thumb rotation`; angle 1000 is
fully open and 0 fully bent, so closing a finger makes its value **fall**.

Repeat on the other hand to confirm rather than assume the complement.

## 6. What we are asking for

1. **Do not ship a site-specific address↔side mapping as an example.** Restore the
   commented-out default, or take the address and side from argv/env:

   ```python
   handler = inspire_sdk.ModbusDataHandler(
       ip=os.environ.get("INSPIRE_HAND_IP", inspire_hand_defaut.defaut_ip),
       LR=os.environ.get("INSPIRE_HAND_LR", "r"),
       device_id=1)
   ```

2. **Say in the README that the mapping is per-robot and must be verified**, with the
   finger-close procedure from §5. One paragraph would have saved us this analysis.

3. **Consider making the bridge refuse to publish until told a side explicitly** — no
   default for `LR`. A wrong side is silent and destructive; a missing side is loud and
   harmless.

4. If the RH56 firmware does expose a side or a serial number over Modbus, expose it in
   the state message so consumers can check the wiring instead of trusting a constant.

## 7. Our workaround

We treat both addresses as unassigned until read on the robot, and we record the answer
in `docs/g1_contract.yaml` rather than in code. Our launcher derives the hand model from
`--ee` so the model and the topics cannot disagree
(`teleop/robot_control/hand_config.py: select_model_for_ee`), but that does not help
with left/right — the bridge decides that, and it decides it from this constant.
