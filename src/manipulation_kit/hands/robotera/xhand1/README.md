# XHAND1 — RobotEra dexterous hand

A five-finger hand with twelve active joints, driven over its own RS-485 link
(3 Mbit/s, through a USB-RS485 adapter on the robot's onboard computer) rather
than through the arm's end-module CAN passthrough. `d1-firmwared` drives it as
model `robotera/xhand1` through its hand driver library; this package
holds only what the hand IS and how a glove drives it.

The six-joint XHAND1 **Lite** is a different joint table:
`manipulation_kit.hands.robotera.xhand1_lite`.

**Status: written from the vendor manual and the driver's descriptor. The
retarget map has not driven a hand.** The joint table matches the driver; the
map's thumb and the index spread direction need checking on hardware (see
below).

## Sources

| source | what it gave |
|---|---|
| RobotEra, "X-Hand 1 Product Manual V1.0" (2024-10-15), section 3.2 | the joint ranges in degrees (`JOINT_LIMITS_DEG`) and the mass (`weights 1.1Kg` in the specification table) |
| the hand driver's descriptor, as `d1-firmwared` publishes it | names, order and unit, mirrored in `axes.py`; the daemon's `open` preset (0 rad on all twelve joints) |
| IIT `yarp-device-xhand` | the joint names, of which the driver's are snake-case forms |

## Layout

| file | what |
|---|---|
| `axes.py` | `AXIS_NAMES`, `JOINT_LIMITS_DEG`, `JOINT_LIMITS_RAD`, `JOINTS` (the end-effector descriptor: twelve `rad` joints with the manual's limits) |
| `retarget.py` | the glove retarget map (`build_retarget`, `RetargetConfig`, `JointMap`) |

## Joints

| # | name | range (deg) | range (rad) |
|---|---|---|---|
| 0 | `thumb_bend` | 0 .. 90 | 0 .. 1.5708 |
| 1 | `thumb_rota1` | -60 .. 90 | -1.0472 .. 1.5708 |
| 2 | `thumb_rota2` | 0 .. 90 | 0 .. 1.5708 |
| 3 | `index_bend` | -5 .. 17 | -0.0873 .. 0.2967 |
| 4 | `index_j1` | 0 .. 110 | 0 .. 1.9199 |
| 5 | `index_j2` | 0 .. 110 | 0 .. 1.9199 |
| 6 | `mid_j1` | 0 .. 110 | 0 .. 1.9199 |
| 7 | `mid_j2` | 0 .. 110 | 0 .. 1.9199 |
| 8 | `ring_j1` | 0 .. 110 | 0 .. 1.9199 |
| 9 | `ring_j2` | 0 .. 110 | 0 .. 1.9199 |
| 10 | `pinky_j1` | 0 .. 110 | 0 .. 1.9199 |
| 11 | `pinky_j2` | 0 .. 110 | 0 .. 1.9199 |

The names are the hand driver's snake-case forms of IIT
`yarp-device-xhand`'s; the degrees are the manual's, converted exactly (the
manual's own radian column rounds them to 1.57 and 1.92).

## Retargeting from a glove

```python
from manipulation_kit.hands import get_retarget

xhand = get_retarget("robotera/xhand1")
xhand.required_channels()      # check these against the glove's declaration
xhand.joint_targets(pose)      # 12 x (rad | None), descriptor order
```

`pose` is a `manipulation_kit.gloves.HandPose`. A `None` target means one of
that joint's input channels has no reading: hold the joint at its last
commanded value. The default map, and what is not verified about it, is
written out in `retarget.py`'s module docstring; in short:

- fingers: `j1` from the knuckle flexion, `j2` from the middle-joint
  flexion, both over 0 .. 110 degrees, 0 rad open;
- index spread: from the bipolar `index_mp_swing` over -5 .. 5 degrees
  so a neutral finger lands on 0 rad. **Direction unverified.**
- thumb: `thumb_bend` from `thumb_cm_yaw`, `thumb_rota1` from
  `thumb_cm_pitch`, `thumb_rota2` from `thumb_mp_pitch`, each over
  0 .. 90 degrees. **Needs on-hand tuning**: the joint names do not say which
  motion each thumb joint is, and the negative half of `thumb_rota1`
  is left unused because the manual does not say what it is.

## What is not here

- **No `toolconfig`.** Registering a tool with the arm controller needs a
  tool point, a centre of mass and an inertia as well as the mass. The
  manual gives the mass (1.1 kg) and none of the others, and this package
  does not register guessed numbers for a hand nobody has measured. Until a
  measured entry lands here, a client registers the tool itself
  (`POST /v1/arm/{side}/tool`).
- **No `description`.** No CAD or URDF for this hand is in this repository.
