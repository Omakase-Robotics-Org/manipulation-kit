# XHAND1 Lite — RobotEra dexterous hand, six-joint variant

The Lite is the reduced XHAND1: one flexion joint per finger and two thumb
joints, six in all. It uses the full XHAND1's RS-485 real-time exchange
(3 Mbit/s, 8N1, all twelve slots in every exchange), and only slots 0-5 are
joints. `d1-firmwared` drives it as model `robotera/xhand1_lite` through its
hand driver library. This package holds the joint table and the glove
retarget map. The full XHAND1 is `manipulation_kit.hands.robotera.xhand1`,
and its joint names and ranges must not be used for a Lite.

**Status:** the joint ranges are a provisional, tested range (below). The
retarget map's thumb mapping is reasoned from the vendor's joint names and
needs on-hand tuning.

## Sources

| source | what it gave |
|---|---|
| RobotEra xbot SDK, `robot_controller/robot_controller.py` @ `f542ed2` (`control_xhand_lite_grasp`) | the joint order (vendor names `thumb_bend_joint, thumb_rota_joint1, index_joint1, mid_joint1, ring_joint1, pinky_joint1`) |
| the hand driver's descriptor, as `d1-firmwared` publishes it | the driver's names for the same joints and the limits below, mirrored in `axes.py` |

## Joints

| # | name | range (rad), provisional |
|---|---|---|
| 0 | `thumb_bend` | -0.05 .. 1.95 |
| 1 | `thumb_rota1` | -0.05 .. 1.29 |
| 2 | `index_j1` | -0.05 .. 1.51 |
| 3 | `mid_j1` | -0.05 .. 1.51 |
| 4 | `ring_j1` | -0.05 .. 1.53 |
| 5 | `pinky_j1` | -0.05 .. 1.46 |

RobotEra publishes no range for the Lite. Each maximum is an angle the joint
has been driven to under position control, rounded down; none is a
mechanical end stop. 0 rad is the open hand; the floor of -0.05 rad is a
small margin so that a hand at rest just below 0 is inside its range. The
retarget map's open end stays at 0 rad.

## Retargeting from a glove

```python
from manipulation_kit.hands import get_retarget

lite = get_retarget("robotera/xhand1_lite")
lite.required_channels()     # check against the glove's declaration
lite.joint_targets(pose)     # 6 x (rad | None); None = hold that joint
```

- Fingers: `<finger>_j1` is driven from the mean of the finger's MP and
  PIP flexion. The DIP has weight 0, as in the DH116S map. The output runs
  from 0 rad (open) to the joint's maximum.
- Thumb (provisional): `thumb_bend` is driven from `thumb_cm_yaw`, and
  `thumb_rota1` from the mean of `thumb_cm_pitch` and `thumb_mp_pitch`.
  The reasoning comes from the vendor's names. On the full XHAND1, "bend" is
  the name of the index finger's sideways spread, and the thumb's two "rota"
  joints form its curling chain. **Needs on-hand tuning.** Every source,
  weight and interval is `RetargetConfig` data.
- With the LitchiBot glove, `thumb_rota1` is held: that glove declares
  `thumb_cm_pitch` uncalibrated. To drive the joint with that glove, re-source
  it, for example from `thumb_mp_pitch` alone.

## What is not here

- **No `toolconfig`.** None of the sources above gives a mass, tool point,
  centre of mass or inertia for the Lite.
- **No `description`.** No CAD or URDF for this hand is in this repository.
