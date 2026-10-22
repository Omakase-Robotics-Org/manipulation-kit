# Wuji Hand 2 — Wuji Technology dexterous hand

A five-finger hand with twenty actuated joints (four per finger), connected
over 100BASE-TX Ethernet and driven through the vendor's SDK (`wuji-sdk`,
Python module `wuji_sdk`). `d1-firmwared` does not drive it itself: an
`end_effector.shm/1` provider process does (the daemon's side is declared
`driver = "shm"`), and declares model `wuji/wujihand2` with the joints in
`axes.py`. This package holds only what the hand IS and how a glove drives
it.

**Status:** the retarget map is the table that drove a right Wuji Hand 2
from a LitchiBot glove on hardware (2026-09-25), moved into this package's
layers. The thumb is held by default; its map (below) was checked on the
vendor's URDF against recorded glove sessions, not yet on a hand.
The left hand has not been driven.

## Sources

| source | what it gave |
|---|---|
| the vendor URDFs `wuji_v2/urdf/right.urdf` and `left.urdf` (shipped with the vendor's glove retarget application) | joint limits, and the direction of every joint (forward kinematics) |
| `wuji_sdk` (2026.9.22) on a right hand, FW 2.10.0 | the joint labels `thumb_S1 .. pinky_S4` and their index order (`hand.joint(i).label`) |
| the glove-to-Wuji table that ran on that hand | the default retarget map and the calibration below |

## Joints

Radians; identical on both sides.

| # | name | what | min | max |
|---|---|---|---|---|
| 0 | `thumb_S1` | thumb base flexion (`cmc_flex`) | -1.187 | 1.291 |
| 1 | `thumb_S2` | thumb base abduction (`cmc_abd`) | -1.484 | 0.698 |
| 2 | `thumb_S3` | thumb second joint (`mcp`) | -1.047 | 1.570 |
| 3 | `thumb_S4` | thumb last joint (`ip`) | -1.047 | 1.570 |
| 4, 8, 12, 16 | `<finger>_S1` | knuckle flexion (`mcp_flex`) | -1.047 | 1.570 |
| 5, 9, 13, 17 | `<finger>_S2` | spread (`mcp_abd`) | -0.698 | 0.698 |
| 6, 10, 14, 18 | `<finger>_S3` | middle joint flexion (`pip`) | -1.047 | 2.094 |
| 7, 11, 15, 19 | `<finger>_S4` | last joint flexion (`dip`) | -1.047 | 1.570 |

`<finger>` is `index`, `middle`, `ring`, `pinky`, in that order. On both
sides positive is flexion, positive `S2` moves a finger towards the
little-finger side, positive `thumb_S1` moves the thumb tip towards the palm
and positive `thumb_S2` towards the index finger. The left URDF mirrors the
flexion axes (`-1 0 0` against `1 0 0`) to make that so.

## Retargeting from a glove

```python
from manipulation_kit.hands import get_retarget

wuji = get_retarget("wuji/wujihand2")
wuji.required_channels()     # check against the glove's declaration
wuji.joint_targets(pose)     # 20 x (rad | None); None = hold that joint
```

| joint | default source | interval (rad) |
|---|---|---|
| `<finger>_S1` | `<finger>_mp_pitch` | 0 .. 1.570 |
| `<finger>_S2` | `<finger>_mp_swing` | -0.648 .. 0.648 |
| `pinky_S2` | `pinky_mp_swing` | -0.45 .. 0.45 |
| `<finger>_S3` | `<finger>_pip_pitch` | 0 .. 2.094 |
| `<finger>_S4` | `<finger>_dip_pitch` | 0 .. 1.570 |
| `thumb_S1 .. S4` | held at 0 rad by default | |
| `thumb_S1` (released) | `thumb_cm_yaw` | 0 .. 0.8 |
| `thumb_S2` (released) | `thumb_cm_yaw` | 0 .. 0.6 |
| `thumb_S3` (released) | `thumb_mp_pitch` | 0 .. 1.0 |
| `thumb_S4` (released) | `thumb_mp_pitch` | 0 .. 0.6 |

- The spread's ±0.648 rad is the URDF's ±0.698 less a 0.05 rad margin;
  the pinky stops at ±0.45 rad, the spread it has reached and held on the
  hand.
- The spread is scaled once, by the swing channel's range. A glove that
  measures the wearer's own spread declares it as the channel's
  `default_range`, and the operator calibration then carries no swing
  range: a second, wider range on top caps the joint short of its interval.
- The thumb: opposition (`thumb_cm_yaw`) moves `S1` and `S2` together,
  because on the vendor URDF the tip crosses the palm through `S1`
  (`cmc_flex`) and `S2` (`cmc_abd`) only adducts it towards the index
  finger; flexion (`thumb_mp_pitch`) moves `S3` and `S4` together.
  `config_for(declaration)` releases the thumb when the glove can drive
  both channels. Forward kinematics of the right URDF with all four at
  their interval's top: the thumb tip is 27 mm from the little finger's
  base.
- `hold` and `hold_rad` (twenty entries each, SDK order) are
  `RetargetConfig` fields, so a binding sets them:
  `[bind.retarget_config]` `hold = [true, true, true, true, false, ...]`.
- **Spread and the glove's declaration.** A glove that declares its finger
  swing channels `uncalibrated` is refused at bind time for a bind that
  follows `S2`; hold the four `S2` joints (`hold` true at indices 5, 9,
  13, 17). A glove that declares the middle spread synthesized from the
  index and ring is refused for `middle_mp_swing`; hold `middle_S2`
  (index 9).
- The left glove's swing sign has not been checked. If a left hand spreads
  the wrong way, set `invert = true` on the four `S2` maps in the left
  bind.

### The calibration that reproduces the table that ran

The hardware table mapped glove degrees straight to joint radians. Its glove
half is an operator calibration (`manipulation_kit.glove_calibration.v1`):

| channel | lo (deg) | hi (deg) |
|---|---|---|
| `index_mp_pitch` | 0 | 50 |
| `middle/ring/pinky_mp_pitch` | 0 | 75 |
| `<finger>_pip_pitch` | 0 | 95 |
| `<finger>_dip_pitch` | 0 | 80 |

The index knuckle's 50 degrees is the table's 1.5 gain on 0 .. 75 degrees.
The table's spread (±18.564 degrees of swing, ±0.324 rad, onto ±0.648 rad)
is a swing range, which belongs to the glove's declaration rather than to
this file. `tests/data/gloves/litchibot_wujihand2.calibration.json` holds
the flexion in radians, and `tests/hands/test_wujihand2.py` checks that
it, that swing range and the default map give the table's outputs.

### Absolute and range mapping

`RetargetConfig.mapping` is `"absolute"` by default: the glove's finger angles
(radians) drive the matching joints 1:1, less a per-joint `zero_rad`, clamped
to the joint's limits, and no channel range is read. Pass the angles:
`wuji.joint_targets(pose, angles=hand_angles)`. `"range"` stretches the
operator's calibrated range onto the interval in the table above; set it per
binding with `[bind.retarget_config] mapping = "range"`. The thumb follows
the table in both modes. See the module docstring of `retarget.py`.

## What is not here

- **No `toolconfig`.** No measured mass, centre of mass or tool point of the
  hand on the D1's flange exists yet. A provider declares no tool until one
  is measured, so the integrator registers it.
- **No `description`.** The vendor URDFs and meshes are not redistributed
  here.

### From a UDCAP glove

`manipulation_kit.hands.wuji.wujihand2.udcap.udcap_config()` is the absolute
map for a UDCAP glove: the fingers and the thumb follow the glove's angles
1:1, and `middle_S2` is held at 0 (the glove's spreads are measured against
the middle finger). The same config as a binding is
`docs/binding-examples/udcap-wuji-right.toml`. The thumb's `S2` follows
`thumb_cm_yaw` with sign +1, which has not been seen on a hand.
