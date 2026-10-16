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
layers. The thumb mapping has never driven a hand and is held by default.
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
| `<finger>_S3` | `<finger>_pip_pitch` | 0 .. 2.094 |
| `<finger>_S4` | `<finger>_dip_pitch` | 0 .. 1.570 |
| `thumb_S1 .. S4` | held at 0 rad | |

- The spread's ±0.648 rad is the URDF's ±0.698 less a 0.05 rad margin.
- The thumb is held, as it was on the hand. Its provisional maps
  (`thumb_cm_pitch`, `thumb_cm_yaw` over 0 .. 0.698, `thumb_mp_pitch`,
  `thumb_ip_pitch`) are read only when `hold` is cleared for that joint.
- `hold` and `hold_rad` (twenty entries each, SDK order) are
  `RetargetConfig` fields, so a binding sets them:
  `[bind.retarget_config]` `hold = [true, true, true, true, false, ...]`.
- **Spread and today's gloves.** The LitchiBot and UDCAP glove drivers both
  declare their finger swing channels `uncalibrated`, so a bind that
  follows `S2` is refused at bind time. To bind, hold the four `S2` joints
  (`hold` true at indices 5, 9, 13, 17). The spread drove the hand through
  a path that read the glove's swing without its declaration.
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
| `<finger>_mp_swing` | -18.564 | 18.564 |

The index knuckle's 50 degrees is the table's 1.5 gain on 0 .. 75 degrees;
the swing's ±18.564 degrees (±0.324 rad) is its 2 x gain onto ±0.648 rad.
`tests/data/gloves/litchibot_wujihand2.calibration.json` holds it in
radians, and `tests/hands/test_wujihand2.py` checks that this calibration
and the default map give the table's outputs.

## What is not here

- **No `toolconfig`.** No measured mass, centre of mass or tool point of the
  hand on the D1's flange exists yet. A provider declares no tool until one
  is measured, so the integrator registers it.
- **No `description`.** The vendor URDFs and meshes are not redistributed
  here.
