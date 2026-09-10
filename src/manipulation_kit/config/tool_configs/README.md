# Tool physical configs (generated — do not hand-edit values)

Per-end-effector physical registration data for the arm controller
(`Arm::setTool`): TCP offset, mass, COM, inertia. **The source of truth
lives in the dx-manipulator repo** (`hands/<maker>/<model>/toolconfig.py`),
next to the hand each file describes; these JSONs are exports:

```sh
python -m manipulation_kit.hands.export_tool_config d1/parallel_gripper  tool_configs/d1_parallel_gripper.json
python -m manipulation_kit.hands.export_tool_config leadshine/dh116s     tool_configs/leadshine_dh116s.json
```

To change a value, change it in dx-manipulator and re-export.

## Selecting the mounted tool

`omakase_arm::ToolConfig::load()` (arm.h) resolves, in order:

1. `OMAKASE_ARM_TOOL_CONFIG=<path to json>` — explicit override; errors throw
   (never silently falls back to the wrong tool).
2. `config/tool_config.json` — the active default. It is a **copy** of the
   file in this directory matching the EE actually mounted on the robot;
   currently ships as the DH116S hand (the EE on d1-1). **Replace it when
   swapping the end effector** (e.g. `cp
   config/tool_configs/d1_parallel_gripper.json config/tool_config.json`
   after going back to the stock gripper).
3. Built-in `defaultGripper()` fallback, with a loud stderr warning.

The chosen source and its TCP/mass/COM are always logged to stderr —
registering the wrong tool fails torque-mode entry
(ARM_ERR_RequestSensorMode=6) or mis-compensates gravity (wrist sag).

DH116S values: TCP 100 mm / COM z 40 mm / mass **0.4 kg** — the
registration set supplied by the arm engineer 2026-08-08 (compliance-mode
fix on d1-1). It supersedes the measured 2026-07 set (TCP 210 mm / COM z
120 mm, Shu 2026-07-18; 0.380 kg weighed at the robot, Shu 2026-07-29);
the reference-point change behind the TCP/COM shift is documented as an
open question in `hands/leadshine/dh116s/toolconfig.py`. Inertia is a
solid-cylinder estimate pending load identification (see the `_comment`
field of each file).
