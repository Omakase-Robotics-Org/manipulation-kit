"""D1 stock parallel gripper tool physical config
(see :mod:`manipulation_kit.hands.toolconfig`).

The REGISTERED values — what an arm controller is told is bolted to the
flange — are the exact ones d1-sdk hardcoded as
``omakase_arm::ToolConfig::defaultGripper()`` (arm.h), carried over verbatim
so the stock default is data like every other tool, not code:

    TCP 136 mm along flange Z, mass 1.5 kg, COM z 68 mm,
    cylinder-approximated inertia 0.003 / 0.003 / 0.001.

Provenance of each: mass 1.5 kg 【MEASURED — Shu weighed the gripper at the
robot 2026-07-29 (「グリッパーは1.5kg」)】. TCP 136 mm 【confirmed by the
vendor CAD: the jaw tips are at Z = 143.5 mm, so 136 mm lands on the pad
face】. COM at 68 mm along Z 【validated on the real D1 as a LEVER: the
earlier meter-valued variant was read by the controller as ~0 mm, so the
tool was modelled at the flange and the wrist visibly sagged in
compliance/torque modes; moving the COM out to 68 mm fixed it — see the
arm.h comment】. Inertia 【estimated, cylinder approximation, same caveat as
arm.h】.

All three registered numbers now have evidence behind them. That was not
true before 2026-07-29, and the way it was settled is worth keeping.

The CAD says 0.328 kg. The CAD is wrong.
---------------------------------------
When the vendor CAD arrived (see ``descriptions/``) it summed to 0.3279 kg —
``base_link`` 0.244769 + 2 x 0.041574 jaws — against the 1.5 kg registered
here, a factor of 4.6. Three numbers described one object:

    registered here / d1-sdk defaultGripper()      1.5    kg   COM z 68.0 mm
    vendor CAD, as received                        0.3279 kg   COM z 51.1 mm
    d1-isaaclab sim primitives, per side           0.37   kg   (no COM claim)

Shu settled it with a scale: **1.5 kg**. So:

* **1.5 kg is correct** and stays. It was never a guess, only undocumented.
* **The CAD export is incomplete.** It models outer shells only. Put the
  missing 1.1721 kg on the jaws and they would have to be 15 065 kg/m^3,
  denser than lead; put it on ``base_link``'s modelled 143.2 cm^3 and that is
  9 894 kg/m^3, denser than steel. Over the body's bounding envelope
  (585.6 cm^3, only 24 % of it filled by the mesh) the same mass is
  2 420 kg/m^3 — ordinary for a housing containing the motor, gearbox,
  leadscrew and PCB the export left out. The implied CAD densities give the
  same verdict: 1 709 kg/m^3 for the body, 998 kg/m^3 for the jaws. Neither
  is a metal, and 998 is water — these are default material values.
* **0.37 kg was never a measurement of anything**: the sum of the placeholder
  link masses in d1-sdk ``description/d1/d1.urdf``, tracked by d1-isaaclab as
  ``end_effectors.SIM_GRIPPER_MASS_KG``. It is ~4x light. Correcting it
  changes the dynamics the hikido policy was trained against, so it is a
  retraining decision, not a wiring fix.

The bundled ``descriptions/gripper.urdf`` therefore does NOT ship the CAD
inertials. Its link masses sum to the measured 1.5 kg, with the missing mass
on ``base_link`` and its COM solved so the assembly COM lands on the
validated 68 mm; that inertial is marked in the file as an estimate and the
CAD originals are quoted beside it. Shipping a URDF whose masses are 4.6x
light, with CAD provenance making them look authoritative, is how a wrong
number ends up in a simulator or a payload-aware planner.

General lesson, and the reason to distrust the next CAD drop too: **CAD
exports in this pipeline under-report mass.** The DH116S is registered at
0.359 kg and weighs ~380 g on the same scale (+6 %, accepted — see
:mod:`manipulation_kit.hands.leadshine.dh116s.toolconfig`). That +6 % is what
mounting hardware looks like. +358 % is not mounting hardware; it is an
export that left the contents of the housing out. Treat CAD inertials as
suspect by default and weigh the thing.
"""

from __future__ import annotations

from ...toolconfig import ToolConfig

#: Total gripper mass as WEIGHED at the robot (Shu 2026-07-29). Equals the
#: registered ``mass_kg`` below, and the sum of the link masses in
#: ``descriptions/gripper.urdf``. The one authoritative figure.
MEASURED_MASS_KG = 1.5

#: What the vendor CAD claimed when it arrived: ``base_link`` + both jaws.
#: Kept as a RECORD of a known-wrong number, so the disagreement stays
#: visible and a future CAD drop can be compared against it. Do not use it
#: for dynamics — see the module docstring.
CAD_MASS_KG = 0.327917

#: COM of that CAD mass in the flange frame, mm. Also superseded: it puts the
#: assembly COM at 38 mm, contradicting the validated 68 mm by 30 mm.
CAD_COM_MM = (0.14, -7.60, 51.06)

#: Distance from the flange to the jaw TIPS along +Z, mm. This one the CAD got
#: right, and it is what confirms the registered 136 mm TCP (7.5 mm inside the
#: tips, on the pad face).
CAD_JAW_TIP_Z_MM = 143.5

_PROVENANCE = (
    "D1 stock parallel gripper — registered values carried over verbatim "
    "from d1-sdk omakase_arm::ToolConfig::defaultGripper(). Mass 1.5 kg "
    "MEASURED at the robot (Shu 2026-07-29), confirming the long-registered "
    "value; TCP 136 mm confirmed by the vendor CAD (jaw tips at 143.5 mm); "
    "the 68 mm COM lever was validated on the real D1. The vendor CAD's own "
    "0.328 kg is a shell-only export and is 4.6x light — see toolconfig.py. "
    "Inertia is a cylinder-approximation estimate."
)


def tool_config() -> ToolConfig:
    """Standard per-model factory (see ``manipulation_kit.hands.get_tool_config``)."""
    return ToolConfig(
        model="d1/parallel_gripper",
        tcp_xyz_mm=(0.0, 0.0, 136.0),
        mass_kg=MEASURED_MASS_KG,
        com_mm=(0.0, 0.0, 68.0),
        inertia_kgm2=(0.003, 0.0, 0.0, 0.003, 0.0, 0.001),  # estimated
        provenance=_PROVENANCE,
    )
