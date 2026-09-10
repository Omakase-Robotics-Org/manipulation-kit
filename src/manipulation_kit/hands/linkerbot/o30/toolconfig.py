"""LinkerHand O30 tool physical config — what the arm controller must know
when this hand is mounted (see :mod:`manipulation_kit.hands.toolconfig`).

**Provisional registration. Only the mass is measured.** The O30 replaced the
DH116S on the D1 flange in 2026-08; TCP, COM and inertia have not been
measured or supplied by the arm engineer yet, and this module records where
each number came from so nobody mistakes an estimate for a measurement.

Mass — the one number that IS measured
--------------------------------------
* **0.75 kg registered.** Weighed by the user at the robot, 2026-08-10:
  ~750 g for the assembly hanging off the flange.
* Vendor spec sheet (official Chinese manual): **730 g for the hand alone**.
  The 20 g difference is the mounting hardware, which is exactly the kind of
  gap the DH116S showed too (0.359 kg catalogue → 0.380 kg weighed, +21 g of
  screws / adapter face / connector pigtail). Gravity compensation has to
  hold up the assembly, not the catalogue part, so 0.75 kg is registered.
* A ~740 g figure circulating in press coverage of the O30 is a media
  re-print, not a source. It is consistent with the spec sheet and the scale;
  it is not what is registered.

COM — provisional, from the vendor URDF, and the URDF disagrees with itself
--------------------------------------------------------------------------
``linker-bot/linkerhand-urdf`` → ``O30/urdf_0803-right/
linkerhand_O30i_right-0803.urdf`` (21 links). Composing every link's inertial
block at the zero pose gives a combined COM of about

    (x 6.5, y 12.2, z 80.5) mm   in hand_base_link (z = the long axis)

registered here as-is. **But that URDF's total mass is 0.1824 kg — about a
quarter of the 0.75 kg on the scale.** The CAD export evidently carries the
shells and not the motors, so the mass distribution behind that COM is only
as good as a hollow model: use it as an order-of-magnitude placeholder, not
as a measurement. Two further caveats:

* the value is in the HAND's base frame; the mounting adapter between flange
  and hand adds an unknown offset along z, so the registered z is a LOWER
  bound on the flange-referenced COM;
* the parallel-gripper precedent in this repo is a warning, not a comfort —
  there, vendor CAD was off by 358 % against the scale
  (:mod:`manipulation_kit.hands.d1.parallel_gripper.toolconfig`).

TCP — deliberately not invented
-------------------------------
Registered as the flange origin ``(0, 0, 0)``, which means "no tool point is
registered yet", not "the TCP is at the flange". Choosing a real TCP for a
five-finger hand is a DESIGN decision (fingertip? grasp center between thumb
and index? palm face?), not a measurement, and the DH116S history shows how
much it moves: TCP 210 mm measured 2026-07-18, then re-registered at 100 mm
by the arm engineer on 2026-08-08 when the reference point changed. Pick the
O30's grasp center with the arm engineer, measure it, then register it here.

Inertia — left at zero on purpose
---------------------------------
The vendor arm demo marks the inertia array optional ("可以不填"), and mass +
COM dominate gravity compensation. Rather than ship a uniform-box estimate
that would read like data, the array is zero = "not filled in". Load
identification or the engineer's set supersedes it.

How d1-sdk consumes this
------------------------
Robot stacks do not import this module: they load the exported JSON that
``omakase_arm::ToolConfig::fromJsonFile`` reads, so no gripper's numbers are
hardcoded in C++ again. Export it with::

    python -m manipulation_kit.hands.export_tool_config linkerbot/o30 \\
        ~/omakaseos/config/tool_o30.json

which writes exactly this document (values as registered today — the
``kinematics``/``dynamics`` arrays are the only machine-read fields)::

    {
      "_comment": "LinkerBot LinkerHand O30 20-DoF hand on the D1 flange ...",
      "schema": "manipulation_kit.hands.tool_config.v1",
      "model": "linkerbot/o30",
      "units": {
        "kinematics": "mm (xyz), deg (rpy, XYZ order)",
        "mass": "kg",
        "com": "mm",
        "inertia": "kg*m^2, row-major upper-triangular (Ixx,Ixy,Ixz,Iyy,Iyz,Izz)"
      },
      "kinematics": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      "dynamics": [0.75, 6.5, 12.2, 80.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    }

The write target belongs to the ROBOT repo (d1-sdk / omakaseos config), never
to this one — this package owns the numbers, not their deployment.

Vendor spec sheet, for the record (official manual, 2026-08)
------------------------------------------------------------
20 DoF / 20 joints, direct drive; CAN / CAN FD at 500 Hz; supply DC 24-48 V;
0.4 A idle and 1.1 A average unloaded at 24 V, 150 W peak (~6.3 A at 24 V);
repeatability 0.2 mm; 0.8 s to open/close; fingertip force 24 N (thumb) /
30 N (other four), 70 N five-finger grip; 6x11 pressure-sensor array,
9.6 x 16.47 mm active area, 50 g trigger, 20 N/cm^2 range, 200 FPS.
Envelope 200 x 92 x 47 mm.
"""

from __future__ import annotations

from ...toolconfig import ToolConfig

#: Mass the arm controller is told to hold up — the assembly on the flange.
REGISTERED_MASS_KG = 0.75

#: Mass as weighed by the user at the robot, 2026-08-10 (~750 g). The repo
#: invariant is "register what was weighed"; the vendor's 730 g hand-only
#: figure is the catalogue part, not the assembly.
MEASURED_MASS_KG = 0.75

#: Vendor spec-sheet mass of the bare hand (official manual): 730 g.
SPEC_MASS_KG = 0.730

#: Vendor spec-sheet envelope, mm (L x W x H).
ENVELOPE_MM = (200.0, 92.0, 47.0)

#: PROVISIONAL COM, mm — composed from the vendor URDF at the zero pose. That
#: URDF totals 0.1824 kg (motors missing), and the mounting adapter offset is
#: not included: order-of-magnitude only. See the module docstring.
COM_MM = (6.5, 12.2, 80.5)

#: TCP is UNDETERMINED. Zero = "nothing registered yet", not a measurement.
TCP_XYZ_MM = (0.0, 0.0, 0.0)

_PROVENANCE = (
    "LinkerBot LinkerHand O30 20-DoF hand on the D1 flange (replaced the "
    "DH116S 2026-08). PROVISIONAL registration: mass 0.75 kg is the only "
    "measured number (weighed at the robot by the user 2026-08-10; vendor "
    "spec sheet says 730 g for the bare hand, the ~20 g difference is "
    "mounting hardware). COM is composed from the vendor URDF "
    "linker-bot/linkerhand-urdf O30/urdf_0803-right at the zero pose, whose "
    "own total mass is 0.1824 kg (~4x light, motors missing) and which "
    "excludes the mounting adapter offset — order-of-magnitude only. TCP is "
    "registered as the flange origin because the grasp center has not been "
    "chosen or measured. Inertia is deliberately zero (vendor marks it "
    "optional). Settle TCP/COM/inertia with the arm engineer or by "
    "measurement before relying on compliance or torque modes."
)


def tool_config() -> ToolConfig:
    """Standard per-model factory (see ``manipulation_kit.hands.get_tool_config``)."""
    return ToolConfig(
        model="linkerbot/o30",
        tcp_xyz_mm=TCP_XYZ_MM,          # UNDETERMINED — flange origin
        mass_kg=REGISTERED_MASS_KG,     # 0.75 kg, weighed 2026-08-10
        com_mm=COM_MM,                  # provisional, vendor URDF composition
        inertia_kgm2=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),  # not filled in
        provenance=_PROVENANCE,
        # Everything above except the mass is a placeholder, and this is the
        # part a CONSUMER can act on: the numbers get registered into an arm
        # controller's dynamics model, and the provenance string is not read at
        # that moment. Mounting this hand moves the registered tool point from
        # the DH116S's 100 mm to the flange origin.
        provisional=True,
        unmeasured=("tcp", "com", "inertia"),
    )
