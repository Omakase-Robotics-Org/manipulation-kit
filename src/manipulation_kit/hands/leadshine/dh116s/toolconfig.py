"""DH116S tool physical config — what the arm controller must know when
this hand is mounted (see :mod:`manipulation_kit.hands.toolconfig`).

Values (registration set of 2026-08-08)
---------------------------------------
TCP 100 mm and COM 40 mm along flange Z, mass 0.4 kg, inertia
0.0006 / 0.0008 / 0.0002 kg*m^2. This is the set the arm engineer supplied
on 2026-08-08 to fix the compliance-mode misbehavior seen on d1-1 with the
previous registration. It arrived as the vendor-API C arrays

    kinePara = {0.0, 0.0, 0.1, 0.0, 0.0, 0.0}                  # m, deg
    dynPara  = {0.4, 0.0, 0.0, 0.04,
                0.0006, 0.0, 0.0, 0.0008, 0.0, 0.0002}         # kg, m, kg*m^2

registered here verbatim, with lengths converted m -> mm for this schema.

What this supersedes, and what is still open
--------------------------------------------
The previous registration was TCP 210 mm / COM z 120 mm 【measured — Shu
2026-07-18】 and mass 0.380 kg 【weighed at the robot — Shu 2026-07-29,
registered 2026-07-31; the weighing story is preserved below】. Three
things about the change are NOT yet documented and should be settled with
the engineer before anyone treats these numbers as measurements:

* **TCP 210 -> 100 mm, COM 120 -> 40 mm.** Half or less of the measured
  values. On the same physical hand this looks like a change of the
  registration reference point (fingertips -> palm face, or similar), not
  a re-measurement — but that is an inference, not a record.
* **Mass 0.380 -> 0.4 kg.** +20 g on the weighed value; rounding or a
  re-weigh, unknown which.
* **The inertia array disagrees with its own derivation.** The engineer's
  accompanying comment computes a 0.4 kg uniform cylinder as
  Ixx = Iyy ≈ 0.0008, Izz ≈ 0.0003, but the array registers
  (Ixx, Iyy, Izz) = (0.0006, 0.0008, 0.0002). The array is registered
  as supplied. The vendor demo marks inertia optional ("可以不填"); mass +
  COM dominate gravity compensation, so a same-order estimate is
  sufficient either way.

Mass history (kept because the method matters)
----------------------------------------------
0.359 kg came from the hand's own spec/CAD. Weighing the assembly actually
hanging off the flange gave **~380 g** (Shu 2026-07-29): +21 g, ~6 %,
which is mounting screws, the adapter face and a connector pigtail.
Gravity compensation has to hold up the assembly, not the catalogue part.
That 6 % gap is what mounting hardware looks like; see
:mod:`manipulation_kit.hands.d1.parallel_gripper.toolconfig`, where the same
spec-vs-scale comparison is off by 358 % and the CAD is the wrong party.
"""

from __future__ import annotations

from ...toolconfig import ToolConfig

#: Mass the arm controller is told to hold up — the registration set
#: supplied by the arm engineer 2026-08-08.
REGISTERED_MASS_KG = 0.4

#: Mass of the hand as weighed at the robot, 2026-07-29 (Shu): ~380 g.
#: Kept as the measurement RECORD (the scale beats the spec sheet); the
#: 2026-08-08 registration rounds it up by 20 g for reasons not yet
#: documented — see the module docstring.
MEASURED_MASS_KG = 0.380

_PROVENANCE = (
    "Leadshine DH116S 5-finger hand on the D1 flange. Registration set "
    "supplied by the arm engineer 2026-08-08 (compliance-mode fix on "
    "d1-1): TCP 100 mm, mass 0.4 kg, COM z 40 mm, inertia "
    "0.0006/0.0008/0.0002. Supersedes the measured set of 2026-07 "
    "(TCP 210 mm / COM z 120 mm, Shu 2026-07-18; 0.380 kg weighed, Shu "
    "2026-07-29) — the reference-point change behind the TCP/COM shift "
    "is not yet documented. Inertia is a cylinder-order ESTIMATE pending "
    "load identification."
)


def tool_config() -> ToolConfig:
    """Standard per-model factory (see ``manipulation_kit.hands.get_tool_config``)."""
    return ToolConfig(
        model="leadshine/dh116s",
        tcp_xyz_mm=(0.0, 0.0, 100.0),   # engineer registration 2026-08-08
        mass_kg=REGISTERED_MASS_KG,     # 0.4 kg (weighed 0.380, see docstring)
        com_mm=(0.0, 0.0, 40.0),        # engineer registration 2026-08-08
        inertia_kgm2=(0.0006, 0.0, 0.0, 0.0008, 0.0, 0.0002),  # as supplied
        provenance=_PROVENANCE,
    )
