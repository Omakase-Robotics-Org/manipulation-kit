"""Tool physical config — the hand-owned answer to "what is mounted on the arm".

Arm controllers (D1's D1 arm in particular) need the mounted tool
registered with their dynamics model: TCP offset for kinematics, and
mass / COM / inertia for gravity compensation and the torque-mode entry
check. Registering the WRONG tool is the failure mode this module exists
to kill: d1-sdk used to hardcode the stock parallel gripper's numbers in
C++ (``ToolConfig::defaultGripper``), so mounting a DH116S raised
controller errors (unmodeled mass/lever → ARM_ERR_RequestSensorMode) or
silently mis-compensated gravity.

Per Shu's abstraction rule the per-model physical data lives HERE, next to
the hand it describes (``hands/<maker>/<model>/toolconfig.py`` exposing
``tool_config() -> ToolConfig``), and robot stacks resolve it through
:func:`manipulation_kit.hands.get_tool_config` — the same seam as ``get_hand`` /
``get_retarget``. The D1 stock gripper is one instance of the abstraction
(``"d1/parallel_gripper"``), not a special case in robot code.

Units follow the vendor OnSetTool convention that d1-sdk feeds the
controller: millimeters for TCP position and COM, degrees (XYZ order) for
TCP rotation, kg for mass, kg*m^2 for inertia. Inertia is row-major
upper-triangular ``(Ixx, Ixy, Ixz, Iyy, Iyz, Izz)`` — NOT ``(xx, yy,
zz, ...)`` — matching ``m_ToolDyn[10]``.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import List, Tuple

SCHEMA = "manipulation_kit.hands.tool_config.v1"


@dataclass(frozen=True)
class ToolConfig:
    """Physical registration data for one mounted end effector.

    ``kinematics()`` / ``dynamics()`` flatten to the exact 6- and 10-element
    arrays the arm controller's OnSetTool call takes (and that d1-sdk's
    ``omakase_arm::ToolConfig`` mirrors).
    """

    model: str                              # "<maker>/<model>" id
    tcp_xyz_mm: Tuple[float, float, float]  # TCP offset from flange, mm
    mass_kg: float
    com_mm: Tuple[float, float, float]      # COM offset from flange, mm
    # (Ixx, Ixy, Ixz, Iyy, Iyz, Izz) about the COM, kg*m^2 — vendor order.
    inertia_kgm2: Tuple[float, float, float, float, float, float] = (
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    tcp_rpy_deg: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # XYZ order
    provenance: str = ""                    # where the numbers came from
    #: True when at least one registered number is a placeholder rather than a
    #: measurement. It is not documentation: a consumer REGISTERS these numbers
    #: with an arm controller's dynamics model, and a docstring cannot warn
    #: anybody at that moment. The O30 is the case that needed it — a measured
    #: mass beside a TCP of (0, 0, 0) that means "no tool point chosen yet",
    #: mounted where a DH116S had 100 mm registered, so the tool point moves
    #: 100 mm on the swap with nothing said out loud.
    provisional: bool = False
    #: Which fields are the placeholders, for a message worth reading:
    #: ``("tcp", "com", "inertia")``.
    unmeasured: Tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if self.unmeasured and not self.provisional:
            raise ValueError(
                f"{self.model}: unmeasured={self.unmeasured} but "
                "provisional=False — a placeholder that does not announce "
                "itself is the failure this flag exists to prevent")

    def caveat(self) -> str:
        """One line a consumer can log when it registers this tool. '' if measured."""
        if not self.provisional:
            return ""
        what = ", ".join(self.unmeasured) if self.unmeasured else "some values"
        return (f"{self.model}: PROVISIONAL tool registration — {what} "
                f"{'is' if len(self.unmeasured) == 1 else 'are'} a placeholder, "
                "not a measurement. Do not rely on compliance or torque modes "
                "until settled.")

    def kinematics(self) -> List[float]:
        """``[x, y, z, rx, ry, rz]`` — mm / deg, vendor kinePara[6] layout."""
        return [*map(float, self.tcp_xyz_mm), *map(float, self.tcp_rpy_deg)]

    def dynamics(self) -> List[float]:
        """``[mass, com_x, com_y, com_z, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]`` —
        vendor dynPara[10] layout (kg / mm / kg*m^2)."""
        return [float(self.mass_kg), *map(float, self.com_mm),
                *map(float, self.inertia_kgm2)]

    def to_json_dict(self) -> dict:
        """The JSON document d1-sdk's ``ToolConfig::fromJsonFile`` consumes.

        Only ``kinematics`` and ``dynamics`` are machine-read on the C++
        side; everything else is provenance / human documentation.
        """
        return {
            "_comment": self.provenance,
            "schema": SCHEMA,
            "model": self.model,
            "provisional": self.provisional,
            "unmeasured": list(self.unmeasured),
            "units": {
                "kinematics": "mm (xyz), deg (rpy, XYZ order)",
                "mass": "kg",
                "com": "mm",
                "inertia": "kg*m^2, row-major upper-triangular "
                           "(Ixx,Ixy,Ixz,Iyy,Iyz,Izz)",
            },
            "kinematics": self.kinematics(),
            "dynamics": self.dynamics(),
        }

    def to_json(self) -> str:
        return json.dumps(self.to_json_dict(), indent=2) + "\n"
