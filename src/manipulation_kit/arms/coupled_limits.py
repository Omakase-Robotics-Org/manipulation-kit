"""Joint limits that depend on another joint — the wrist roll of the D1 arm.

The per-joint box (``KinematicChain.limits()``, from the URDF) says J7 may go
to +/-90 deg at any J6. On the hardware it may not: the hand, the wrist camera
plate and its cables catch on the J6 link, so the further the wrist is pitched
the less far it can roll. Measured by hand on d1-2 on 2026-09-22 after a live
Approach solved J7 = -90 deg at J6 = 55.2 deg and the wrist stopped at
-39.5 deg (``tests/data/d1_2_approach_20260922``): |J7| reaches 65 deg at
J6 = 30 deg and 39 deg at J6 = 55 deg, the same for either sign.

A :class:`CoupledJointLimit` is that table as DATA
(``config/coupled_joint_limits.json``, one entry per arm revision, with its
provenance). It is enforced where the box is:

* inside the IK iteration (:func:`manipulation_kit.arms.ik.solve_ik` projects
  every update onto it, so the convergence test only ever sees a posture the
  wrist can hold, and a target that needs more roll fails honestly rather
  than being clipped afterwards — the planner then tries its other roll
  candidates);
* in :meth:`manipulation_kit.arms.kinematics.GuardedArm.posture_violation`,
  the posture check ``solve_ee`` makes on every solution (reason
  ``joint_limit``).

Linear between the measured points, extrapolated linearly outside them, capped
at the nominal box, minus :attr:`CoupledJointLimit.margin_deg`. Below the first
measured driver angle the value is an EXTRAPOLATION — :meth:`measured` says so.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

import numpy as np

#: the bundled table, one entry per arm revision
DEFAULT_CONFIG = (Path(__file__).resolve().parents[1] / "config"
                  / "coupled_joint_limits.json")

#: a posture this close to a coupled limit [deg] is called out in a plan's notes
NEAR_LIMIT_DEG = 10.0


@dataclass(frozen=True)
class CoupledJointLimit:
    """``|q[driven]| <= limit_deg(|q[driver]|)``, joints 1-based as on the arm."""

    name: str
    driver_joint: int
    driven_joint: int
    #: ``(|driver| deg, |driven| max deg)``, measured, ascending in driver
    points_deg: Tuple[Tuple[float, float], ...]
    margin_deg: float
    #: the per-joint box of the driven joint; the coupled limit never exceeds it
    nominal_deg: float
    measured_driver_range_deg: Tuple[float, float]
    provenance: str = ""
    revision: str = ""

    def __post_init__(self) -> None:
        if len(self.points_deg) < 2:
            raise ValueError(f"coupled limit {self.name!r} needs at least two "
                             f"measured points, got {self.points_deg!r}")
        xs = [p[0] for p in self.points_deg]
        if any(b <= a for a, b in zip(xs, xs[1:])):
            raise ValueError(f"coupled limit {self.name!r}: driver angles must "
                             f"ascend, got {xs}")
        for v in (*xs, *(p[1] for p in self.points_deg), self.margin_deg,
                  self.nominal_deg):
            if not math.isfinite(float(v)):
                raise ValueError(f"coupled limit {self.name!r}: non-finite {v!r}")

    # -- the table ---------------------------------------------------------- #
    def _stop_deg(self, driver_deg: float) -> float:
        """The table at ``|driver_deg|``: piecewise linear, linear past the
        ends, never negative, NOT capped."""
        x = abs(float(driver_deg))
        pts = self.points_deg
        # the segment x falls in, or the end segment it extrapolates from
        i = 0
        while i < len(pts) - 2 and x > pts[i + 1][0]:
            i += 1
        (x0, y0), (x1, y1) = pts[i], pts[i + 1]
        return float(max(y0 + (x - x0) * (y1 - y0) / (x1 - x0), 0.0))

    def hardware_limit_deg(self, driver_deg: float) -> float:
        """The measured / extrapolated stop [deg], no margin, capped at the box."""
        return float(min(self._stop_deg(driver_deg), self.nominal_deg))

    def limit_deg(self, driver_deg: float) -> float:
        """The ENFORCED bound on ``|driven|`` [deg]: the stop minus the margin,
        capped at the nominal box — where the extrapolated stop lies beyond the
        box (|J6| below ~25 deg on the D1) the box alone binds, unnarrowed."""
        return float(min(max(self._stop_deg(driver_deg) - self.margin_deg,
                             0.0), self.nominal_deg))

    def measured(self, driver_deg: float) -> bool:
        """Is ``driver_deg`` inside the range the table was measured over?"""
        lo, hi = self.measured_driver_range_deg
        return lo <= abs(float(driver_deg)) <= hi

    # -- on a joint vector (radians, 0-based) ------------------------------- #
    @property
    def _i(self) -> Tuple[int, int]:
        return self.driver_joint - 1, self.driven_joint - 1

    def bound_rad(self, q) -> float:
        d, _ = self._i
        return math.radians(self.limit_deg(math.degrees(float(q[d]))))

    def margin_to_limit_deg(self, q) -> float:
        """``limit - |driven|`` [deg]; negative = violated."""
        d, v = self._i
        return (self.limit_deg(math.degrees(float(q[d])))
                - abs(math.degrees(float(q[v]))))

    def project(self, q) -> np.ndarray:
        """``q`` with the driven joint clipped into the coupled bound."""
        q = np.array(q, dtype=float)
        _, v = self._i
        b = self.bound_rad(q)
        q[v] = min(max(q[v], -b), b)
        return q

    def violation(self, q, *, tol_deg: float = 1e-6) -> Optional[str]:
        """Why ``q`` breaks this limit, or ``None``."""
        d, v = self._i
        margin = self.margin_to_limit_deg(q)
        if margin >= -tol_deg:
            return None
        return (f"J{self.driven_joint}={math.degrees(float(q[v])):+.1f} deg is "
                f"past the coupled {self.name} limit +/-"
                f"{self.limit_deg(math.degrees(float(q[d]))):.1f} deg at "
                f"J{self.driver_joint}={math.degrees(float(q[d])):+.1f} deg "
                f"({self.describe_source(math.degrees(float(q[d])))})")

    def describe_source(self, driver_deg: float) -> str:
        stop = self.hardware_limit_deg(driver_deg)
        where = ("measured range" if self.measured(driver_deg) else
                 "EXTRAPOLATED, outside the measured J%d range %g-%g deg"
                 % ((self.driver_joint,) + tuple(self.measured_driver_range_deg)))
        return (f"stop {stop:.1f} deg minus {self.margin_deg:g} deg margin, "
                f"{where}; {self.revision or 'unnamed revision'}")

    @classmethod
    def from_json(cls, block: Mapping[str, Any], *, revision: str = ""
                  ) -> "CoupledJointLimit":
        return cls(name=str(block["name"]),
                   driver_joint=int(block["driver_joint"]),
                   driven_joint=int(block["driven_joint"]),
                   points_deg=tuple((float(a), float(b))
                                    for a, b in block["points_deg"]),
                   margin_deg=float(block["margin_deg"]),
                   nominal_deg=float(block["nominal_deg"]),
                   measured_driver_range_deg=tuple(
                       float(v) for v in block["measured_driver_range_deg"]),
                   provenance=str(block.get("provenance", "")),
                   revision=revision)


def load_coupled_limits(revision: Optional[str] = None, *,
                        path: Optional[Path] = None
                        ) -> Tuple[CoupledJointLimit, ...]:
    """The coupled limits of one arm revision (default: the file's default).

    An unknown revision is a ``ValueError``: running with NO coupled limit
    because a name was mistyped is the outcome that must not happen silently.
    """
    doc = json.loads(Path(path or DEFAULT_CONFIG).read_text(encoding="utf-8"))
    name = revision or doc["default_revision"]
    revisions: Dict[str, Any] = doc["revisions"]
    if name not in revisions:
        raise ValueError(f"unknown arm revision {name!r} for coupled joint "
                         f"limits; known: {sorted(revisions)}")
    return tuple(CoupledJointLimit.from_json(b, revision=name)
                 for b in revisions[name]["coupled"])


def wrist_roll_limit(revision: Optional[str] = None) -> CoupledJointLimit:
    """The D1 arm's wrist-roll (J7 on J6) coupled limit."""
    for lim in load_coupled_limits(revision):
        if lim.name == "wrist_roll":
            return lim
    raise ValueError(f"arm revision {revision!r} has no wrist_roll limit")


def wrist_roll_limit_deg(j6_deg: float, *, revision: Optional[str] = None
                         ) -> float:
    """The enforced bound on |J7| [deg] at wrist pitch ``j6_deg``, margin
    included (see :class:`CoupledJointLimit`)."""
    return wrist_roll_limit(revision).limit_deg(j6_deg)


def project_all(limits: Sequence[CoupledJointLimit], q) -> np.ndarray:
    for lim in limits:
        q = lim.project(q)
    return np.asarray(q, dtype=float)


def first_violation(limits: Sequence[CoupledJointLimit], q) -> Optional[str]:
    for lim in limits:
        why = lim.violation(q)
        if why is not None:
            return why
    return None
