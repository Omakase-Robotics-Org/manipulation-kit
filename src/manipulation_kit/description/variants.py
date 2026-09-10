"""The named URDF variants ``mkit-urdf export`` can produce.

Public names are hyphenated and robot-shaped (``d1-wholebody-gripper``), because
they are what a customer or a consumer repo writes down. They map onto the
exporter's older snake_case flavour ids, which stay as-is so a d1-sdk-era
consumer's ``PROVENANCE.json`` still reads true.

``d1-wholebody-o30`` is REGISTERED AND UNBUILDABLE on purpose. The O30 is a real
decision already taken (it replaces the DH116S on D1), so a consumer asking for
it should get the actual reason it cannot be built yet — no CAD, no measured TCP
— rather than "unknown variant", which reads like a typo and sends people
looking in the wrong place.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional


@dataclass(frozen=True)
class Variant:
    """One exportable robot variant."""

    #: Public, hyphenated name — the one users type.
    name: str
    #: Exporter flavour id, or ``None`` when the variant cannot be built.
    flavour: Optional[str]
    #: One line for ``--help``.
    summary: str
    #: If unbuildable, exactly what is missing.
    blocked_by: Optional[str] = None

    @property
    def buildable(self) -> bool:
        return self.flavour is not None


VARIANTS: Dict[str, Variant] = {
    v.name: v for v in (
        Variant(
            "d1-wholebody-gripper", "d1_wholebody_gripper",
            "whole D1 (base + lift + neck + both arms) wearing the stock "
            "parallel gripper — the authoritative asset other repos consume",
        ),
        Variant(
            "d1-collision", "d1_collision",
            "the mesh-free guard model d1.urdf alone: primitives-only "
            "collision, loads anywhere with zero assets",
        ),
        Variant(
            "d1-yubi", "d1_yubi",
            "mesh-bearing dual-arm D1 + YUBI hands (RViz, Genesis, MuJoCo)",
        ),
        Variant(
            "d1-arm", "d1_arm",
            "the vendor per-arm packages, copied verbatim (left and right)",
        ),
        Variant(
            "d1-wholebody-o30", None,
            "whole D1 wearing the LinkerHand O30 — NOT YET BUILDABLE",
            blocked_by=(
                "needs O30 CAD + a measured TCP.\n"
                "  Nothing about the O30's SHAPE exists yet: no STEP/STL from "
                "LinkerBot, so\n"
                "  manipulation_kit/hands/linkerbot/o30/ has no descriptions/ "
                "at all, and its\n"
                "  tool config registers the flange origin (kinematics = "
                "[0]*6) precisely\n"
                "  because choosing a TCP without measuring one would be an "
                "invention.\n"
                "  Building this variant needs, in order:\n"
                "    1. vendor CAD (STEP or STL) + its mount flange frame;\n"
                "    2. a TCP measured on the robot, the way the gripper's "
                "136 mm was;\n"
                "    3. a primitive collision approximation for the guard;\n"
                "    4. an END_EFFECTORS entry in "
                "description/d1/tools/generate_d1_urdf.py.\n"
                "  Until then d1-wholebody-gripper is the whole-body asset."
            ),
        ),
    )
}


class VariantNotBuildable(RuntimeError):
    """Raised for a registered variant whose inputs do not exist yet."""


def resolve(name: str) -> Variant:
    """Public variant name -> :class:`Variant`, or a useful error."""
    try:
        variant = VARIANTS[name]
    except KeyError:
        raise SystemExit(
            f"unknown variant {name!r}. Known: "
            + ", ".join(sorted(VARIANTS))) from None
    if not variant.buildable:
        raise VariantNotBuildable(
            f"variant {name!r} cannot be built: {variant.blocked_by}")
    return variant
