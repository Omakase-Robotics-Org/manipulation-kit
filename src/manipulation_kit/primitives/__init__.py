"""manipulation_kit.primitives — the verbs, and the contract every verb keeps.

Shu, 2026-09-19: 「approach とか少し高次のスキルも manip kit に実装するわけで、
それは agent の中ではなくて、普通に primitive の中に入れる」 — approach and the
other higher-level skills belong in the kit as ordinary primitives, not inside
an agent package. So they are here, usable with no model anywhere near them:

    from manipulation_kit.arms import get_arm_kinematics
    from manipulation_kit.primitives import Grasp
    from manipulation_kit.world import ArmView, ObjectView, WorldView

    kin = get_arm_kinematics("d1/arm", quiet=True)
    world = WorldView.of(
        [ObjectView("red_block", p=(0.42, 0.12, 0.10), size=(0.05, 0.04, 0.05))],
        arms=[ArmView(s, joints=kin.joints(s)) for s in ("left", "right")],
        grippers=[GripperView(s, 0.0) for s in ("left", "right")])

    verb = Grasp(object="red_block", approach="top_down")
    plan = verb.plan(world, kin)          # pure: nothing moved
    if plan.ok:
        run(plan, executor)               # manipulation_kit.executor
        assert verb.verifier(world)(observe()).verdict == "true"

The contract — ``preconditions`` / pure ``plan`` / measured ``verifier`` — is
written out in :mod:`.types`, which is also the module a consumer reads to
learn the refusal vocabulary. :mod:`.approach` is where orientation is DERIVED
so a model never emits one, and :mod:`.planning` is the single place a
primitive touches the IK and the guard.

The verbs: ``Approach Grasp Lift Carry Place Release Nudge Retreat GoHome``,
plus the ``Pour`` CONTRACT whose body is a learned policy
(:class:`~.types.LearnedPrimitive`).
"""

from .approach import (APPROACH_DIRECTION, APPROACH_DOC, JAW_OPEN_M, TOOL_Z_M,
                       choose_side, grasp_orientation, link7_from_tool,
                       tool_from_link7)
from .planning import ARRIVE_TOL_M, Kin, joint_ramp, solve_path
from .types import (APPROACHES, AUTO, BOTH, GOHOME_SIDE_CHOICES, GRIPS,
                    NUDGE_FRAMES, NUDGE_GRID_M,
                    NUDGE_MAX_YAW_RAD, PLAN_REASONS, PRIMITIVE_CONTRACT, SIDES,
                    SIDE_CHOICES, GripStep, JointStep, LearnedPrimitive, Plan,
                    PlanError, Primitive, SettleStep, Unmet, Verdict,
                    VerdictReport, Verifier, Waypoint)
from .verbs import (BY_VERB, PRIMITIVES, Approach, Carry, GoHome, Grasp, Lift,
                    Nudge, Place, Pour, Release, Retreat, by_verb, snap)

__all__ = [
    # verbs
    "Approach", "Grasp", "Lift", "Carry", "Place", "Release", "Nudge",
    "Retreat", "GoHome", "Pour", "PRIMITIVES", "BY_VERB", "by_verb",
    # contract
    "Primitive", "LearnedPrimitive", "PRIMITIVE_CONTRACT",
    "Plan", "PlanError", "Waypoint", "JointStep", "GripStep", "SettleStep",
    "Unmet", "Verdict", "VerdictReport", "Verifier",
    # vocabulary
    "APPROACHES", "APPROACH_DIRECTION", "APPROACH_DOC", "AUTO", "BOTH",
    "GOHOME_SIDE_CHOICES", "GRIPS",
    "NUDGE_FRAMES", "NUDGE_GRID_M", "NUDGE_MAX_YAW_RAD", "PLAN_REASONS",
    "SIDES", "SIDE_CHOICES", "snap",
    # geometry helpers consumers legitimately need
    "JAW_OPEN_M", "TOOL_Z_M", "choose_side", "grasp_orientation",
    "link7_from_tool", "tool_from_link7",
    # planning
    "Kin", "solve_path", "joint_ramp", "ARRIVE_TOL_M",
]
