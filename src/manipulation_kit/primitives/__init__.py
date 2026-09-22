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

    verb = Grasp(object="red_block", direction="down")
    plan = verb.plan(world, kin)          # pure: nothing moved
    if plan.ok:
        run(plan, executor)               # manipulation_kit.executor
        assert verb.verifier(world)(observe()).verdict == "true"

The contract — ``preconditions`` / pure ``plan`` / measured ``verifier`` — is
written out in :mod:`.types`, which is also the module a consumer reads to
learn the refusal vocabulary. :mod:`.orientation` is where orientation is
DERIVED (``align_tool``, from a :class:`~manipulation_kit.world.Direction`) so a
model never emits one, and :mod:`.planning` is the single place a
primitive touches the IK and the guard.

The verbs: ``Approach Grasp Lift Carry Place Release Nudge Retreat GoHome``,
the ``Pour`` CONTRACT whose body is a learned policy
(:class:`~.types.LearnedPrimitive`), and the two contact verbs ``Probe`` and
``Press`` (:mod:`.contact`), which travel along a direction until something
resists and report where.

Beside them, three modules that are about CAPABILITY rather than about any
model, and are therefore here rather than in ``examples/agent/``:

``offer.py``      what can this robot do right now — every candidate planned
                  through the same IK, clamp and guard before it is offered,
                  with the refusals kept
``schema.py``     the canonical argument metadata and a dependency-free JSON
                  Schema export, plus ``decode`` for the way back
``arguments.py``  the one table both of those and every ``preconditions`` read
``reach.py``      which hand can do the WHOLE task, planned before anything
                  moves

None of them imports a provider SDK and none of them adds a dependency: the
package still installs as numpy + scipy. What stays outside the wheel is the
part that knows a model exists — prompts, authentication, request envelopes,
ranking, menu capping, the scripted policy and the runnable loops.
"""

from .orientation import (TOOL_Z_M, align_tool,
                          choose_side, grasp_orientation, grasp_width,
                          jaw_axis, link7_from_tool, roll_tool,
                          tool_from_link7, tool_revision)
from .grasp_geometry import (PAD, TIP, GraspReference, GraspSpec, fits,
                             graspable_width_m, grasp_pose, roll_candidates,
                             standoff_point, support_of)
from .arguments import ARGUMENTS, Argument, check_arguments, names_for
from .offer import (Offered, Refused, candidates_for, label_for, offer,
                    why_nothing)
from .planning import (ARRIVE_TOL_M, PATH_TOL_M, PATH_TOL_RAD, Kin,
                       joint_ramp, missing_arms, solve_path)
from .reach import ChainLink, ChainPlan, SideChoice, plan_chain
from .reach import choose_side as choose_side_for_task
from .schema import (NOT_MODEL_BINDABLE, decode, direction_doc, domains,
                     domains_in, tool_schemas, verbs_in)
from .types import (AUTO, BOTH, ContactCriterion, ContactStep,
                    GOHOME_SIDE_CHOICES, GRIPS,
                    GRASP_DIRECTIONS, NUDGE_FRAMES, NUDGE_GRID_M,
                    NUDGE_MAX_YAW_RAD, PLAN_REASONS, PRIMITIVE_CONTRACT, SIDES,
                    SIDE_CHOICES, UNMET_CODES, GripStep, JointStep,
                    LearnedPrimitive, Plan, PlanBinding,
                    PlanError, Primitive, SettleStep, Step, Unmet, Verdict,
                    VerdictReport, Verifier, Waypoint)
from .verbs import (BY_VERB, PRIMITIVES, Approach, Carry, GoHome, Grasp, Lift,
                    Nudge, Place, Pour, Release, Retreat, by_verb, snap)
from .contact import (ContactPlane, Press, Probe, fit_plane, record_contacts,
                      surface_from_contacts)

__all__ = [
    # the model-independent action boundary (offer / schema / reach)
    "offer", "Offered", "Refused", "candidates_for", "label_for", "why_nothing",
    "tool_schemas", "decode", "domains", "domains_in", "verbs_in",
    "NOT_MODEL_BINDABLE", "direction_doc",
    "ARGUMENTS", "Argument", "check_arguments", "names_for",
    "plan_chain", "choose_side_for_task", "ChainPlan", "ChainLink", "SideChoice",
    # verbs
    "Approach", "Grasp", "Lift", "Carry", "Place", "Release", "Nudge",
    "Retreat", "GoHome", "Pour", "Probe", "Press", "PRIMITIVES", "BY_VERB",
    "by_verb",
    # contact: measured contacts into the world
    "record_contacts", "fit_plane", "ContactPlane", "surface_from_contacts",
    # contract
    "Primitive", "LearnedPrimitive", "PRIMITIVE_CONTRACT",
    "Plan", "PlanBinding", "PlanError", "Waypoint", "JointStep", "GripStep",
    "SettleStep", "ContactStep", "ContactCriterion", "Step", "UNMET_CODES",
    "Unmet", "Verdict", "VerdictReport", "Verifier",
    # vocabulary
    "AUTO", "BOTH", "GOHOME_SIDE_CHOICES", "GRASP_DIRECTIONS", "GRIPS",
    "NUDGE_FRAMES", "NUDGE_GRID_M", "NUDGE_MAX_YAW_RAD", "PLAN_REASONS",
    "SIDES", "SIDE_CHOICES", "snap",
    # geometry helpers consumers legitimately need
    "TOOL_Z_M", "align_tool", "choose_side",
    "grasp_orientation", "roll_tool", "grasp_width", "jaw_axis",
    "link7_from_tool", "tool_from_link7", "tool_revision",
    # where and how the hand meets an object
    "GraspReference", "GraspSpec", "PAD", "TIP", "fits", "graspable_width_m",
    "grasp_pose", "roll_candidates", "standoff_point", "support_of",
    # planning
    "Kin", "solve_path", "joint_ramp", "missing_arms",
    "ARRIVE_TOL_M", "PATH_TOL_M", "PATH_TOL_RAD",
]
