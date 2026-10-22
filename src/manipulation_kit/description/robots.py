"""The robot models this package describes, selected by name.

A robot model is one physical arrangement of hardware: which arms, where they
are mounted, what else is there (a body, a lift, a neck, a chassis) and what
the arms wear. Every program that needs a robot's shape — the daemon's motion
guard, the teleop server, a 3D view, a simulator — names the model and takes
that model's description bundle (:mod:`manipulation_kit.description.bundle`):
its URDFs, its collision model, its guard policy and its HOME pose. Nothing
else carries a copy, and no program special-cases a model by any other sign.

========================  ====================================================
model                     what it is
========================  ====================================================
``d1``                    the D1 robot: chassis, lift, torso, head and two
                          arms
``d1-arms-stand``         two D1 arms (the same arm and controller) on a
                          stand, with no D1 body
========================  ====================================================

Naming
------
A model id names the kinematic structure, never a use, a site or a tool:
lowercase, hyphen-separated, ``<platform>`` for a whole robot of that
platform, ``<platform>-<parts>-<mount>`` for parts of it mounted on something
else (``d1-arms-stand``: the D1's two arms on a stand; one arm would be
``d1-arm-stand``). Words such as ``bench``, ``demo`` or a site name are not
used: the same structure keeps its id wherever and for whatever it is used.
End effectors are never part of an id: what a flange carries is chosen from
the hand catalog (:mod:`manipulation_kit.hands.catalog`) and declared by its
provider at run time.

End effectors
-------------
:attr:`RobotModel.end_effector_collision` says whether the guard checks
whatever a flange carries. On ``d1`` it does (the guard model carries the
hand envelope, and a daemon applies the declared hand's catalog envelope).
On ``d1-arms-stand`` it does not: the arms end at their flanges, the guard
checks arm against arm, each arm against itself and the joint limits, and
nothing mounted on a flange is collision-checked.

Recorded input
--------------
:attr:`RobotModel.recorded_input` says whether recorded or synthetic operator
input (a replayed capture, a mock device) may drive this model's real arms:
``refused`` on ``d1``; ``operator_override`` on ``d1-arms-stand``, where a
teleop server may allow it while the operator has switched its override on.

A new model is an entry here plus its URDFs (in the D1 generator's
``OUTPUTS`` while it is built from D1 parts) and its guard profile
(:data:`~manipulation_kit.description.bundle.GUARD_PROFILES`); its bundle and
its lock follow from those.

``d1-arms-stand`` is also a guard profile of the ``d1`` model: a daemon is
always a D1 and selects the profile by name (``[arm] guard_profile``) from the
one D1 bundle it reads; this model's own bundle (the stand's URDFs, for a 3D
view) carries the same profile. It replaces the ``d1-bench`` profile, which
described the same arms on a stand as a D1 whose body checks are off.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Mapping, Tuple

#: The ``robot.json`` format every bundle carries.
ROBOT_SCHEMA = "manipulation_kit.robot_model/1"

#: The values of :attr:`RobotModel.recorded_input`.
RECORDED_INPUT = ("refused", "operator_override")


@dataclass(frozen=True)
class RobotModel:
    """One robot model."""

    #: The name every program selects it by (``d1``, ``d1-arms-stand``).
    id: str
    #: One line for an operator.
    summary: str
    #: The mesh-free primitives URDF the motion guard parses, in
    #: ``description/d1/``.
    guard_urdf: str
    #: The articulated URDF a 3D view draws (CAD visuals by reference), in
    #: ``description/d1/``.
    view_urdf: str
    #: The ``mkit-urdf export`` variant of :attr:`view_urdf`.
    view_variant: str
    #: The guard profiles the bundle carries; the first is the one a real
    #: robot of this model runs.
    guard_profiles: Tuple[str, ...]
    #: The devices a robot of this model has, in d1-firmwared's words.
    devices: Tuple[str, ...]
    #: Whether the guard checks what a flange carries (see the module).
    end_effector_collision: bool
    #: Whether recorded or synthetic input may drive the real arms
    #: (:data:`RECORDED_INPUT`, see the module).
    recorded_input: str
    #: Operator side -> the URDF suffix of that arm's chain (the D1's
    #: crossed convention, manipulation_kit.arms.sides.URDF_SUFFIX).
    arm_suffixes: Mapping[str, str] = field(
        default_factory=lambda: {"left": "R", "right": "L"})

    def document(self) -> dict:
        """``robot.json``: the model as its bundle states it."""
        return {
            "schema": ROBOT_SCHEMA,
            "model": self.id,
            "summary": self.summary,
            "urdf": {"guard": "kinematics/" + self.guard_urdf,
                     "view": "kinematics/" + self.view_urdf},
            "guard_profile": self.guard_profiles[0],
            "guard_profiles": list(self.guard_profiles),
            "devices": list(self.devices),
            "end_effectors": {
                "chosen_from": "hand catalog, declared by the provider at run time",
                "collision_checked": self.end_effector_collision,
            },
            "recorded_input": self.recorded_input,
            "arms": {side: {"urdf_suffix": suffix,
                            "joints": [f"Joint{i}_{suffix}" for i in range(1, 8)]}
                     for side, suffix in sorted(self.arm_suffixes.items())},
        }


ROBOT_MODELS: Dict[str, RobotModel] = {
    m.id: m for m in (
        RobotModel(
            id="d1",
            summary="the D1 robot: chassis, lift, torso, head and two arms",
            guard_urdf="d1.urdf",
            view_urdf="d1_wholebody_gripper.urdf",
            view_variant="d1-wholebody-gripper",
            guard_profiles=("d1", "d1-arms-stand", "sim"),
            devices=("arms", "end_effectors", "lift", "neck", "eyes", "chassis"),
            end_effector_collision=True,
            recorded_input="refused",
        ),
        RobotModel(
            id="d1-arms-stand",
            summary="two D1 arms on a stand, with no D1 body",
            guard_urdf="d1_arms_stand.urdf",
            view_urdf="d1_arms_stand_visual.urdf",
            view_variant="d1-arms-stand",
            guard_profiles=("d1-arms-stand",),
            devices=("arms", "end_effectors"),
            end_effector_collision=False,
            recorded_input="operator_override",
        ),
    )
}

#: The model a program assumes when nothing names one.
DEFAULT_MODEL = "d1"


def robot_model(name: str = DEFAULT_MODEL) -> RobotModel:
    """Robot model ``name``; ``KeyError`` naming the known ones if unknown."""
    try:
        return ROBOT_MODELS[name]
    except KeyError:
        raise KeyError(f"unknown robot model {name!r}; known: "
                       + ", ".join(sorted(ROBOT_MODELS))) from None
