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
                          arms wearing the stock parallel gripper
``arm-ee-bench``          two D1 arms (the same arm and controller) on a
                          stand with no D1 body, each wearing a Wuji Hand 2
========================  ====================================================

A new model is an entry here plus its URDFs (in the D1 generator's
``OUTPUTS`` while it is built from D1 parts) and its guard profile
(:data:`~manipulation_kit.description.bundle.GUARD_PROFILES`); its bundle and
its lock follow from those.

``arm-ee-bench`` replaces the ``d1-bench`` guard profile of the ``d1`` model,
which described the same rig as a D1 whose body checks are off while still
drawing and modelling the D1 body. ``d1-bench`` stays in the ``d1`` bundle so
a daemon that selects it by its record keeps working until it selects the
``arm-ee-bench`` model instead.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Mapping, Tuple

#: The ``robot.json`` format every bundle carries.
ROBOT_SCHEMA = "manipulation_kit.robot_model/1"


@dataclass(frozen=True)
class RobotModel:
    """One robot model."""

    #: The name every program selects it by (``d1``, ``arm-ee-bench``).
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
    #: Operator side -> the end effector it wears (a hand catalog model).
    end_effectors: Mapping[str, str] = field(default_factory=dict)
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
            "arms": {side: {"urdf_suffix": suffix,
                            "joints": [f"Joint{i}_{suffix}" for i in range(1, 8)],
                            "end_effector": self.end_effectors.get(side)}
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
            guard_profiles=("d1", "d1-bench", "sim"),
            devices=("arms", "grippers", "lift", "neck", "eyes", "chassis"),
            end_effectors={"left": "d1/parallel_gripper",
                           "right": "d1/parallel_gripper"},
        ),
        RobotModel(
            id="arm-ee-bench",
            summary="two D1 arms on a stand with no D1 body, each wearing a "
                    "Wuji Hand 2",
            guard_urdf="arm_ee_bench.urdf",
            view_urdf="arm_ee_bench_visual.urdf",
            view_variant="arm-ee-bench",
            guard_profiles=("arm-ee-bench",),
            devices=("arms", "hands"),
            end_effectors={"left": "wuji/wujihand2",
                           "right": "wuji/wujihand2"},
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
