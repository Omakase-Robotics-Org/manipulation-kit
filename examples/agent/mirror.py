"""The demo robot: a kinematic mirror plus a scene the block moves in.

This exists so ``astra_loop`` can take a ROBOT rather than build one inline.
The old loop hardcoded a ``Mirror`` class, its own private step walker and a
block attached to the hand, and then said "swap the executor; the loop does
not change" — which was not true, because the walker was the loop's (R,
section 1).

Here the mirror is an ``Executor`` and nothing else, so the loop runs it
through ``manipulation_kit.executor.run`` exactly as it runs the firmware
transport. A mirror teleports the object to the tool and has no dynamics, no
contact and no settle: it is the right fidelity for testing that a PLAN is
well formed, and no fidelity at all for whether a grasp holds.
"""

from __future__ import annotations


import numpy as np

from manipulation_kit.executor import KinematicExecutor
from manipulation_kit.primitives.approach import tool_from_link7
from scene import BLOCK_P, observe


class MirrorRobot:
    """A ``KinematicExecutor`` and the scene it is moving things in."""

    OBJECT = "red_block"

    def __init__(self, kin, block_p=BLOCK_P):
        self.kin = kin
        self.executor = KinematicExecutor(kin)
        for side in ("left", "right"):
            self.executor.next_object[side] = self.OBJECT
        self.block = np.array(block_p, dtype=float)
        self._wrap_send()

    def _wrap_send(self) -> None:
        """The block follows a closed hand. Done by decorating the executor so
        the LOOP never walks the steps itself."""
        inner = self.executor.send_joints

        def send(q16, *, t: float) -> None:
            inner(q16, t=t)
            self._follow()

        self.executor.send_joints = send      # type: ignore[assignment]
        inner_grip = self.executor.set_gripper

        def grip(side: str, closedness: float, *, grip: str) -> None:
            inner_grip(side, closedness, grip=grip)
            self._follow()

        self.executor.set_gripper = grip      # type: ignore[assignment]

    def _follow(self) -> None:
        for side in ("left", "right"):
            if self.executor.held.get(side) == self.OBJECT:
                self.block = tool_from_link7(*self.kin.ee_pose(side))[0].copy()

    def world(self):
        held = {s: self.executor.held.get(s) for s in ("left", "right")}
        return observe(self.kin, block_p=self.block,
                       closed=dict(self.executor.grippers), held=held)


class SceneMirrorRobot(MirrorRobot):
    """``MirrorRobot`` over a MEASURED scene (``--scene`` / ``--perceive``):
    the observed world is the scene's objects, with the named object following
    a closed hand.

    THE OBJECT MAY NOT BE THERE YET. Since ``--perceive``'s default leaves the
    things to the model — which declares them with ``declare_scene`` on turn 0
    — the tracked object arrives after construction, and refusing to build
    without it would make the zero-shot path impossible to start. It is picked
    up the first time it appears.
    """

    def __init__(self, kin, world0, obj: str):
        self.OBJECT = obj
        self.world0 = world0
        target = [o for o in world0.objects if o.name == obj]
        super().__init__(kin, block_p=target[0].p if target
                         else (0.0, 0.0, 0.0))
        self.tracking = bool(target)

    def declare(self, objects) -> None:
        """Replace or add scene objects, by name. The model's own measurement.

        Names are matched exactly and everything else about the world — the
        arms, the grippers, the frames — is untouched: this is an OBSERVATION
        of the things, not a new world.
        """
        import dataclasses as _dc  # noqa: PLC0415
        incoming = {o.name: o for o in objects}
        kept = [incoming.pop(o.name, o) for o in self.world0.objects]
        self.world0 = _dc.replace(self.world0,
                                  objects=tuple(kept) + tuple(incoming.values()))
        target = [o for o in self.world0.objects if o.name == self.OBJECT]
        if target and not self.tracking:
            self.block = np.array(target[0].p, dtype=float)
            self.tracking = True
        elif target and not any(self.executor.held.get(s) == self.OBJECT
                                for s in ("left", "right")):
            # Not in a hand: believe the new measurement over the old one.
            self.block = np.array(target[0].p, dtype=float)

    def world(self):
        import dataclasses as _dc  # noqa: PLC0415
        base = super().world()
        objects = tuple(
            _dc.replace(o, p=self.block.copy())
            if (o.name == self.OBJECT and self.tracking) else o
            for o in self.world0.objects)
        return _dc.replace(base, objects=objects, frames=self.world0.frames)
