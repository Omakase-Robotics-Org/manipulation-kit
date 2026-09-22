"""A REAL D1 as the loop's robot: firmware transport plus an observation source.

The half the kit deliberately does not own is the observation: nothing in
``manipulation_kit`` opens a camera. What it does own is the ROBOT half of a
``WorldView`` — the arms, their tool poses and the grippers — and that is what
this adapter fills in from ``RawState``. The objects come from a scene file
you measured (``--scene``), which is the documented first-hour path.

See the README's Quickstart for the calibration this assumes and for what
``UNKNOWN`` means when a producer cannot supply a piece of evidence.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
from scipy.spatial.transform import Rotation as R

from manipulation_kit.primitives.orientation import tool_from_link7
from manipulation_kit.world import (ArmView, ContainerView, Frame, FrameGraph,
                                    GripperView, ObjectView, SurfaceView,
                                    WorldView)

KINDS = {"object": ObjectView, "container": ContainerView, "surface": SurfaceView}


def load_scene(path: Path) -> Dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def objects_from(scene: Dict[str, Any]) -> List[ObjectView]:
    """Build the measured objects from a scene file. Units: metres, radians."""
    out: List[ObjectView] = []
    for item in scene.get("objects", ()):
        kind = KINDS[item.get("kind", "object")]
        extra: Dict[str, Any] = {}
        if kind is ContainerView and "interior" in item:
            extra["interior"] = item["interior"]
        if kind is ContainerView and "rim_height_m" in item:
            extra["rim_height_m"] = item["rim_height_m"]
        if kind is ContainerView and "interior_measured" in item:
            # A scene file that GIVES an interior used to be believed
            # unconditionally, because ``ContainerView`` only clears the flag
            # for the interior it invents itself. A perceived interior is a
            # number AND a guess (``examples/agent/perceive.py`` writes 85% of
            # the measured outside), and ``Place`` refuses to drop into a
            # guessed interior — which it cannot do if the file cannot say so.
            extra["interior_measured"] = bool(item["interior_measured"])
        out.append(kind(
            item["name"], p=item["p"], size=item["size"],
            r=R.from_quat(item["quat_xyzw"]) if "quat_xyzw" in item
            else R.from_euler("z", float(item.get("yaw_rad", 0.0))),
            frame_id=item.get("frame_id", "base"),
            colour=item.get("colour"), stamp=float(item.get("stamp", 0.0)),
            # A perceived or model-declared object arrives with a confidence
            # below 1 and it has to SURVIVE the file, or the one honest thing
            # about the number is the thing that gets dropped on the way in.
            confidence=float(item.get("confidence", 1.0)),
            **extra))
    return out


def frames_from(scene: Dict[str, Any], *, now: float) -> FrameGraph:
    graph = FrameGraph(now=now)
    for frame in scene.get("frames", ()):
        graph.add(Frame(
            frame["frame_id"], frame.get("parent", "base"), p=frame["p"],
            r=R.from_quat(frame["quat_xyzw"]) if "quat_xyzw" in frame
            else R.from_euler("z", float(frame.get("yaw_rad", 0.0))),
            stamp=float(frame.get("stamp", now)),
            max_age_s=frame.get("max_age_s"),
            valid=bool(frame.get("valid", True))))
    return graph


class LiveRobot:
    """``RawState`` from the daemon + a measured scene file -> ``WorldView``.

    The gripper's ``held_object`` is the one piece of bookkeeping the firmware
    cannot supply: the daemon reports that the jaws stalled on SOMETHING, not
    what. So it is tracked here, from what the loop asked for, and it is the
    ASSOCIATION half of the pickup predicate — the verifiers return UNKNOWN
    rather than TRUE when it is absent, which is why this is a named field and
    not an assumption.
    """

    def __init__(self, executor, kin, scene: Optional[Dict[str, Any]] = None):
        self.executor = executor
        self.kin = kin
        self.scene = scene or {"objects": [], "frames": []}
        self.held: Dict[str, Optional[str]] = {"left": None, "right": None}
        self.revision = 0

    def expect(self, side: str, name: Optional[str]) -> None:
        """Record what the next stroke on ``side`` is closing on."""
        self.held[side] = name

    def declare(self, objects) -> None:
        """Replace or add scene objects, by name — the loop's ``declare_scene``.

        The scene file is the THINGS half of the observation and nothing in
        the kit can supply it, so a model that has looked at the frame and
        said where something is has produced the only measurement there is.
        Stored back into the same dict the file was read from, so a run's
        ``scene_perceived.json`` and the world the plan was built in stay the
        same shape.
        """
        incoming = {}
        for view in objects:
            item = {"name": view.name, "kind": view.kind,
                    "frame_id": view.frame_id,
                    "p": [float(v) for v in view.p],
                    "size": [float(v) for v in view.size],
                    "quat_xyzw": [float(v) for v in view.r.as_quat()],
                    "confidence": float(view.confidence)}
            interior = getattr(view, "interior", None)
            if interior is not None:
                item["interior"] = [float(v) for v in interior]
                item["interior_measured"] = bool(
                    getattr(view, "interior_measured", False))
            rim = getattr(view, "rim_height_m", None)
            if rim is not None:
                item["rim_height_m"] = float(rim)
            incoming[view.name] = item
        objects_json = list(self.scene.get("objects", ()))
        merged = [incoming.pop(o.get("name"), o) for o in objects_json]
        self.scene["objects"] = merged + list(incoming.values())

    def world(self) -> WorldView:
        state = self.executor.state()
        self.revision += 1
        arms, grippers = [], []
        for side in ("left", "right"):
            q = state.joints.get(side)
            if q is None:
                continue
            saved = np.array(self.kin.joints(side), dtype=float)
            try:
                self.kin.set_joints(side, q)
                p, r = tool_from_link7(*self.kin.ee_pose(side))
            finally:
                self.kin.set_joints(side, saved)
            arms.append(ArmView(side, joints=q, tool_p=p, tool_r=r,
                                mode=state.extra.get(f"{side}_mode") or "position",
                                error_code=int(state.extra.get(f"{side}_error_code", 0) or 0),
                                stationary=state.stationary))
            closedness = state.grippers.get(side)
            if closedness is None:
                continue        # UNKNOWN, not "open"
            holding = bool(state.holding.get(side, False))
            grippers.append(GripperView(
                side, closedness, holding=holding,
                held_object=self.held.get(side) if holding else None,
                jaw_gap_m=state.extra.get(f"{side}_jaw_gap_m"),
                jaw_stalled=state.extra.get(f"{side}_jaw_stalled")))
        return WorldView.of(objects=objects_from(self.scene),
                            frames=frames_from(self.scene, now=state.stamp),
                            arms=arms, grippers=grippers,
                            stamp=state.stamp, revision=self.revision)
