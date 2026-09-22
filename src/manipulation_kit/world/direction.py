"""A direction is a VALUE: a unit vector and the frame it is expressed in.

Before 0.16.0 "which way" had five encodings — a four-word approach enum whose
geometry lived in a string-keyed table, ``Nudge.frame`` + ``dx/dy/dz``,
``Retreat``'s hardcoded ``-tool_z``, and ``Lift``'s hardcoded ``+z`` — and a
verb that wanted a degree of freedom the enum did not have grew a named scalar
instead (``jaw_turn_deg``), which is how a planner knob reached a model's tool
schema. One type replaces all of them:

    Direction((0, 0, -1))                    # base frame: straight down
    Direction((0, 0, 1), frame=TOOL)         # along the hand's own approach axis
    Direction((1, 0, 0), frame="object:cup") # the cup's own +x

A direction says which way the TOOL TRAVELS. ``down`` is a descent onto the
object; ``forward`` moves away from the robot (+x). Resolution into the base
frame goes through the same :class:`~.frames.FrameGraph` every pose goes
through and raises :class:`~.frames.FrameError` like every other resolution
here: an unknown frame is never silently treated as the base.

The named aliases are data, not code paths — ``ALIASES["down"]`` is exactly
``Direction((0, 0, -1))`` and plans identically to it
(``tests/world/test_direction.py``).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple, Union

import numpy as np
from scipy.spatial.transform import Rotation as R

from .frames import BASE, UNKNOWN_FRAME, FrameError

#: the hand's own frame (the TCP frame: +z is the approach axis, +x the jaw gap)
TOOL = "tool"
#: ``object:<name>`` — the named object's own axes, resolved through its pose
OBJECT_PREFIX = "object:"
#: the fixed frame names; ``object:<name>`` is the open third kind
FRAMES: Tuple[str, ...] = (BASE, TOOL)


def object_frame(name: str) -> str:
    """The frame name for ``name``'s own axes."""
    return f"{OBJECT_PREFIX}{name}"


def _check_frame(frame: Any) -> str:
    if not isinstance(frame, str):
        raise ValueError(f"a direction's frame must be a string, got "
                         f"{type(frame).__name__}")
    if frame in FRAMES:
        return frame
    if frame.startswith(OBJECT_PREFIX) and len(frame) > len(OBJECT_PREFIX):
        return frame
    raise ValueError(f"unknown direction frame {frame!r}; the frames are "
                     f"{list(FRAMES)} and 'object:<name>'")


def _unit(v: Any) -> Tuple[float, float, float]:
    try:
        arr = np.asarray(v, dtype=float).reshape(3)
    except (TypeError, ValueError):
        raise ValueError(f"a direction is three numbers, got {v!r}") from None
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"a direction must be finite, got {list(arr)}")
    norm = float(np.linalg.norm(arr))
    if norm < 1e-9:
        raise ValueError("a direction cannot be the zero vector")
    arr = arr / norm
    return (float(arr[0]), float(arr[1]), float(arr[2]))


@dataclass(frozen=True)
class Direction:
    """A unit vector and the frame it is expressed in. Never a name.

    ``v`` need not arrive normalised; it is stored as a normalised tuple so a
    ``Direction`` is hashable and compares by value. ``via`` is an optional
    human description ("toward cup") that does not take part in equality.
    """

    v: Tuple[float, float, float]
    frame: str = BASE
    via: str = field(default="", compare=False)

    def __post_init__(self) -> None:
        object.__setattr__(self, "v", _unit(self.v))
        object.__setattr__(self, "frame", _check_frame(self.frame))

    # -- the value ----------------------------------------------------------- #
    def vector(self) -> np.ndarray:
        """The unit vector in ITS OWN frame."""
        return np.array(self.v, dtype=float)

    def opposite(self) -> "Direction":
        return Direction(tuple(-c for c in self.v), self.frame)

    def resolve(self, world, *, side: Optional[str] = None,
                tool_r: Optional[R] = None) -> np.ndarray:
        """The unit vector in BASE.

        ``TOOL`` needs the hand: ``side`` names it and its measured
        ``ArmView.tool_r`` is used, unless the caller passes the tool
        orientation it is planning from (``tool_r``). ``object:<name>`` resolves
        through that object's pose in ``world.frames``. Anything that does not
        resolve raises :class:`FrameError`, never falls back to base.
        """
        rot = frame_rotation(self.frame, world, side=side, tool_r=tool_r)
        v = self.vector()
        return v if rot is None else np.asarray(rot.apply(v), dtype=float)

    # -- rendering ----------------------------------------------------------- #
    def alias(self) -> Optional[str]:
        """The alias this direction IS, or ``None``."""
        for name, known in ALIASES.items():
            if known == self:
                return name
        return None

    def label(self) -> str:
        """``"down"``, ``"toward cup"``, ``"-along_tool"``, or the vector."""
        name = self.alias()
        if name is not None:
            return name
        if self.via:
            return self.via
        back = self.opposite().alias()
        if back is not None:
            return f"-{back}"
        return (f"({self.v[0]:+.3f}, {self.v[1]:+.3f}, {self.v[2]:+.3f}) "
                f"in {self.frame}")

    def as_argument(self) -> Union[str, Dict[str, Any]]:
        """The compact form a model writes: the alias, else ``{axis, frame}``."""
        name = self.alias()
        if name is not None:
            return name
        return {"axis": [round(c, 6) + 0.0 for c in self.v], "frame": self.frame}

    def to_json(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {"axis": [round(c, 6) + 0.0 for c in self.v],
                               "frame": self.frame, "label": self.label()}
        return out

    def __repr__(self) -> str:
        return f"Direction({self.label()})"


#: The named directions. Base-frame unless stated: +x forward, +y the robot's
#: LEFT, +z up. They are how a model says "down"; they are not a closed set —
#: any ``{axis, frame}`` is a direction too.
ALIASES: Dict[str, Direction] = {
    "down": Direction((0.0, 0.0, -1.0)),
    "up": Direction((0.0, 0.0, 1.0)),
    "forward": Direction((1.0, 0.0, 0.0)),
    "backward": Direction((-1.0, 0.0, 0.0)),
    "left": Direction((0.0, 1.0, 0.0)),
    "right": Direction((0.0, -1.0, 0.0)),
    "along_tool": Direction((0.0, 0.0, 1.0), frame=TOOL),
}

#: one line per alias, for generated prompt text (never hand-copied)
ALIAS_MEANING: Dict[str, str] = {
    "down": "travel straight down (base -z)",
    "up": "travel straight up (base +z)",
    "forward": "travel away from the robot (base +x)",
    "backward": "travel toward the robot (base -x)",
    "left": "travel toward the robot's left (base +y)",
    "right": "travel toward the robot's right (base -y)",
    "along_tool": "travel along the hand's own approach axis (tool +z)",
}


def frame_rotation(frame: str, world, *, side: Optional[str] = None,
                   tool_r: Optional[R] = None) -> Optional[R]:
    """The rotation that takes ``frame`` coordinates into BASE, or ``None``
    for the base itself. Raises :class:`FrameError` when it cannot be known."""
    frame = _check_frame(frame)
    if frame == BASE:
        return None
    if frame == TOOL:
        if tool_r is not None:
            return tool_r
        if side is None:
            raise FrameError(UNKNOWN_FRAME, TOOL,
                             "a tool-frame direction needs the hand it is "
                             "expressed in (side=)")
        arm = None if world is None else world.arm(side)
        if arm is None or arm.tool_r is None:
            raise FrameError(UNKNOWN_FRAME, TOOL,
                             f"the {side} arm reports no tool orientation")
        return arm.tool_r
    name = frame[len(OBJECT_PREFIX):]
    item = None if world is None else world.find(name)
    if item is None:
        raise FrameError(UNKNOWN_FRAME, frame,
                         f"there is no object called {name!r} to take the "
                         f"axes of")
    return item.pose_in_base(world.frames)[1]


def parse_direction(value: Union[Direction, str, Sequence[float],
                                 Mapping[str, Any]]) -> Direction:
    """A direction from any of the forms a caller or a model may write.

    ``"down"`` (an alias), ``[x, y, z]`` (base frame), or
    ``{"axis": [x, y, z], "frame": "tool"}``. Raises ``ValueError`` naming the
    aliases for anything else.
    """
    if isinstance(value, Direction):
        return value
    if isinstance(value, str):
        try:
            return ALIASES[value]
        except KeyError:
            raise ValueError(
                f"unknown direction {value!r}; the named directions are "
                f"{sorted(ALIASES)}, or give {{'axis': [x, y, z], 'frame': "
                f"...}}") from None
    if isinstance(value, Mapping):
        extra = sorted(set(value) - {"axis", "frame"})
        if extra or "axis" not in value:
            raise ValueError(f"a direction object is {{'axis': [x, y, z], "
                             f"'frame': ...}}, got keys {sorted(value)}")
        return Direction(tuple(value["axis"]), value.get("frame", BASE))
    if isinstance(value, (bytes, bool)) or value is None:
        raise ValueError(f"not a direction: {value!r}")
    return Direction(tuple(value), BASE)


def toward(world, *, name: str, from_p) -> Direction:
    """The base-frame direction from ``from_p`` to ``name``'s centre."""
    item = world.find(name)
    if item is None:
        raise FrameError(UNKNOWN_FRAME, object_frame(name),
                         f"there is no object called {name!r}")
    p = np.asarray(item.pose_in_base(world.frames)[0], dtype=float).reshape(3)
    d = p - np.asarray(from_p, dtype=float).reshape(3)
    if float(np.linalg.norm(d)) < 1e-9:
        raise ValueError(f"already at {name}; 'toward' has no direction")
    return Direction(tuple(d), BASE, via=f"toward {name}")


__all__ = ["ALIASES", "ALIAS_MEANING", "BASE", "Direction", "FRAMES",
           "OBJECT_PREFIX", "TOOL", "frame_rotation", "object_frame",
           "parse_direction", "toward"]

