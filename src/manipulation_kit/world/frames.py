"""Frames that travel WITH the pose, and refuse rather than average.

Every pose this package carries names the frame it was measured in. That is
not bookkeeping: on D1 the useful frames are estimates with a validity
condition attached, and the failure mode is silent.

``d1-inference``'s ``scene/table_frame.py`` is the case that taught it. The
table frame is a homography fitted from ONE neck pose; move the neck and the
fit still evaluates, still returns a plausible plane, and is wrong by
centimetres. That module answers by CHECKING the neck pose and refusing,
rather than averaging over two fits. This module puts the same rule in the
type system: a frame carries a ``stamp`` and a ``max_age_s``, and resolving a
pose through a frame older than that raises :class:`FrameError` with reason
``frame_stale`` instead of returning a number. A frame nobody registered
raises ``unknown_frame``.

Both reasons are in the primitive refusal vocabulary
(:mod:`manipulation_kit.primitives`), so "I cannot act because the table frame
is 40 seconds old" reaches the caller as a typed reason rather than as a
plausible pose.

Nothing here opens a camera or estimates anything. Producers
(``d1-inference/scene``, the Isaac env server, an ArUco detector) build the
graph; the kit reads it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

#: The frame every plan is expressed in: the robot base / torso platform,
#: +x forward, +y the robot's left, +z up. It is the frame
#: :meth:`manipulation_kit.arms.kinematics.ArmKinematics.ee_pose` reports in,
#: so it is the root of the graph by definition and needs no registration.
BASE = "base"

#: typed refusal reasons this module can produce
UNKNOWN_FRAME = "unknown_frame"
FRAME_STALE = "frame_stale"

#: How far ahead of ``now`` a frame stamp may sit before it is treated as
#: stale rather than as fresh. Small, because the only legitimate cause is
#: clock granularity between the producer and the observation stamp.
FUTURE_TOL_S = 0.050


class FrameError(LookupError):
    """A pose could not be resolved into the base frame.

    ``reason`` is one of :data:`UNKNOWN_FRAME` / :data:`FRAME_STALE` — the same
    strings the primitives report, so a caller never has to parse a message.
    """

    def __init__(self, reason: str, frame_id: str, detail: str = ""):
        self.reason = reason
        self.frame_id = frame_id
        self.detail = detail
        super().__init__(f"{reason}: {frame_id}" + (f" ({detail})" if detail else ""))


@dataclass(frozen=True)
class Frame:
    """One registered frame: where it sits in its parent, and when that was true.

    ``max_age_s=None`` means the frame is a fixed mount and never expires (a
    bolted-on camera, the lift column). Anything ESTIMATED should give a real
    age: an ArUco read, a table homography, a detector's own frame.

    ``valid=False`` is the explicit "this estimate is known bad" flag —
    ``table_frame`` sets it when the neck has moved since the fit. It refuses
    with ``frame_stale`` regardless of age, because that is what it is.
    """

    frame_id: str
    parent: str
    p: np.ndarray
    r: R
    stamp: float = 0.0
    max_age_s: Optional[float] = None
    valid: bool = True

    def __post_init__(self) -> None:
        p = np.array(self.p, dtype=float).reshape(3)
        p.setflags(write=False)
        object.__setattr__(self, "p", p)
        if not isinstance(self.r, R):
            raise TypeError(f"frame {self.frame_id!r}: r must be a scipy Rotation")
        if self.frame_id == BASE:
            raise ValueError(f"{BASE!r} is the graph root and is never registered")
        if self.max_age_s is not None and self.max_age_s <= 0:
            raise ValueError(f"frame {self.frame_id!r}: max_age_s must be positive")
        if not math.isfinite(float(self.stamp)):
            raise ValueError(f"frame {self.frame_id!r}: stamp must be finite — "
                             f"a NaN stamp is never older than any limit and "
                             f"would read as fresh forever")

    def age_s(self, now: float) -> float:
        return float(now) - float(self.stamp)

    def fresh(self, now: float, *, future_tol_s: float = FUTURE_TOL_S) -> bool:
        """Is this frame usable at ``now``?

        Three ways to fail, and the third one is new: a frame stamped in the
        FUTURE has a negative age, which passes every ``age <= max_age`` test
        there is and would stay "fresh" indefinitely. A clock that disagrees
        with the producer's is a measurement problem, not a licence to act.
        """
        if not self.valid:
            return False
        if not math.isfinite(float(now)):
            return False
        if self.age_s(now) < -float(future_tol_s):
            return False
        return self.max_age_s is None or self.age_s(now) <= self.max_age_s


@dataclass
class FrameGraph:
    """A tree of frames rooted at :data:`BASE`, resolved or refused.

    The graph is deliberately tiny and deliberately not clever: no
    interpolation, no extrapolation, no averaging of two estimates of the same
    frame. Registering a frame twice REPLACES it, because a newer estimate is
    the answer and a blend of two is nobody's measurement.
    """

    frames: Dict[str, Frame] = field(default_factory=dict)
    #: "now" for staleness decisions. The kit does not read a clock (a plan
    #: must be reproducible), so the producer sets this to the observation
    #: time when it builds the world.
    now: float = 0.0

    @classmethod
    def of(cls, frames: Iterable[Frame] = (), *, now: float = 0.0) -> "FrameGraph":
        graph = cls(now=now)
        for frame in frames:
            graph.add(frame)
        return graph

    def add(self, frame: Frame) -> "FrameGraph":
        self.frames[frame.frame_id] = frame
        return self

    def copy(self, *, now: Optional[float] = None) -> "FrameGraph":
        """A detached copy, optionally re-clocked.

        ``WorldView.with_(stamp=...)`` used to keep the OLD graph and its old
        ``now``, so a later world stamp did not age a single transform and a
        30-second-old table frame stayed "fresh" for as long as the test kept
        stamping worlds. Re-clocking is what makes a newer observation
        actually ask the freshness question again.
        """
        return FrameGraph(frames=dict(self.frames),
                          now=self.now if now is None else float(now))

    def revision(self) -> Tuple[Tuple[str, float, bool], ...]:
        """A hashable fingerprint of every registered transform's identity.

        A plan is checked against the transforms that were in force when it
        was built; this is the thing an executor compares before it moves
        (see ``manipulation_kit.primitives.types.PlanBinding``).
        """
        return tuple(sorted((f.frame_id, float(f.stamp), bool(f.valid))
                            for f in self.frames.values()))

    def known(self, frame_id: str) -> bool:
        return frame_id == BASE or frame_id in self.frames

    def pose_in_base(self, frame_id: str) -> Tuple[np.ndarray, R]:
        """``(p, r)`` of ``frame_id`` expressed in :data:`BASE`.

        Raises :class:`FrameError` — ``unknown_frame`` for a frame (or a
        parent) nobody registered and for a cycle, ``frame_stale`` for one past
        its ``max_age_s`` or explicitly marked invalid.
        """
        p = np.zeros(3)
        r = R.identity()
        seen = []
        current = frame_id
        while current != BASE:
            if current in seen:
                raise FrameError(UNKNOWN_FRAME, frame_id,
                                 "cycle through " + " -> ".join(seen + [current]))
            seen.append(current)
            frame = self.frames.get(current)
            if frame is None:
                raise FrameError(UNKNOWN_FRAME, frame_id,
                                 f"{current!r} is not registered")
            if not frame.fresh(self.now):
                age = frame.age_s(self.now)
                detail = (f"{current!r} is marked invalid" if not frame.valid
                          else f"{current!r} is {age:.1f}s old, limit "
                               f"{frame.max_age_s:.1f}s")
                raise FrameError(FRAME_STALE, frame_id, detail)
            p = frame.p + frame.r.apply(p)
            r = frame.r * r
            current = frame.parent
        return p, r

    def to_base(self, p, r: Optional[R] = None, *, frame_id: str = BASE):
        """Re-express a pose given in ``frame_id`` in :data:`BASE`.

        With ``r=None`` only the position is returned, so a producer that has
        a point and no orientation does not have to invent one.
        """
        # A copy, not a view: producers freeze their arrays (``setflags(write=False)``)
        # and recent scipy refuses read-only input in ``Rotation.apply``.
        p = np.array(p, dtype=float, copy=True).reshape(3)
        if frame_id == BASE:
            return (p, r) if r is not None else p
        fp, fr = self.pose_in_base(frame_id)
        p_base = fp + fr.apply(p)
        return (p_base, fr * r) if r is not None else p_base
