"""The loop's ROBOT: an executor, a world source, and the kinematic model.

``LiveRobot`` is what an agent loop drives — on a real D1, in Isaac, or in the
kinematic mirror, with the SAME loop, policy, prompt and trace. Only two
things differ between them, and they are this module's two seams:

``executor``      how a checked plan is played (the kit's ``Executor``
                  protocol): ``FirmwareExecutor`` on hardware,
                  ``KinematicExecutor`` in the mirror, ``IsaacExecutor`` (in
                  d1-isaaclab) in sim
``source``        what produces the :class:`~manipulation_kit.world.WorldView`
                  each turn (:class:`WorldSource`): on hardware the declared
                  scene plus the executor's measured robot half
                  (:class:`SceneSource`); in sim the ground-truth objects RPC

Which pair is built is a FLAG, resolved through a registry::

    robot = LiveRobot.from_flag("firmware", url="http://d1-2:4750",
                                policy=policy, scene=scene)
    with robot:                      # lease / heartbeat / release, Ctrl-C too
        ...

``firmware`` and ``kinematic`` are built in. Anything else registers itself —
:func:`register_executor` in-process, or an entry point in the
``manipulation_kit.executors`` group from an installed package (this is how
d1-isaaclab provides ``isaac`` without the kit importing it) — or is named
directly with ``executor_class="module:factory"``.

Moved here from ``examples/agent/live.py`` and ``mirror.py`` (design C.8).
What changed on the way, and why:

* the held object RIDES THE TOOL (:mod:`manipulation_kit.world.attach`,
  review 11). ``live.py`` rebuilt the scene objects unchanged after every
  action, so a real lift measured zero rise; and it took the held object's
  NAME from what the loop asked for (``robot.expect``). Now the producer
  associates the object it can see between the pads when the gripper starts
  reporting holding, records the grasp transform at that instant, and
  publishes the attached pose — ``provenance="attached"`` — until the hand
  lets go, when the object stays where the hand had it, ``predicted``, until
  somebody looks again;
* ``LiveRobot`` is a context manager, so the example's ``ExitStack`` around
  the executor is gone.
"""

from __future__ import annotations

import dataclasses
import importlib
import json
from pathlib import Path
from typing import (Any, Callable, Dict, Iterable, List, Mapping, Optional,
                    Protocol, Sequence, Tuple, runtime_checkable)

import numpy as np
from scipy.spatial.transform import Rotation as R

from ..primitives.orientation import tool_from_link7
from ..world import (ArmView, ContactView, ContainerView, Frame, FrameGraph, GripperView,
                     ObjectView, SurfaceView, WorldView)
from ..world.attach import (GraspTransform, grasp_transform, with_attached)
from ..world.frames import BASE
from .policy import OperatorPolicy

SIDES: Tuple[str, ...] = ("left", "right")
KINDS = {"object": ObjectView, "container": ContainerView, "surface": SurfaceView}
DEFAULT_URL = "http://127.0.0.1:4750"

#: The entry-point group an out-of-tree package registers an executor under:
#:
#:     [project.entry-points."manipulation_kit.executors"]
#:     isaac = "agent_eval.kit_executor:isaac"
ENTRY_POINT_GROUP = "manipulation_kit.executors"

#: executors that exist out of tree, and where — so an unknown flag can say
#: what to install rather than only that it failed
KNOWN_ELSEWHERE: Dict[str, str] = {
    "isaac": ("d1-isaaclab (scripts/eval/agent_eval: IsaacExecutor + "
              "IsaacWorldSource). Install it into this environment so it "
              "registers the 'isaac' entry point, or pass "
              "--executor-class agent_eval.kit_executor:isaac with it on "
              "the path"),
}


# --------------------------------------------------------------------------- #
# the scene file — a measurement somebody made, read one way
# --------------------------------------------------------------------------- #

def load_scene(path: Path, *, profile: Any = None,
               allow_failed_gate: bool = False) -> Dict[str, Any]:
    """A scene file, with its ``robot`` block resolved against ``profile``
    (a :class:`~manipulation_kit.description.robot_profile.RobotProfile` or
    the path of the robot's ``omakase.camera_calibration/2`` file — a
    ``--robot-profile`` flag) or, without one, against the file the block
    names (``"robot": {"profile": "PATH"}``, relative to the scene file). The
    scene's own ``robot`` keys override the profile's. ``allow_failed_gate``
    accepts a calibration layer whose gate FAILED."""
    from ..description.robot_profile import with_profile  # noqa: PLC0415
    path = Path(path)
    scene = json.loads(path.read_text(encoding="utf-8"))
    return with_profile(scene, profile, relative_to=path.parent,
                        allow_failed_gate=allow_failed_gate)


def _profile_option(options: Dict[str, Any]) -> Any:
    """``profile=`` (and ``allow_failed_gate=``) out of a factory's options,
    resolved: a RobotProfile, or the path of a calibration file."""
    from ..description.robot_profile import RobotProfile  # noqa: PLC0415
    allow = bool(options.pop("allow_failed_gate", False))
    return RobotProfile.resolve(options.pop("profile", None),
                                allow_failed_gate=allow)


def _robot_block(scene: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    """``scene["robot"]`` with any profile it names folded in."""
    from ..description.robot_profile import scene_robot_block  # noqa: PLC0415
    return scene_robot_block(scene)


def objects_from(scene: Dict[str, Any]) -> List[ObjectView]:
    """Build the objects of a scene file. Units: metres, radians.

    Their ``provenance`` is ``declared`` unless the file says otherwise:
    somebody measured them with a tape, or a model said where they are — a
    statement, not a sight.
    """
    out: List[ObjectView] = []
    for item in scene.get("objects", ()):
        kind = KINDS[item.get("kind", "object")]
        extra: Dict[str, Any] = {}
        if kind is ContainerView and "interior" in item:
            extra["interior"] = item["interior"]
        if kind is ContainerView and "rim_height_m" in item:
            extra["rim_height_m"] = item["rim_height_m"]
        if kind is ContainerView and "interior_measured" in item:
            # A perceived interior is a number AND a guess; ``Place`` refuses
            # to drop into a guessed interior, which it cannot do if the file
            # cannot say so.
            extra["interior_measured"] = bool(item["interior_measured"])
        if kind is SurfaceView:
            # How the top's HEIGHT is known travels into the world, not just
            # the file (Astra review 8).
            for key in ("plane_source", "height_uncertainty_m"):
                if item.get(key) is not None:
                    extra[key] = item[key]
        out.append(kind(
            item["name"], p=item["p"], size=item["size"],
            r=R.from_quat(item["quat_xyzw"]) if "quat_xyzw" in item
            else R.from_euler("z", float(item.get("yaw_rad", 0.0))),
            frame_id=item.get("frame_id", "base"),
            colour=item.get("colour"), stamp=float(item.get("stamp", 0.0)),
            # A perceived or model-declared object arrives with a confidence
            # below 1 and it has to SURVIVE the file.
            confidence=float(item.get("confidence", 1.0)),
            provenance=item.get("provenance", "declared"),
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


def hand_open_gap_m(scene: Optional[Dict[str, Any]]) -> Optional[float]:
    """The robot's measured driven-open gripper gap from a scene file [m].

    A scene is a measurement of one robot at one table, so it can carry that
    robot's hand too: ``"robot": {"hand": {"open_gap_m": 0.0605}}``. A
    FALLBACK for a transport that cannot report the hand (the kinematic
    mirror); on hardware the executor publishes the daemon's own ``open_rad``
    as ``HandState.open_gap_m`` and that wins. ``None`` = nominal.
    """
    value = (_robot_block(scene).get("hand") or {}).get("open_gap_m")
    if value is None:
        return None
    gap = float(value)
    if not (0.0 < gap < 0.2):
        raise ValueError(f"scene robot.hand.open_gap_m must be a gap in metres, "
                         f"got {value!r}")
    return gap


def with_declared_hand(world: WorldView,
                       scene: Optional[Dict[str, Any]]) -> WorldView:
    """``world`` with the scene's measured hand on every gripper that has no
    measured opening of its own (see :func:`hand_open_gap_m`)."""
    gap = hand_open_gap_m(scene)
    if gap is None:
        return world
    return world.with_(grippers={
        side: (g if g.open_gap_m is not None
               else dataclasses.replace(g, open_gap_m=gap))
        for side, g in world.grippers.items()})


WRIST_INTRINSICS = ("fx", "fy", "cx", "cy", "width", "height")
#: the lens-model keys a wrist block may add (``WristIntrinsics``)
WRIST_LENS_KEYS = ("model", "k", "valid_radius_px")


def _wrist_kwargs(block: Mapping[str, Any]) -> Dict[str, Any]:
    missing = [k for k in WRIST_INTRINSICS if k not in block]
    if missing:
        raise ValueError(f"scene robot.wrist_camera needs {list(WRIST_INTRINSICS)}"
                         f"; missing {missing}")
    out: Dict[str, Any] = {k: (int(block[k]) if k in ("width", "height")
                               else float(block[k])) for k in WRIST_INTRINSICS}
    out["model"] = str(block.get("model", "pinhole"))
    if out["model"] == "fisheye":
        out["k"] = tuple(float(c) for c in block["k"])
    if block.get("valid_radius_px") is not None:
        out["valid_radius_px"] = float(block["valid_radius_px"])
    return out


def per_side_wrist(intrinsics: Optional[Mapping[str, Any]]
                   ) -> Dict[str, Dict[str, Any]]:
    """Wrist intrinsics as ``{side: kwargs}``: a per-side mapping as it is,
    a single flat block (one lens model for both hands) copied to each."""
    if not intrinsics:
        return {}
    if "fx" in intrinsics:
        return {side: dict(intrinsics) for side in SIDES}
    return {side: dict(block) for side, block in intrinsics.items()
            if side in SIDES and block}


def wrist_camera_from_scene(scene: Optional[Dict[str, Any]], *,
                            measured_only: bool = False
                            ) -> Optional[Dict[str, Dict[str, Any]]]:
    """The wrist lens intrinsics a scene (or the robot profile it names)
    records, ``robot.wrist_camera``, as ``{side: kwargs}`` for
    :meth:`manipulation_kit.perception.WristCamera.from_flange`.

    The block is either per side (``{"left": {...}, "right": {...}}`` — what
    a :class:`~manipulation_kit.description.robot_profile.RobotProfile`
    carries, fisheye ``model``/``k``/``valid_radius_px`` included) or one flat
    block for both hands. The mount is the kit's
    (:mod:`manipulation_kit.perception.wrist`); the focal length is the
    stream's and is NOT defaulted anywhere — without it there is no wrist
    camera model, and a policy that requires a look before a stroke refuses
    to start rather than grasping blind.

    A block marked ``"measured": false`` is a PLACEHOLDER. It lets the
    kinematic mirror dry-run the look policy; with ``measured_only``
    (hardware) it is no camera model at all, so a live run stops with
    ``look_unavailable`` instead of projecting through invented numbers.
    """
    block = _robot_block(scene).get("wrist_camera")
    if not block:
        return None
    out: Dict[str, Dict[str, Any]] = {}
    for side, one in per_side_wrist(block).items():
        if measured_only and one.get("measured", True) is False:
            continue
        out[side] = _wrist_kwargs(one)
    return out or None


def head_camera_from_scene(scene: Optional[Dict[str, Any]]):
    """The head-camera model a perceived scene recorded, or ``None``.

    The camera travels IN the scene file (``_perceive.camera``) rather than as
    a second set of flags, so the pose ``locate`` projects through is provably
    the one the frame was perceived with.
    """
    block = (scene or {}).get("_perceive", {}).get("camera")
    if not block:
        return None
    from ..perception import HeadCamera  # noqa: PLC0415
    return HeadCamera.from_json(block)


# --------------------------------------------------------------------------- #
# the world source
# --------------------------------------------------------------------------- #

@runtime_checkable
class WorldSource(Protocol):
    """What produces the :class:`WorldView` each turn.

    One method. A value per call, never a live handle: the verifiers compare
    the world before with the world after. Optional: ``declare(objects)`` (a
    source that takes a model's or a person's statement of where things are),
    ``close()``.
    """

    def observe(self) -> WorldView: ...


def associate(world: WorldView, side: str, *,
              tol_m: Optional[float] = None) -> Optional[str]:
    """The object between ``side``'s pads, by where the objects are: the
    nearest non-surface object to the tool point within ``tol_m``
    (the verifiers' ``ASSOCIATION_TOL_M``). ``None`` when nothing is that close
    or two things are equally close (within 5 mm) — an identity nobody can
    tell is left unknown, and the verifiers then say UNKNOWN."""
    from ..primitives.verifiers import ASSOCIATION_TOL_M  # noqa: PLC0415
    tol = ASSOCIATION_TOL_M if tol_m is None else float(tol_m)
    arm = world.arm(side)
    if arm is None or arm.tool_p is None:
        return None
    ranked = []
    for item in world.objects:
        if isinstance(item, SurfaceView):
            continue
        try:
            p = item.pose_in_base(world.frames)[0]
        except LookupError:
            continue
        ranked.append((float(np.linalg.norm(np.asarray(p) - arm.tool_p)),
                       item.name))
    ranked.sort()
    if not ranked or ranked[0][0] > tol:
        return None
    if len(ranked) > 1 and ranked[1][0] - ranked[0][0] < 0.005:
        return None
    return ranked[0][1]


class SceneSource:
    """Declared things + the executor's MEASURED robot half -> ``WorldView``.

    The half the kit does not own is the observation: nothing in the kit opens
    a camera. The things come from a scene file somebody measured or from
    what a model declared (:meth:`declare`); the arms, tool poses and
    grippers come from the executor's typed ``RawState``.

    ATTACHMENT. When a gripper starts reporting ``holding``, the object between
    its pads (:func:`associate`, or ``identity(side)`` when the transport
    knows) is attached with the grasp transform of THAT observation; while it
    holds, the object's pose is the tool pose composed with it
    (``provenance="attached"``); when it lets go, the object stays where the
    hand had it — ``released_provenance``, ``"predicted"`` for a real robot
    (nobody has looked), ``"observed"`` for a simulator whose world is the
    truth. Every kit verb that opens the jaws ends its plan at the release
    posture, so the hand's pose at the first open observation IS where it let
    go.
    """

    def __init__(self, executor: Any, kin: Any,
                 objects: Iterable[ObjectView] = (), *,
                 frames: Optional[Sequence[Dict[str, Any]]] = None,
                 frame_graph: Optional[FrameGraph] = None,
                 open_gap_m: Optional[float] = None,
                 released_provenance: str = "predicted",
                 identity: Optional[Callable[[str], Optional[str]]] = None):
        self.executor = executor
        self.kin = kin
        self.objects: Dict[str, ObjectView] = {o.name: o for o in objects}
        self._frames_spec = list(frames or ())
        self._frame_graph = frame_graph
        self.open_gap_m = open_gap_m
        self.released_provenance = released_provenance
        self.identity = identity
        self.grasps: Dict[str, GraspTransform] = {}
        #: contacts MEASURED by probe/press runs, kept across turns until the
        #: scene is restated (:meth:`forget_contacts`, ``declare_scene``)
        self.contacts: Tuple[ContactView, ...] = ()
        self.revision = 0
        self._last: Optional[WorldView] = None

    @classmethod
    def from_scene(cls, executor: Any, kin: Any,
                   scene: Optional[Dict[str, Any]] = None,
                   **kwargs: Any) -> "SceneSource":
        scene = scene or {}
        return cls(executor, kin, objects_from(scene),
                   frames=scene.get("frames", ()),
                   open_gap_m=hand_open_gap_m(scene), **kwargs)

    # -- what it is told ---------------------------------------------------- #
    def declare(self, objects: Iterable[ObjectView]) -> None:
        """Replace or add objects, by name — a person's or a model's statement
        of where things are. A statement about the object a hand is HOLDING
        re-anchors that grasp at the current tool pose: the newest sight of the
        thing wins over the transform recorded at the stroke."""
        for view in objects:
            self.objects[view.name] = view
            for side, grasp in list(self.grasps.items()):
                if grasp.name != view.name or self._last is None:
                    continue
                world = self._last.with_(objects=tuple(
                    view if o.name == view.name else o
                    for o in self._last.objects))
                try:
                    self.grasps[side] = grasp_transform(world, side=side,
                                                        name=view.name)
                except LookupError:
                    pass

    def remember_contacts(self, contacts: Iterable[ContactView]) -> None:
        """Keep the contacts a run measured (the WHOLE history the loop folded,
        not only the new ones): they are measurements, and every later
        observation carries them in ``WorldView.contacts``."""
        self.contacts = tuple(contacts)

    def forget_contacts(self) -> None:
        """The scene was restated (``declare_scene``): its contacts go with
        it. A surface they fitted stays, as the SurfaceView with
        ``plane_source="contact"`` it was published as."""
        self.contacts = ()

    # -- what it measures --------------------------------------------------- #
    def _frames(self, now: float) -> FrameGraph:
        if self._frame_graph is not None:
            return self._frame_graph.copy(now=now)
        return frames_from({"frames": self._frames_spec}, now=now)

    def _robot_half(self, state) -> Tuple[List[ArmView], List[GripperView]]:
        arms, grippers = [], []
        for side in SIDES:
            arm = state.arms.get(side)
            if arm is None:
                continue
            q = arm.q
            saved = np.array(self.kin.joints(side), dtype=float)
            try:
                self.kin.set_joints(side, q)
                p, r = tool_from_link7(*self.kin.ee_pose(side))
            finally:
                self.kin.set_joints(side, saved)
            # The controller's own mode and error code, typed. A latched arm
            # stops the run in the kit (``controller_fault``).
            arms.append(ArmView(side, joints=q, tool_p=p, tool_r=r,
                                mode=arm.mode, error_code=arm.error_code,
                                stationary=arm.stationary))
            hand = state.hands.get(side)
            if hand is None or hand.closedness is None:
                continue        # UNKNOWN, not "open"
            grippers.append(GripperView(
                side, hand.closedness, holding=bool(hand.holding),
                jaw_gap_m=hand.jaw_gap_m, jaw_stalled=hand.stalled,
                open_gap_m=(hand.open_gap_m if hand.open_gap_m is not None
                            else self.open_gap_m)))
        return arms, grippers

    def observe(self) -> WorldView:
        state = self.executor.state()
        self.revision += 1
        arms, grippers = self._robot_half(state)
        world = WorldView.of(
            list(self.objects.values()), frames=self._frames(state.stamp),
            arms=arms, grippers=grippers, stamp=state.stamp,
            revision=self.revision,
            firmware_spec=getattr(self.executor, "firmware_spec", None) or "")
        if self.contacts:
            world = world.with_(contacts=self.contacts)
        # LET GO: the object stays where the hand has it now.
        let_go: Dict[str, str] = {}
        for side in list(self.grasps):
            gripper = world.gripper(side)
            if gripper is not None and gripper.holding:
                continue
            grasp = self.grasps.pop(side)
            let_go[grasp.name] = side
            try:
                at = with_attached(world, side=side, name=grasp.name,
                                   grasp=grasp).find(grasp.name)
            except LookupError:
                continue
            self.objects[grasp.name] = dataclasses.replace(
                at, provenance=self.released_provenance)
            world = world.with_(objects=tuple(
                self.objects[grasp.name] if o.name == grasp.name else o
                for o in world.objects))
        # TOOK HOLD: record the grasp at this observation.
        for side in SIDES:
            gripper = world.gripper(side)
            if gripper is None or not gripper.holding or side in self.grasps:
                continue
            name = (self.identity(side) if self.identity is not None
                    else associate(world, side))
            passed = [n for n, giver in let_go.items() if giver != side]
            if (name is None or name in let_go) and len(passed) == 1:
                # HAND TO HAND, in one observation (``handover``): the other
                # hand let go of it and this one took hold. Where the giver
                # let go is not where the giver IS — it backed out after
                # opening — so the object is put at THIS hand's pad centre,
                # which is where the receiving grasp planned it (a horizontal
                # pad grasp centres the pads on the object), orientation as
                # released. An inference, published as ``attached``.
                name = passed[0]
                world = self._transferred(world, side, name)
            if name is None or world.find(name) is None:
                continue
            try:
                self.grasps[side] = grasp_transform(world, side=side, name=name)
            except LookupError:
                continue
        for side, grasp in self.grasps.items():
            try:
                world = with_attached(world, side=side, name=grasp.name,
                                      grasp=grasp)
            except LookupError:
                continue
        self._last = world
        return world

    def _transferred(self, world: WorldView, side: str, name: str
                     ) -> WorldView:
        item = world.find(name)
        arm = world.arm(side)
        if item is None or arm is None or arm.tool_p is None:
            return world
        try:
            _p, r = item.pose_in_base(world.frames)
        except LookupError:
            return world
        moved = dataclasses.replace(item, p=np.asarray(arm.tool_p, dtype=float),
                                    r=r, frame_id=BASE, provenance="attached")
        self.objects[name] = moved
        return world.with_(objects=tuple(moved if o.name == name else o
                                         for o in world.objects))

    def nearest(self, side: str) -> Optional[str]:
        """:func:`associate` against the current objects, for a transport that
        has to decide at the stroke what its jaws closed on (the mirror)."""
        world = self._last if self._last is not None else self.observe()
        tool = getattr(self.executor, "tool_pose", None)
        if tool is not None:
            p, r = tool(side)
            arms = dict(world.arms)
            was = arms.get(side)
            if was is not None:
                arms[side] = dataclasses.replace(was, tool_p=p, tool_r=r)
                world = world.with_(arms=arms)
        return associate(world, side)


# --------------------------------------------------------------------------- #
# the robot
# --------------------------------------------------------------------------- #

class LiveRobot:
    """An executor, a world source and a kinematic model — what a loop drives.

    A CONTEXT MANAGER: entering it enters the executor (lease, position mode,
    heartbeat, for the firmware one) and leaving it releases the executor and
    closes whatever the factory handed over (a socket client), on every exit
    including Ctrl-C.
    """

    def __init__(self, executor: Any, source: WorldSource, kin: Any, *,
                 head_camera: Any = None,
                 wrist_intrinsics: Optional[Mapping[str, float]] = None,
                 perceiver: Any = None, name: str = "",
                 closing: Sequence[Any] = ()):
        self.executor = executor
        self.source = source
        self.kin = kin
        self.head_camera = head_camera
        #: side -> WristCamera.from_flange kwargs (a flat block = both hands)
        self.wrist_intrinsics: Dict[str, Dict[str, Any]] = per_side_wrist(
            wrist_intrinsics)
        self.perceiver = perceiver
        #: the RobotProfile this robot was built with (``from_flag(profile=)``)
        self.profile: Any = None
        self.name = name
        self._closing = tuple(closing)
        self._entered: List[Any] = []

    # -- the loop's view of it --------------------------------------------- #
    def world(self) -> WorldView:
        return self.source.observe()

    @property
    def can_declare(self) -> bool:
        return callable(getattr(self.source, "declare", None))

    def declare(self, objects: Sequence[ObjectView]) -> None:
        if not self.can_declare:
            raise TypeError(f"this robot's world source "
                            f"({type(self.source).__name__}) does not take a "
                            f"declared scene")
        self.source.declare(objects)

    def remember_contacts(self, contacts: Sequence[ContactView]) -> None:
        """Hand measured contacts to the world source, when it keeps them."""
        remember = getattr(self.source, "remember_contacts", None)
        if callable(remember):
            remember(contacts)

    def forget_contacts(self) -> None:
        forget = getattr(self.source, "forget_contacts", None)
        if callable(forget):
            forget()

    def has_wrist_camera(self) -> bool:
        return bool(self.wrist_intrinsics)

    def cameras(self, world: Optional[WorldView] = None) -> Dict[str, Any]:
        """The camera MODELS available right now, by name: ``head`` when the
        scene carries one, ``left_wrist`` / ``right_wrist`` at the arms'
        measured joints when the wrist intrinsics are known."""
        out: Dict[str, Any] = {}
        if self.head_camera is not None:
            out["head"] = self.head_camera
        if self.wrist_intrinsics:
            from ..perception import WristCamera  # noqa: PLC0415
            world = world if world is not None else self.world()
            for side in SIDES:
                arm = world.arm(side)
                if arm is None or side not in self.wrist_intrinsics:
                    continue
                saved = np.array(self.kin.joints(side), dtype=float)
                try:
                    self.kin.set_joints(side, arm.joints)
                    out[f"{side}_wrist"] = WristCamera.from_kin(
                        self.kin, side, **self.wrist_intrinsics[side])
                finally:
                    self.kin.set_joints(side, saved)
        return out

    # -- lifecycle ---------------------------------------------------------- #
    def __enter__(self) -> "LiveRobot":
        enter = getattr(self.executor, "__enter__", None)
        if enter is not None:
            enter()
            self._entered.append(self.executor)
        return self

    def __exit__(self, *exc: Any) -> None:
        try:
            for item in reversed(self._entered):
                item.__exit__(*exc)
        finally:
            self._entered.clear()
            for item in (self.source,) + self._closing:
                close = getattr(item, "close", None)
                if callable(close):
                    close()

    # -- construction -------------------------------------------------------- #
    @classmethod
    def from_flag(cls, name: str, *, kin: Any = None,
                  policy: Optional[OperatorPolicy] = None,
                  scene: Optional[Dict[str, Any]] = None,
                  url: Optional[str] = None,
                  executor_class: Optional[str] = None,
                  **options: Any) -> "LiveRobot":
        """The robot an ``--executor NAME`` flag means.

        ``name`` is looked up in the registry (built-ins, then
        :func:`register_executor`, then the ``manipulation_kit.executors``
        entry points); ``executor_class="module:attr"`` names a factory — or an
        ``Executor`` class, wrapped with a :class:`SceneSource` — directly.
        The factory gets ``kin``, ``policy`` (the RESOLVED one: its
        ``vel_ratio`` and timeouts configure the executor), ``scene``,
        ``url`` and any ``options``. ``profile=`` (a
        :class:`~manipulation_kit.description.robot_profile.RobotProfile`, or
        the path of the robot's calibration file) is
        folded into the scene's ``robot`` block first, so the hand gap and
        the MEASURED wrist lenses reach every executor the same way. Raises :class:`UnknownExecutor` with the
        registered names and what to install.
        """
        factory = (_load_attr(executor_class) if executor_class
                   else executor_factory(name))
        profile = _profile_option(options)
        if profile is not None:
            # the robot's measured numbers, under whatever the scene restates
            from ..description.robot_profile import with_profile  # noqa: PLC0415
            scene = with_profile(scene, profile)
        kin = kin if kin is not None else _default_kin()
        policy = policy if policy is not None else OperatorPolicy()
        built = factory(kin=kin, policy=policy, scene=scene, url=url,
                        **options)
        if not isinstance(built, LiveRobot):
            # an Executor, not a robot: give it the declared scene as its world
            built = cls(built, SceneSource.from_scene(built, kin, scene), kin,
                        head_camera=head_camera_from_scene(scene),
                        wrist_intrinsics=wrist_camera_from_scene(scene),
                        name=name or str(executor_class))
        built.profile = profile
        return built

    @classmethod
    def firmware(cls, url: str = DEFAULT_URL, *, kin: Any = None,
                 policy: Optional[OperatorPolicy] = None,
                 scene: Optional[Dict[str, Any]] = None,
                 perceiver: Any = None, **options: Any) -> "LiveRobot":
        """A real D1 through d1-firmwared, configured by ``policy`` (its
        ``vel_ratio`` for velocity AND acceleration, its arrival timeout, its
        stroke timeout or — ``None`` — the daemon document's)."""
        profile = _profile_option(options)
        if profile is not None:
            from ..description.robot_profile import with_profile  # noqa: PLC0415
            scene = with_profile(scene, profile)
        robot = _firmware(kin=kin if kin is not None else _default_kin(),
                          policy=policy or OperatorPolicy(), scene=scene,
                          url=url, **options)
        robot.perceiver = perceiver
        robot.profile = profile
        return robot


class KinematicMirror(LiveRobot):
    """The offline stand-in: a ``KinematicExecutor`` and the scene it moves.

    It mirrors — it does not simulate. There is no dynamics, no contact and no
    settle: the right fidelity for testing that a PLAN is well formed and that
    the loop around it behaves, and none at all for whether a grasp holds.

    The jaws close on whatever :func:`associate` finds between the pads at the
    stroke (nothing, if the plan closed on air), that object rides the tool by
    the one attachment rule, and a released object is ``observed`` where the
    hand let go — the mirror's world IS the truth, as a simulator's is.
    """

    def __init__(self, kin: Any, world0: Optional[WorldView] = None, *,
                 scene: Optional[Dict[str, Any]] = None,
                 open_gap_m: Optional[float] = None,
                 wrist_intrinsics: Optional[Mapping[str, float]] = None,
                 head_camera: Any = None):
        from ..executor import KinematicExecutor  # noqa: PLC0415
        if open_gap_m is None:
            open_gap_m = hand_open_gap_m(scene)
        executor = KinematicExecutor(kin, open_gap_m=open_gap_m)
        if world0 is not None and scene is None:
            objects, frame_graph, frames = world0.objects, world0.frames, None
        else:
            objects = objects_from(scene or {})
            frame_graph, frames = None, (scene or {}).get("frames", ())
        source = SceneSource(executor, kin, objects, frames=frames,
                             frame_graph=frame_graph, open_gap_m=open_gap_m,
                             released_provenance="observed",
                             identity=lambda side: executor.held.get(side))
        inner = executor.set_gripper

        def set_gripper(side: str, closedness: float, *, grip: str) -> None:
            # The mirror's "contact": what the jaws close on is what is
            # between them, decided at the stroke.
            if closedness >= executor.hold_at and executor.held.get(side) is None:
                executor.next_object[side] = source.nearest(side)
            inner(side, closedness, grip=grip)

        executor.set_gripper = set_gripper      # type: ignore[assignment]
        super().__init__(executor, source, kin,
                         head_camera=(head_camera if head_camera is not None
                                      else head_camera_from_scene(scene)),
                         wrist_intrinsics=(wrist_intrinsics
                                           if wrist_intrinsics is not None
                                           else wrist_camera_from_scene(scene)),
                         name="kinematic")


# --------------------------------------------------------------------------- #
# the executor registry
# --------------------------------------------------------------------------- #

ExecutorFactory = Callable[..., Any]
_REGISTRY: Dict[str, ExecutorFactory] = {}


class UnknownExecutor(LookupError):
    """An ``--executor`` flag that names nothing this environment can build."""


def register_executor(name: str, factory: ExecutorFactory, *,
                      replace: bool = False) -> None:
    """Make ``--executor NAME`` build ``factory(kin=, policy=, scene=, url=,
    **options)`` — a :class:`LiveRobot`, or an ``Executor`` the kit wraps with
    a :class:`SceneSource`."""
    if not name or not isinstance(name, str):
        raise ValueError("an executor needs a name")
    if name in _REGISTRY and not replace and _REGISTRY[name] is not factory:
        raise ValueError(f"an executor called {name!r} is already registered; "
                         f"pass replace=True to replace it")
    _REGISTRY[name] = factory


def unregister_executor(name: str) -> None:
    _REGISTRY.pop(name, None)


def _entry_points() -> Dict[str, Any]:
    try:
        from importlib import metadata  # noqa: PLC0415
    except ImportError:                 # pragma: no cover - py<3.8
        return {}
    found = metadata.entry_points()
    if hasattr(found, "select"):
        group = found.select(group=ENTRY_POINT_GROUP)
    else:                               # python 3.9: a dict of groups
        group = found.get(ENTRY_POINT_GROUP, ())
    return {ep.name: ep for ep in group}


def registered_executors() -> List[str]:
    """Every name ``--executor`` accepts here: built in, registered, and
    advertised by an installed package's entry point."""
    return sorted(set(_REGISTRY) | set(_entry_points()))


def executor_factory(name: str) -> ExecutorFactory:
    if name in _REGISTRY:
        return _REGISTRY[name]
    points = _entry_points()
    if name in points:
        try:
            factory = points[name].load()
        except Exception as exc:  # noqa: BLE001 - say which plugin broke
            raise UnknownExecutor(
                f"--executor {name}: the entry point {points[name].value!r} "
                f"(group {ENTRY_POINT_GROUP!r}) is installed but failed to "
                f"load: {exc!r}") from exc
        register_executor(name, factory)
        return factory
    hint = KNOWN_ELSEWHERE.get(name)
    raise UnknownExecutor(
        f"--executor {name}: no such executor in this environment. "
        f"Available: {', '.join(registered_executors()) or 'none'}."
        + (f" {name!r} is provided by {hint}." if hint else
           f" An installed package registers one under the "
           f"{ENTRY_POINT_GROUP!r} entry-point group, or name it directly "
           f"with --executor-class module:factory."))


def _load_attr(spec: str) -> ExecutorFactory:
    module, _, attr = str(spec).partition(":")
    if not module or not attr:
        raise UnknownExecutor(f"--executor-class wants 'module:attr', got "
                              f"{spec!r}")
    try:
        target = importlib.import_module(module)
        for part in attr.split("."):
            target = getattr(target, part)
    except (ImportError, AttributeError) as exc:
        raise UnknownExecutor(f"--executor-class {spec}: {exc}") from exc
    return target


def _default_kin():
    from ..arms import get_arm_kinematics  # noqa: PLC0415
    return get_arm_kinematics("d1/arm", quiet=True)


def _firmware(*, kin: Any, policy: OperatorPolicy,
              scene: Optional[Dict[str, Any]] = None,
              url: Optional[str] = None, **options: Any) -> LiveRobot:
    from ..executors.firmware import FirmwareExecutor  # noqa: PLC0415
    executor = FirmwareExecutor(base_url=url or DEFAULT_URL,
                                vel_ratio=policy.vel_ratio,
                                acc_ratio=policy.vel_ratio,
                                arrive_timeout_s=policy.arrive_timeout_s,
                                stroke_timeout_s=policy.stroke_timeout_s,
                                **options)
    return LiveRobot(executor, SceneSource.from_scene(executor, kin, scene),
                     kin, head_camera=head_camera_from_scene(scene),
                     wrist_intrinsics=wrist_camera_from_scene(
                         scene, measured_only=True),
                     name="firmware")


def _kinematic(*, kin: Any, policy: OperatorPolicy,
               scene: Optional[Dict[str, Any]] = None,
               url: Optional[str] = None,
               world0: Optional[WorldView] = None, **options: Any
               ) -> KinematicMirror:
    return KinematicMirror(kin, world0, scene=scene, **options)


register_executor("firmware", _firmware)
register_executor("kinematic", _kinematic)


__all__ = ["DEFAULT_URL", "ENTRY_POINT_GROUP", "ExecutorFactory",
           "KNOWN_ELSEWHERE", "KinematicMirror", "LiveRobot", "SceneSource",
           "UnknownExecutor", "WorldSource", "associate", "executor_factory",
           "frames_from", "hand_open_gap_m", "head_camera_from_scene",
           "load_scene", "objects_from", "per_side_wrist", "register_executor",
           "registered_executors", "unregister_executor", "with_declared_hand",
           "wrist_camera_from_scene"]
