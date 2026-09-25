"""The scene as an obstacle set: arm links against what the world publishes.

The motion guard (:mod:`manipulation_kit.guard`) keeps the arms off the robot's
own body and off each other. It is stdlib-only on purpose and it never sees a
:class:`~manipulation_kit.world.WorldView`, so before 0.16.0 NOTHING kept an
arm off the table: run 5 latched the right forearm into the wagon top with
every check passing, and the example answered with a policy patch
(an approach-direction allow list) for a geometry gap. This module is the scene half of
what :class:`~manipulation_kit.arms.guard.GuardGate` is for the body.

WHAT IS CHECKED, exactly:

* **Links.** The arm STRUCTURE of the side being planned — every collision
  primitive of ``Base_*`` .. ``Link7_*`` in the bundled ``d1.urdf``, turned
  into a capsule by :func:`manipulation_kit.guard.urdf_model.prim_to_world`,
  the SAME conversion the body guard uses (so the scene sees the arm the
  guard sees, radii included). The tool side (``TCP_Link`` and the hand) is
  NOT checked, for the reason the guard gives: the hand is the working
  surface and must reach the thing it grasps and the table under it. The
  hand's clearance in TRANSIT is given by construction instead — see
  :func:`~manipulation_kit.primitives.planning._over_the_top`.
* **Obstacles.** Every :class:`~manipulation_kit.world.SurfaceView`,
  :class:`~manipulation_kit.world.ContainerView` and
  :class:`~manipulation_kit.world.ObjectView` in the world, as an oriented
  box, EXCEPT the ones a verb names (its target, its destination) and
  whatever a gripper is holding (:func:`obstacles_of`).
* **Margins, per obstacle.** A probed surface is known to a few millimetres,
  a model-declared box to its declared uncertainty; one global margin would be
  either too loose for the first or refuse everything near the second
  (:class:`ClearancePolicy`). A surface's HEIGHT uncertainty inflates its box
  along its own normal, not sideways: a single camera does not know how high
  the table is, it does know where its edges are in the image. Boxes keep
  their full pose rotation (a probed wall's normal is horizontal).
* **Contact legs.** A probe or a press is DRIVEN INTO the surface it
  measures — its leg ends up to ``max_travel_m`` past it by design. The gate
  for such a plan is built with :meth:`SceneGate.for_contact`, which leaves
  out the one obstacle the leg's ray meets first (the surface being measured)
  and keeps every other one.
* **Droop.** The real D1 arm sags about a centimetre at a long reach (F16).
  That used to be the ``MKIT_SUPPORT_CLEARANCE_M`` environment variable; it is
  now :attr:`ClearancePolicy.droop_margin_m`, a typed number the operator
  policy sets, and it is added both to every obstacle's required clearance and
  to the grasp's fingertip floor.

SAMPLING, stated rather than inherited. :func:`~manipulation_kit.guard.geometry
.seg_aabb_distance` samples a segment at 24 points whatever its length, which
on a 264 mm forearm capsule is 11 mm between samples. This gate samples by
DISTANCE instead: points along each capsule at most
:attr:`ClearancePolicy.segment_spacing_m` apart, and postures along a swept
interval such that no capsule endpoint moves more than
:attr:`ClearancePolicy.sweep_step_m` between two checked postures. Distance to
a convex box is 1-Lipschitz, so the true clearance can be below the sampled one
by at most half of each spacing; :attr:`SceneGate.sampling_allowance_m` is that
sum and it is ADDED to every required clearance, not left to the margin to
absorb by luck.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

from ..arms import sides as arm_sides
from ..world import ContainerView, FrameError, ObjectView, SurfaceView, WorldView

#: ``plane_source`` values that mean the surface was MEASURED by contact
#: (step 4's probe publishes one of these). Anything else — declared,
#: known-length, provisional, unstated — is a declaration.
PROBED_SOURCES: Tuple[str, ...] = ("probed", "contact")


@dataclass(frozen=True)
class ClearancePolicy:
    """Every number the scene gate uses, in one typed place.

    The defaults are the RIGID-model defaults, so a kinematic run plans what
    it always planned. ``droop_margin_m`` is the one an operator changes for
    a real arm: measured on d1-2 (2026-09-22 run 7, x 0.48 m) the pad tips met
    the wagon top with a 3 mm floor and the controller raised error 15; the
    robot ran at a 15 mm floor, i.e. ``droop_margin_m = 0.012``. Step 7's
    ``OperatorPolicy`` is where that number lives for a live run; here it is
    attached to the kinematics a plan is made with (:func:`policy_of`).
    """

    #: vertical sag of the real arm under its own weight, added to every
    #: obstacle clearance and to the top-down grasp floor [m]. 0 = rigid.
    droop_margin_m: float = 0.0
    #: clearance a link must keep from a surface measured by contact [m]
    probed_surface_margin_m: float = 0.005
    #: ... from a surface that was declared (or perceived) [m]
    declared_surface_margin_m: float = 0.010
    #: ... from a declared object / container that states no uncertainty [m]
    declared_object_margin_m: float = 0.010
    #: largest distance between two checked points on one capsule [m]
    segment_spacing_m: float = 0.004
    #: largest capsule-endpoint travel between two checked postures [m]
    sweep_step_m: float = 0.006

    def __post_init__(self) -> None:
        for name in ("droop_margin_m", "probed_surface_margin_m",
                     "declared_surface_margin_m", "declared_object_margin_m"):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"ClearancePolicy.{name} must be a finite "
                                 f"non-negative number of metres, got {value!r}")
            object.__setattr__(self, name, value)
        for name in ("segment_spacing_m", "sweep_step_m"):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"ClearancePolicy.{name} must be positive, "
                                 f"got {value!r}")
            object.__setattr__(self, name, value)

    @property
    def sampling_allowance_m(self) -> float:
        """How far the true clearance can sit below the sampled one."""
        return 0.5 * (self.segment_spacing_m + self.sweep_step_m)

    def to_json(self) -> Dict[str, float]:
        return {name: float(getattr(self, name)) for name in (
            "droop_margin_m", "probed_surface_margin_m",
            "declared_surface_margin_m", "declared_object_margin_m",
            "segment_spacing_m", "sweep_step_m")}


DEFAULT_POLICY = ClearancePolicy()

#: the attribute of an arm-kinematics object that carries its policy
POLICY_ATTR = "clearance_policy"


def policy_of(kin) -> ClearancePolicy:
    """The clearance policy the plan is made under: the one attached to the
    kinematics (``kin.clearance_policy``), else :data:`DEFAULT_POLICY`.

    The kinematics object is the model of THIS arm; droop is a property of
    this arm. Attaching the policy there is what lets every verb's
    ``plan(world, kin)`` see it without a new argument on thirty call sites.
    """
    policy = getattr(kin, POLICY_ATTR, None)
    if policy is None:
        inner = getattr(kin, "kin", None)     # a planning.Kin wrapper
        policy = getattr(inner, POLICY_ATTR, None) if inner is not None else None
    if policy is None:
        return DEFAULT_POLICY
    if not isinstance(policy, ClearancePolicy):
        raise TypeError(f"{POLICY_ATTR} must be a ClearancePolicy, got "
                        f"{type(policy).__name__}")
    return policy


def set_policy(kin, policy: ClearancePolicy) -> None:
    """Attach ``policy`` to an arm-kinematics object (see :func:`policy_of`)."""
    if not isinstance(policy, ClearancePolicy):
        raise TypeError("policy must be a ClearancePolicy")
    setattr(kin, POLICY_ATTR, policy)


# --------------------------------------------------------------------------- #
# obstacles
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class Obstacle:
    """One oriented box the arm links must keep ``margin_m`` away from.

    ``p``/``r`` are the box centre and orientation in the base frame,
    ``half`` its half-extents in its own axes (already inflated by any height
    uncertainty), ``margin_m`` the clearance required of a link, and
    ``basis`` a short phrase saying where that margin came from — it is
    printed in a refusal, so a model knows which declaration to correct.
    """

    name: str
    p: np.ndarray
    r: R
    half: np.ndarray
    margin_m: float
    kind: str = "object"
    basis: str = ""

    def __post_init__(self) -> None:
        object.__setattr__(self, "p", np.asarray(self.p, dtype=float).reshape(3))
        object.__setattr__(self, "half",
                           np.asarray(self.half, dtype=float).reshape(3))

    @property
    def top_z(self) -> float:
        """Highest point of the (inflated) box in the base frame."""
        m = self.r.as_matrix()
        return float(self.p[2] + np.abs(m[2]) @ self.half)

    def footprint_distance(self, xy) -> float:
        """Horizontal distance from ``xy`` to the box's footprint (0 inside)."""
        m = self.r.as_matrix()
        # project the box onto the floor plane: its xy half-extent along its
        # own two most horizontal axes is enough for a level or yawed box
        local = m.T @ np.array([xy[0] - self.p[0], xy[1] - self.p[1], 0.0])
        outside = np.maximum(np.abs(local[:2]) - self.half[:2], 0.0)
        return float(np.linalg.norm(outside))

    def to_json(self) -> Dict[str, object]:
        return {"name": self.name, "kind": self.kind,
                "p": [round(float(v), 4) for v in self.p],
                "half": [round(float(v), 4) for v in self.half],
                "margin_m": round(self.margin_m, 4), "basis": self.basis}


def _margin_for(item: ObjectView, policy: ClearancePolicy
                ) -> Tuple[float, float, str]:
    """``(margin, extra vertical half-extent, basis)`` for one world item."""
    declared = getattr(item, "uncertainty_m", None)
    if isinstance(item, SurfaceView):
        source = item.plane_source or "unstated"
        if declared is not None:
            margin, basis = float(declared), (f"{source} surface, declared "
                                              f"+-{float(declared) * 1000:.0f} mm")
        elif source in PROBED_SOURCES:
            margin, basis = policy.probed_surface_margin_m, f"{source} surface"
        else:
            margin, basis = (policy.declared_surface_margin_m,
                             f"{source} surface")
        dz = float(item.height_uncertainty_m or 0.0)
        if dz:
            basis += f", height +-{dz * 1000:.0f} mm"
        return margin, dz, basis
    if declared is not None:
        return float(declared), 0.0, f"declared +-{float(declared) * 1000:.0f} mm"
    return policy.declared_object_margin_m, 0.0, "declared, no stated uncertainty"


def held_names(world: WorldView) -> Tuple[str, ...]:
    """Every object a gripper reports holding — it rides the tool."""
    out = []
    for gripper in world.grippers.values():
        if gripper.held_object:
            out.append(gripper.held_object)
    return tuple(out)


def obstacles_of(world: WorldView, *, exclude: Sequence[str] = (),
                 policy: ClearancePolicy = DEFAULT_POLICY,
                 ) -> Tuple[Tuple[Obstacle, ...], Tuple[str, ...]]:
    """The world's obstacles, minus ``exclude`` and what the hands hold.

    Returns ``(obstacles, unresolved)``: an item whose frame cannot be
    resolved is not silently dropped — its name comes back in ``unresolved``
    so the plan can say it was not checked against it.
    """
    skip = set(exclude) | set(held_names(world))
    obstacles: List[Obstacle] = []
    unresolved: List[str] = []
    for item in world.objects:
        if item.name in skip:
            continue
        try:
            p, r = item.pose_in_base(world.frames)
        except FrameError:
            unresolved.append(item.name)
            continue
        margin, dz, basis = _margin_for(item, policy)
        half = np.asarray(item.size, dtype=float) / 2.0
        if dz:
            # the uncertainty is along the surface's OWN normal — its local
            # +z (``SurfaceView.from_plane`` puts the measured normal there):
            # the base vertical for a table, horizontal for a probed wall
            half = half + np.array([0.0, 0.0, dz])
        obstacles.append(Obstacle(item.name, p, r, half, margin,
                                  kind=item.kind, basis=basis))
    return tuple(obstacles), tuple(unresolved)


# --------------------------------------------------------------------------- #
# the arm's capsules, from the guard's own collision model
# --------------------------------------------------------------------------- #

class _ArmCapsules:
    """Forward kinematics of one arm's checked capsules, numpy for speed.

    The kinematic tree, the collision primitives and the primitive -> capsule
    rule are the guard's (:class:`~manipulation_kit.guard.urdf_model
    .UrdfModel`, :func:`~manipulation_kit.guard.urdf_model.prim_to_world`).
    ``prim_to_world`` is applied ONCE per primitive, in its own link's frame
    (identity transform), which gives the capsule in link coordinates; a
    capsule is affine in the link transform, so posing it is then one
    rotation and one translation. Only the chain product is redone in numpy,
    because the pure-python FK of the whole body costs ~1 ms and a swept
    check asks for it thousands of times a plan.
    ``tests/primitives/test_clearance.py`` pins the result to the guard's own
    ``MotionGuard._arm_capsules`` to 1e-9.
    """

    def __init__(self, model, side: str, spacing_m: float):
        from ..guard import geometry as g  # noqa: PLC0415
        from ..guard.guard import is_ee_body  # noqa: PLC0415
        from ..guard.urdf_model import prim_to_world  # noqa: PLC0415
        suffix = arm_sides.URDF_SUFFIX[side]
        joint_names = arm_sides.ARM_JOINTS[side]
        base = f"Base_{suffix}"
        tfs0 = model.link_transforms({})
        self._root = _mat(tfs0[model.parent_link[base]])
        chain = []
        link = base
        while True:
            chain.append(link)
            nxt = [j for j in model.children.get(link, [])
                   if not is_ee_body(j.child)]
            if not nxt:
                break
            link = nxt[0].child
        self.links = tuple(chain)
        self._joint = []
        for link in chain:
            j = model.joints[link]
            qi = joint_names.index(j.name) if j.name in joint_names else None
            axis = np.asarray(j.axis, dtype=float)
            axis = axis / max(float(np.linalg.norm(axis)), 1e-12)
            self._joint.append((_mat(j.origin), axis,
                                qi if j.type == "revolute" else None))
        # per link: capsule endpoints in LINK coordinates, sample points, radii
        self._local = []          # (link index, a, b, r)
        sample_pts, sample_r, sample_link = [], [], []
        for index, link in enumerate(chain):
            for prim in model.collisions.get(link, []):
                if "_cap_" in prim.name:      # end-sphere duplicates
                    continue
                kind, cap = prim_to_world(prim, link, g.Tf(), prefer_aabb=False)
                if kind != "capsule":      # pragma: no cover - model shape
                    raise ValueError(f"{link} collision {prim.name} is not a "
                                     f"capsule")
                a_l = np.asarray(cap.a, dtype=float)
                b_l = np.asarray(cap.b, dtype=float)
                self._local.append((index, a_l, b_l, float(cap.r)))
                n = max(1, int(math.ceil(float(np.linalg.norm(b_l - a_l))
                                         / spacing_m)))
                s = np.linspace(0.0, 1.0, n + 1)[:, None]
                sample_pts.append((index, a_l + (b_l - a_l) * s))
                sample_r.append(np.full(n + 1, float(cap.r)))
                sample_link += [link] * (n + 1)
        self._samples = sample_pts
        self.sample_radii = np.concatenate(sample_r)
        self.sample_links = tuple(sample_link)

    def link_frames(self, q) -> List[np.ndarray]:
        q = np.asarray(q, dtype=float).reshape(7)
        tf = self._root
        out = []
        for origin, axis, qi in self._joint:
            tf = tf @ origin
            if qi is not None:
                tf = tf @ _axis_angle(axis, float(q[qi]))
            out.append(tf)
        return out

    def capsules(self, q) -> List[Tuple[str, np.ndarray, np.ndarray, float]]:
        """``[(link, a, b, radius)]`` in the base frame at joints ``q`` [rad]."""
        frames = self.link_frames(q)
        return [(self.links[i], frames[i][:3, :3] @ a + frames[i][:3, 3],
                 frames[i][:3, :3] @ b + frames[i][:3, 3], r)
                for i, a, b, r in self._local]

    def endpoints(self, frames) -> np.ndarray:
        return np.array([[frames[i][:3, :3] @ a + frames[i][:3, 3],
                          frames[i][:3, :3] @ b + frames[i][:3, 3]]
                         for i, a, b, _r in self._local])

    def samples(self, frames) -> np.ndarray:
        """Every sample point of every capsule, base frame, (N, 3)."""
        return np.vstack([pts @ frames[i][:3, :3].T + frames[i][:3, 3]
                          for i, pts in self._samples])


def _mat(tf) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = np.asarray(tf.R, dtype=float)
    m[:3, 3] = np.asarray(tf.t, dtype=float)
    return m


def _axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    """Homogeneous rotation about a UNIT ``axis`` (Rodrigues, as the guard's
    ``geometry.rot_axis``)."""
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    C = 1.0 - c
    m = np.eye(4)
    m[:3, :3] = ((c + x * x * C, x * y * C - z * s, x * z * C + y * s),
                 (y * x * C + z * s, c + y * y * C, y * z * C - x * s),
                 (z * x * C - y * s, z * y * C + x * s, c + z * z * C))
    return m


_MODEL_CACHE: Dict[str, object] = {}


def _guard_model(kin):
    """The URDF model the body guard checks, or the bundled one."""
    guard = getattr(getattr(getattr(kin, "kin", kin), "gate", None), "guard", None)
    model = getattr(guard, "model", None)
    if model is not None and hasattr(model, "collisions"):
        return model
    if "default" not in _MODEL_CACHE:
        from ..guard.urdf_model import UrdfModel  # noqa: PLC0415
        _MODEL_CACHE["default"] = UrdfModel()
    return _MODEL_CACHE["default"]


# --------------------------------------------------------------------------- #
# the gate
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class ClearanceReport:
    """The closest approach of one arm's links to the scene.

    ``clearance_m`` is signed distance from the link capsule's SURFACE to the
    bare obstacle box (negative = inside it), ``required_m`` what this
    obstacle demands (its margin + droop + the sampling allowance), and
    ``depth_m = required_m - clearance_m`` how far the link is inside that
    envelope — the number a refusal carries as ``residual_m`` and the amount
    a declaration or a path has to change by.
    """

    ok: bool
    obstacle: str = ""
    link: str = ""
    clearance_m: float = math.inf
    required_m: float = 0.0
    fraction: float = 0.0          # where along a swept interval (0..1)
    postures_checked: int = 0
    basis: str = ""

    @property
    def depth_m(self) -> float:
        return float(self.required_m - self.clearance_m)

    def describe(self, side: str) -> str:
        if not self.obstacle:
            return "no obstacle in the scene"
        inside = ("inside the box itself" if self.clearance_m < 0.0
                  else f"{self.clearance_m * 1000:.0f} mm from it")
        return (f"the {side} arm's {self.link} would pass {inside}, "
                f"{self.depth_m * 1000:.0f} mm inside the "
                f"{self.required_m * 1000:.0f} mm clearance {self.obstacle!r} "
                f"requires ({self.basis})")


@dataclass
class SceneGate:
    """Arm links vs the scene: the check the body guard cannot make.

    Built per plan (:meth:`of`) from the world the plan is made in, with the
    verb's own target and destination excluded. ``pose_ok`` answers for one
    posture, ``swept_ok`` for the joint-space interval between two.
    """

    obstacles: Tuple[Obstacle, ...]
    policy: ClearancePolicy = DEFAULT_POLICY
    unresolved: Tuple[str, ...] = ()
    model: object = None
    #: for a contact plan (:meth:`for_contact`): the surface it measures
    contact_target: str = ""
    _arms: Dict[str, _ArmCapsules] = field(default_factory=dict, repr=False)

    @classmethod
    def for_contact(cls, world: WorldView, kin, p_start, direction,
                    travel_m: float, *, exclude: Sequence[str] = (),
                    policy: Optional[ClearancePolicy] = None) -> "SceneGate":
        """The gate for a plan whose last leg is a CONTACT leg.

        The leg starts with the tool point at ``p_start`` and travels up to
        ``travel_m`` along the base-frame ``direction``, deliberately past
        the surface it is measuring. That surface — the first obstacle the
        leg's ray meets, hand width and required clearance included — is left
        out (:attr:`contact_target` names it); every other obstacle still
        gates the standoff transit and the leg.
        """
        policy = policy if policy is not None else policy_of(kin)
        gate = cls.of(world, kin, exclude=exclude, policy=policy)
        hit = first_hit(gate, p_start, direction, travel_m)
        if hit:
            gate = cls.of(world, kin, exclude=tuple(exclude) + (hit,),
                          policy=policy)
        gate.contact_target = hit
        return gate

    @classmethod
    def of(cls, world: WorldView, kin=None, *, exclude: Sequence[str] = (),
           policy: Optional[ClearancePolicy] = None) -> "SceneGate":
        policy = policy if policy is not None else policy_of(kin)
        obstacles, unresolved = obstacles_of(world, exclude=exclude,
                                             policy=policy)
        return cls(obstacles, policy, unresolved, _guard_model(kin))

    # -- sampling, stated ----------------------------------------------- #
    @property
    def sampling_allowance_m(self) -> float:
        return self.policy.sampling_allowance_m

    def required_m(self, obstacle: Obstacle) -> float:
        """Clearance a link must keep from ``obstacle``, everything included."""
        return (obstacle.margin_m + self.policy.droop_margin_m
                + self.sampling_allowance_m)

    def describe_sampling(self) -> str:
        return (f"links sampled every {self.policy.segment_spacing_m * 1000:.0f}"
                f" mm, postures every {self.policy.sweep_step_m * 1000:.0f} mm "
                f"of link travel; the {self.sampling_allowance_m * 1000:.1f} mm "
                f"this can miss by is added to every required clearance")

    # -- geometry -------------------------------------------------------- #
    def _arm(self, side: str) -> _ArmCapsules:
        if side not in self._arms:
            if self.model is None:
                self.model = _guard_model(None)
            self._arms[side] = _ArmCapsules(self.model, side,
                                            self.policy.segment_spacing_m)
        return self._arms[side]

    def capsules(self, side: str, q):
        """``[(link, a, b, radius)]`` of the checked links at joints ``q``."""
        return self._arm(side).capsules(q)

    def _stacked(self):
        """The obstacles as arrays, built once: centres, rotations, halves,
        required clearances."""
        cache = getattr(self, "_stack", None)
        if cache is None:
            cache = (np.array([ob.p for ob in self.obstacles]),
                     np.array([ob.r.as_matrix() for ob in self.obstacles]),
                     np.array([ob.half for ob in self.obstacles]),
                     np.array([self.required_m(ob) for ob in self.obstacles]))
            object.__setattr__(self, "_stack", cache)
        return cache

    def _depths(self, arm: _ArmCapsules, frames):
        """Per obstacle: (depth into its envelope, clearance, worst sample)."""
        centres, rots, halves, required = self._stacked()
        pts = arm.samples(frames)
        # every sample in every obstacle's own axes: (n_obs, n_pts, 3)
        local = np.einsum("opk,okj->opj", pts[None, :, :] - centres[:, None, :],
                          rots)
        d = np.abs(local) - halves[:, None, :]
        outside = np.linalg.norm(np.maximum(d, 0.0), axis=2)
        inside = np.minimum(np.max(d, axis=2), 0.0)
        clearance = outside + inside - arm.sample_radii[None, :]
        worst = np.argmin(clearance, axis=1)
        closest = clearance[np.arange(len(self.obstacles)), worst]
        return required - closest, closest, worst

    def _report_at(self, arm: _ArmCapsules, depth, clearance, worst, index: int,
                   **extra) -> ClearanceReport:
        ob = self.obstacles[index]
        return ClearanceReport(bool(depth[index] <= 0.0), ob.name,
                               arm.sample_links[int(worst[index])],
                               float(clearance[index]),
                               self.required_m(ob), basis=ob.basis, **extra)

    # -- the two questions ----------------------------------------------- #
    def pose_ok(self, kin, side: str, q) -> ClearanceReport:
        """Does the ``side`` arm at joints ``q`` keep clear of the scene?

        The report names the obstacle the arm is deepest into (or, when it
        clears everything, the one it comes closest to).
        """
        if not self.obstacles:
            return ClearanceReport(True, postures_checked=1)
        arm = self._arm(side)
        depth, clearance, worst = self._depths(arm, arm.link_frames(q))
        return self._report_at(arm, depth, clearance, worst,
                               int(np.argmax(depth)), postures_checked=1)

    def swept_ok(self, kin, side: str, q_from, q_to) -> ClearanceReport:
        """Does every posture on the joint-space line ``q_from -> q_to`` clear?

        Postures are sampled so that no capsule endpoint travels more than
        :attr:`ClearancePolicy.sweep_step_m` between two of them (``q_from``
        itself is the previous interval's end and is not re-judged).

        AN ARM ALREADY INSIDE AN ENVELOPE MAY LEAVE IT. A posture that starts
        inside an obstacle's margin — measured state, a droop margin raised
        after the grasp — must still be allowed to move AWAY, or a retreat
        from the table would be refused for being near the table. So a
        sample that violates an obstacle is accepted when ``q_from`` already
        violated that same obstacle at least as deeply; only getting deeper,
        or entering a new envelope, is refused.
        """
        if not self.obstacles:
            return ClearanceReport(True)
        q_from = np.asarray(q_from, dtype=float).reshape(7)
        q_to = np.asarray(q_to, dtype=float).reshape(7)
        arm = self._arm(side)
        frames0 = arm.link_frames(q_from)
        frames1 = arm.link_frames(q_to)
        travel = float(np.max(np.linalg.norm(
            arm.endpoints(frames1) - arm.endpoints(frames0), axis=2)))
        n = max(1, int(math.ceil(travel / self.policy.sweep_step_m)))
        start, _, _ = self._depths(arm, frames0)
        # an envelope the arm starts in may be held, not deepened
        allowed = np.maximum(start, 0.0) + 1e-6
        best = None
        for k in range(1, n + 1):
            frames = frames1 if k == n else arm.link_frames(
                q_from + (q_to - q_from) * (k / n))
            depth, clearance, worst = self._depths(arm, frames)
            over = depth - np.where(start > 0.0, allowed, 0.0)
            bad = int(np.argmax(over))
            if over[bad] > 0.0:
                return self._report_at(arm, depth, clearance, worst, bad,
                                       fraction=k / n, postures_checked=k)
            i = int(np.argmax(depth))
            if best is None or depth[i] > best[0]:
                best = (float(depth[i]), k, depth, clearance, worst, i)
        _, k, depth, clearance, worst, i = best
        rep = self._report_at(arm, depth, clearance, worst, i,
                              fraction=k / n, postures_checked=n)
        return _with(rep, ok=True)

    # -- transit height -------------------------------------------------- #
    def in_the_way(self, p0, p1, *, hang_m: float, width_m: float
                   ) -> Tuple[float, Tuple[str, ...]]:
        """The tool height a transit from ``p0`` to ``p1`` must rise to.

        An obstacle is IN THE WAY when the straight tool segment passes over
        its footprint (widened by ``width_m`` + its required clearance) lower
        than its top + required clearance + ``hang_m`` (how far the hand
        hangs below the tool point). An obstacle whose envelope already
        contains an END of the segment is not: the leg is declared to start or
        end there, and rising over it cannot change that — the link gate
        judges such a leg instead.

        Returns ``(height, names)``; ``(-inf, ())`` when nothing is in the way.
        """
        p0 = np.asarray(p0, dtype=float)
        p1 = np.asarray(p1, dtype=float)
        height, names = -math.inf, []
        samples = max(2, int(math.ceil(float(np.linalg.norm(p1[:2] - p0[:2]))
                                       / 0.01)) + 1)
        for ob in self.obstacles:
            reach = width_m + self.required_m(ob)
            ceiling = ob.top_z + self.required_m(ob) + hang_m

            def inside(p):
                return ob.footprint_distance(p[:2]) <= reach and p[2] < ceiling

            if inside(p0) or inside(p1):
                continue
            for s in np.linspace(0.0, 1.0, samples):
                if inside(p0 + (p1 - p0) * s):
                    height = max(height, ceiling)
                    names.append(ob.name)
                    break
        return height, tuple(names)


#: how wide the hand is around a contact leg's ray [m] (half the palm)
CONTACT_RAY_HALF_WIDTH_M = 0.04


def first_hit(gate: "SceneGate", p_start, direction, travel_m: float, *,
              width_m: float = CONTACT_RAY_HALF_WIDTH_M) -> str:
    """The obstacle a tool ray from ``p_start`` along ``direction`` meets first.

    Each box is grown by ``width_m`` plus its required clearance, and the ray
    runs ``travel_m`` plus the pad tips' reach past the tool point. ``""``
    when it meets nothing.
    """
    from .grasp_geometry import PAD  # noqa: PLC0415
    p0 = np.asarray(p_start, dtype=float).reshape(3)
    d = np.asarray(direction, dtype=float).reshape(3)
    d = d / max(float(np.linalg.norm(d)), 1e-12)
    reach = float(travel_m) + PAD.lead_m
    best, name = math.inf, ""
    for ob in gate.obstacles:
        m = ob.r.as_matrix()
        o = m.T @ (p0 - ob.p)
        v = m.T @ d
        half = ob.half + width_m + gate.required_m(ob)
        t0, t1 = 0.0, reach
        for k in range(3):
            if abs(v[k]) < 1e-12:
                if abs(o[k]) > half[k]:
                    t0, t1 = 1.0, 0.0
                    break
                continue
            a, b = (-half[k] - o[k]) / v[k], (half[k] - o[k]) / v[k]
            t0, t1 = max(t0, min(a, b)), min(t1, max(a, b))
        if t0 <= t1 and t0 < best:
            best, name = t0, ob.name
    return name


def _with(rep: ClearanceReport, **changes) -> ClearanceReport:
    import dataclasses  # noqa: PLC0415
    return dataclasses.replace(rep, **changes)


__all__ = ["ClearancePolicy", "ClearanceReport", "DEFAULT_POLICY", "Obstacle",
           "PROBED_SOURCES", "SceneGate", "first_hit", "held_names",
           "obstacles_of",
           "policy_of", "set_policy"]
