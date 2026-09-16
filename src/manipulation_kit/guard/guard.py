"""D1 dual-arm software motion guard: joint-limit clamp + self-collision /
torso keep-out check for commanded joint vectors, BEFORE they reach the
hardware.

Model source: description/d1/d1.urdf (full-body primitives-only URDF).
Frame + conventions match include/omakase_arm/{safety_zones.h,
collision_model.h} and config/safety_zones.json:

  * angles in DEGREES, SDK order J1..J7;
  * SDK ArmSide 'A' = "_R" link tree = physical LEFT arm (+y);
    SDK ArmSide 'B' = "_L" link tree = physical RIGHT arm (-y);
  * torso keep-out margin default 0.03 m, arm-arm min distance 0.06 m
    (safety_zones.json), self margin 0.0 (collision_model.h defaults).

The guard is OPT-IN: nothing in the SDK behaves differently unless a
caller constructs a MotionGuard / wraps its robot in GuardedRobot (see
sdk/TJ.py: OMAKASE_ARM_GUARD=1).

Checks performed by MotionGuard.check():
  1. per-joint limits (from the URDF), clamped or rejected;
  2. every arm capsule vs the torso/head/chassis keep-out boxes;
  3. arm-arm minimum capsule distance;
  4. same-arm self collision (non-adjacent capsule pairs, with the
     "bridged by a short link" skip rule of collision_model.h).

COLLISION POLICY — what the guard does and does NOT protect
-----------------------------------------------------------
The guard protects the ARM STRUCTURE (shoulder → upper arm → elbow →
forearm → wrist) from the torso and from the other arm.  The
END-EFFECTORS are the robot's working surfaces: they MUST be free to
contact the world and each other (bimanual manipulation, hand-to-hand
hand-offs), so every body DISTAL OF THE TOOL MOUNTING FLANGE is excluded
from ALL collision checks (body, arm-arm and self).

The mounting-flange boundary is the fixed tool joint JointTCP (Link7 →
TCP_Link) in d1.urdf: Link7 and everything proximal is arm structure and
stays checked; TCP_Link plus the whole YUBI hand (palm / camera /
fingers) is the tool side and is excluded.  See EE_LINK_PREFIXES /
is_ee_body().

(Owner directive, 2026-07-20 VR-teleop field feedback: "EEは無視した範囲
でIKのガード計算した方がいい" — bimanual contact is intended, so the
hands must not be blocked from touching things or each other.)

Compared to the C++ ArmCollisionModel this model covers the arm chain up
to Link7 (the C++ model stops at Joint7); the TCP flange and YUBI hand
are modelled for FK/rendering but excluded from the guard checks by the
policy above.
"""
from __future__ import annotations

import math

from . import geometry as g
from .urdf_model import UrdfModel, prim_to_world, DEFAULT_URDF

ARM_SIDES = {"A": "R", "B": "L"}  # SDK ArmSide -> URDF link suffix
JOINTS_PER_ARM = 7

# Links that sit at/inside the shoulder mount by construction — never
# checked against the body (mirrors collision_model.h first_link_to_check).
DEFAULT_BODY_EXEMPT_LINKS = ("Base_R", "Base_L")

# --- end-effector / tool exclusion (collision policy, see module docstring) --
# Everything DISTAL of the tool mounting flange (JointTCP: Link7 -> TCP_Link).
# These bodies are the working surfaces and are excluded from every collision
# check so bimanual tasks (hands touching the world and each other) are not
# blocked.  Link7 and everything proximal is arm STRUCTURE and stays checked.
EE_LINK_PREFIXES = ("TCP_Link", "yubi")


def is_ee_body(link: str) -> bool:
    """True if `link` is an end-effector / tool body distal of the mounting
    flange (TCP flange or any YUBI hand part).  Such bodies are excluded
    from ALL guard collision checks — see the module docstring."""
    return any(link.startswith(p) for p in EE_LINK_PREFIXES)

# Body boxes the arms legitimately pass through / live next to.
# ("*_exempt" collision names are also skipped automatically.)
DEFAULT_DISABLED_BODY_BOXES = (
    # chassis geometry is only valid at the lift extension captured in the
    # CAD; enable explicitly when the lift is at that pose.
    "chassis_cover",
    "chassis_body",
    "lift_pole",
)

# --- upper-chest keep-out (fixes the "elbow sinks into the torso" false
# negative, VR-teleop field feedback 2026-07-20) ---------------------------
# The measured torso AABBs from d1.urdf leave a coverage NOTCH at the upper
# chest / shoulder band (z 0.45..0.56 m): there the only real body is the
# CAD shoulder shell, which is tagged "..._exempt" in the URDF (the shoulder
# mount barrels pass through it) and so contributes no keep-out.  The chest
# boxes that remain there — torso_frame (y +-0.088) / torso_core (y +-0.055)
# — are far NARROWER than the real trunk (CAD shell y +-0.15), so an elbow /
# forearm driven across the chest at working height slipped through.
#
# We close the notch WITHOUT loosening anything: an explicit per-arm chest
# keep-out box that fills the band at the real trunk width, but is offset to
# the CENTRE + FAR half so each arm may still occupy its OWN-SIDE shoulder
# space (where its Base/Link1 barrels legitimately live) at HOME.  The
# dangerous configuration — the elbow/forearm crossing the midline or
# pressing into the sternum — is what this catches.  Radii + margin absorb
# the split.
#
# lo/hi are root-frame AABBs (metres).  Arm 'A' = physical LEFT (+y): it may
# keep its +y shoulder space, so its box covers y in [-0.13, +0.06].  Arm
# 'B' = physical RIGHT (-y): mirror, y in [-0.06, +0.13].
DEFAULT_CHEST_KEEPOUT = {
    "A": ((-0.1245, -0.13, 0.45), (0.1245, 0.06, 0.60)),
    "B": ((-0.1245, -0.06, 0.45), (0.1245, 0.13, 0.60)),
}
# Shoulder-mount links whose barrels live inside the chest band at HOME and
# so are exempt from the chest keep-out (Base is already globally body-exempt;
# Link1 is the rigid shoulder barrel at y ~ 0.196).  Link2 (upper arm) onward
# is NOT exempt — that is the elbow/forearm structure we must protect.
DEFAULT_CHEST_EXEMPT_LINKS = ("Base_R", "Base_L", "Link1_R", "Link1_L")


class GuardViolation(Exception):
    """Raised (in mode='raise') when a commanded pose is rejected."""

    def __init__(self, report):
        self.report = report
        super().__init__(str(report))


class GuardReport:
    """Outcome of a check.  ok == True means safe to send."""

    __slots__ = ("ok", "violations", "clamped", "min_body_clearance",
                 "min_arm_arm", "min_self_clearance", "min_chest_clearance")

    def __init__(self):
        self.ok = True
        self.violations = []      # human-readable strings
        self.clamped = False      # limits were clamped (only in clamp mode)
        self.min_body_clearance = 1e9
        self.min_arm_arm = 1e9
        self.min_self_clearance = 1e9
        self.min_chest_clearance = 1e9

    def add(self, msg):
        self.ok = False
        self.violations.append(msg)

    def __str__(self):
        if self.ok:
            return ("guard: OK (body %.3f m, chest %.3f m, arm-arm %.3f m, "
                    "self %.3f m)"
                    % (self.min_body_clearance, self.min_chest_clearance,
                       self.min_arm_arm, self.min_self_clearance))
        return "guard: REJECT — " + "; ".join(self.violations)


class MotionGuard:
    """URDF-driven joint-limit + self-collision guard for the D1 arms.

    Parameters
    ----------
    urdf_path : str
        Full-body primitives URDF (default: description/d1/d1.urdf).
    body_margin_m : float
        Required clearance between any arm capsule and the torso/head
        keep-out boxes (default 0.03 = safety_zones.json min_body_clearance).
    arm_arm_margin_m : float
        Minimum distance between the two arms' capsules (default 0.06 =
        safety_zones.json min_arm_arm_distance_m).
    self_margin_m : float
        Extra margin for same-arm non-adjacent pairs (default 0.0, as in
        collision_model.h).
    clamp_limits : bool
        True (default): out-of-limit joints are clamped to the limit and
        the check continues on the clamped pose.  False: out-of-limit is a
        violation.
    check_body / check_arm_arm / check_self : bool
        Enable/disable individual check classes.
    disabled_body_boxes : iterable of str
        Names of body collision boxes to skip (default: the chassis/lift
        boxes, whose position depends on the lift extension).
    box_pad_m : dict {box_name: pad_metres} or None
        Per-box size correction applied to the body AABBs built from the
        URDF, in metres and on every face: a POSITIVE pad inflates the box
        (lo - pad, hi + pad), a NEGATIVE one shrinks it.  Use it where the
        measured hardware differs from the CAD primitive (e.g. a shell that
        bulges beyond its box) without editing the URDF, which the C++ side
        shares.  Unlike `body_margin_m` — one clearance applied to every box
        — this changes the geometry of ONE named box, so a single tight
        region can be padded without widening the whole body keep-out.  An
        unknown box name raises ValueError (a typo must not silently pad
        nothing).
    chest_keepout : dict {side: (lo, hi)} or None
        Per-arm upper-chest keep-out AABBs (root frame) that close the
        shoulder-band coverage notch left by the exempt CAD shoulder shell
        (see DEFAULT_CHEST_KEEPOUT).  None disables the chest keep-out.
    finger_angles_rad : dict
        Optional finger joint angles (default all 0.0 = fingers straight).
    """

    def __init__(self, urdf_path: str = DEFAULT_URDF, *,
                 body_margin_m: float = 0.03,
                 arm_arm_margin_m: float = 0.06,
                 self_margin_m: float = 0.0,
                 clamp_limits: bool = True,
                 check_body: bool = True,
                 check_arm_arm: bool = True,
                 check_self: bool = True,
                 body_exempt_links=DEFAULT_BODY_EXEMPT_LINKS,
                 disabled_body_boxes=DEFAULT_DISABLED_BODY_BOXES,
                 box_pad_m=None,
                 chest_keepout=DEFAULT_CHEST_KEEPOUT,
                 chest_exempt_links=DEFAULT_CHEST_EXEMPT_LINKS,
                 finger_angles_rad=None):
        self.model = UrdfModel(urdf_path)
        self.body_margin_m = body_margin_m
        self.arm_arm_margin_m = arm_arm_margin_m
        self.self_margin_m = self_margin_m
        self.clamp_limits = clamp_limits
        self.check_body = check_body
        self.check_arm_arm = check_arm_arm
        self.check_self = check_self
        self.body_exempt_links = set(body_exempt_links)
        self.disabled_body_boxes = set(disabled_body_boxes)
        self.box_pad_m = dict(box_pad_m or {})
        self.chest_keepout = dict(chest_keepout) if chest_keepout else {}
        self.chest_exempt_links = set(chest_exempt_links)
        self.finger_angles_rad = dict(finger_angles_rad or {})

        m = self.model
        self.joint_names = {
            side: [f"Joint{i}_{suf}" for i in range(1, 8)]
            for side, suf in ARM_SIDES.items()}
        self.limits_deg = {
            side: m.limits_deg(names)
            for side, names in self.joint_names.items()}

        # Arm link sets (everything below each mount), ordered by tree depth
        # so the "bridged" rule can measure chain length between links.
        self.arm_links = {
            side: [l for l in m.descendants(f"Base_{suf}")]
            for side, suf in ARM_SIDES.items()}
        for side in self.arm_links:
            self.arm_links[side].sort(key=m.chain_depth)
        arm_link_all = {l for ls in self.arm_links.values() for l in ls}

        # Static body AABBs (root frame; computed once).
        tf0 = m.link_transforms({})
        self.body_aabbs = []
        for link in m.links:
            if link in arm_link_all:
                continue
            for prim in m.collisions[link]:
                kind, shape = prim_to_world(prim, link, tf0[link],
                                            prefer_aabb=True)
                if kind != "aabb":
                    raise ValueError(
                        f"body link {link} collision {prim.name} is not an "
                        "axis-aligned box; the guard expects static body "
                        "geometry as AABBs")
                name, lo, hi = shape
                if name.endswith("_exempt"):
                    continue
                self.body_aabbs.append((name, lo, hi))

        # Per-box size correction (box_pad_m).  Applied AFTER the URDF boxes
        # are built, because the pad is keyed by the primitive name the URDF
        # supplies.  Validate first: a name that matches no box would pad
        # nothing at all, which is a configuration error, not a no-op.
        if self.box_pad_m:
            known = {name for name, _, _ in self.body_aabbs}
            for name in self.box_pad_m:
                if name not in known:
                    raise ValueError(
                        f"box_pad_m names unknown body box {name!r}; known "
                        f"boxes: {sorted(known)}")
            self.body_aabbs = [
                (name,
                 tuple(v - self.box_pad_m.get(name, 0.0) for v in lo),
                 tuple(v + self.box_pad_m.get(name, 0.0) for v in hi))
                for name, lo, hi in self.body_aabbs]

    # ------------------------------------------------------------------ #
    def clamp(self, side: str, joints_deg):
        """Per-joint limit clamp.  Returns (clamped_list, was_clamped)."""
        if side not in ARM_SIDES:
            raise ValueError(f"arm must be 'A' or 'B', got {side!r}")
        if len(joints_deg) != JOINTS_PER_ARM:
            raise ValueError(f"expected {JOINTS_PER_ARM} joints, got "
                             f"{len(joints_deg)}")
        out, changed = [], False
        for q, (lo, hi) in zip(joints_deg, self.limits_deg[side]):
            c = min(max(float(q), lo), hi)
            if c != float(q):
                changed = True
            out.append(c)
        return out, changed

    # ------------------------------------------------------------------ #
    def _arm_capsules(self, side: str, joints_deg):
        """Checked world capsules of one arm at the given joint vector (deg).

        End-effector / tool bodies (is_ee_body: TCP flange + YUBI hand) are
        NOT returned — they are excluded from every collision check by the
        guard's collision policy (module docstring).  `tfs` still holds the
        FK of every link (including the EE) for callers that need it."""
        m = self.model
        q = {n: math.radians(v)
             for n, v in zip(self.joint_names[side], joints_deg)}
        q.update(self.finger_angles_rad)
        tfs = m.link_transforms(q)
        caps = []
        for link in self.arm_links[side]:
            if is_ee_body(link):              # tool side: excluded from checks
                continue
            for prim in m.collisions[link]:
                if "_cap_" in prim.name:      # capsule end-sphere duplicates
                    continue
                kind, shape = prim_to_world(prim, link, tfs[link],
                                            prefer_aabb=False)
                assert kind == "capsule"
                caps.append(shape)
        return caps, tfs

    def _chain_points(self, side, tfs):
        """Joint origin positions along the arm's serial chain (for the
        bridged-short-link rule), ordered root -> tip."""
        pts = {}
        for link in self.arm_links[side]:
            pts[link] = tfs[link].t
        return pts

    def _bridged(self, side, cap_i, cap_j, tfs):
        """True if the chain STRICTLY BETWEEN the two capsules' links is
        shorter in total than the two radii (the capsules then overlap by
        construction -> skip the pair).  Mirrors collision_model.h: for its
        pair (i, j) it sums |pts[k+1]-pts[k]| for k in i+1..j-1, i.e. the
        segments of the intermediate links only."""
        m = self.model
        order = {l: i for i, l in enumerate(self.arm_links[side])}
        li, lj = cap_i.link, cap_j.link
        if order[li] > order[lj]:
            li, lj = lj, li
        # walk parents from lj toward li; each hop l->p contributes the
        # length of p's link segment EXCEPT the final hop into li itself
        # (that segment belongs to capsule i, not to the gap between them).
        total = 0.0
        l = lj
        while l in m.parent_link:
            p = m.parent_link[l]
            if p == li:
                return total < cap_i.r + cap_j.r
            if p not in order:      # left the arm subtree: not the same chain
                return False
            total += g.norm(g.sub(tfs[l].t, tfs[p].t))
            l = p
        return False

    def _adjacent(self, la, lb):
        return self.model.parent_link.get(la) == lb or \
            self.model.parent_link.get(lb) == la

    # ------------------------------------------------------------------ #
    def check(self, joints_a_deg=None, joints_b_deg=None) -> GuardReport:
        """Full check of one or both arms' commanded joint vectors (deg,
        SDK order).  Pass one arm as None to skip it (its checks — and the
        arm-arm check — are then skipped)."""
        rep = GuardReport()
        arms = {}
        for side, joints in (("A", joints_a_deg), ("B", joints_b_deg)):
            if joints is None:
                continue
            clamped, changed = self.clamp(side, joints)
            if changed:
                if self.clamp_limits:
                    rep.clamped = True
                else:
                    for i, (q, c) in enumerate(zip(joints, clamped)):
                        if float(q) != c:
                            lo, hi = self.limits_deg[side][i]
                            rep.add(f"arm {side} J{i + 1}={float(q):.1f} deg "
                                    f"outside [{lo:.0f}, {hi:.0f}]")
            arms[side] = clamped
        if not rep.ok:
            return rep

        caps, tfs = {}, {}
        for side, joints in arms.items():
            caps[side], tfs[side] = self._arm_capsules(side, joints)

        if self.check_body:
            for side in arms:
                for c in caps[side]:
                    if c.link in self.body_exempt_links:
                        continue
                    for name, lo, hi in self.body_aabbs:
                        if name in self.disabled_body_boxes:
                            continue
                        d = g.seg_aabb_distance(c.a, c.b, lo, hi) - c.r
                        rep.min_body_clearance = min(rep.min_body_clearance, d)
                        if d < self.body_margin_m:
                            rep.add(f"arm {side} link {c.link} within "
                                    f"{max(d, 0):.3f} m of body box {name} "
                                    f"(margin {self.body_margin_m:.3f})")
                # per-arm upper-chest keep-out (closes the shoulder-band notch
                # where only the exempt CAD shell exists — see DEFAULT_CHEST_
                # KEEPOUT): the elbow / forearm crossing into the chest.
                kbox = self.chest_keepout.get(side)
                if kbox is not None:
                    lo, hi = kbox
                    for c in caps[side]:
                        if c.link in self.chest_exempt_links:
                            continue
                        d = g.seg_aabb_distance(c.a, c.b, lo, hi) - c.r
                        rep.min_chest_clearance = min(rep.min_chest_clearance, d)
                        if d < self.body_margin_m:
                            rep.add(f"arm {side} link {c.link} within "
                                    f"{max(d, 0):.3f} m of chest keep-out "
                                    f"(margin {self.body_margin_m:.3f})")
        if self.check_arm_arm and "A" in arms and "B" in arms:
            for ca in caps["A"]:
                if ca.link in self.body_exempt_links:
                    continue
                for cb in caps["B"]:
                    if cb.link in self.body_exempt_links:
                        continue
                    d = g.seg_seg_distance(ca.a, ca.b, cb.a, cb.b) - ca.r - cb.r
                    rep.min_arm_arm = min(rep.min_arm_arm, d)
                    if d < self.arm_arm_margin_m:
                        rep.add(f"arm A {ca.link} vs arm B {cb.link}: "
                                f"{max(d, 0):.3f} m < "
                                f"{self.arm_arm_margin_m:.3f}")
        if self.check_self:
            for side in arms:
                cs = [c for c in caps[side]
                      if c.link not in self.body_exempt_links]
                for i in range(len(cs)):
                    for j in range(i + 1, len(cs)):
                        ci, cj = cs[i], cs[j]
                        if ci.link == cj.link:
                            continue
                        if self._adjacent(ci.link, cj.link):
                            continue
                        if self._bridged(side, ci, cj, tfs[side]):
                            continue
                        d = (g.seg_seg_distance(ci.a, ci.b, cj.a, cj.b)
                             - ci.r - cj.r - self.self_margin_m)
                        rep.min_self_clearance = min(rep.min_self_clearance, d)
                        if d < 0.0:
                            rep.add(f"arm {side} self: {ci.link} vs "
                                    f"{cj.link} penetrating {-d:.3f} m")
        return rep

    # convenience -------------------------------------------------------- #
    def guard(self, joints_a_deg=None, joints_b_deg=None):
        """check() and raise GuardViolation on rejection.  Returns the
        (possibly limit-clamped) joint vectors."""
        rep = self.check(joints_a_deg, joints_b_deg)
        if not rep.ok:
            raise GuardViolation(rep)
        out = []
        for side, joints in (("A", joints_a_deg), ("B", joints_b_deg)):
            out.append(None if joints is None else self.clamp(side, joints)[0])
        return out[0], out[1]


class GuardedRobot:
    """Opt-in wrapper around vendor_arm_sdk.robot.ArmRobot that runs
    every set_joint_cmd_pose() through a MotionGuard before forwarding.

    All other attributes/methods delegate untouched to the wrapped robot,
    so `GuardedRobot(robot, guard=None)` (or enabled=False) behaves
    byte-identically to the bare robot.

    Because set_joint_cmd_pose() is per-arm but arm-arm safety needs both
    arms, the wrapper remembers the last commanded pose of the other arm
    (seed it with seed_pose(), e.g. from feedback joints, for full coverage
    from the first command; until both arms are known the arm-arm check is
    skipped).

    on_reject: "block" (default) — drop the command, print the reason and
    return 2 (the SDK's False); or "raise" — raise GuardViolation.
    """

    def __init__(self, robot, guard: MotionGuard = None, *,
                 enabled: bool = True, on_reject: str = "block",
                 log=print):
        self._robot = robot
        self._guard = guard
        self._enabled = enabled and guard is not None
        if on_reject not in ("block", "raise"):
            raise ValueError("on_reject must be 'block' or 'raise'")
        self._on_reject = on_reject
        self._log = log
        self._last = {"A": None, "B": None}

    def seed_pose(self, arm: str, joints_deg):
        """Tell the guard where an arm currently is (e.g. feedback joints)."""
        self._last[arm] = list(joints_deg)

    def set_joint_cmd_pose(self, arm: str, joints: list):
        if not self._enabled:
            return self._robot.set_joint_cmd_pose(arm, joints)
        other = "B" if arm == "A" else "A"
        qa = joints if arm == "A" else self._last["A"]
        qb = joints if arm == "B" else self._last["B"]
        rep = self._guard.check(qa, qb)
        if not rep.ok:
            if self._on_reject == "raise":
                raise GuardViolation(rep)
            self._log(f"[pyguard] BLOCKED set_joint_cmd_pose({arm!r}): {rep}")
            return 2  # SDK convention: 2 == False
        clamped, _ = self._guard.clamp(arm, joints)
        self._last[arm] = clamped
        return self._robot.set_joint_cmd_pose(arm, clamped)

    def __getattr__(self, name):
        return getattr(self._robot, name)
