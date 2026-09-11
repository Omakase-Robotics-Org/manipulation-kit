"""Pure-python URDF loader + forward kinematics for the D1 full-body model.

Parses description/d1/d1.urdf (primitives-only collision URDF) with
xml.etree — no ROS, no numpy.  Provides:

  * joint limits (revolute joints, radians in the file),
  * a kinematic tree with FK to any link frame,
  * per-link collision primitives (box / cylinder / sphere) converted to
    world-frame check shapes:
      - cylinder  -> capsule (segment along local z +- length/2, radius r)
      - sphere    -> degenerate capsule (point, radius r)
      - box on a STATIC body link -> axis-aligned box (asserted axis-aligned)
      - box on a MOVING link -> conservative bounding capsule along the box's
        longest axis (radius = half diagonal of the two shorter sides)

Collision elements whose name ends with "_exempt" are parsed but excluded
from checking (e.g. the shoulder-cover shell band the arms pass through).
Cylinder "cap" spheres emitted by the URDF generator (name contains
"_cap_") duplicate their cylinder's endpoints and are skipped too.
"""
from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET

from . import geometry as g

#: The guard model, resolved INSIDE the installed package — never from a
#: ``$D1_SDK_DIR`` checkout. ``manipulation_kit.description`` ships
#: ``d1/d1.urdf`` as package data, so this works identically from a git
#: checkout, a wheel and a zipapp-free installed tree.
DEFAULT_URDF = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "description", "d1", "d1.urdf"))


def _floats(s, default):
    if s is None:
        return default
    return tuple(float(v) for v in s.split())


class Joint:
    __slots__ = ("name", "type", "parent", "child", "origin", "axis",
                 "lower", "upper")

    def __init__(self, el):
        self.name = el.get("name")
        self.type = el.get("type")
        self.parent = el.find("parent").get("link")
        self.child = el.find("child").get("link")
        o = el.find("origin")
        xyz = _floats(o.get("xyz") if o is not None else None, (0.0, 0.0, 0.0))
        rpy = _floats(o.get("rpy") if o is not None else None, (0.0, 0.0, 0.0))
        self.origin = g.from_rpy_xyz(*rpy, *xyz)
        a = el.find("axis")
        self.axis = _floats(a.get("xyz") if a is not None else None, (0.0, 0.0, 1.0))
        lim = el.find("limit")
        # Prismatic joints carry <limit> too (metres, not radians). The guard
        # only ever asks about revolute ones, but the IK substrate in
        # manipulation_kit.arms.urdf_chain needs the bounds of whatever it
        # actuates, and parsing them here keeps ONE URDF reader in the package.
        if self.type in ("revolute", "prismatic") and lim is not None:
            self.lower = float(lim.get("lower"))
            self.upper = float(lim.get("upper"))
        else:
            self.lower = self.upper = None


class CollisionPrim:
    """One parsed <collision> element (link-local)."""
    __slots__ = ("name", "kind", "origin", "size")

    def __init__(self, name, kind, origin, size):
        self.name = name
        self.kind = kind          # "box" | "cylinder" | "sphere"
        self.origin = origin      # Tf, link-local
        self.size = size          # box: (sx,sy,sz); cyl: (r,len); sph: (r,)


class UrdfModel:
    def __init__(self, path: str = DEFAULT_URDF):
        self.path = path
        root = ET.parse(path).getroot()
        self.name = root.get("name")
        self.joints = {}          # child link name -> Joint
        self.children = {}        # link -> [Joint]
        self.parent_link = {}     # link -> parent link name
        for jel in root.findall("joint"):
            j = Joint(jel)
            self.joints[j.child] = j
            self.children.setdefault(j.parent, []).append(j)
            self.parent_link[j.child] = j.parent
        self.links = [l.get("name") for l in root.findall("link")]
        roots = [l for l in self.links if l not in self.parent_link]
        if len(roots) != 1:
            raise ValueError(f"URDF must have exactly one root link, got {roots}")
        self.root = roots[0]
        self.collisions = {}      # link -> [CollisionPrim]
        for lel in root.findall("link"):
            prims = []
            for cel in lel.findall("collision"):
                gm = cel.find("geometry")
                if gm is None:
                    continue
                o = cel.find("origin")
                xyz = _floats(o.get("xyz") if o is not None else None, (0.0, 0.0, 0.0))
                rpy = _floats(o.get("rpy") if o is not None else None, (0.0, 0.0, 0.0))
                origin = g.from_rpy_xyz(*rpy, *xyz)
                name = cel.get("name") or f"{lel.get('name')}_col{len(prims)}"
                box = gm.find("box")
                cyl = gm.find("cylinder")
                sph = gm.find("sphere")
                if box is not None:
                    prims.append(CollisionPrim(name, "box", origin,
                                               _floats(box.get("size"), None)))
                elif cyl is not None:
                    prims.append(CollisionPrim(
                        name, "cylinder", origin,
                        (float(cyl.get("radius")), float(cyl.get("length")))))
                elif sph is not None:
                    prims.append(CollisionPrim(name, "sphere", origin,
                                               (float(sph.get("radius")),)))
                else:
                    raise ValueError(
                        f"unsupported collision geometry on link {lel.get('name')} "
                        "(this URDF must stay primitives-only)")
            self.collisions[lel.get("name")] = prims

    # -- limits ------------------------------------------------------------
    def revolute_joints(self):
        return [j for j in self.joints.values() if j.type == "revolute"]

    def limits_deg(self, joint_names):
        """[(lower_deg, upper_deg)] for the given revolute joint names."""
        out = []
        for n in joint_names:
            j = next((x for x in self.joints.values() if x.name == n), None)
            if j is None or j.lower is None:
                raise KeyError(f"no revolute joint named {n}")
            out.append((math.degrees(j.lower), math.degrees(j.upper)))
        return out

    # -- FK ----------------------------------------------------------------
    def link_transforms(self, q_rad: dict):
        """World (root-frame) Tf of every link.  q_rad maps revolute joint
        name -> angle in RADIANS; missing joints default to 0."""
        tfs = {self.root: g.Tf()}
        stack = [self.root]
        while stack:
            parent = stack.pop()
            for j in self.children.get(parent, []):
                tf = tfs[parent].mul(j.origin)
                if j.type == "revolute":
                    tf = tf.mul(g.rot_axis(j.axis, q_rad.get(j.name, 0.0)))
                tfs[j.child] = tf
                stack.append(j.child)
        return tfs

    def descendants(self, link):
        out, stack = [], [link]
        while stack:
            l = stack.pop()
            out.append(l)
            for j in self.children.get(l, []):
                stack.append(j.child)
        return out

    def chain_depth(self, link):
        d, l = 0, link
        while l in self.parent_link:
            l = self.parent_link[l]
            d += 1
        return d


class WorldCapsule:
    __slots__ = ("name", "link", "a", "b", "r")

    def __init__(self, name, link, a, b, r):
        self.name, self.link, self.a, self.b, self.r = name, link, a, b, r


def prim_to_world(prim: CollisionPrim, link: str, tf: g.Tf, prefer_aabb=True):
    """Convert a link-local primitive to a world check shape.

    Returns ("capsule", WorldCapsule) or ("aabb", (name, lo, hi)).
    Boxes become AABBs only when prefer_aabb (static body links) and their
    world rotation is axis-aligned; otherwise a conservative bounding
    capsule along the longest box axis.
    """
    w = tf.mul(prim.origin)
    if prim.kind == "sphere":
        return ("capsule", WorldCapsule(prim.name, link, w.t, w.t, prim.size[0]))
    if prim.kind == "cylinder":
        r, ln = prim.size
        h = ln / 2.0
        a = w.apply((0.0, 0.0, -h))
        b = w.apply((0.0, 0.0, h))
        return ("capsule", WorldCapsule(prim.name, link, a, b, r))
    # box
    sx, sy, sz = prim.size
    R = w.R
    is_axis_aligned = all(
        abs(abs(R[i][j]) - (1.0 if i == j else 0.0)) < 1e-6 or
        abs(R[i][j]) < 1e-6
        for i in range(3) for j in range(3))
    if prefer_aabb and is_axis_aligned:
        half_local = (sx / 2.0, sy / 2.0, sz / 2.0)
        # rotate half extents through |R| to get world half extents
        half = tuple(
            abs(R[i][0]) * half_local[0] + abs(R[i][1]) * half_local[1]
            + abs(R[i][2]) * half_local[2] for i in range(3))
        lo = tuple(w.t[i] - half[i] for i in range(3))
        hi = tuple(w.t[i] + half[i] for i in range(3))
        return ("aabb", (prim.name, lo, hi))
    # oriented box -> bounding capsule along longest axis
    dims = [(sx, (1.0, 0.0, 0.0)), (sy, (0.0, 1.0, 0.0)), (sz, (0.0, 0.0, 1.0))]
    dims.sort(key=lambda d: -d[0])
    (dmax, ax), (d2, _), (d3, _) = dims
    r = 0.5 * math.sqrt(d2 * d2 + d3 * d3)
    h = dmax / 2.0
    a = w.apply(tuple(c * -h for c in ax))
    b = w.apply(tuple(c * h for c in ax))
    return ("capsule", WorldCapsule(prim.name, link, a, b, r))
