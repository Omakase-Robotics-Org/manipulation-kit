"""The description bundle: one versioned, digested copy of a robot's shape.

Several programs need the same facts about the D1 — the daemon's motion guard,
a planner, a 3D view, a simulator — and each used to take its own copy of the
files it needed out of this package. A copy that is not identified drifts
silently. The bundle is the identified copy: a directory of plain files, a
``manifest.json`` that lists each file with its SHA-256, a
``description_version`` and one ``digest`` over all of them. Two programs that
report the same digest are using the same description, byte for byte.

What is in it (bundle schema ``manipulation_kit.description_bundle/1``)
----------------------------------------------------------------------

==========================================  ==================================
path                                        what it is
==========================================  ==================================
``robot.json``                              which robot model the bundle
                                            describes (schema
                                            ``manipulation_kit.robot_model/1``):
                                            its URDFs, its guard profile, its
                                            devices, each arm's joints and
                                            end effector
``kinematics/d1.urdf``                      the mesh-free whole-body model the
                                            motion guard parses (kinematics
                                            plus collision primitives)
``kinematics/d1_wholebody_gripper.urdf``    the articulated whole body (lift,
                                            neck) with visual mesh references
``collision/collision.json``                ``d1.urdf`` as data: every joint
                                            and every collision primitive, so
                                            a consumer needs no URDF parser
``guard/profile.<name>.json``               a motion guard policy (checks,
                                            margins, keep-outs, exemptions):
                                            ``d1``, ``d1-bench``, ``sim``
                                            (:data:`GUARD_PROFILES`)
``poses/home_pose.json``                    the HOME pose, verbatim
``end_effectors/tool_configs/<id>.json``    the tool configs, verbatim
``hands/catalog.json``                      the hand catalog (schema
                                            ``manipulation_kit.hand_catalog.v1``:
                                            models, aliases, envelopes),
                                            verbatim
``visual/visual.json``                      every mesh the bundled URDFs
                                            reference, with its licence class
==========================================  ==================================

That table is the ``d1`` model's bundle. One bundle is built per robot model
(:mod:`manipulation_kit.description.robots`): another model's bundle has the
same layout with its own URDFs (``robot.json``'s ``urdf.guard`` and
``urdf.view``), its own collision model and its own guard profile, and its
own lock. ``mkit-description --model <id>`` selects one; ``d1`` is the
default.

Every file is GENERATED from this package's own single sources (the URDF
generator's output, ``config/``, the guard's defaults); nothing in a bundle is
authored by hand. The CAD meshes themselves are not in the bundle:
``visual/visual.json`` says, per mesh, whether it ships in this package
(``shipped``), is withheld for licensing and lives in the private assets
repository (``withheld``), or is an optional decorative visual (``optional``).

The digest
----------
``digest = "sha256:" + sha256( for each file sorted by path:
path + "\\n" + sha256_hex(file bytes) + "\\n" )``. ``manifest.json`` itself is
not part of the digest. Generated JSON is written with sorted keys, two-space
indentation, LF line ends and a final newline, so the bytes — and so the
digest — depend only on the content.

``description_version``
-----------------------
A semantic version of the CONTENT, separate from the package version: the
package version moves with every change to the code, the description version
only when a bundled byte changes. It is recorded, with the digest it names, in
the committed lock file ``description/bundle.lock.json`` (``d1``) or
``description/bundle.lock.<model>.json`` (any other model); a test rebuilds
each bundle and fails when its digest is not the locked one. To change the
description: change its source, then ``mkit-description lock
--description-version X.Y.Z`` (refused when the digest changed and the version
did not, or when a version is reused for another digest).
"""
from __future__ import annotations

import argparse
import hashlib
import inspect
import json
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Mapping, Optional, Tuple

from .. import assets
from . import ROOT
from .robots import DEFAULT_MODEL, ROBOT_MODELS, robot_model

#: The bundle format this module writes and reads.
BUNDLE_SCHEMA = "manipulation_kit.description_bundle/1"
#: The hand catalog's bundle path; a bundle without it is refused.
HAND_CATALOG = "hands/catalog.json"
#: The robot model statement every bundle carries.
ROBOT_JSON = "robot.json"
#: Files every ``d1`` bundle must list (besides the per-profile and tool
#: files); :func:`required_files` names any model's.
REQUIRED_FILES = ("kinematics/d1.urdf", "collision/collision.json",
                  "guard/profile.d1.json", HAND_CATALOG, ROBOT_JSON)


def required_files(model: str = DEFAULT_MODEL) -> tuple:
    """The files a bundle of robot model ``model`` must list."""
    m = robot_model(model)
    return ("kinematics/" + m.guard_urdf, "collision/collision.json",
            f"guard/profile.{m.guard_profiles[0]}.json", HAND_CATALOG, ROBOT_JSON)


#: The lock file's format.
LOCK_SCHEMA = "manipulation_kit.description_bundle_lock/1"
#: The committed lock of the ``d1`` bundle: the description version and the
#: digest it names (:func:`lock_path` names any model's).
LOCK_PATH = ROOT / "bundle.lock.json"
#: The robot model a bundle describes when none is named.
ROBOT = DEFAULT_MODEL


def lock_path(model: str = DEFAULT_MODEL) -> Path:
    """The committed lock of robot model ``model``'s bundle."""
    robot_model(model)
    return LOCK_PATH if model == DEFAULT_MODEL else ROOT / f"bundle.lock.{model}.json"
#: The manifest's file name inside a bundle directory (not digested).
MANIFEST_NAME = "manifest.json"

_CONFIG = ROOT.parent / "config"


class BundleError(ValueError):
    """A bundle directory that does not match its manifest, or a bad lock."""

    def __init__(self, problems):
        self.problems = list(problems)
        super().__init__("; ".join(self.problems))


def _json_bytes(obj) -> bytes:
    return (json.dumps(obj, indent=2, sort_keys=True) + "\n").encode("utf-8")


def _floats(text: Optional[str], default: Tuple[float, ...]) -> List[float]:
    if text is None:
        return list(default)
    return [float(v) for v in text.split()]


def _origin(el) -> Dict[str, List[float]]:
    o = el.find("origin")
    return {"xyz": _floats(o.get("xyz") if o is not None else None, (0.0, 0.0, 0.0)),
            "rpy": _floats(o.get("rpy") if o is not None else None, (0.0, 0.0, 0.0))}


def collision_model(urdf_path: Path) -> dict:
    """``urdf_path`` (a primitives-only URDF) as plain data.

    Joints keep the URDF's raw origin (``xyz`` metres, ``rpy`` radians,
    fixed-axis roll-pitch-yaw), axis and limits; collision primitives keep
    their link-local origin and size (box ``[sx, sy, sz]``, cylinder
    ``[radius, length]``, sphere ``[radius]``), in document order."""
    root = ET.parse(str(urdf_path)).getroot()
    joints = []
    for j in root.findall("joint"):
        entry = {"name": j.get("name"), "type": j.get("type"),
                 "parent": j.find("parent").get("link"),
                 "child": j.find("child").get("link"),
                 "origin": _origin(j)}
        if j.get("type") != "fixed":
            a = j.find("axis")
            entry["axis"] = _floats(a.get("xyz") if a is not None else None,
                                    (1.0, 0.0, 0.0))
            lim = j.find("limit")
            if lim is not None and lim.get("lower") is not None:
                entry["limit"] = [float(lim.get("lower")), float(lim.get("upper"))]
        joints.append(entry)
    links = []
    for link in root.findall("link"):
        prims = []
        for c in link.findall("collision"):
            g = c.find("geometry")
            if g is None:
                continue
            kind = g[0].tag
            if kind == "box":
                size = _floats(g[0].get("size"), ())
            elif kind == "cylinder":
                size = [float(g[0].get("radius")), float(g[0].get("length"))]
            elif kind == "sphere":
                size = [float(g[0].get("radius"))]
            else:
                raise BundleError([f"{urdf_path.name}: link {link.get('name')}: "
                                   f"{kind} collision geometry; the bundled "
                                   "collision model is primitives-only"])
            prims.append({"name": c.get("name"), "kind": kind,
                          "origin": _origin(c), "size": size})
        links.append({"name": link.get("name"), "collisions": prims})
    return {"robot": root.get("name"), "source": "kinematics/" + urdf_path.name,
            "units": {"length": "m", "angle": "rad"},
            "joints": joints, "links": links}


#: The guard profiles a daemon selects between, and what selects each.
#:
#: Each is the guard's own defaults (``d1``) with the listed changes. Only
#: ``d1-bench`` loosens anything, and only what a bench is physically
#: missing: a bench carries the two arms on a stand with no torso, head or
#: chassis, so the body checks (body boxes and chest keep-out) would judge a
#: body that is not there. The arm-to-arm and same-arm checks, and every
#: margin, stay as on the robot. ``sim`` reproduces the robot exactly; it
#: differs from ``d1`` only in what selects it.
#:
#: ``selected_by`` states the rule the daemon is to apply: ``d1`` is the real
#: backend's default; ``d1-bench`` only with the real backend AND a
#: ``d1-bench`` record written by the bench host package (never by a
#: configuration file); ``sim`` only with the simulated backend.
GUARD_PROFILES = {
    "d1": {"selected_by": {"backend": "real", "record": None},
           "summary": "the D1 robot: every check on, reference margins",
           "changes": {}},
    "d1-bench": {"selected_by": {"backend": "real", "record": "d1-bench"},
                 "summary": "two D1 arms on a bench stand with no torso, head "
                            "or chassis: body checks off, arm checks as on "
                            "the robot",
                 "changes": {"check_body": False, "chest_keepout": None}},
    "sim": {"selected_by": {"backend": "sim", "record": None},
            "summary": "the simulated D1: identical to d1",
            "changes": {}},
    # The arm + end-effector bench rig as a robot model of its own: there is
    # no body in its URDF, so the body checks have nothing to judge, and the
    # chest keep-out (fixed boxes where a D1's torso is) is off. The arm-arm
    # and same-arm checks and every margin are the robot's. Selected by the
    # model (robot.json's guard_profile); ``selected_by`` keeps the shape a
    # daemon that selects by backend and record parses, and names the real
    # backend with no record.
    "arm-ee-bench": {"selected_by": {"backend": "real", "record": None},
                     "summary": "two D1 arms on a stand with no D1 body: no "
                                "body to keep out, arm checks as on the robot",
                     "changes": {"chest_keepout": None}},
}

#: The profile file format.
GUARD_PROFILE_SCHEMA = "manipulation_kit.guard_profile/1"


def _guard_defaults() -> dict:
    from ..guard import guard as kg  # noqa: PLC0415

    sig = inspect.signature(kg.MotionGuard.__init__).parameters
    keepout = {side: {"lo": list(lo), "hi": list(hi)}
               for side, (lo, hi) in sorted(kg.DEFAULT_CHEST_KEEPOUT.items())}
    return {
        "check_body": sig["check_body"].default,
        "check_arm_arm": sig["check_arm_arm"].default,
        "check_self": sig["check_self"].default,
        "body_margin_m": sig["body_margin_m"].default,
        "arm_arm_margin_m": sig["arm_arm_margin_m"].default,
        "self_margin_m": sig["self_margin_m"].default,
        "body_exempt_links": list(kg.DEFAULT_BODY_EXEMPT_LINKS),
        "disabled_body_boxes": list(kg.DEFAULT_DISABLED_BODY_BOXES),
        "chest_keepout": keepout,
        "chest_exempt_links": list(kg.DEFAULT_CHEST_EXEMPT_LINKS),
    }


def guard_profile(name: str = "d1") -> dict:
    """Guard profile ``name`` (a key of :data:`GUARD_PROFILES`).

    The values start from :mod:`manipulation_kit.guard.guard` itself (its
    module constants and :class:`~manipulation_kit.guard.guard.MotionGuard`'s
    keyword defaults), so ``d1`` cannot disagree with the guard this package
    runs, and the other profiles differ from it only by their listed
    ``changes``. ``daemon_keys`` repeats the margins under the names of the
    daemon's ``[arm]`` configuration keys."""
    from ..guard import guard as kg  # noqa: PLC0415

    try:
        spec = GUARD_PROFILES[name]
    except KeyError:
        raise KeyError(f"unknown guard profile {name!r}; known: "
                       + ", ".join(sorted(GUARD_PROFILES))) from None
    policy = _guard_defaults()
    policy.update(spec["changes"])
    return {
        "schema": GUARD_PROFILE_SCHEMA,
        "profile": name,
        "summary": spec["summary"],
        "selected_by": dict(spec["selected_by"]),
        "units": {"length": "m"},
        "arm_sides": dict(sorted(kg.ARM_SIDES.items())),
        **policy,
        "ee_link_prefixes": list(kg.EE_LINK_PREFIXES),
        "hand_envelope_prefix": kg.HAND_ENVELOPE_PREFIX,
        "daemon_keys": {"arm.guard_body": policy["check_body"],
                        "arm.guard_body_margin_m": policy["body_margin_m"],
                        "arm.guard_arm_arm_margin_m": policy["arm_arm_margin_m"]},
    }


def motion_guard_kwargs(profile: Mapping) -> dict:
    """``MotionGuard(**motion_guard_kwargs(guard_profile(name)))`` builds the
    kit's guard under that profile."""
    keepout = profile["chest_keepout"]
    return {
        "check_body": profile["check_body"],
        "check_arm_arm": profile["check_arm_arm"],
        "check_self": profile["check_self"],
        "body_margin_m": profile["body_margin_m"],
        "arm_arm_margin_m": profile["arm_arm_margin_m"],
        "self_margin_m": profile["self_margin_m"],
        "body_exempt_links": tuple(profile["body_exempt_links"]),
        "disabled_body_boxes": tuple(profile["disabled_body_boxes"]),
        "chest_keepout": None if keepout is None else {
            side: (tuple(b["lo"]), tuple(b["hi"])) for side, b in keepout.items()},
        "chest_exempt_links": tuple(profile["chest_exempt_links"]),
    }


def _license_class(package_rel: str) -> str:
    parent = package_rel.rsplit("/", 1)[0]
    if parent in assets.EXTERNAL_ASSET_DIRS:
        return "withheld"
    if parent == assets.OPTIONAL_VISUAL_DIR:
        return "optional"
    return "shipped"


def visual_refs(urdfs: Mapping[str, Path]) -> dict:
    """Every ``<mesh filename>`` in ``urdfs`` (bundle path -> source URDF),
    resolved package-relative, with its licence class."""
    pkg = assets.PACKAGE_ROOT
    meshes = []
    for bundle_path, src in sorted(urdfs.items()):
        seen = set()
        for m in ET.parse(str(src)).getroot().iter("mesh"):
            ref = m.get("filename")
            if ref in seen:
                continue
            seen.add(ref)
            rel = (src.parent / ref).resolve().relative_to(pkg).as_posix()
            meshes.append({"urdf": bundle_path, "filename": ref,
                           "package_path": rel, "license": _license_class(rel)})
    meshes.sort(key=lambda e: (e["urdf"], e["filename"]))
    return {"license_classes": {
                "shipped": "in this package",
                "withheld": "CAD withheld for licensing; in the private assets "
                            "repository at the same package path",
                "optional": "decorative visual, not shipped; fetched separately"},
            "meshes": meshes}


def bundle_files(model: str = DEFAULT_MODEL) -> Dict[str, bytes]:
    """Robot model ``model``'s bundle files, bundle path -> bytes, built from
    this package."""
    m = robot_model(model)
    guard_urdf = ROOT / "d1" / m.guard_urdf
    view_urdf = ROOT / "d1" / m.view_urdf
    guard_path, view_path = "kinematics/" + m.guard_urdf, "kinematics/" + m.view_urdf
    files: Dict[str, bytes] = {
        guard_path: guard_urdf.read_bytes(),
        view_path: view_urdf.read_bytes(),
        "collision/collision.json": _json_bytes(collision_model(guard_urdf)),
        "poses/home_pose.json": (_CONFIG / "home_pose.json").read_bytes(),
        "visual/visual.json": _json_bytes(visual_refs({
            guard_path: guard_urdf, view_path: view_urdf})),
        ROBOT_JSON: _json_bytes(m.document()),
    }
    for name in m.guard_profiles:
        files[f"guard/profile.{name}.json"] = _json_bytes(guard_profile(name))
    for p in sorted((_CONFIG / "tool_configs").glob("*.json")):
        files[f"end_effectors/tool_configs/{p.name}"] = p.read_bytes()
    files[HAND_CATALOG] = (ROOT.parent / "hands" / "catalog.json").read_bytes()
    return dict(sorted(files.items()))


def file_hashes(files: Mapping[str, bytes]) -> Dict[str, str]:
    return {path: hashlib.sha256(data).hexdigest() for path, data in sorted(files.items())}


def digest_of(hashes: Mapping[str, str]) -> str:
    """The bundle digest over ``{path: sha256_hex}`` (see the module docstring)."""
    h = hashlib.sha256()
    for path in sorted(hashes):
        h.update(f"{path}\n{hashes[path]}\n".encode("utf-8"))
    return "sha256:" + h.hexdigest()


def digest(model: str = DEFAULT_MODEL) -> str:
    """The digest of robot model ``model``'s bundle as this package builds it
    now."""
    return digest_of(file_hashes(bundle_files(model)))


def read_lock(path: Path = LOCK_PATH) -> dict:
    lock = json.loads(Path(path).read_text(encoding="utf-8"))
    if lock.get("schema") != LOCK_SCHEMA:
        raise BundleError([f"{path}: schema must be {LOCK_SCHEMA!r}"])
    return lock


def description_version(model: str = DEFAULT_MODEL) -> str:
    """The locked ``description_version`` of robot model ``model``'s bundle."""
    return read_lock(lock_path(model))["description_version"]


def manifest(files: Optional[Mapping[str, bytes]] = None,
             version: Optional[str] = None,
             model: str = DEFAULT_MODEL) -> dict:
    """The ``manifest.json`` of robot model ``model``'s bundle built from
    ``files``."""
    files = bundle_files(model) if files is None else files
    hashes = file_hashes(files)
    return {
        "schema": BUNDLE_SCHEMA,
        "model": model,
        "description_version": version or description_version(model),
        "digest": digest_of(hashes),
        "files": [{"path": p, "sha256": hashes[p], "bytes": len(files[p])}
                  for p in sorted(files)],
    }


def write_bundle(out_dir, *, force: bool = False,
                 model: str = DEFAULT_MODEL) -> dict:
    """Write robot model ``model``'s bundle into ``out_dir`` (created; must be
    empty unless ``force``) and return its manifest."""
    out = Path(out_dir)
    if out.exists() and any(out.iterdir()) and not force:
        raise BundleError([f"{out} is not empty (pass force to overwrite)"])
    files = bundle_files(model)
    man = manifest(files, model=model)
    for path, data in files.items():
        dest = out / path
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_bytes(data)
    (out / MANIFEST_NAME).write_bytes(_json_bytes(man))
    return man


def verify_bundle(bundle_dir) -> List[str]:
    """Problems with the bundle in ``bundle_dir``; empty when it is intact.

    Checks the schema, that every listed file exists with its SHA-256, that
    no unlisted file is present, and that the digest is the digest of the
    listed hashes."""
    root = Path(bundle_dir)
    try:
        man = json.loads((root / MANIFEST_NAME).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return [f"{root / MANIFEST_NAME}: {exc}"]
    problems = []
    if man.get("schema") != BUNDLE_SCHEMA:
        problems.append(f"schema {man.get('schema')!r} is not {BUNDLE_SCHEMA!r}")
    listed = {}
    for entry in man.get("files", []):
        path, want = entry.get("path"), entry.get("sha256")
        listed[path] = want
        f = root / path
        if not f.is_file():
            problems.append(f"{path}: listed but missing")
            continue
        got = hashlib.sha256(f.read_bytes()).hexdigest()
        if got != want:
            problems.append(f"{path}: sha256 {got} != manifest {want}")
    present = {p.relative_to(root).as_posix() for p in root.rglob("*")
               if p.is_file()} - {MANIFEST_NAME}
    for extra in sorted(present - set(listed)):
        problems.append(f"{extra}: present but not in the manifest")
    model = man.get("model")
    if model not in ROBOT_MODELS:
        problems.append(f"model {model!r} is not a known robot model "
                        f"({', '.join(sorted(ROBOT_MODELS))})")
        model = DEFAULT_MODEL
    for required in required_files(model):
        if required not in listed:
            problems.append(f"{required}: required but not in the manifest")
    if digest_of(listed) != man.get("digest"):
        problems.append(f"digest {man.get('digest')} is not the digest of the "
                        f"listed files ({digest_of(listed)})")
    return problems


@dataclass(frozen=True)
class Bundle:
    """A verified bundle directory."""

    root: Path
    manifest: dict

    @property
    def digest(self) -> str:
        return self.manifest["digest"]

    @property
    def description_version(self) -> str:
        return self.manifest["description_version"]

    @property
    def model(self) -> str:
        """The robot model the bundle describes."""
        return self.manifest["model"]

    def robot(self) -> dict:
        """``robot.json``, schema-checked and naming the manifest's model."""
        from .robots import ROBOT_SCHEMA  # noqa: PLC0415

        doc = self.read_json(ROBOT_JSON)
        if doc.get("schema") != ROBOT_SCHEMA or doc.get("model") != self.model:
            raise BundleError([f"{ROBOT_JSON}: schema {doc.get('schema')!r} "
                               f"model {doc.get('model')!r} is not "
                               f"{ROBOT_SCHEMA!r} {self.model!r}"])
        return doc

    def path(self, bundle_path: str) -> Path:
        if bundle_path not in {e["path"] for e in self.manifest["files"]}:
            raise KeyError(bundle_path)
        return self.root / bundle_path

    def read_json(self, bundle_path: str):
        return json.loads(self.path(bundle_path).read_text(encoding="utf-8"))

    def hand_catalog(self) -> dict:
        """``hands/catalog.json``, schema-checked."""
        from ..hands.catalog import CATALOG_SCHEMA  # noqa: PLC0415

        doc = self.read_json(HAND_CATALOG)
        if doc.get("schema") != CATALOG_SCHEMA:
            raise BundleError([f"{HAND_CATALOG}: schema {doc.get('schema')!r} "
                               f"is not {CATALOG_SCHEMA!r}"])
        return doc


def load_bundle(bundle_dir) -> Bundle:
    """The bundle in ``bundle_dir``, verified; :class:`BundleError` if not."""
    problems = verify_bundle(bundle_dir)
    if problems:
        raise BundleError(problems)
    root = Path(bundle_dir)
    return Bundle(root, json.loads((root / MANIFEST_NAME).read_text(encoding="utf-8")))


def lock_problems(lock: dict, built_digest: str) -> List[str]:
    """Why ``lock`` does not describe a bundle whose digest is ``built_digest``."""
    problems = []
    history = lock.get("history", {})
    if lock.get("digest") != built_digest:
        problems.append(
            f"the bundle's digest is {built_digest}, the lock names "
            f"{lock.get('digest')} for description_version "
            f"{lock.get('description_version')}: run `mkit-description lock "
            "--description-version <new>`")
    if history.get(lock.get("description_version")) != lock.get("digest"):
        problems.append("the lock's history does not record its own version")
    if len(set(history.values())) != len(history):
        problems.append("the lock's history names one digest under two versions")
    return problems


def update_lock(new_version: Optional[str], path: Optional[Path] = None,
                model: str = DEFAULT_MODEL) -> dict:
    """Record robot model ``model``'s current digest in its lock under
    ``new_version``."""
    path = lock_path(model) if path is None else path
    built = digest(model)
    lock = read_lock(path) if Path(path).exists() else {
        "schema": LOCK_SCHEMA, "model": model, "history": {}}
    history = dict(lock.get("history", {}))
    version = new_version or lock.get("description_version")
    if version is None:
        raise BundleError(["no description_version: pass one"])
    if history.get(version, built) != built:
        raise BundleError([f"description_version {version} already names "
                           f"{history[version]}; a changed description needs a "
                           "new version"])
    for v, d in history.items():
        if d == built and v != version:
            raise BundleError([f"this content is already description_version {v}"])
    history[version] = built
    lock.update({"description_version": version, "digest": built,
                 "files": file_hashes(bundle_files(model)), "history": history})
    Path(path).write_bytes(_json_bytes(lock))
    return lock


# ---------------------------------------------------------------- CLI --
def main(argv=None) -> int:
    """``mkit-description`` — build, digest and verify the description bundle.

    mkit-description build --out DIR [--force]   write the bundle directory
    mkit-description digest                       print the digest
    mkit-description check                        the build matches the lock
    mkit-description verify DIR [--expect-digest D]   check a bundle directory
    mkit-description lock --description-version V     record a new digest
    mkit-description models                       list the robot models

    ``--model <id>`` (before the command) selects the robot model; ``d1`` by
    default. ``check`` checks every model's bundle against its lock.
    """
    ap = argparse.ArgumentParser(prog="mkit-description",
                                 description=main.__doc__.split("\n")[0])
    ap.add_argument("--model", default=DEFAULT_MODEL, choices=sorted(ROBOT_MODELS))
    sub = ap.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("build", help="write the bundle into a directory")
    b.add_argument("--out", required=True)
    b.add_argument("--force", action="store_true")
    sub.add_parser("digest", help="print the bundle digest and version")
    sub.add_parser("check", help="fail when the bundle differs from the lock")
    v = sub.add_parser("verify", help="verify a bundle directory")
    v.add_argument("dir")
    v.add_argument("--expect-digest", default=None)
    lk = sub.add_parser("lock", help="record the current digest (maintainers)")
    lk.add_argument("--description-version", default=None)
    sub.add_parser("models", help="list the robot models")
    args = ap.parse_args(argv)
    model = args.model

    try:
        if args.cmd == "build":
            man = write_bundle(args.out, force=args.force, model=model)
            print(f"wrote {len(man['files'])} files to {args.out}: "
                  f"description_version {man['description_version']} {man['digest']}")
        elif args.cmd == "digest":
            print(f"{description_version(model)} {digest(model)}")
        elif args.cmd == "check":
            failed = False
            for name in sorted(ROBOT_MODELS):
                problems = lock_problems(read_lock(lock_path(name)), digest(name))
                for p in problems:
                    print(f"mkit-description: {name}: {p}", file=sys.stderr)
                failed = failed or bool(problems)
                if not problems:
                    print(f"{name}: bundle matches the lock: "
                          f"{description_version(name)} {digest(name)}")
            if failed:
                return 1
        elif args.cmd == "verify":
            problems = verify_bundle(args.dir)
            if not problems and args.expect_digest:
                got = json.loads((Path(args.dir) / MANIFEST_NAME).read_text())["digest"]
                if got != args.expect_digest:
                    problems.append(f"digest {got} != expected {args.expect_digest}")
            for p in problems:
                print(f"mkit-description: {p}", file=sys.stderr)
            if problems:
                return 1
            print(f"{args.dir}: intact")
        elif args.cmd == "lock":
            lock = update_lock(args.description_version, model=model)
            print(f"locked description_version {lock['description_version']} "
                  f"{lock['digest']}")
        elif args.cmd == "models":
            for name, m in sorted(ROBOT_MODELS.items()):
                print(f"{name}\t{m.summary}")
    except BundleError as exc:
        for p in exc.problems:
            print(f"mkit-description: {p}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
