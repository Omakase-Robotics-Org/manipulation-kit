"""The hand catalog: one index over every end effector this package knows.

What already exists for a model is spread over the tree: its joint table
(``hands/<maker>/<model>/axes.py``), its retarget map and the committed table
of it (``retarget.py``, ``retarget_table.json``), its exported tool config
(``config/tool_configs/<maker>_<model>.json``). :func:`catalog` lists them per
model id, and adds the one thing a safety check needs that no file held for
most hands: the hand's ENVELOPE, the capsules (in the flange frame: +z the
approach axis) that stand for the hand when an arm guard checks it against
the robot's body and the other arm.

Envelopes
---------
An envelope's id is derived from its capsules (``capsules-`` + the first 16
hex digits of the SHA-256 of their canonical JSON), so the id names exactly
one shape, carries no vendor or model name, and two models with the same
shape share it. A consumer that receives an id can check it against the
capsules it holds.

``verified`` says whether the capsules were checked against the hand's own
geometry. Only the parallel gripper's are (they cover every collision box of
its description; see ``HAND_ENVELOPE`` in the URDF generator, which this
module reads rather than copies). Every five-finger hand gets
:data:`FIVE_FINGER_DEFAULT`, a deliberately large capsule that was NOT
measured on any of them, and is marked ``verified = False``: a consumer that
enforces safety should treat an unverified envelope as an unknown hand
(refuse, or widen its margins), not as a measurement.

Model ids
---------
The id is ``"<maker>/<model>"``, the package path. :data:`MODEL_ALIASES` maps
spellings seen in the field to the canonical id; :func:`canonical_model`
resolves one (and refuses an id that is neither).

The catalog is committed as ``hands/catalog.json`` (``mkit-toolconfig
catalog``) and ships in the wheel; :func:`load` and :func:`entry` read it.
A test fails when it differs from what :func:`build` computes.
"""
from __future__ import annotations

import hashlib
import importlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

#: The ``schema`` string of the serialized catalog.
CATALOG_SCHEMA = "manipulation_kit.hand_catalog.v1"

#: The frame every envelope is written in.
ENVELOPE_FRAME = "flange"

Capsule = Tuple[str, Tuple[float, float, float], Tuple[float, float, float], float]


@dataclass(frozen=True)
class Envelope:
    """A hand's protrusion as capsules ``(part, a, b, radius)``, metres."""

    capsules: Tuple[Capsule, ...]
    verified: bool
    #: Where the numbers come from, one sentence.
    source: str

    @property
    def id(self) -> str:
        return "capsules-" + self.digest()[:16]

    def digest(self) -> str:
        doc = [[p, list(a), list(b), r] for p, a, b, r in self.capsules]
        text = json.dumps(doc, sort_keys=True, separators=(",", ":"))
        return hashlib.sha256(text.encode("utf-8")).hexdigest()

    def to_json(self) -> Dict[str, object]:
        return {"id": self.id, "digest": self.digest(), "frame": ENVELOPE_FRAME,
                "verified": self.verified, "source": self.source,
                "capsules": [{"part": p, "a": list(a), "b": list(b), "radius": r}
                             for p, a, b, r in self.capsules]}


def _gripper_envelope() -> Envelope:
    from ..description.d1.tools.generate_d1_urdf import HAND_ENVELOPE  # noqa: PLC0415

    return Envelope(tuple((p, tuple(a), tuple(b), r) for p, a, b, r in HAND_ENVELOPE),
                    True, "covers every collision box of the gripper description, "
                    "jaws over their whole travel (HAND_ENVELOPE in the D1 URDF generator)")


#: The envelope of every five-finger hand until one is measured: one capsule
#: along the approach axis from 0.04 to 0.20 m, radius 0.07 m, i.e. a palm up
#: to 140 mm across reaching 270 mm from the flange. NOT measured on any hand.
FIVE_FINGER_DEFAULT = Envelope(
    (("hand", (0.0, 0.0, 0.04), (0.0, 0.0, 0.20), 0.07),), False,
    "a conservative bound, not measured on the hand: a palm up to 140 mm "
    "across, reaching 270 mm from the flange")


@dataclass(frozen=True)
class HandEntry:
    """What the package holds for one model."""

    model: str
    #: ``"gripper"`` or ``"hand"``.
    kind: str
    #: Number of joints a provider declares (``None`` = not tabled here).
    joints: Optional[int]
    #: The retarget map's table, package-relative, or ``None`` (no map).
    retarget_table: Optional[str]
    #: The exported tool config, package-relative, or ``None`` (none measured).
    tool_config: Optional[str]
    envelope: Envelope

    def to_json(self) -> Dict[str, object]:
        return {"model": self.model, "kind": self.kind, "joints": self.joints,
                "retarget_table": self.retarget_table, "tool_config": self.tool_config,
                "envelope": self.envelope.id}


#: The models, in id order: (id, kind, has a version-2 retarget map).
MODELS: Tuple[Tuple[str, str, bool], ...] = (
    ("d1/parallel_gripper", "gripper", False),
    ("leadshine/dh116s", "hand", True),
    ("linkerbot/o30", "hand", False),
    ("robotera/xhand1", "hand", True),
    ("robotera/xhand1_lite", "hand", True),
    ("wuji/wujihand2", "hand", True),
)

#: Other spellings of a model id seen in the field -> the canonical id.
MODEL_ALIASES: Dict[str, str] = {"wuji/hand2": "wuji/wujihand2"}

_PKG = Path(__file__).resolve().parents[1]


def canonical_model(model: str) -> str:
    """``model`` or its canonical id; ``ValueError`` if neither is catalogued."""
    known = {m for m, _, _ in MODELS}
    if model in known:
        return model
    if model in MODEL_ALIASES:
        return MODEL_ALIASES[model]
    raise ValueError(f"no hand model {model!r} in the catalog "
                     f"(known: {sorted(known)}, aliases: {sorted(MODEL_ALIASES)})")


def _joint_count(model: str) -> Optional[int]:
    maker, _, name = model.partition("/")
    try:
        axes = importlib.import_module(f"manipulation_kit.hands.{maker}.{name}.axes")
    except ImportError:
        return None
    joints = getattr(axes, "JOINTS", None)
    if joints is not None:
        return len(joints)
    return getattr(axes, "NUM_AXES", None)


def _rel(path: Path) -> Optional[str]:
    return path.relative_to(_PKG).as_posix() if path.exists() else None


def catalog() -> Tuple[HandEntry, ...]:
    """Every catalogued model, in id order."""
    gripper = _gripper_envelope()
    out: List[HandEntry] = []
    for model, kind, has_map in MODELS:
        maker, _, name = model.partition("/")
        table = _PKG / "hands" / maker / name / "retarget_table.json"
        tool = _PKG / "config" / "tool_configs" / f"{maker}_{name}.json"
        out.append(HandEntry(
            model, kind, None if kind == "gripper" else _joint_count(model),
            _rel(table) if has_map else None, _rel(tool),
            gripper if kind == "gripper" else FIVE_FINGER_DEFAULT))
    return tuple(out)


def envelopes() -> Dict[str, Envelope]:
    """Every envelope :func:`catalog` uses, by id (source checkout only)."""
    return {e.envelope.id: e.envelope for e in catalog()}


def build() -> Dict[str, object]:
    """The catalog document, computed from the code. Needs a source checkout
    (it reads the gripper's capsules from the URDF generator, which is not
    part of the wheel); an installed package reads :func:`load` instead."""
    entries = catalog()
    envs = envelopes()
    return {"schema": CATALOG_SCHEMA,
            "models": [e.to_json() for e in entries],
            "aliases": dict(sorted(MODEL_ALIASES.items())),
            "envelopes": [env.to_json() for _, env in sorted(envs.items())]}


def dumps() -> str:
    """Canonical text of the catalog (what ``hands/catalog.json`` holds)."""
    return json.dumps(build(), indent=2, sort_keys=True) + "\n"


#: The committed catalog.
CATALOG_FILE = Path(__file__).resolve().parent / "catalog.json"


def load() -> Dict[str, object]:
    """The committed catalog document (works from an installed wheel)."""
    doc = json.loads(CATALOG_FILE.read_text(encoding="utf-8"))
    if doc.get("schema") != CATALOG_SCHEMA:
        raise ValueError(f"{CATALOG_FILE}: schema must be {CATALOG_SCHEMA!r}")
    return doc


def entry(model: str) -> Dict[str, object]:
    """The committed entry of ``model`` (an alias resolves), with its
    envelope document inlined under ``"envelope"``."""
    model = canonical_model(model)
    doc = load()
    envs = {e["id"]: e for e in doc["envelopes"]}
    found = next(e for e in doc["models"] if e["model"] == model)
    return {**found, "envelope": envs[found["envelope"]]}
