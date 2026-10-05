"""Per-binding overrides of a hand's ``RetargetConfig``, validated at load.

A whole-hand bind may carry a ``retarget_config`` table. Its keys are field
names of the resolved hand map's ``RetargetConfig`` dataclass
(``manipulation_kit.hands.<maker>.<model>.retarget.RetargetConfig``); every
field not named keeps the hand's default. :func:`build_retarget_config`
checks each value against the TYPE OF THAT FIELD'S DEFAULT, builds the
config (so the dataclass's own checks run too), and returns it ready for
``get_retarget(model, config=...)``.

Value shapes, by the default's type:

=================  ==========================================================
default            TOML value
=================  ==========================================================
``float``          a number (an integer is accepted; ``true`` is not)
``int``            an integer
``bool``           ``true`` / ``false``
``str``            a string. A field whose name ends in ``_channel`` must be a
                   glove channel name (:mod:`manipulation_kit.gloves.channels`)
``tuple``          an array of the SAME LENGTH, each element of the type of
                   the default's element (``invert = [false, true, ...]``)
``dict`` of        a table of joint name -> partial joint map; named keys
``JointMap``       override that joint's default map, the rest are kept::

                       [bind.retarget_config.joints.thumb_rota1]
                       sources = { thumb_mp_pitch = 1.0 }   # channel = weight
                       lo = 0.12
                       hi = 1.29
                       invert = false
=================  ==========================================================

``sources`` REPLACES the joint's source list (it is not merged channel by
channel), so the table says the whole weighted mean.
"""
from __future__ import annotations

import dataclasses
import importlib
from typing import Any, Dict, List, Mapping, Tuple

from ..gloves.channels import spec_of


def _is_number(v: Any) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _check_scalar(default: Any, value: Any, where: str, problems: List[str]):
    """``value`` coerced to ``default``'s type, or ``None`` with a problem."""
    if isinstance(default, bool):
        if isinstance(value, bool):
            return value
        problems.append(f"{where}: expected true/false, got {value!r}")
    elif isinstance(default, int):
        if isinstance(value, int) and not isinstance(value, bool):
            return value
        problems.append(f"{where}: expected an integer, got {value!r}")
    elif isinstance(default, float):
        if _is_number(value):
            return float(value)
        problems.append(f"{where}: expected a number, got {value!r}")
    elif isinstance(default, str):
        if isinstance(value, str):
            return value
        problems.append(f"{where}: expected a string, got {value!r}")
    else:
        problems.append(f"{where}: this field cannot be set from a binding file")
    return None


def _joint_maps(default: Dict[str, Any], value: Any, where: str,
                problems: List[str]) -> Dict[str, Any]:
    out = dict(default)
    if not isinstance(value, Mapping):
        problems.append(f"{where}: expected a table of joint name -> joint map")
        return out
    for joint, raw in value.items():
        w = f"{where}.{joint}"
        if joint not in default:
            problems.append(f"{w}: no such joint (joints: {', '.join(default)})")
            continue
        if not isinstance(raw, Mapping):
            problems.append(f"{w}: expected a table with sources/lo/hi/invert")
            continue
        base = default[joint]
        changes: Dict[str, Any] = {}
        for key, v in raw.items():
            if key == "sources":
                if not isinstance(v, Mapping) or not v:
                    problems.append(f"{w}.sources: expected a non-empty table "
                                    "of channel = weight")
                    continue
                pairs = []
                for ch, weight in v.items():
                    if spec_of(ch) is None:
                        problems.append(f"{w}.sources: {ch!r} is not a glove channel")
                    elif not _is_number(weight):
                        problems.append(f"{w}.sources.{ch}: weight must be a number")
                    else:
                        pairs.append((ch, float(weight)))
                changes["sources"] = tuple(pairs)
            elif key in ("lo", "hi", "invert"):
                got = _check_scalar(getattr(base, key), v, f"{w}.{key}", problems)
                if got is not None:
                    changes[key] = got
            else:
                problems.append(f"{w}: unknown key {key!r} (sources, lo, hi, invert)")
        out[joint] = dataclasses.replace(base, **changes)
    return out


def config_class(model: str):
    """The ``RetargetConfig`` dataclass of a ``"<maker>/<model>"`` hand map."""
    maker, _, name = model.partition("/")
    mod = importlib.import_module(f"manipulation_kit.hands.{maker}.{name}.retarget")
    cls = getattr(mod, "RetargetConfig", None)
    if cls is None or not dataclasses.is_dataclass(cls):
        raise ValueError(f"{model} has no RetargetConfig dataclass")
    return cls


def build_retarget_config(model: str, overrides: Mapping[str, Any],
                          where: str = "retarget_config") -> Tuple[Any, List[str]]:
    """``(config, problems)``: ``model``'s ``RetargetConfig`` with
    ``overrides`` applied, or ``(None, [every problem])``."""
    problems: List[str] = []
    try:
        cls = config_class(model)
    except (ImportError, ValueError) as exc:
        return None, [f"{where}: {exc}"]
    defaults = cls()
    fields = {f.name for f in dataclasses.fields(cls) if f.init}
    kwargs: Dict[str, Any] = {}
    for key, value in overrides.items():
        w = f"{where}.{key}"
        if key not in fields:
            problems.append(f"{w}: {model} RetargetConfig has no field {key!r} "
                            f"(fields: {', '.join(sorted(fields))})")
            continue
        default = getattr(defaults, key)
        if isinstance(default, dict):
            kwargs[key] = _joint_maps(default, value, w, problems)
        elif isinstance(default, tuple):
            if not isinstance(value, list) or len(value) != len(default):
                problems.append(f"{w}: expected an array of {len(default)} values")
                continue
            items = [_check_scalar(d, v, f"{w}[{i}]", problems)
                     for i, (d, v) in enumerate(zip(default, value))]
            if all(x is not None for x in items):
                kwargs[key] = tuple(items)
        else:
            got = _check_scalar(default, value, w, problems)
            if got is None:
                continue
            if key.endswith("_channel") and spec_of(got) is None:
                problems.append(f"{w}: {got!r} is not a glove channel")
                continue
            kwargs[key] = got
    if problems:
        return None, problems
    try:
        return cls(**kwargs), []
    except (ValueError, TypeError) as exc:
        return None, [f"{where}: {exc}"]
