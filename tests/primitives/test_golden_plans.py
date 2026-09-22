"""Golden plans: the alias directions reproduce the pre-Direction plans.

``tests/data/golden_plans/pre_direction_e1dce97.json`` was captured on
``feat/perceive-head`` @ ``e1dce97`` — before 0.16.0 replaced
``approach: str`` + ``jaw_turn_deg`` with ``direction: Direction`` +
``roll_rad`` — by planning every verb the example scenes exercise with the
OLD API (each approach name x each jaw turn, both arms, the whole
Approach->Place chain, the corrections) and recording the plans. Each case is
stored with the arguments it was captured with (``captured_as``) and the
arguments the NEW API takes for the same request (``args``: the alias with the
same vector, ``roll_rad = radians(jaw_turn_deg)``).

This test replays every case with the new API and requires the waypoints,
the joint path (per-waypoint step count, last posture and posture sum), the
gripper strokes and the refusals to agree to 1e-9. It is the evidence that
the vocabulary change changed no motion.

Deliberately updating it (a later step that changes grasp geometry on
purpose): ``python tests/primitives/test_golden_plans.py --regenerate`` writes
the current plans into the file; review the diff and say why in the commit.

The old names map onto aliases BY VECTOR (``alias_mapping`` in the file): the
old side names said where the hand came FROM, a Direction says where it
TRAVELS, so ``side_left`` (in from the robot's left, travelling -y) is
``right``.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import pytest

GOLDEN = (Path(__file__).resolve().parents[1] / "data" / "golden_plans"
          / "pre_direction_e1dce97.json")
TOL = 1e-9
#: notes the new API adds that the old one did not (the planner reporting the
#: roll it used, since a model no longer sets one)
NEW_NOTE_PREFIXES = ("jaws rolled ",)


# --------------------------------------------------------------------------- #
# replay
# --------------------------------------------------------------------------- #

def _objects(items):
    from scipy.spatial.transform import Rotation as R

    from manipulation_kit.world import ContainerView, ObjectView, SurfaceView
    kinds = {"object": ObjectView, "container": ContainerView,
             "surface": SurfaceView}
    out = []
    for item in items:
        kind = kinds[item.get("kind", "object")]
        extra = {}
        if kind is ContainerView and "interior" in item:
            extra["interior"] = item["interior"]
        if kind is ContainerView and "rim_height_m" in item:
            extra["rim_height_m"] = item["rim_height_m"]
        out.append(kind(item["name"], p=item["p"], size=item["size"],
                        r=R.from_euler("z", float(item.get("yaw_rad", 0.0))),
                        frame_id=item.get("frame_id", "base"),
                        colour=item.get("colour"), **extra))
    return out


def _world(kin, items):
    from manipulation_kit.primitives.orientation import tool_from_link7
    from manipulation_kit.world import ArmView, GripperView, WorldView
    arms, grippers = [], []
    for side in ("left", "right"):
        p, r = tool_from_link7(*kin.ee_pose(side))
        arms.append(ArmView(side, joints=kin.joints(side), tool_p=p, tool_r=r,
                            mode="position"))
        grippers.append(GripperView(side, 0.0, holding=False, jaw_gap_m=0.04))
    return WorldView.of(_objects(items), arms=arms, grippers=grippers)


def _steps(steps):
    from manipulation_kit.primitives.types import GripStep, JointStep, SettleStep
    out = []
    for s in steps:
        if isinstance(s, JointStep):
            q = [float(v) for v in s.q]
            last = out[-1] if out else None
            if (last is not None and "joints" in last
                    and last["joints"] == s.side
                    and last["waypoint"] == int(s.waypoint)):
                last["n"] += 1
                last["q_last"] = q
                last["q_sum"] = [a + b for a, b in zip(last["q_sum"], q)]
            else:
                out.append({"joints": s.side, "waypoint": int(s.waypoint),
                            "n": 1, "q_last": q, "q_sum": list(q)})
        elif isinstance(s, GripStep):
            out.append({"grip": s.side, "closedness": float(s.closedness),
                        "preset": s.grip, "waypoint": int(s.waypoint)})
        elif isinstance(s, SettleStep):
            out.append({"settle": float(s.timeout_s)})
    return out


def _result(result):
    if not getattr(result, "ok", False):
        return {"kind": "refusal", "reason": result.reason,
                "waypoint_index": result.waypoint_index,
                "waypoint_label": result.waypoint_label,
                "residual_m": (None if not math.isfinite(result.residual_m)
                               else float(result.residual_m)),
                "stage": result.stage, "side": result.side,
                "unmet": [u.code for u in result.unmet]}
    return {"kind": "plan", "primitive": result.primitive, "side": result.side,
            "waypoints": [{"label": w.label, "p": [float(v) for v in w.p],
                           "quat_xyzw": [float(v) for v in w.r.as_quat()],
                           "allow_via": bool(w.allow_via),
                           "arrive": bool(w.arrive)} for w in result.waypoints],
            "steps": _steps(result.steps),
            "notes": [n for n in result.notes
                      if not n.startswith(NEW_NOTE_PREFIXES)]}


def replay(case, kin, world):
    from manipulation_kit.primitives import reach
    from manipulation_kit.primitives.verbs import BY_VERB
    args = dict(case["args"])
    if case["verb"] == "chain":
        chain = reach.plan_chain(world, kin, **args)
        return {"kind": "chain", "side": chain.side,
                "links": [{"verb": link.verb, "result": _result(link.result)}
                          for link in chain.links]}
    return _result(BY_VERB[case["verb"]](**args).plan(world, kin))


# --------------------------------------------------------------------------- #
# comparison
# --------------------------------------------------------------------------- #

def _close(a, b, path, errors, tol=TOL):
    if isinstance(a, dict) and isinstance(b, dict):
        if sorted(a) != sorted(b):
            errors.append(f"{path}: keys {sorted(a)} != {sorted(b)}")
            return
        for key in a:
            scale = max(1, int(a.get("n", 1))) if key == "q_sum" else 1
            _close(a[key], b[key], f"{path}.{key}", errors, tol * scale)
        return
    if isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            errors.append(f"{path}: {len(a)} items != {len(b)}")
            return
        for i, (x, y) in enumerate(zip(a, b)):
            _close(x, y, f"{path}[{i}]", errors, tol)
        return
    if isinstance(a, float) or isinstance(b, float):
        if a is None or b is None or not math.isclose(float(a), float(b),
                                                      rel_tol=0.0, abs_tol=tol):
            errors.append(f"{path}: {a!r} != {b!r}")
        return
    if a != b:
        errors.append(f"{path}: {a!r} != {b!r}")


def _load():
    return json.loads(GOLDEN.read_text(encoding="utf-8"))


def test_alias_directions_reproduce_the_pre_direction_plans(d1_arm):
    golden = _load()
    worlds = {name: _world(d1_arm, items)
              for name, items in golden["scenes"].items()}
    planned = sum(1 for c in golden["cases"] if c["result"]["kind"] != "refusal")
    assert len(golden["cases"]) >= 200 and planned >= 75, \
        "the golden set shrank; it is the evidence, not a sample"
    failures = []
    for index, case in enumerate(golden["cases"]):
        errors = []
        _close(case["result"], replay(case, d1_arm, worlds[case["scene"]]),
               f"case {index} {case['scene']}/{case['verb']} {case['args']}",
               errors)
        failures += errors[:3]
    assert not failures, "\n".join(failures[:30])


def test_the_golden_mapping_is_by_vector_not_by_name():
    """The alias each old name became has the old name's TRAVEL vector."""
    from manipulation_kit.world import ALIASES
    old_vectors = {"top_down": (0, 0, -1), "front": (1, 0, 0),
                   "side_left": (0, -1, 0), "side_right": (0, 1, 0)}
    mapping = _load()["alias_mapping"]
    for old, alias in mapping.items():
        assert ALIASES[alias].frame == "base"
        assert ALIASES[alias].v == pytest.approx(old_vectors[old]), old


# --------------------------------------------------------------------------- #
# deliberate regeneration
# --------------------------------------------------------------------------- #

def _regenerate() -> None:  # pragma: no cover - maintenance entry point
    from manipulation_kit.arms import get_arm_kinematics

    def rounded(x):
        if isinstance(x, float):
            return float(f"{x:.13g}")
        if isinstance(x, list):
            return [rounded(v) for v in x]
        if isinstance(x, dict):
            return {k: rounded(v) for k, v in x.items()}
        return x

    golden = _load()
    kin = get_arm_kinematics("d1/arm", quiet=True)
    for side in ("left", "right"):
        kin.set_joints(side, kin.home(side))
    worlds = {name: _world(kin, items)
              for name, items in golden["scenes"].items()}
    for case in golden["cases"]:
        case["result"] = rounded(replay(case, kin, worlds[case["scene"]]))
    head = {k: v for k, v in golden.items() if k not in ("scenes", "cases")}
    with GOLDEN.open("w", encoding="utf-8") as f:
        f.write("{" + ",\n".join(f"{json.dumps(k)}:{json.dumps(v)}"
                                 for k, v in head.items()) + ",\n")
        f.write('"scenes":' + json.dumps(golden["scenes"],
                                         separators=(",", ":")) + ",\n")
        f.write('"cases":[\n')
        f.write(",\n".join(json.dumps(c, separators=(",", ":"))
                           for c in golden["cases"]))
        f.write("\n]}\n")
    print(f"rewrote {GOLDEN} ({len(golden['cases'])} cases)")


if __name__ == "__main__":  # pragma: no cover
    if "--regenerate" in sys.argv:
        _regenerate()
