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

WHERE 1e-9 HOLDS. The joint path is the output of an iterative IK solve, and
its last bits depend on the numeric stack it ran on: the same code planned
with numpy 2.0.2 / scipy 1.13.1 (CI's Python 3.9 job) takes 26 knots where
the capture took 18 on one chain (case 86), and on another CI runner a nudge
that the guard refuses at its first knot here walks 15 mm first (case 45).
Both are knife edges of the solver, not of the vocabulary: e1dce97 itself,
captured on numpy 2.0.2 / scipy 1.13.1, disagrees with this file on 14 of
the 229 cases and the new API reproduces THAT capture to 1e-9 as well. So
the file records the stack it was captured on (``numeric_stack``) and

* on that stack every field is compared at 1e-9 (the full proof);
* elsewhere every verdict, refusal reason, side, stage, label, stroke and
  note must still be IDENTICAL, waypoints agree to ``PORTABLE_TOL`` (a later
  link of a chain starts from the posture the solver reached, so its
  waypoints inherit the solver's last bits: 7e-5 m on case 86), and the
  solver's own path is compared by its final posture only, to
  ``PORTABLE_Q_TOL`` (knot counts, posture sums and a refusal's residual are
  the solver's, not the plan's). A vocabulary mistake — a mirrored axis, a
  90 deg roll — moves these by centimetres and quarter turns.

THE SCENE GATE (0.16.0, step 5) is switched off for that proof: it is a
proof about the VOCABULARY, and the gate is a new check that refuses some of
these plans on purpose. A second test replays every case with the gate on and
requires every case to be unchanged EXCEPT the ones listed, by number, in
:data:`SCENE_REFUSED` — each a horizontal approach whose arm comes within
11-15 mm of a declared table or box, inside the 15 mm a declared obstacle
requires (10 mm margin + 5 mm sampling allowance). Those become
``guard_reject`` refusals naming the obstacle. No top-down case changes.

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
#: off the capture stack: waypoints (m, quaternion components) ...
PORTABLE_TOL = 1e-3
#: ... and the solver's final posture (rad)
PORTABLE_Q_TOL = 1e-2
#: fields that are the IK solver's path rather than the plan's geometry
SOLVER_PATH_KEYS = ("n", "q_sum", "residual_m")
#: notes the new API adds that the old one did not (the planner reporting the
#: roll it used, since a model no longer sets one)
NEW_NOTE_PREFIXES = ("jaws rolled ",)

#: The cases the scene gate (0.16.0, step 5) changes, BY NUMBER, and the
#: obstacle each refusal must name. Measured on the capture stack: all are
#: ``direction: forward`` (for a chain, its approach link), all were plans
#: at e1dce97, and in every one the arm's closest link (Link4 = forearm,
#: Link7 = wrist) comes 11-15 mm from the named obstacle — inside the 15 mm
#: a declared, uncertainty-free obstacle requires. The body guard cannot see
#: a table; this is run 5's horizontal approach, refused with a number.
SCENE_REFUSED = {
    # demo: table top z 0.01, block at z 0.05; box (container) beside it
    2: "box", 3: "table", 4: "table", 6: "box", 10: "box", 32: "box",
    36: "box",
    # tabletop: the same table, forearm 11-13 mm above it
    53: "table", 54: "table", 55: "table", 57: "table", 61: "table",
    83: "table", 87: "table",
    # d1-2_tape_cup: wrist 13 mm from the wagon top's edge
    103: "table", 104: "table", 105: "table", 134: "table",
    # yawed: forearm 13-15 mm above the table
    151: "table", 152: "table", 154: "table", 184: "table",
}


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

def numeric_stack():
    """What the joint path's last bits depend on: the linear-algebra stack
    and the CPU it dispatches for."""
    import platform

    import numpy
    import scipy
    cpu = platform.processor() or platform.machine()
    try:
        with open("/proc/cpuinfo", encoding="utf-8") as f:
            for line in f:
                if line.startswith("model name"):
                    cpu = line.split(":", 1)[1].strip()
                    break
    except OSError:
        pass
    return {"numpy": numpy.__version__, "scipy": scipy.__version__,
            "cpu": cpu}


def _close(a, b, path, errors, tol=TOL, portable=False):
    if isinstance(a, dict) and isinstance(b, dict):
        if sorted(a) != sorted(b):
            errors.append(f"{path}: keys {sorted(a)} != {sorted(b)}")
            return
        for key in a:
            if portable and key in SOLVER_PATH_KEYS:
                continue
            scale = max(1, int(a.get("n", 1))) if key == "q_sum" else 1
            key_tol = tol
            if portable:
                key_tol = PORTABLE_Q_TOL if key == "q_last" else PORTABLE_TOL
            _close(a[key], b[key], f"{path}.{key}", errors, key_tol * scale,
                   portable)
        return
    if isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            errors.append(f"{path}: {len(a)} items != {len(b)}")
            return
        for i, (x, y) in enumerate(zip(a, b)):
            _close(x, y, f"{path}[{i}]", errors, tol, portable)
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


def _without_the_scene_gate(monkeypatch):
    from manipulation_kit.primitives import clearance, verbs
    monkeypatch.setattr(verbs, "_scene_for",
                        lambda primitive, world, kin: clearance.SceneGate(()))


def test_alias_directions_reproduce_the_pre_direction_plans(d1_arm,
                                                            monkeypatch):
    _without_the_scene_gate(monkeypatch)
    golden = _load()
    worlds = {name: _world(d1_arm, items)
              for name, items in golden["scenes"].items()}
    planned = sum(1 for c in golden["cases"] if c["result"]["kind"] != "refusal")
    assert len(golden["cases"]) >= 200 and planned >= 75, \
        "the golden set shrank; it is the evidence, not a sample"
    # off the capture stack, only the solver's own path is loosened (above)
    portable = golden.get("numeric_stack") != numeric_stack()
    failures = []
    for index, case in enumerate(golden["cases"]):
        errors = []
        _close(case["result"], replay(case, d1_arm, worlds[case["scene"]]),
               f"case {index} {case['scene']}/{case['verb']} {case['args']}",
               errors, portable=portable)
        failures += errors[:3]
    assert not failures, "\n".join(failures[:30])


def _scene_refusal(case, kin, world):
    """The PlanError a listed case now ends in (a chain: its first link)."""
    from manipulation_kit.primitives import reach
    from manipulation_kit.primitives.verbs import BY_VERB
    args = dict(case["args"])
    if case["verb"] == "chain":
        links = reach.plan_chain(world, kin, **args).links
        assert len(links) == 1, [link.verb for link in links]
        return links[0].result
    return BY_VERB[case["verb"]](**args).plan(world, kin)


def test_the_scene_gate_changes_only_the_listed_cases(d1_arm):
    """With the gate ON: every case as captured, except :data:`SCENE_REFUSED`,
    each of which is now a ``guard_reject`` naming its obstacle with a
    positive penetration depth."""
    from manipulation_kit.primitives.types import GUARD_REJECT
    golden = _load()
    worlds = {name: _world(d1_arm, items)
              for name, items in golden["scenes"].items()}
    portable = golden.get("numeric_stack") != numeric_stack()
    failures = []
    for index, case in enumerate(golden["cases"]):
        world = worlds[case["scene"]]
        if index in SCENE_REFUSED:
            error = _scene_refusal(case, d1_arm, world)
            name = SCENE_REFUSED[index]
            if (getattr(error, "ok", True) or error.reason != GUARD_REJECT
                    or f"obstacle:{name}" not in error.attempted
                    or f"{name!r}" not in error.detail):
                # Off the capture stack a knife-edge case (151/154/184 are
                # 0.09 mm inside the envelope here) may plan on the solver's
                # other path; then it must be the captured plan.
                errors = []
                if portable:
                    _close(case["result"], replay(case, d1_arm, world),
                           f"case {index}", errors, portable=True)
                if not portable or errors:
                    failures.append(f"case {index}: expected a guard_reject "
                                    f"naming {name!r}, got {error}")
            continue
        errors = []
        _close(case["result"], replay(case, d1_arm, world),
               f"case {index} {case['scene']}/{case['verb']} {case['args']}",
               errors, portable=portable)
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

    # the file is the PRE-scene evidence: capture with the scene gate off
    from manipulation_kit.primitives import clearance, verbs
    verbs._scene_for = lambda primitive, world, kin: clearance.SceneGate(())
    golden = _load()
    kin = get_arm_kinematics("d1/arm", quiet=True)
    for side in ("left", "right"):
        kin.set_joints(side, kin.home(side))
    worlds = {name: _world(kin, items)
              for name, items in golden["scenes"].items()}
    for case in golden["cases"]:
        case["result"] = rounded(replay(case, kin, worlds[case["scene"]]))
    golden["numeric_stack"] = numeric_stack()
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
