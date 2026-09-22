"""Golden plans: every verb the example scenes exercise, planned and pinned.

``tests/data/golden_plans/plans.json`` holds a set of planning requests —
every grasp direction for every object, both arms and ``auto``, the whole
Approach->Place chain, the corrections — in the scenes the examples use, and
the plan (or the typed refusal) the kit produced for each. This test replays
every case and requires the waypoints, the joint path (per-waypoint step
count, last posture and posture sum), the gripper strokes, the notes and the
refusals to agree.

HISTORY. Captured on ``feat/perceive-head`` @ ``e1dce97`` with the OLD API
(``approach`` x ``jaw_turn_deg``) and replayed identically by 0.16.0 step 2
(``Direction``): that was the proof the vocabulary change moved nothing. Each
case still records the old arguments it was first captured with
(``captured_as``). Step 3 (grasp geometry) then changed the geometry ON
PURPOSE and regenerated it — the standoff is measured from the object's
silhouette, the descent floor is the measured support, the roll is the kit's
one sweep (so the old per-``jaw_turn`` cases collapsed into one each) — and
added the cases the old set could not pin: a side approach that PLANS (the
old set's 70 side cases were all refusals, so it could not tell ``left`` from
``right``), a fingertip grasp, a tilted object, an object declared into its
table, and a tall cup. The per-number reasons are in that commit.

WHERE 1e-9 HOLDS. The joint path is the output of an iterative IK solve, and
its last bits depend on the numeric stack it ran on: the same code planned
with numpy 2.0.2 / scipy 1.13.1 (CI's Python 3.9 job) takes 26 knots where
the capture took 18 on one chain, and on another CI runner a nudge that the
guard refuses at its first knot here walks 15 mm first. Both are knife edges
of the solver, not of the geometry. So the file records the stack it was
captured on (``numeric_stack``) and

* on that stack every field is compared at 1e-9 (the full proof);
* elsewhere every verdict, refusal reason, side, stage, label, stroke and
  note must still be IDENTICAL, waypoints agree to ``PORTABLE_TOL`` (a later
  link of a chain starts from the posture the solver reached, so its
  waypoints inherit the solver's last bits), and the solver's own path is
  compared by its final posture only, to ``PORTABLE_Q_TOL`` (knot counts,
  posture sums and a refusal's residual are the solver's, not the plan's). A
  geometry mistake — a mirrored axis, a 90 deg roll — moves these by
  centimetres and quarter turns.

THE SCENE GATE (0.16.0, step 5) is switched off for that proof: it is a
proof about the VOCABULARY, and the gate is a new check that refuses some of
these plans on purpose. A second test replays every case with the gate on and
requires every case to be unchanged EXCEPT the ones listed, by number, in
:data:`SCENE_REFUSED` — each a side-on approach whose arm comes within
12-15 mm of a declared table or shelf, inside the 15 mm a declared obstacle
requires (10 mm margin + 5 mm sampling allowance), or whose standoff posture
is refused by the body guard with every route around it refused by the
table. Those become refusals naming the obstacle. No top-down case changes.

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
          / "plans.json")
TOL = 1e-9
#: off the capture stack: waypoints (m, quaternion components) ...
PORTABLE_TOL = 1e-3
#: ... and the solver's final posture (rad)
PORTABLE_Q_TOL = 1e-2
#: fields that are the IK solver's path rather than the plan's geometry
SOLVER_PATH_KEYS = ("n", "q_sum", "residual_m")

#: The cases the scene gate (0.16.0, step 5) changes, BY NUMBER, and the
#: obstacle each refusal must name — re-derived on the capture stack from the
#: phase-B golden (steps 3+4+5+7 merged; ``plans.json``, 163 cases). Every
#: one is a SIDE-ON approach (``forward``, or ``left``/``right`` at the shelf)
#: that was a plan with the gate off; no top-down case changes. Each is a
#: ``guard_reject`` naming the obstacle, except where noted.
SCENE_REFUSED = {
    # demo / tabletop, forward onto red_block: the posture at the (step-3,
    # silhouette-measured) standoff is refused by the body guard, and every
    # route around it is refused by the table — the refusal names both
    1: "table", 3: "table", 5: "table", 13: "table", 17: "table",
    32: "table", 34: "table", 36: "table", 44: "table", 48: "table",
    # d1-2_tape_cup, forward onto the cube: the wrist (Link7) passes 12 mm
    # from the wagon top on the way to 'grasp' (the cup refuses the other roll)
    70: "table", 74: "table", 77: "table",
    # yawed, forward onto the block: forearm (Link4) 13 mm from the table
    93: "table", 95: "table", 109: "table",
    # side_shelf: the forearm passes 14-15 mm from the shelf (0-1 mm inside
    # its 15 mm — knife-edge). 140/142: the squared roll's own failure is an
    # IK miss at the standoff; the quarter turn was the shelf's, and the
    # refusal carries obstacle:shelf in ``attempted``
    140: "shelf", 142: "shelf", 144: "shelf",
}
#: the listed cases whose refusal reason is not ``guard_reject``
SCENE_REFUSED_REASON = {140: "ik_fail", 142: "ik_fail"}


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
        if "euler_xyz_rad" in item:
            r = R.from_euler("xyz", [float(a) for a in item["euler_xyz_rad"]])
        else:
            r = R.from_euler("z", float(item.get("yaw_rad", 0.0)))
        out.append(kind(item["name"], p=item["p"], size=item["size"], r=r,
                        frame_id=item.get("frame_id", "base"),
                        colour=item.get("colour"), **extra))
    return out


def _world(kin, items, open_gap_m=None):
    from manipulation_kit.primitives.orientation import tool_from_link7
    from manipulation_kit.world import ArmView, GripperView, WorldView
    arms, grippers = [], []
    for side in ("left", "right"):
        p, r = tool_from_link7(*kin.ee_pose(side))
        arms.append(ArmView(side, joints=kin.joints(side), tool_p=p, tool_r=r,
                            mode="position"))
        grippers.append(GripperView(side, 0.0, holding=False, jaw_gap_m=0.04,
                                    open_gap_m=open_gap_m))
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
            "notes": list(result.notes)}


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


def _worlds(kin, golden):
    """One world per scene, with the hand opening the scene declares (the
    committed d1-2 scene carries the 60.5 mm d1-2 measures)."""
    hands = golden.get("scene_hand_open_gap_m", {})
    return {name: _world(kin, items, hands.get(name))
            for name, items in golden["scenes"].items()}


#: the set may not shrink below this (it is the evidence, not a sample)
MIN_CASES = 150
MIN_PLANNED = 60


def test_the_golden_set_pins_a_planned_side_approach_both_ways():
    """The old set could not tell ``left`` from ``right``: every one of its
    side cases was a refusal, so swapping the two still passed. These cases
    PLAN, and only with the vector the alias names."""
    golden = _load()
    side = [c for c in golden["cases"]
            if c["verb"] in ("approach", "grasp")
            and c["args"].get("direction") in ("left", "right")]
    planned = {(c["args"]["direction"], c["result"].get("side"))
               for c in side if c["result"]["kind"] == "plan"}
    assert ("left", "right") in planned and ("right", "left") in planned, planned
    for case in side:
        if case["result"]["kind"] != "plan":
            continue
        quat = case["result"]["waypoints"][-1]["quat_xyzw"]
        from scipy.spatial.transform import Rotation as R
        axis = R.from_quat(quat).as_matrix()[:, 2]
        want = {"left": (0, 1, 0), "right": (0, -1, 0)}[case["args"]["direction"]]
        assert axis == pytest.approx(want, abs=1e-6), case["args"]


def test_every_golden_case_replays(d1_arm, monkeypatch):
    _without_the_scene_gate(monkeypatch)
    golden = _load()
    worlds = _worlds(d1_arm, golden)
    planned = sum(1 for c in golden["cases"] if c["result"]["kind"] != "refusal")
    assert len(golden["cases"]) >= MIN_CASES and planned >= MIN_PLANNED, \
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
    """The PlanError a listed case now ends in (a chain: its refused link)."""
    from manipulation_kit.primitives import reach
    from manipulation_kit.primitives.verbs import BY_VERB
    args = dict(case["args"])
    if case["verb"] == "chain":
        links = reach.plan_chain(world, kin, **args).links
        return links[-1].result
    return BY_VERB[case["verb"]](**args).plan(world, kin)


def test_the_scene_gate_changes_only_the_listed_cases(d1_arm):
    """With the gate ON: every case as captured, except :data:`SCENE_REFUSED`,
    each of which is now a refusal (:data:`SCENE_REFUSED_REASON`, else
    ``guard_reject``) naming its obstacle."""
    from manipulation_kit.primitives.types import GUARD_REJECT
    golden = _load()
    worlds = _worlds(d1_arm, golden)
    portable = golden.get("numeric_stack") != numeric_stack()
    failures = []
    for index, case in enumerate(golden["cases"]):
        world = worlds[case["scene"]]
        if index in SCENE_REFUSED:
            error = _scene_refusal(case, d1_arm, world)
            name = SCENE_REFUSED[index]
            reason = SCENE_REFUSED_REASON.get(index, GUARD_REJECT)
            if (getattr(error, "ok", True) or error.reason != reason
                    or f"obstacle:{name}" not in error.attempted
                    or (reason == GUARD_REJECT
                        and f"{name!r}" not in error.detail)):
                # Off the capture stack a knife-edge case (140/142/144 are
                # 0-1 mm inside the envelope here) may plan on the solver's
                # other path; then it must be the captured plan.
                errors = []
                if portable:
                    _close(case["result"], replay(case, d1_arm, world),
                           f"case {index}", errors, portable=True)
                if not portable or errors:
                    failures.append(f"case {index}: expected a {reason} "
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
    # the file is the PRE-scene evidence: capture with the scene gate off
    from manipulation_kit.primitives import clearance, verbs
    verbs._scene_for = lambda primitive, world, kin: clearance.SceneGate(())
    golden = _load()
    kin = get_arm_kinematics("d1/arm", quiet=True)
    for side in ("left", "right"):
        kin.set_joints(side, kin.home(side))
    worlds = _worlds(kin, golden)
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
