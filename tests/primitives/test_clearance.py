"""The scene gate: arm links against the tables, containers and objects.

Against the real bundled ``d1.urdf``, the real DLS IK and the real
MotionGuard; the obstacles are the committed d1-2 scene
(``examples/agent/scenes/d1-2_tape_cup.json``) and small variations of it.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

from manipulation_kit.primitives import Approach, Grasp, planning, reach, verbs
from manipulation_kit.primitives.clearance import (ClearancePolicy, SceneGate,
                                                   first_hit,
                                                   obstacles_of, policy_of,
                                                   set_policy)
from manipulation_kit.primitives.orientation import tool_from_link7
from manipulation_kit.primitives.planning import Kin, solve_path
from manipulation_kit.primitives.types import GUARD_REJECT, JointStep, Waypoint
from manipulation_kit.world import (ArmView, ContainerView, GripperView,
                                    ObjectView, SurfaceView, WorldView)

REPO = Path(__file__).resolve().parents[2]
D1_2_SCENE = REPO / "examples" / "agent" / "scenes" / "d1-2_tape_cup.json"
#: the roll the d1-2 top-down grasp plans at from HOME (the golden file's
#: case 109: the right arm's jaws turned -90 deg)
ROLL = -math.pi / 2


# --------------------------------------------------------------------------- #
# worlds
# --------------------------------------------------------------------------- #

def _objects_of_scene(path=D1_2_SCENE):
    kinds = {"object": ObjectView, "container": ContainerView,
             "surface": SurfaceView}
    out = []
    for item in json.loads(path.read_text(encoding="utf-8"))["objects"]:
        kind = kinds[item["kind"]]
        extra = {}
        if kind is ContainerView:
            extra = {"interior": item["interior"],
                     "rim_height_m": item["rim_height_m"]}
        out.append(kind(item["name"], p=item["p"], size=item["size"],
                        r=R.from_euler("z", float(item.get("yaw_rad", 0.0))),
                        frame_id=item.get("frame_id", "base"),
                        colour=item.get("colour"), **extra))
    return out


def _world(kin, objects):
    arms, grippers = [], []
    for side in ("left", "right"):
        p, r = tool_from_link7(*kin.ee_pose(side))
        arms.append(ArmView(side, joints=kin.joints(side), tool_p=p, tool_r=r,
                            mode="position"))
        # d1-2's measured driven-open gap (the scene file's robot.hand)
        grippers.append(GripperView(side, 0.0, holding=False, jaw_gap_m=0.04,
                                    open_gap_m=0.0605))
    return WorldView.of(objects, arms=arms, grippers=grippers)


def _d1_2(kin):
    return _world(kin, _objects_of_scene())


def _tool_path(kin, side, steps):
    saved = np.array(kin.joints(side), dtype=float)
    out = []
    try:
        for step in steps:
            if isinstance(step, JointStep) and step.side == side:
                kin.set_joints(side, step.q)
                out.append((step.waypoint, tool_from_link7(*kin.ee_pose(side))[0]))
    finally:
        kin.set_joints(side, saved)
    return out


@pytest.fixture
def no_scene_gate(monkeypatch):
    """The planner as it was before 0.16.0: body guard only."""
    monkeypatch.setattr(verbs, "_scene_for",
                        lambda primitive, world, kin: SceneGate(()))


@pytest.fixture
def policy_restored(d1_arm):
    before = getattr(d1_arm, "clearance_policy", None)
    yield d1_arm
    if before is None:
        if hasattr(d1_arm, "clearance_policy"):
            del d1_arm.clearance_policy
    else:
        d1_arm.clearance_policy = before


# --------------------------------------------------------------------------- #
# the three D.1 tests
# --------------------------------------------------------------------------- #

#: a 100 mm crate standing on the d1-2 wagon top between HOME and the cube
CRATE = ObjectView("crate", p=(0.39, -0.20, 0.216), size=(0.04, 0.20, 0.10))
CUBE_BEYOND = ObjectView("cube", p=(0.47, -0.20, 0.191),
                         size=(0.045, 0.05, 0.05))


def _crate_world(kin, *extra):
    table = [o for o in _objects_of_scene() if o.name == "table"]
    return _world(kin, table + [CUBE_BEYOND] + list(extra))


def test_a_transit_over_a_wagon_rises_above_it(d1_arm, monkeypatch):
    """Up-and-over is the DEFAULT shape of a free transit, by construction.

    A crate on the wagon stands between the right hand at HOME and the cube.
    The approach rises until the hand clears it — its top, plus the 15 mm a
    declared box requires, plus how far the hand hangs below the tool point —
    traverses, and descends onto the standoff; and the plan says so.
    """
    world = _crate_world(d1_arm, CRATE)
    plan = Approach(object="cube", side="right", direction="down",
                    roll_rad=ROLL).plan(world, d1_arm)
    assert plan.ok, str(plan)
    note = [n for n in plan.notes if n.startswith("transit to 'standoff' rose")]
    assert note and "'crate'" in note[0], plan.notes
    height = float(re.search(r"z=([0-9.]+) m", note[0]).group(1))
    gate = SceneGate.of(world, d1_arm, exclude=("cube",))
    crate = next(ob for ob in gate.obstacles if ob.name == "crate")
    # the rise height is the crate's envelope plus a hanging hand, not a guess
    assert height >= crate.top_z + gate.required_m(crate) + 0.029 - 1e-9
    # every tool point over the crate's footprint (widened by the hand) is at
    # or above that height, within the path window the solver walks in
    over = [p for _, p in _tool_path(d1_arm, "right", plan.steps)
            if crate.footprint_distance(p[:2]) <= planning.TRANSIT_HALF_WIDTH_M]
    assert over, "the transit never passed over the crate at all"
    assert min(p[2] for p in over) >= height - planning.PATH_TOL_M
    # ...and it arrives where the plan said
    last = _tool_path(d1_arm, "right", plan.steps)[-1][1]
    assert np.linalg.norm(last - plan.waypoints[-1].p) < planning.PATH_TOL_M

    # THE TEST DISCRIMINATES: without the scene gate the same approach flies
    # the old detour, lower over the crate than the hand needs
    monkeypatch.setattr(verbs, "_scene_for",
                        lambda primitive, world, kin: SceneGate(()))
    old = Approach(object="cube", side="right", direction="down",
                   roll_rad=ROLL).plan(world, d1_arm)
    assert old.ok
    old_over = [p for _, p in _tool_path(d1_arm, "right", old.steps)
                if crate.footprint_distance(p[:2])
                <= planning.TRANSIT_HALF_WIDTH_M]
    assert min(p[2] for p in old_over) < height - planning.PATH_TOL_M


def test_a_forearm_through_the_table_is_refused_with_the_obstacle_named(d1_arm):
    """The gate's own verdict, on a posture whose forearm is in a table.

    A wagon top declared 20 mm above the lowest point of the right forearm
    capsule at HOME: the report names the table and the link, the clearance
    is the (negative) penetration, and the depth is that plus the clearance
    the table requires. A swept interval that ENTERS the table is refused the
    same way; ``solve_path`` turns that into the body guard's own reason.
    """
    world = _d1_2(d1_arm)
    gate0 = SceneGate.of(world, d1_arm)
    q_home = d1_arm.home("right")
    forearm = [c for c in gate0.capsules("right", q_home) if c[0] == "Link4_L"]
    (link, a, b, r), = forearm
    low = min(a[2], b[2]) - r
    x, y = (a[:2] + b[:2]) / 2.0
    top = low + 0.020
    wagon = SurfaceView("wagon_top", p=(x, y, top - 0.01), size=(0.4, 0.4, 0.02))
    gate = SceneGate((obstacles_of(world.with_(objects=(wagon,)))[0]),
                     ClearancePolicy())
    report = gate.pose_ok(d1_arm, "right", q_home)
    assert not report.ok
    assert report.obstacle == "wagon_top"
    assert report.link.startswith("Link")          # arm structure, not the hand
    assert report.clearance_m < -0.015             # inside the box itself
    assert report.depth_m == pytest.approx(
        report.required_m - report.clearance_m)
    assert "'wagon_top'" in report.describe("right")
    # ENTERING it from a clear posture is refused too, and named: the same
    # table 30 mm below the forearm (clear of its 15 mm), then a joint move
    # that drops the forearm into it
    wagon = SurfaceView("wagon_top", p=(x, y, low - 0.030 - 0.01),
                        size=(0.4, 0.4, 0.02))
    gate = SceneGate(obstacles_of(world.with_(objects=(wagon,)))[0])
    assert gate.pose_ok(d1_arm, "right", q_home).ok
    dropped = None
    for j in range(7):
        for sign in (1.0, -1.0):
            q = np.array(q_home, dtype=float)
            q[j] += 0.25 * sign
            if gate.pose_ok(d1_arm, "right", q).clearance_m < -0.005:
                dropped = q
                break
        if dropped is not None:
            break
    assert dropped is not None
    swept = gate.swept_ok(d1_arm, "right", q_home, dropped)
    assert not swept.ok and swept.obstacle == "wagon_top"
    assert 0.0 < swept.fraction <= 1.0 and swept.depth_m > 0.0


def test_the_run5_horizontal_approach_on_the_d1_2_scene_is_refused_with_the_wagon_named(
        d1_arm, no_scene_gate, monkeypatch):
    """Run 5: the right arm approached the cube horizontally and swept the
    wagon top. Without the scene gate that grasp PLANS — with the wrist
    inside the table; with it, it is a ``guard_reject`` naming the table
    with a positive depth, which is what makes ``ASTRA_APPROACH_ALLOW``
    unnecessary."""
    world = _d1_2(d1_arm)
    blind = Grasp(object="cube", side="right", direction="forward",
                  roll_rad=ROLL).plan(world, d1_arm)
    assert blind.ok, str(blind)
    gate = SceneGate.of(world, d1_arm, exclude=("cube",))
    worst = min((gate.pose_ok(d1_arm, "right", s.q) for s in blind.joint_steps()),
                key=lambda rep: rep.clearance_m)
    assert worst.obstacle == "table" and worst.clearance_m < 0.0, worst

    monkeypatch.undo()                              # the gate back on
    error = Grasp(object="cube", side="right", direction="forward",
                  roll_rad=ROLL).plan(world, d1_arm)
    assert not error.ok
    assert error.reason == GUARD_REJECT
    assert "'table'" in error.detail
    assert "obstacle:table" in error.attempted
    assert math.isfinite(error.residual_m) and error.residual_m > 0.0


def test_the_grasp_descent_is_still_straight(d1_arm):
    """``allow_via=False`` legs are never re-shaped by the scene.

    The grasp's descent (waypoint 1) stays on the standoff -> grasp line even
    when its transit (waypoint 0) was planned up-and-over; and a leg marked
    ``allow_via=False`` whose straight line passes low over the crate is NOT
    lifted over it — it is walked straight or refused.
    """
    world = _crate_world(d1_arm, CRATE)
    plan = Grasp(object="cube", side="right", direction="down",
                 roll_rad=ROLL).plan(world, d1_arm)
    assert plan.ok, str(plan)
    assert [w.allow_via for w in plan.waypoints] == [True, False]
    assert any("rose" in n for n in plan.notes), plan.notes
    a, b = plan.waypoints[0].p, plan.waypoints[1].p
    axis = (b - a) / np.linalg.norm(b - a)
    descent = [p for index, p in _tool_path(d1_arm, "right", plan.steps)
               if index == 1]
    assert descent
    for p in descent:
        off = (p - a) - axis * float(np.dot(p - a, axis))
        assert np.linalg.norm(off) < 0.005, np.linalg.norm(off)

    # a constrained leg across the crate: no up-and-over, whatever happens
    p_goal = np.array([0.47, -0.20, 0.30])
    r_goal = plan.waypoints[0].r
    gate = SceneGate.of(world, d1_arm, exclude=("cube",))
    with Kin(d1_arm, world, scene=gate) as borrowed:
        start = borrowed.tool_pose("right")[0]
        steps, error, notes = solve_path(
            borrowed, "right", [Waypoint("across", p_goal, r_goal,
                                         allow_via=False)],
            primitive="test")
    assert not any("rose" in n or "via" in n for n in notes), notes
    line = (p_goal - start) / np.linalg.norm(p_goal - start)
    for _, p in _tool_path(d1_arm, "right", steps):
        off = (p - start) - line * float(np.dot(p - start, line))
        assert np.linalg.norm(off) < planning.PATH_TOL_M


# --------------------------------------------------------------------------- #
# the false-positive check, and the rest of the brief
# --------------------------------------------------------------------------- #

def test_a_reachable_top_down_grasp_on_the_d1_2_scene_is_not_refused(
        d1_arm, policy_restored):
    """The number that decides usability: the committed d1-2 scene, wagon top
    and cup declared, the whole top-down pick-and-place plans — with the
    gate on, and again with the real arm's 12 mm droop margin."""
    world = _d1_2(d1_arm)
    gate = SceneGate.of(world, d1_arm, exclude=("cube",))
    assert {ob.name for ob in gate.obstacles} == {"table", "cup"}
    for droop in (0.0, 0.012):
        set_policy(d1_arm, ClearancePolicy(droop_margin_m=droop))
        chain = reach.plan_chain(world, d1_arm, obj="cube", destination="cup",
                                 side="right", direction="down", roll_rad=ROLL)
        verbs_planned = [link.verb for link in chain.links
                         if getattr(link.result, "ok", False)]
        assert verbs_planned == ["approach", "grasp", "lift", "carry", "place"], \
            [str(link.result) for link in chain.links]
        grasp = chain.links[1].result
        # the arm really was checked against the table, and clears it
        worst = max((SceneGate.of(world, d1_arm, exclude=("cube",))
                     .pose_ok(d1_arm, "right", s.q) for s in grasp.joint_steps()),
                    key=lambda rep: rep.depth_m)
        assert worst.ok and worst.depth_m < 0.0, worst
        # the droop margin reaches the fingertip floor
        tips = [n for n in grasp.notes if "keeps the pad tips" in n]
        mm = float(re.search(r"tips ([0-9.]+) mm", tips[0]).group(1))
        assert mm == pytest.approx(3.0 + droop * 1000.0, abs=1.5), tips


def test_the_scene_gate_states_its_sampling_and_margin(d1_arm):
    """The gate says how it samples, adds what the sampling can miss to every
    requirement, and the claim holds against a dense reference."""
    policy = ClearancePolicy()
    assert policy.sampling_allowance_m == pytest.approx(
        0.5 * (policy.segment_spacing_m + policy.sweep_step_m))
    world = _d1_2(d1_arm)
    gate = SceneGate.of(world, d1_arm, exclude=("cube",))
    text = gate.describe_sampling()
    assert f"{policy.segment_spacing_m * 1000:.0f} mm" in text
    assert f"{policy.sweep_step_m * 1000:.0f} mm" in text
    table = next(ob for ob in gate.obstacles if ob.name == "table")
    assert gate.required_m(table) == pytest.approx(
        policy.declared_surface_margin_m + policy.droop_margin_m
        + policy.sampling_allowance_m)

    # per obstacle: probed < declared < a stated uncertainty; a HEIGHT
    # uncertainty grows the box vertically only
    probed = SurfaceView("p", p=(0.5, 0, 0.1), size=(0.4, 0.4, 0.02),
                         plane_source="probed")
    provisional = SurfaceView("v", p=(0.5, 0, 0.1), size=(0.4, 0.4, 0.02),
                              plane_source="provisional",
                              height_uncertainty_m=0.05)
    boxed = ObjectView("b", p=(0.5, 0, 0.1), size=(0.1, 0.1, 0.1),
                       uncertainty_m=0.02)
    obs = {ob.name: ob for ob in obstacles_of(
        WorldView.of([probed, provisional, boxed]))[0]}
    assert obs["p"].margin_m == policy.probed_surface_margin_m
    assert obs["v"].margin_m == policy.declared_surface_margin_m
    assert np.allclose(obs["v"].half, [0.2, 0.2, 0.01 + 0.05])
    assert obs["b"].margin_m == 0.02 and "declared +-20 mm" in obs["b"].basis

    # THE CLAIM HOLDS: over swept intervals near and into the wagon top, the
    # clearance the gate reports is never more than the allowance above the
    # truth (0.5 mm spacing, 400 postures per interval). Postures around the
    # run-5 wrist-in-the-table pose, where the check matters.
    q_near = np.array([1.2187, 1.4379, -1.5428, -1.7760, -1.4356, 0.0020,
                       -0.1450])          # golden case 111's standoff posture
    rng = np.random.default_rng(5)
    arm = gate._arm("right")
    dense = SceneGate(gate.obstacles, ClearancePolicy(segment_spacing_m=0.0005))
    dense_arm = dense._arm("right")
    worst_gap, near = 0.0, 0
    for _ in range(30):
        q0 = q_near + rng.uniform(-0.3, 0.3, 7)
        q1 = q0 + rng.uniform(-0.25, 0.25, 7)
        swept = gate.swept_ok(d1_arm, "right", q0, q1)
        if not swept.ok:
            continue
        truth = math.inf
        for f in np.linspace(0.0, 1.0, 400)[1:]:
            _, clearance, _ = dense._depths(dense_arm, dense_arm.link_frames(
                q0 + (q1 - q0) * f))
            truth = min(truth, float(np.min(clearance)))
        near += truth < 0.05
        worst_gap = max(worst_gap, swept.clearance_m - truth)
    assert near >= 5, "the check never came near an obstacle"
    assert worst_gap <= policy.sampling_allowance_m + 1e-9, worst_gap
    assert arm.sample_radii.shape[0] > dense_arm.sample_radii.shape[0] / 10


def test_the_capsules_are_the_guards_capsules(d1_arm):
    """The scene sees the arm the body guard sees: same links, same radii,
    same endpoints to 1e-9 over random postures, both arms."""
    from manipulation_kit.arms import sides
    from manipulation_kit.guard.guard import MotionGuard
    guard = MotionGuard()
    gate = SceneGate((), ClearancePolicy(), (), guard.model)
    rng = np.random.default_rng(11)
    for side in ("left", "right"):
        for _ in range(10):
            q = rng.uniform(-1.2, 1.2, 7)
            theirs, _ = guard._arm_capsules(sides.SDK_SIDE[side],
                                            list(np.degrees(q)))
            mine = gate.capsules(side, q)
            assert [c.link for c in theirs] == [m[0] for m in mine]
            for c, (link, a, b, r) in zip(theirs, mine):
                assert np.allclose(a, c.a, atol=1e-9)
                assert np.allclose(b, c.b, atol=1e-9)
                assert r == c.r


def test_an_arm_already_inside_an_envelope_may_leave_it(d1_arm):
    """Retreating from the table is not refused for being near the table —
    only going deeper, or into a new envelope, is."""
    world = _d1_2(d1_arm)
    gate0 = SceneGate.of(world, d1_arm)
    q_home = np.asarray(d1_arm.home("right"), dtype=float)
    (link, a, b, r), = [c for c in gate0.capsules("right", q_home)
                        if c[0] == "Link4_L"]
    x, y = (a[:2] + b[:2]) / 2.0
    top = min(a[2], b[2]) - r + 0.005              # 5 mm into the forearm
    wagon = SurfaceView("wagon_top", p=(x, y, top - 0.01), size=(0.5, 0.5, 0.02))
    gate = SceneGate(obstacles_of(world.with_(objects=(wagon,)))[0])
    start = gate.pose_ok(d1_arm, "right", q_home)
    assert not start.ok
    # find a joint move that lowers the violation and one that deepens it
    out = in_ = None
    for j in range(7):
        for sign in (1.0, -1.0):
            q = q_home.copy()
            q[j] += 0.05 * sign
            depth = gate.pose_ok(d1_arm, "right", q).depth_m
            if depth < start.depth_m - 1e-3 and out is None:
                out = q
            if depth > start.depth_m + 1e-3 and in_ is None:
                in_ = q
    assert out is not None and in_ is not None
    assert gate.swept_ok(d1_arm, "right", q_home, out).ok
    deeper = gate.swept_ok(d1_arm, "right", q_home, in_)
    assert not deeper.ok and deeper.obstacle == "wagon_top"


def test_the_target_and_the_held_object_are_not_obstacles(d1_arm):
    """A grasp must reach its object; a carried object rides the tool."""
    world = _d1_2(d1_arm)
    grippers = dict(world.grippers)
    grippers["left"] = GripperView("left", 1.0, holding=True, held_object="cup")
    world = world.with_(grippers=grippers)
    gate = verbs._scene_for(Grasp(object="cube"), world, d1_arm)
    assert {ob.name for ob in gate.obstacles} == {"table"}
    gate = verbs._scene_for(verbs.Place(object="cube", to="table"), world,
                            d1_arm)
    assert {ob.name for ob in gate.obstacles} == set()


def test_the_droop_margin_is_a_typed_policy_on_the_kinematics(policy_restored):
    kin = policy_restored
    assert policy_of(kin).droop_margin_m == 0.0      # the documented default
    set_policy(kin, ClearancePolicy(droop_margin_m=0.012))
    assert policy_of(kin).droop_margin_m == 0.012
    world = _d1_2(kin)
    plan = Grasp(object="cube", side="right", direction="down",
                 roll_rad=ROLL).plan(world, kin)
    assert plan.ok, str(plan)
    tips = [n for n in plan.notes if "keeps the pad tips" in n]
    clearance_mm = float(re.search(r"tips ([0-9.]+) mm", tips[0]).group(1))
    assert clearance_mm == pytest.approx(3.0 + 12.0, abs=1.5)
    with pytest.raises(TypeError):
        set_policy(kin, {"droop_margin_m": 0.01})
    with pytest.raises(ValueError):
        ClearancePolicy(droop_margin_m=-0.001)


def test_no_env_var_feeds_the_support_clearance():
    """``MKIT_SUPPORT_CLEARANCE_M`` is gone: setting it changes nothing, and
    no source file reads it."""
    code = ("from manipulation_kit.primitives import orientation as o, "
            "clearance as c; print(o.SUPPORT_CLEARANCE_M, "
            "c.DEFAULT_POLICY.droop_margin_m)")
    env = dict(os.environ, MKIT_SUPPORT_CLEARANCE_M="0.05")
    out = subprocess.run([sys.executable, "-c", code], env=env, check=True,
                         capture_output=True, text=True).stdout.split()
    assert [float(v) for v in out] == [0.003, 0.0]
    reads = []
    for base in ("src", "examples"):
        for path in (REPO / base).rglob("*.py"):
            text = path.read_text(encoding="utf-8")
            for line in text.splitlines():
                if re.search(r"(environ(\.get)?|getenv)\s*[(\[]\s*['\"]"
                             r"MKIT_SUPPORT_CLEARANCE_M", line):
                    reads.append(f"{path}: {line.strip()}")
    assert not reads, reads
    source = (REPO / "src/manipulation_kit/primitives/orientation.py").read_text()
    assert "os.environ" not in source and "\nimport os\n" not in source


#: sha256 of every file of ``manipulation_kit.guard`` as it is on
#: ``origin/main`` (and on ``feat/perceive-head`` @ 1b5829c). The guard is
#: stdlib-only and shared with firmware-side hosts; the scene gate re-uses it
#: and must not change it.
GUARD_SHA256 = {
    "__init__.py": "8603dd5a7ccd069f486433927be106819b8f2fc1019ca6d556dd0d9364100385",
    "geometry.py": "10c7fc71bdc2db53124ba91a78ff9f511baacabeedc49e65d46885225dc2f64e",
    "guard.py": "a99f2c1ab15ce2088b23c00c05c9f07eb6dd30346f75a87f671dc95342aadbd9",
    "urdf_model.py": "a0b9f5b46e88a4bae993ce4a07b80add66cb41a385f005490c3ebd0c807d05a3",
}


def test_guard_package_is_untouched():
    guard = REPO / "src" / "manipulation_kit" / "guard"
    found = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
             for p in sorted(guard.glob("*.py"))}
    assert found == GUARD_SHA256
    # and it still imports nothing but the standard library
    for path in guard.glob("*.py"):
        for line in path.read_text(encoding="utf-8").splitlines():
            if line.startswith(("import ", "from ")):
                assert not re.match(r"(import|from) (numpy|scipy|manipulation_kit\.world)",
                                    line), (path, line)


# --------------------------------------------------------------------------- #
# contact legs (step 4's Probe / Press): driven INTO what they measure
# --------------------------------------------------------------------------- #

def _probe_leg(side, p_start, d, travel_m):
    """The two waypoints ``contact._contact_plan`` solves: the standoff, then
    the leg to ``max_travel_m`` along ``d`` — deliberately past the surface."""
    from manipulation_kit.primitives import orientation as ap
    r_tool = ap.align_tool(side, np.asarray(d, dtype=float))
    p_start = np.asarray(p_start, dtype=float)
    return [Waypoint("probe_start", p_start, r_tool, allow_via=True),
            Waypoint("contact_limit", p_start + np.asarray(d) * travel_m, r_tool,
                     allow_via=False)]


def test_a_probe_leg_is_not_refused_by_the_surface_it_measures(d1_arm):
    """A 150 mm probe down onto the d1-2 wagon top ends with the wrist in the
    table — which is what a probe is for. Gated by ``SceneGate.for_contact``
    the leg plans, with the table named as the contact target and the cup
    still an obstacle; gated as an ordinary leg it is refused by the table,
    so the exclusion is what lets it through."""
    world = _d1_2(d1_arm)
    down = np.array([0.0, 0.0, -1.0])
    # over the wagon top (x 0.31-0.71, |y| < 0.30), clear of cube and cup
    p_start, travel = (0.36, -0.25, 0.23), 0.15
    waypoints = _probe_leg("right", p_start, down, travel)

    gate = SceneGate.for_contact(world, d1_arm, p_start, down, travel)
    assert gate.contact_target == "table"
    assert {ob.name for ob in gate.obstacles} == {"cube", "cup"}
    with Kin(d1_arm, world, scene=gate) as borrowed:
        steps, error, _ = solve_path(borrowed, "right", waypoints,
                                     primitive="probe")
    assert error is None, str(error)
    end = _tool_path(d1_arm, "right", steps)[-1][1]
    assert end[2] < 0.166 - 0.05              # 50 mm "through" the table top

    blind = SceneGate.of(world, d1_arm)
    with Kin(d1_arm, world, scene=blind) as borrowed:
        _, error, _ = solve_path(borrowed, "right", waypoints, primitive="probe")
    assert error is not None and "obstacle:table" in error.attempted


def test_a_probed_wall_keeps_its_rotation_and_its_normal_uncertainty():
    """Probed surfaces are thin slabs whose own +z is the measured normal
    (``SurfaceView.from_plane``); a wall's is horizontal. The obstacle keeps
    the pose rotation, and the height uncertainty grows it along that normal."""
    # a wall facing the robot: its +z is base -x, top face at x = 0.60
    rot = R.from_matrix(np.column_stack([[0, 1, 0], [0, 0, -1], [-1, 0, 0]]))
    wall = SurfaceView("wall", p=(0.602, 0.0, 0.3), size=(0.3, 0.3, 0.004),
                       r=rot, plane_source="contact", height_uncertainty_m=0.005)
    table = SurfaceView("table", p=(0.45, 0.0, 0.10), size=(0.4, 0.6, 0.02))
    world = WorldView.of([wall, table])
    obs = {ob.name: ob for ob in obstacles_of(world)[0]}
    assert obs["wall"].margin_m == ClearancePolicy().probed_surface_margin_m
    assert np.allclose(obs["wall"].half, [0.15, 0.15, 0.002 + 0.005])
    assert np.allclose(obs["wall"].r.as_matrix(), rot.as_matrix())
    gate = SceneGate(tuple(obs.values()))
    forward, down = np.array([1.0, 0, 0]), np.array([0, 0, -1.0])
    assert first_hit(gate, (0.40, 0.0, 0.30), forward, 0.30) == "wall"
    assert first_hit(gate, (0.40, 0.0, 0.30), down, 0.30) == "table"
    assert first_hit(gate, (0.40, 0.0, 0.30), -forward, 0.10) == ""
