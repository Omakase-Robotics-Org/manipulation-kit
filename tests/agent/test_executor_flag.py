"""``--executor firmware | isaac | kinematic``: one loop, the executor a flag.

Shu, 2026-09-22 18:02Z: 「これは実機で動かす前提だけど、まず sim で動かしたい、
というケースはフラグで isaac を指定できるようにはしておきたい」. The kit does
not import d1-isaaclab; d1-isaaclab registers itself (an entry point in the
``manipulation_kit.executors`` group). Here a FAKE registered executor stands in
for it, in-tree, by all three routes: in-process registration, an entry point,
and ``--executor-class module:factory``.
"""

from __future__ import annotations

import sys
import types

import pytest

from manipulation_kit.agent import (ENTRY_POINT_GROUP, KinematicMirror,
                                    LiveRobot, OperatorPolicy, UnknownExecutor,
                                    register_executor, run,
                                    unregister_executor)
from manipulation_kit.agent import robot as robot_module
from manipulation_kit.executor import KinematicExecutor, run_steps
from manipulation_kit.primitives import Place


class SpyExecutor(KinematicExecutor):
    """A kinematic mirror that records how it was configured and how every
    plan was handed to it — the numbers L13 says must be the same."""

    def __init__(self, kin, *, arrive_timeout_s, stroke_timeout_s):
        super().__init__(kin)
        self.configured = {"arrive_timeout_s": arrive_timeout_s,
                           "stroke_timeout_s": stroke_timeout_s}
        self.stroke_timeout_s = stroke_timeout_s
        self.runs = []

    def run_plan(self, plan, *, hz=50.0, arrive_tol_rad=None,
                 arrive_timeout_s=None, stroke_timeout_s=None, gate=None):
        self.runs.append({"arrive_timeout_s": arrive_timeout_s,
                          "stroke_timeout_s": stroke_timeout_s,
                          "gate_timeout_s": gate.timeout_s})
        return run_steps(plan, self, hz=hz, arrive_tol_rad=arrive_tol_rad,
                         arrive_timeout_s=arrive_timeout_s,
                         stroke_timeout_s=stroke_timeout_s, gate=gate)


def fake_isaac(*, kin, policy, scene=None, url=None, world0=None, **options):
    """What d1-isaaclab's factory does, shaped the same: build the executor
    FROM THE POLICY and hand back a LiveRobot."""
    from scene import DEMO_WRIST_CAMERA, demo_scene
    world, _ = demo_scene()
    robot = KinematicMirror(kin, world, wrist_intrinsics=DEMO_WRIST_CAMERA)
    robot.name = f"fake-isaac@{url}"
    robot.url = url
    return robot


@pytest.fixture
def fake_registered():
    register_executor("fake-isaac", fake_isaac, replace=True)
    yield
    unregister_executor("fake-isaac")


def test_the_executor_flag_resolves_a_registered_executor(agent_examples,
                                                          fake_registered,
                                                          monkeypatch, capsys):
    robot = LiveRobot.from_flag("fake-isaac", url="tcp://sim:8977")
    assert robot.name == "fake-isaac@tcp://sim:8977"
    # the example resolves the same flag and runs the same loop on it
    import astra_loop
    built = []
    real = LiveRobot.from_flag

    def spy(name, **kw):
        built.append((name, kw.get("url")))
        return real(name, **kw)

    monkeypatch.setattr(LiveRobot, "from_flag", staticmethod(spy))
    assert astra_loop.main(["--dry-run", "--executor", "fake-isaac"]) == 0
    assert built[-1] == ("fake-isaac", "http://127.0.0.1:4750")
    assert '"stop": "goal_verified"' in capsys.readouterr().out
    # and `--executor isaac --isaac-url ...` is exactly this, once d1-isaaclab
    # has registered the name
    register_executor("isaac", fake_isaac, replace=True)
    try:
        assert astra_loop.main(["--dry-run", "--executor", "isaac",
                                "--isaac-url", "tcp://sim:8977"]) == 0
    finally:
        unregister_executor("isaac")
    assert built[-1] == ("isaac", "tcp://sim:8977")
    assert '"stop": "goal_verified"' in capsys.readouterr().out


def test_an_entry_point_and_a_class_path_register_it_too(agent_examples,
                                                         monkeypatch):
    class EntryPoint:
        name, value = "sim-ep", "some_pkg.kit_executor:isaac"

        def load(self):
            return fake_isaac

    monkeypatch.setattr(robot_module, "_entry_points",
                        lambda: {"sim-ep": EntryPoint()})
    try:
        assert "sim-ep" in robot_module.registered_executors()
        assert LiveRobot.from_flag("sim-ep", url="u").name == "fake-isaac@u"
    finally:
        unregister_executor("sim-ep")
    module = types.ModuleType("fake_sim_pkg")
    module.factory = fake_isaac
    # an Executor rather than a robot is wrapped with the declared scene
    module.bare = lambda *, kin, policy, scene=None, url=None: \
        KinematicExecutor(kin)
    monkeypatch.setitem(sys.modules, "fake_sim_pkg", module)
    assert LiveRobot.from_flag("x", executor_class="fake_sim_pkg:factory",
                               url="v").name == "fake-isaac@v"
    wrapped = LiveRobot.from_flag("x", executor_class="fake_sim_pkg:bare",
                                  scene={"objects": [
                                      {"name": "cup", "kind": "container",
                                       "p": [0.4, 0, 0.05],
                                       "size": [0.08, 0.08, 0.1]}]})
    assert isinstance(wrapped.executor, KinematicExecutor)
    assert wrapped.world().find("cup").provenance == "declared"


def test_an_unknown_executor_flag_fails_with_a_clear_message(agent_examples,
                                                             capsys):
    with pytest.raises(UnknownExecutor) as caught:
        LiveRobot.from_flag("isaac")
    message = str(caught.value)
    assert "d1-isaaclab" in message and "firmware" in message
    assert "kinematic" in message
    with pytest.raises(UnknownExecutor) as caught:
        LiveRobot.from_flag("warp-drive")
    assert ENTRY_POINT_GROUP in str(caught.value)
    assert "--executor-class" in str(caught.value)
    with pytest.raises(UnknownExecutor):
        LiveRobot.from_flag("x", executor_class="no_such_module_here:thing")
    # the example says it as a usage error, not a traceback
    import astra_loop
    with pytest.raises(SystemExit) as exit_:
        astra_loop.main(["--dry-run", "--executor", "isaac",
                         "--isaac-url", "tcp://127.0.0.1:8977"])
    assert exit_.value.code == 2
    assert "d1-isaaclab" in capsys.readouterr().err


def test_a_broken_plugin_says_which_one(monkeypatch):
    class Broken:
        name, value = "sim-broken", "broken_pkg:isaac"

        def load(self):
            raise ImportError("No module named 'isaacsim'")

    monkeypatch.setattr(robot_module, "_entry_points",
                        lambda: {"sim-broken": Broken()})
    with pytest.raises(UnknownExecutor) as caught:
        LiveRobot.from_flag("sim-broken")
    assert "broken_pkg:isaac" in str(caught.value)
    assert "isaacsim" in str(caught.value)


def test_timeouts_are_resolved_once(agent_examples):
    """L13 / Astra review 12: the example set 4 s on the executor and the
    generic ``run()`` then used its own 3 s. The policy's numbers now reach
    the executor's constructor AND every run, the tool gate included."""
    from scene import DEMO_WRIST_CAMERA, demo_scene

    def spy_factory(*, kin, policy, scene=None, url=None, **options):
        world, _ = demo_scene()
        robot = KinematicMirror(kin, world, wrist_intrinsics=DEMO_WRIST_CAMERA)
        robot.executor = SpyExecutor(
            kin, arrive_timeout_s=policy.arrive_timeout_s,
            stroke_timeout_s=policy.stroke_timeout_s or 9.0)
        robot.source.executor = robot.executor
        robot.source.identity = lambda side: robot.executor.held.get(side)
        return robot

    register_executor("spy", spy_factory, replace=True)
    try:
        import astra_loop
        for stroke, expect in ((11.0, 11.0), (None, 9.0)):
            policy = OperatorPolicy(arrive_timeout_s=7.5,
                                    stroke_timeout_s=stroke,
                                    look_before_stroke=False, max_turns=3)
            robot = LiveRobot.from_flag("spy", policy=policy)
            model = astra_loop.ScriptedModel()
            run(goal=Place(object="red_block", to="box"), robot=robot,
                policy=policy, ask=model, on_side=model.use_side)
            spy = robot.executor
            assert spy.configured["arrive_timeout_s"] == 7.5
            assert spy.runs, "nothing ran"
            for handed in spy.runs:
                assert handed["arrive_timeout_s"] == 7.5
                assert handed["gate_timeout_s"] == 7.5
                # None = the EXECUTOR's own bound (on firmware, the daemon
                # document's), resolved once and passed to every run
                assert handed["stroke_timeout_s"] == expect
    finally:
        unregister_executor("spy")


def test_the_firmware_executor_is_built_from_the_policy(monkeypatch):
    """No robot: the executor class is replaced by a recorder."""
    import manipulation_kit.executors.firmware as fw

    made = {}

    class Recorder:
        def __init__(self, **kw):
            made.update(kw)

    monkeypatch.setattr(fw, "FirmwareExecutor", Recorder)
    policy = OperatorPolicy(vel_ratio=0.3, arrive_timeout_s=6.0)
    robot = LiveRobot.from_flag("firmware", url="http://d1-x:4750",
                                policy=policy)
    assert isinstance(robot.executor, Recorder)
    assert made == {"base_url": "http://d1-x:4750", "vel_ratio": 0.3,
                    "acc_ratio": 0.3, "arrive_timeout_s": 6.0,
                    "stroke_timeout_s": None}
