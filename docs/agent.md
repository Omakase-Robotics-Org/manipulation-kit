# `manipulation_kit.agent` — the loop, the policy, the robot

One loop, one operator policy, one prompt and one trace, on the real robot, in
Isaac, or in the kinematic mirror. Only the executor and the world source
differ, and which pair is built is a flag.

```python
from manipulation_kit.agent import DecisionTrace, LiveRobot, OperatorPolicy, run
from manipulation_kit.description.robot_profile import RobotProfile
from manipulation_kit.primitives import Place

policy = OperatorPolicy(max_grip="soft", allowed_directions=("down",),
                        vel_ratio=0.15, droop_margin_m=0.012)   # d1-2
with LiveRobot.from_flag("firmware", url="http://d1-2:4750", policy=policy,
                         scene=scene, profile=RobotProfile.named("d1-2")) as robot:
    trace = run(goal=Place(object="cube", to="cup"), robot=robot,
                policy=policy, ask=my_model, system=MY_PROMPT,
                trace=DecisionTrace(run_dir / "trace.jsonl"))
print(trace.stop, trace.summary())
```

- `ask(messages, tools) -> {"name", "arguments", "call_id", "claimed"}` is the
  only thing that knows a model exists.
- `OperatorPolicy` is the operator's knobs, as fields (`to_json`/`from_json`,
  CLI flags via `OperatorPolicy.add_arguments`). `clamp(call)` lowers the grip
  and contact caps and refuses what the policy forbids with kit `Unmet`s.
- `look_before_stroke` (default on): before every `grasp` stroke the loop
  answers with one wrist look (`WristCamera.project_object`) from the posture
  the stroke closes from, and a `locate` on that wrist camera corrects the
  hand. It needs **measured wrist intrinsics per robot**: a robot profile's
  `wrist_cameras` (d1-2's two fisheyes were measured 2026-09-22 with
  d1-inference `d1-calibrate-wrist`, `calibration/wrist_fisheye.py`), or a
  scene's `robot.wrist_camera`. The lens model is used as measured
  (`model: fisheye`, `k1..k4`, pixels outside `valid_radius_px` not trusted).
  Without intrinsics the loop refuses to start (`look_unavailable`); a block
  marked `"measured": false` is used by the kinematic mirror only.
  **First live run on a robot: `--no-look-before-stroke`.** The wrist
  lens's EXTRINSIC is still the kit's nominal plate geometry, and the look
  policy has not been exercised on hardware; own the blind grasp for the first
  session and turn the look on once a wrist photo is seen to agree with the
  projection.
- `droop_margin_m` (`--droop-margin-m`): how far the real arm sags below the
  commanded pose, added to the fingertip floor of a descent and to every
  scene clearance (`primitives.clearance.ClearancePolicy`). 0.0 = the rigid
  model; **d1-2 measured 0.012**.
- `allowed_directions` restricts every verb whose direction is how the hand
  ARRIVES — `Primitive.DIRECTION_ARRIVES`, collected in
  `agent.policy.DIRECTED_VERBS` (approach, grasp, probe, press, handover) — and
  not a Lift's "up" or a Retreat.
- A held object rides the tool (`manipulation_kit.world.attach`) and carries
  `provenance="attached"`; the verifiers say "inferred, not sighted" for it.
  One that passed hand to hand (`handover`) is re-attached at the receiver's
  pad centre.
- Contacts measured by `probe`/`press` are kept across turns
  (`LiveRobot.remember_contacts` / `forget_contacts`, which `declare_scene`
  calls); a contact leg's scene check leaves out only the surface the leg is
  aimed at (`SceneGate.for_contact`).
- `handover` is refused under `look_before_stroke` (`look_not_possible`: its
  receiving close has no posture a look can precede); with the look on, the
  model composes it — approach the held object with the other hand, grasp
  (looked), release, retreat.

## The robot profile

One typed file per robot (`manipulation_kit.description.robot_profile`):
the hand's driven-open gap, the head camera's measured mount, the wrist
lenses. `RobotProfile.named("d1-2")` is committed with the kit;
`RobotProfile.from_files(name, head_calibration=..., wrist={"left": ...,
"right": ...})` reads the d1-inference artefacts verbatim. A scene names one
(`"robot": {"profile": "d1-2"}`) and may override any of its keys;
`astra_loop.py --robot-profile NAME|FILE` and `perceive.py --robot-profile`
do the same from the command line. The head camera applies the mount and
says `calibrated: true` only then.

## The example's one deliberate lie

`astra_loop.py --dry-run` plays `examples/agent/scripted.py`, which claims
"done" at the carry, one verb early. The run then prints `CLAIMED DONE, NOT
MEASURED, at turn 4` and goes on to the place: that line is the point — the
loop ends on a measurement, not on what the model says.

## Executors: `--executor NAME`

`firmware` and `kinematic` are built in. Anything else is found, in order, in
`register_executor(name, factory)`, then the `manipulation_kit.executors`
entry-point group of any installed distribution, or named directly with
`executor_class="module:factory"` (`--executor-class`). A factory is called as
`factory(kin=, policy=, scene=, url=, **options)` and returns a `LiveRobot` (or
a bare `Executor`, which the kit gives the declared scene as its world). It
must build its executor FROM `policy` — that is what makes the timeouts the
same numbers everywhere. The kit never imports the out-of-tree package.

### Isaac (d1-isaaclab)

```python
# d1-isaaclab: scripts/eval/agent_eval/kit_executor.py
from urllib.parse import urlsplit
from manipulation_kit.agent import LiveRobot

def isaac(*, kin, policy, scene=None, url=None, wrist_intrinsics=None, **_):
    from .client import EnvClient
    from .executor import IsaacExecutor
    from .world import IsaacWorldSource
    where = urlsplit(url or "tcp://127.0.0.1:8977")
    client = EnvClient(where.hostname or "127.0.0.1", where.port or 8977)
    executor = IsaacExecutor(client)
    executor.span = executor.measure_gripper_span()
    source = IsaacWorldSource(client, kin, span=executor.span)
    source.calibrate()                     # frame agreement, MEASURED
    return LiveRobot(executor, source, kin, name="isaac", closing=(client,),
                     wrist_intrinsics=wrist_intrinsics)   # the sim wrist camera's
```

and either register it as an entry point of an installed distribution

```toml
[project.entry-points."manipulation_kit.executors"]
isaac = "agent_eval.kit_executor:isaac"
```

or, with `scripts/eval` on `PYTHONPATH`, name it:
`astra_loop.py --executor isaac --executor-class agent_eval.kit_executor:isaac
--isaac-url tcp://127.0.0.1:8977`. Without either, `--executor isaac` fails
with a message naming d1-isaaclab.
