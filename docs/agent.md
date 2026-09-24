# `manipulation_kit.agent` — the loop, the policy, the robot

One loop, one operator policy, one prompt and one trace, on the real robot, in
Isaac, or in the kinematic mirror. Only the executor and the world source
differ, and which pair is built is a flag.

```python
from manipulation_kit.agent import DecisionTrace, LiveRobot, OperatorPolicy, run
from manipulation_kit.description.camera_calibration import DEFAULT_PATH
from manipulation_kit.description.robot_profile import RobotProfile
from manipulation_kit.primitives import Place

policy = OperatorPolicy(max_grip="soft", allowed_directions=("down",),
                        vel_ratio=0.15, droop_margin_m=0.012)   # d1-2
with LiveRobot.from_flag("firmware", url="http://d1-2:4750", policy=policy,
                         scene=scene, profile=RobotProfile.load(DEFAULT_PATH)) as robot:
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
  hand. It needs **measured wrist intrinsics per robot**: the robot's
  calibration file's `left_wrist`/`right_wrist` lenses (d1-2's two fisheyes were measured 2026-09-22 with
  d1-inference `d1-calibrate-wrist`, `calibration/wrist_fisheye.py`), or a
  scene's `robot.wrist_camera`. The lens model is used as measured
  (`model: fisheye`, `k1..k4`, pixels outside `valid_radius_px` not trusted).
  Without intrinsics the loop refuses to start (`look_unavailable`); a block
  marked `"measured": false` is used by the kinematic mirror only.
  The wrist lens's EXTRINSIC is the calibration file's MEASURED mount when
  it has one (`cameras.<side>_wrist.mount`, seiryu-calib's plate -> optical
  fit in `gripper_R_camera_plate` for the logical left arm,
  `gripper_L_camera_plate` for the right), else the kit's nominal plate
  geometry; the camera model and every `look` record say which
  (`mount: measured | nominal`). **First live run on a robot without a
  measured wrist mount: `--no-look-before-stroke`** — own the blind grasp
  and turn the look on once a wrist photo is seen to agree with the
  projection.
- `run(..., servo=Servo(frame, judge))` — **the look answered by a judge
  that chooses, not by the model** (`manipulation_kit.agent.servo`; System 1
  under a System 2 model). Before EVERY stroke of `SERVO_VERBS` (`grasp` on
  its `object`, `press` on its `target`; not `probe`, which has no named
  target, `handover`, which closes inside its own plan, or `approach`, the
  move to the look) the kit takes a fresh
  wrist photo (`frame(side) -> path`), DRAWS its belief on it — a green cross
  at the projected centre and a green box, the object's projected outline
  grown by `tolerance_m` (default 10 mm) on every side — and asks the judge
  one five-way question — the object is `on` (inside the box), or sticks out
  `left` / `right` / `above` / `below` it in the image, or `not_visible`
  (`judge(look) -> {choice: probability}`). A direction becomes
  one base-frame step through the wrist camera's orientation: the object is
  re-declared that step away as a JUDGEMENT (`provenance="judged"`, the
  judge's probability in `confidence`; verifiers treat it like `declared`,
  never as a sighting) and the hand is moved by the same step with the kit's `Nudge` (coarse 30 mm, then 10
  mm once the answer changes sign), within `max_nudges_per_target`. `on`
  counts as the look and the grasp runs in the same turn; anything else ends
  the servo with a named outcome (`unsure` below `min_confidence`, `nudge_budget`,
  `not_visible`, `unmappable_direction`) that the model reads and chooses on.
  Every judgement and its distribution is in the trace (`record.servo`). The
  kit's own stand-in is `geometry_judge(truth)`, which answers from where a
  known point projects; `examples/agent/jev_servo.py` plugs in Jev-Omni, a
  12B multimodal decision classifier. Why a drawn box and a relative
  question: on rendered wrist frames that classifier answered an open "which
  way" with the same option on every frame; against a drawn mark it placed
  the object correctly, and against the outline box it aligned a block
  declared 25-60 mm off in one or two steps on 5 of 5 kinematic-mirror runs
  (residual 5-30 mm — it accepts a block that overlaps the box's edge, so
  the box is a coarse tolerance, not a fine one; report `jev-servo-loop`).
  Roll is not asked for — it stays planner-only (`roll_candidates`).
  **The model's own wrist look does not replace the servo.** The model may
  `locate` on a wrist camera before a stroke, as before: its re-measurement
  re-declares the object and nudges the hand, and it satisfies the operator
  policy's look. The servo is STILL asked when the stroke comes, on the
  declaration the model left (the trace keeps it: the locate turn's answer and
  world, and the servo's first mark, `declared_p`). If the judge steps, the
  object is re-declared `provenance="judged"` and that is what the stroke
  plans to — the judged pose wins for the alignment; if it says `on`, the
  model's declaration stands. (d1-2, 2026-09-24 01:47Z, `--judge-only`: the
  servo ran only when the look was unmet, the model located before each
  grasp, and all twelve records had `servo: null` while the tape sat cm off
  the jaws.) Every judged stroke is recorded — `record.servo` is never null
  when a servo is configured and the verb is a servo verb; a photo judge
  with no wrist photo (no snapshotter, or a grab without that hand's file)
  is not asked and records `{"skipped": "no wrist frame", ...}`, and the
  model's look rule follows. A judge that answers without a photo says so
  (`photoless(judge)`; `geometry_judge` is one). The marked photo is saved
  beside the trace as `turn{N}_servo_{side}.png` (`_2`, `_3` for later
  steps of the same turn), and `servo_line(record.servo)` is the operator's
  one-liner, which `jev_servo.py` prints per stroke as it happens
  (`servo judged: on 0.81 (left 0.12, ...) — recorded only`). **Not yet
  aligned on hardware**: the judge has been run judge-only on d1-2 once.
  - `Servo(observe_only=True)` (`jev_servo.py --judge-only`): judge and
    record before every stroke, never step; the model's look rule then
    applies as without a servo (the look question when the policy's look is
    unmet, else the stroke runs). The first live sessions run this way.
  - `Servo(refine=True)` (`--refine`, off by default): after `on`, the same
    photo is re-marked on a 3 x 3 grid of 5 mm shifts and the DECLARATION
    moves to the best-judged one (only if it beats the unshifted mark); the
    hand does not move. With a perfect judge the residual ends within 5 mm
    instead of within the 10 mm tolerance. Turn it on only after real photos
    show the judge separates 5 mm.
  - The 12B classifier does not fit a D1's Jetson: run
    `examples/agent/jev_judge_server.py` on a workstation (loopback or a
    Tailscale address, no authentication) and pass `--judge-url`; expect
    ~47 GB of GPU memory, 80-200 ms per warm judgement (~0.7-0.8 s for the
    first) on an RTX PRO 6000, plus the network round trip. `jev_servo.py --rejudge RUN_DIR
    --robot-profile PATH --judge-url URL` judges a finished run's saved wrist
    photos again, offline, and prints each distribution beside the mark.
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

## The robot profile — the values live on the robot

**The kit owns the schema and the reader; the robot holds the values.** No
robot's numbers ship in the wheel. Each robot keeps ONE
`omakase.camera_calibration/2` file, `~/.config/omakase/camera_calibration.json`
(`manipulation_kit.description.camera_calibration`, JSON Schema shipped as
`description/schemas/omakase.camera_calibration-2.schema.json`), written by
seiryu-calib: per camera slot (`head`, `left_wrist`, `right_wrist`) a lens
layer (`intrinsics`, seiryu distortion vocabulary — `kannala_brandt` is the
kit's `fisheye`) and a mount layer (`mount`: the ABSOLUTE optical-frame pose
in `parent_link`, with the nominal it was fitted against), plus the hand's
driven-open gap.

`RobotProfile.load(path)` builds the kit's view of it: the head mount becomes
the `HeadMountDelta` on its recorded nominal (the head camera applies it and
says `calibrated: true` only then), the wrist lenses become per-side
`WristIntrinsics`, the wrist mounts become per-side `WristMount`s (the
ABSOLUTE `plate -> optical` pose; `perception.WristCamera` composes it with
the arm's FK in place of the nominal, and says `mount: "measured"`,
`calibrated: true` only then), `hand` becomes `HandMeasurement`. A wrist
mount expressed in any link but its side's camera plate is refused — the
logical left arm is the URDF `_R` tree (`manipulation_kit.arms.sides`). On
d1-2 (installed 2026-09-24) the measured wrist lenses sit 16-20 mm from the
nominal, ~30 px of jaw-pad shift in the fisheye; the head mount is
seiryu-calib `head-correct` (arm FK as the world reference, 0.43 px).

Every layer has a gate. A layer whose gate is **FAIL is refused** unless the
file records a `gate.override` with a reason, or the caller passes
`allow_failed_gate=True` (`--allow-failed-calibration` on `astra_loop.py` and
`perceive.py`); WARN is accepted with a warning.

`astra_loop.py --robot-profile PATH` and `perceive.py --robot-profile PATH`
default to the robot's `~/.config/omakase/camera_calibration.json` when it
exists (for `astra_loop.py --scene`, a file the scene names comes first), else none. A scene
may name a file (`"robot": {"profile": "PATH"}`, relative to the scene) and
override any of its keys. Offline, d1-2's file is
`tests/data/d1-2.camera_calibration.json`.

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
