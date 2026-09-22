# `manipulation_kit.agent` — the loop, the policy, the robot

One loop, one operator policy, one prompt and one trace, on the real robot, in
Isaac, or in the kinematic mirror. Only the executor and the world source
differ, and which pair is built is a flag.

```python
from manipulation_kit.agent import DecisionTrace, LiveRobot, OperatorPolicy, run
from manipulation_kit.primitives import Place

policy = OperatorPolicy(max_grip="soft", allowed_directions=("down",))
with LiveRobot.from_flag("firmware", url="http://d1-2:4750", policy=policy,
                         scene=scene) as robot:
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
- `look_before_stroke` (default on) needs the wrist lens intrinsics
  (`scene["robot"]["wrist_camera"] = {fx, fy, cx, cy, width, height}`); without
  them the loop refuses to start rather than grasp blind.
- A held object rides the tool (`manipulation_kit.world.attach`) and carries
  `provenance="attached"`; the verifiers say "inferred, not sighted" for it.

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
