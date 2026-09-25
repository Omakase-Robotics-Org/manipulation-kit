"""manipulation_kit.agent — the consumer-shaped layer: policy, loop, robot, trace.

Everything a model-driven loop needs that is NOT model-facing text or a provider call
(design C.8, C.12). Before 0.16.0 it lived in ``examples/agent/`` — the
operator policy as eight environment variables, the executor lifecycle as an
``ExitStack``, the held object as ``robot.expect()`` — where a wheel customer
could not get it and the suite could not see it.

    policy.py   OperatorPolicy — the grip cap, the direction restriction, the
                look before a stroke, the nudge budget, the timeouts, the turn
                cap; ``clamp()`` returns kit ``Unmet`` s
    loop.py     run(): observe -> gate -> run -> verify, provider-independent
    robot.py    LiveRobot (executor + world source, a context manager),
                KinematicMirror, the ``--executor`` registry
    tools.py    declare_scene / locate, the observation tools
    servo.py    Servo: the look before a stroke answered by a judge that
                CHOOSES (on / left / right / above / below the drawn mark),
                the kit stepping the hand on its own grid — System 1
    trace.py    DecisionRecord / DecisionTrace: the claim beside the measurement

    from manipulation_kit.agent import LiveRobot, OperatorPolicy, run
    from manipulation_kit.primitives import Place

    policy = OperatorPolicy(max_grip="soft", vel_ratio=0.15)
    with LiveRobot.from_flag("firmware", url=url, policy=policy,
                             scene=scene) as robot:
        trace = run(goal=Place(object="cube", to="cup"), robot=robot,
                    policy=policy, ask=my_model, system=MY_SYSTEM_TEXT)
    print(trace.summary())

No provider is imported here, and nothing here opens a socket: the firmware
executor is imported only when ``--executor firmware`` builds one.
"""

from .loop import STOP_REASONS, ObservationError, Stop, robot_facts, run
from .policy import (DIRECTION_NOT_ALLOWED, LOOK_REQUIRED, LOOK_UNAVAILABLE,
                     NUDGE_LIMIT, OperatorPolicy, PolicyState)
from .robot import (ENTRY_POINT_GROUP, KinematicMirror, LiveRobot, SceneSource,
                    UnknownExecutor, WorldSource, register_executor,
                    registered_executors, unregister_executor)
from .servo import (CHOICES as SERVO_CHOICES, Servo, ServoLook, ServoReport,
                    geometry_judge)
from .trace import DecisionRecord, DecisionTrace

__all__ = [
    "DIRECTION_NOT_ALLOWED", "ENTRY_POINT_GROUP", "LOOK_REQUIRED",
    "LOOK_UNAVAILABLE", "NUDGE_LIMIT", "SERVO_CHOICES", "STOP_REASONS",
    "DecisionRecord", "DecisionTrace", "KinematicMirror", "LiveRobot",
    "ObservationError", "OperatorPolicy", "PolicyState", "SceneSource",
    "Servo", "ServoLook", "ServoReport", "Stop", "UnknownExecutor",
    "WorldSource", "geometry_judge", "register_executor",
    "registered_executors", "robot_facts", "run", "unregister_executor",
]
