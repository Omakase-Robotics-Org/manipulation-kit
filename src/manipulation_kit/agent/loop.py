"""The provider-independent turn: observe -> gate -> run -> verify.

A function-calling model is shown the task, the world as text (and photos, if
the caller has them) and the kit's verbs as JSON Schema tools; it names ONE
verb with arguments; the kit decodes and re-checks the BOUND call — the
operator policy first, then the planner and the guard — the plan runs on the
robot's executor, and the turn ends with a MEASURED verdict rather than the
model's opinion::

    from manipulation_kit.agent import LiveRobot, OperatorPolicy, run
    policy = OperatorPolicy(max_grip="soft")
    with LiveRobot.from_flag("firmware", url=url, policy=policy,
                             scene=scene) as robot:
        trace = run(goal=Place(object="cube", to="cup"), robot=robot,
                    policy=policy, ask=my_model, system=MY_PROMPT,
                    trace=DecisionTrace(run_dir / "trace.jsonl"))

``ask(messages, tools) -> {"name", "arguments", "call_id", "claimed"}`` is
the ONLY thing that knows a model exists (``name=None`` = the model stopped;
``protocol_error`` = it broke the one-call contract). Everything else — the
arm choice, the policy, the look before a stroke, the stop conditions, the
trace — is here, versioned and tested, instead of in an example.

Stop reasons, never conflated (``trace.stop``):

``goal_verified``      the task verifier measured TRUE
``model_stopped``      the model returned no call; the goal is still measured
``max_turns``          :attr:`OperatorPolicy.max_turns`, the hard cap
``unreachable_task``   no arm plans the whole chain, refused at turn zero
``controller_fault``   the kit stopped a run on a latched controller; not
                       something a model can talk its way out of
``look_unavailable``   the policy requires a wrist look before every stroke
                       and this robot has no wrist camera model
``observation_failed`` the caller's ``observe`` could not produce a fresh
                       observation (:class:`ObservationError`): no blind turn
``interrupted`` / ``exception``  recorded, then re-raised

THE LOOK BEFORE A STROKE (``look_before_stroke``, requirement 2). A ``grasp``
on an object the hand has not looked at from this posture is not run: the
kit projects the object into that hand's wrist camera
(:meth:`~manipulation_kit.perception.WristCamera.project_object`) and tells
the model where it should appear. The model answers with ``grasp`` again (the
photo agrees) or with ``locate`` on the wrist camera — the correction: the
object is re-measured there (``provenance="observed"``) and the hand is moved
by the difference with the kit's own :class:`~manipulation_kit.primitives.Nudge`.
Corrections count against :attr:`OperatorPolicy.max_nudges_per_target`.
"""

from __future__ import annotations

import copy
import dataclasses
import math
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

from ..executor import CONTROLLER_FAULT, run as run_plan
from ..primitives import Place
from ..primitives.offer import arguments as bound_arguments
from ..primitives.offer import check, label_for
from ..primitives.contact import record_contacts
from ..primitives.reach import choose_side
from ..primitives.schema import decode, direction_doc, tool_schemas
from ..primitives.types import (NUDGE_GRID_M, NUDGE_MAX_YAW_RAD,
                                PRECONDITION_UNMET, PlanError, Primitive)
from .policy import LOOK_REQUIRED, OperatorPolicy, PolicyState
from .tools import SCENE_TOOLS, apply_declare_scene, apply_locate, scene_tools
from .tools import locate as locate_point
from .trace import DecisionRecord, DecisionTrace

Ask = Callable[[List[Dict[str, Any]], List[Dict[str, Any]]], Dict[str, Any]]
Observe = Callable[[int, Any], List[Dict[str, Any]]]

STOP_REASONS: Tuple[str, ...] = (
    "goal_verified", "model_stopped", "max_turns", "unreachable_task",
    CONTROLLER_FAULT, "look_unavailable", "observation_failed",
    "interrupted", "exception")


class ObservationError(RuntimeError):
    """A fresh observation (the photos a visual model acts on) could not be
    taken. The loop stops with ``observation_failed`` rather than letting the
    model act on no picture or an old one (Astra review 14)."""


@dataclass
class Stop:
    reason: str
    detail: str = ""


def robot_facts(world: Any = None) -> str:
    """The numbers the kit OWNS, generated rather than quoted in a prompt:
    the jaw capacity (from the hand the world measured, else the nominal
    description), the nudge grid and yaw clamp, the contact modes (``tip``
    flagged experimental until the d1-2 tip trial), the tool revision
    (:func:`~manipulation_kit.primitives.orientation.tool_revision`) and the
    direction vocabulary (:func:`~manipulation_kit.primitives.schema.direction_doc`)."""
    from ..hands.d1.parallel_gripper.description import (  # noqa: PLC0415
        DRIVEN_OPEN_GAP_M)
    from ..primitives.grasp_geometry import (PAD,  # noqa: PLC0415
                                             graspable_width_m)
    from ..primitives.orientation import tool_revision  # noqa: PLC0415
    opening = None
    for gripper in (getattr(world, "grippers", None) or {}).values():
        if gripper.open_gap_m is not None:
            opening = float(gripper.open_gap_m)
            break
    open_m = DRIVEN_OPEN_GAP_M if opening is None else opening
    grid = "/".join(f"{g * 1000:.0f}" for g in NUDGE_GRID_M)
    lines = [
        "ROBOT FACTS (generated by the kit from this robot, not quoted):",
        f"- the jaws open {open_m * 1000:.0f} mm and take at most "
        f"{graspable_width_m(PAD, opening) * 1000:.0f} mm across "
        f"({'measured on this hand' if opening is not None else 'nominal hand description'})",
        f"- `nudge` translations snap to a {grid} mm grid per axis, and its "
        f"`dyaw` is clamped to +-{math.degrees(NUDGE_MAX_YAW_RAD):.0f} degrees "
        f"about the hand's own approach axis",
        "- `contact` is where on the hand the object is taken: \"pad\" (the "
        "default) or \"tip\" for something flat on a surface. \"tip\" is "
        "EXPERIMENTAL: not yet measured on hardware (docs/"
        "probe-hardware-trial.md, tip grasp trial), so prefer \"pad\"",
        f"- tool geometry this robot plans with: {tool_revision()}",
        "- directions (the way the hand TRAVELS):"]
    lines += ["  " + line for line in direction_doc().splitlines()]
    return "\n".join(lines)


def _say(messages: List[Dict[str, Any]], call_id: str, text: str) -> None:
    """Feed a result back CORRELATED with the call that produced it."""
    messages.append({"role": "user",
                     "content": (f"[result of {call_id}] {text}" if call_id
                                 else text)})


def _content(text: str, parts: Sequence[Dict[str, Any]]) -> Any:
    if not parts:
        return text
    return [{"type": "input_text", "text": text}] + list(parts)


def _forget_images(messages: List[Dict[str, Any]], keep: int) -> None:
    """Replace the photos of all but the last ``keep`` observations with their
    labels — an in-memory history that resends every earlier image grows the
    request without telling the model anything (Astra review 15)."""
    with_images = [m for m in messages if isinstance(m.get("content"), list)
                   and any(p.get("type") == "input_image" for p in m["content"])]
    for message in with_images[:max(0, len(with_images) - keep)]:
        message["content"] = [
            {"type": "input_text",
             "text": f"[{p.get('_file', 'photo')}: an earlier photo, not "
                     f"resent]"} if p.get("type") == "input_image" else p
            for p in message["content"]]


def _side_of(primitive: Primitive, world: Any) -> Optional[str]:
    for attr in ("resolve_side", "_side"):
        fn = getattr(primitive, attr, None)
        if fn is not None:
            try:
                side = fn(world)
            except Exception:  # noqa: BLE001 - an unresolvable side is None
                side = None
            if side in ("left", "right"):
                return side
    side = getattr(primitive, "side", None)
    return side if side in ("left", "right") else None


class _Loop:
    """One run's state. :func:`run` is the public door."""

    def __init__(self, *, robot, policy: OperatorPolicy, ask: Ask, goal,
                 task: str, system: str, trace: DecisionTrace,
                 observe: Optional[Observe], on_side, keep_images: int,
                 servo=None):
        if not isinstance(goal, Place):
            raise TypeError("run() takes the task as a Place(object=, to=) "
                            "goal: its verifier is what decides success")
        self.robot = robot
        self.policy = policy
        self.ask = ask
        self.goal = goal
        self.obj, self.destination = goal.object, goal.to
        self.task = task or f"put the {goal.object} in the {goal.to}"
        self.system = system
        self.trace = trace
        self.observe = observe
        self.on_side = on_side
        self.servo = servo
        self.keep_images = int(keep_images)
        self.state = PolicyState()
        #: resolved ONCE (L13): the same numbers the executor was built with
        self.settings = policy.settings(robot.executor)
        # the arm's droop reaches every plan's fingertip floor through the
        # kinematics (primitives.clearance.ClearancePolicy)
        policy.apply_to(robot.kin)
        self.messages: List[Dict[str, Any]] = []
        self.hand = None
        self.goal_ready = False

    # -- the arm, chosen by planning the whole chain ------------------------- #
    def plan_the_hand(self, world) -> None:
        names = set(world.names())
        if self.obj not in names or self.destination not in names:
            return
        # The whole chain, per arm; there is no roll here — every Approach /
        # Grasp in the chain sweeps the candidate rolls of grasp_geometry itself
        # (the ONE sweep, design D.1 row 3), and a Grasp continues the roll
        # its Approach stood at.
        self.hand = choose_side(world, self.robot.kin, obj=self.obj,
                                destination=self.destination)
        if self.goal.side in ("", "auto"):
            self.goal = dataclasses.replace(self.goal, side=self.hand.side)
        if self.on_side is not None:
            self.on_side(self.hand.side)
        self.goal_ready = True

    def _stop_at_zero(self, world, reason: str, detail: str,
                      refused: Optional[List[Dict[str, Any]]] = None) -> Stop:
        record = DecisionRecord(iteration=0, world=world.to_json(),
                                task=self.task)
        record.stop = reason
        record.refused = list(refused or [])
        self.trace.write(record)
        self.trace.save_messages(self.messages)
        return Stop(reason, detail)

    # -- the run -------------------------------------------------------------- #
    def run(self) -> DecisionTrace:
        self.trace.task = self.task
        world0 = self.robot.world()
        self.messages = [{"role": "system", "content": self.system},
                         {"role": "user", "content": f"TASK: {self.task}"},
                         {"role": "user", "content": self.policy.describe()},
                         {"role": "user", "content": robot_facts(world0)}]
        head = getattr(self.robot, "head_camera", None)
        if head is not None:
            self.messages.append({"role": "user", "content": head.to_text()})
        stop = self._start(world0)
        if stop is None:
            stop = Stop("max_turns", f"{self.policy.max_turns} turns without "
                                     f"a measured goal")
            for turn in range(self.policy.max_turns):
                ended = self._turn_recorded(turn)
                if ended is not None:
                    stop = ended
                    break
        self.trace.stop = stop.reason
        self.trace.stop_detail = stop.detail
        return self.trace

    def _start(self, world0) -> Optional[Stop]:
        if self.policy.look_before_stroke and not self.robot.has_wrist_camera():
            # FAIL CLOSED, before anything moves: every task here needs a
            # stroke, and the policy forbids a blind one.
            return self._stop_at_zero(
                world0, "look_unavailable",
                "the operator policy requires a wrist look before every grasp "
                "stroke and this robot has no wrist camera model (no "
                "intrinsics). Give the scene robot.wrist_camera {fx, fy, cx, "
                "cy, width, height}, or run with look_before_stroke=False "
                "(--no-look-before-stroke) and own the blind grasp. A LIVE "
                "run needs the lens's MEASURED intrinsics: a placeholder "
                "marked \"measured\": false is used by the kinematic mirror "
                "only, never on hardware")
        self.plan_the_hand(world0)
        if self.hand is None:
            # NOT AN ERROR: the model declares the things from the photos.
            self.messages.append({"role": "user", "content": (
                f"The scene does not contain {self.obj!r} and "
                f"{self.destination!r} yet. Look at the photographs and call "
                f"declare_scene with where they are, in base metres; use "
                f"locate on the pixel where each one touches the table rather "
                f"than estimating by eye. Then act.")})
        elif not self.hand.reachable:
            # F10/F12: an impossible task is refused at turn zero rather than
            # discovered three verbs in.
            return self._stop_at_zero(
                world0, "unreachable_task", self.hand.reason,
                [c.to_json() for c in self.hand.chains.values()])
        return None

    def _turn_recorded(self, turn: int) -> Optional[Stop]:
        """One turn, written to the trace however it ends — a model error, a
        planning crash or Ctrl-C included (Astra review 15)."""
        record = DecisionRecord(iteration=turn, world={}, task=self.task)
        try:
            stop = self._turn(turn, record)
        except ObservationError as exc:
            record.stop, record.error = "observation_failed", repr(exc)
            stop = Stop("observation_failed", str(exc))
        except BaseException as exc:
            record.error = repr(exc)
            record.stop = ("interrupted" if isinstance(exc, KeyboardInterrupt)
                           else "exception")
            self.trace.write(record)
            self.trace.save_messages(self.messages)
            self.trace.stop = record.stop
            self.trace.stop_detail = repr(exc)
            raise
        self.trace.write(record)
        self.trace.save_messages(self.messages)
        return stop

    def _turn(self, turn: int, record: DecisionRecord) -> Optional[Stop]:
        world = self.robot.world()
        record.world = world.to_json()
        parts = self.observe(turn, world) if self.observe is not None else []
        _forget_images(self.messages, self.keep_images - 1)
        self.messages.append({"role": "user",
                              "content": _content(world.to_text(), parts)})
        cameras = self.robot.cameras(world)
        tools = tool_schemas(world) + scene_tools(cameras)
        call = self.ask(self.messages, tools)
        call_id = call.get("call_id") or ""
        # VERBATIM, and a COPY: nothing the kit does to the call later may
        # rewrite what the model asked for.
        record.choice = copy.deepcopy({"name": call.get("name"),
                                       "arguments": call.get("arguments") or {},
                                       "call_id": call_id})
        record.claimed = call.get("claimed") or None
        arguments = copy.deepcopy(call.get("arguments") or {})
        if call.get("protocol_error"):
            _say(self.messages, call_id, call["protocol_error"])
            return None
        if not call.get("name"):
            return self._stopped(world, record)
        if call["name"] in SCENE_TOOLS:
            return self._observation_tool(call["name"], arguments, world,
                                          cameras, record, call_id)
        if not self.goal_ready:
            _say(self.messages, call_id,
                 f"nothing can be planned until {self.obj!r} and "
                 f"{self.destination!r} are in the scene. Call declare_scene "
                 f"first.")
            return None
        return self._motion(call["name"], arguments, world, cameras, record,
                            call_id)

    # -- the three kinds of answer ------------------------------------------ #
    def _stopped(self, world, record) -> Stop:
        record.stop = "model_stopped"
        if not self.goal_ready:
            return Stop("model_stopped",
                        f"the model stopped without declaring {self.obj!r} "
                        f"and {self.destination!r}; nothing was ever planned")
        report = self.goal.verifier(world)(self.robot.world())
        record.goal_verdict = report.to_json()
        return Stop("goal_verified" if report.verdict == "true"
                    else "model_stopped", report.reason)

    def _observation_tool(self, name, arguments, world, cameras, record,
                          call_id) -> None:
        if name == "declare_scene":
            answer = apply_declare_scene(self.robot, arguments)
            self.plan_the_hand(self.robot.world())
            if self.hand is not None:
                answer += (f". Planning the whole chain says the "
                           f"{self.hand.side} arm"
                           + ("" if self.hand.reachable
                              else f", and nothing reaches yet: "
                                   f"{self.hand.reason}"))
        elif str(arguments.get("camera", "")).endswith("_wrist"):
            answer = self._correct(arguments, world, cameras, record)
        else:
            answer = apply_locate(cameras, world, arguments)
        record.observation_after = {"tool": name, "answer": answer}
        _say(self.messages, call_id, answer)
        return None

    def _refuse(self, record, call_id, primitive, error: PlanError,
                prefix: str = "") -> None:
        record.refused = [error.to_json()]
        _say(self.messages, call_id,
             f"{prefix}{label_for(primitive)} was refused: {error}")

    def _motion(self, name, arguments, world, cameras, record,
                call_id) -> Optional[Stop]:
        primitive = decode(name, arguments, world)
        if isinstance(primitive, PlanError):
            record.refused = [primitive.to_json()]
            _say(self.messages, call_id,
                 f"that call is malformed: {primitive}")
            return None
        side = _side_of(primitive, world)
        arm = world.arm(side) if side else None
        clamped, unmet = self.policy.clamp(
            primitive, self.state, side=side,
            joints=None if arm is None else arm.joints)
        if unmet and all(u.code == LOOK_REQUIRED for u in unmet):
            if self.servo is None or self.servo.observe_only:
                if self.servo is not None:
                    # judge-only: the judge's answer is RECORDED beside the
                    # model's own look; nothing is moved or re-declared
                    self._servo(primitive, side, record, call_id)
                self._look(primitive, side, world, cameras, record, call_id,
                           unmet)
                return None
            # System 1: the kit aligns the hand by judgement and, aligned,
            # runs the stroke the model asked for in this same turn
            aligned = self._servo(primitive, side, record, call_id)
            if not aligned:
                return None
            world = self.robot.world()
            record.world = world.to_json()
            primitive = decode(name, arguments, world)
            if isinstance(primitive, PlanError):
                record.refused = [primitive.to_json()]
                _say(self.messages, call_id,
                     f"after the alignment that call is malformed: {primitive}")
                return None
            arm = world.arm(side) if side else None
            clamped, unmet = self.policy.clamp(
                primitive, self.state, side=side,
                joints=None if arm is None else arm.joints)
        if unmet:
            self._refuse(record, call_id, primitive, PlanError(
                PRECONDITION_UNMET, "; ".join(str(u) for u in unmet),
                primitive=primitive.name(), side=side or "",
                unmet=tuple(unmet)), prefix="by the operator policy, ")
            return None
        notes = self.policy.notes(primitive, clamped)
        if clamped != primitive:
            record.effective = {"name": clamped.name(),
                                "arguments": bound_arguments(clamped)}
        plan = check(clamped, world, self.robot.kin)
        if not getattr(plan, "ok", False):
            self._refuse(record, call_id, clamped, plan)
            return None
        record.offered = [{"id": clamped.name(), "label": label_for(clamped)}]
        record.plan = plan.to_json()
        report = run_plan(plan, self.robot.executor, kin=self.robot.kin,
                          **self.settings)
        record.run = report.to_json()
        ran = getattr(plan, "side", None) or side
        self._bookkeeping(clamped, ran)
        contacts = tuple(getattr(report, "contacts", ()) or ())
        record.contacts = [c.to_json() for c in contacts]
        if report.stop_reason == CONTROLLER_FAULT:
            # Leave recovery to the operator (console: Clear error -> Home).
            return Stop(CONTROLLER_FAULT, report.error)
        after = self.robot.world()
        if contacts:
            after = self._fold_contacts(after, report, clamped)
        verdict = clamped.verifier(world)(after)
        record.verdict = verdict.to_json()
        corrected = self._measured_width(verdict, record)
        if corrected:
            notes = list(notes) + corrected
            after = self.robot.world()
        record.observation_after = after.to_json()
        how = ("ok" if report.completed else
               f"{report.stop_reason} — {report.error}")
        _say(self.messages, call_id,
             ("note: " + "; ".join(notes) + ". " if notes else "")
             + f"{label_for(clamped)}: transport {how}; "
               f"measured {verdict.verdict} — {verdict.reason}")
        goal_report = self.goal.verifier(world)(after)
        record.goal_verdict = goal_report.to_json()
        if goal_report.verdict == "true":
            return Stop("goal_verified", goal_report.reason)
        return None

    def _measured_width(self, verdict, record) -> list:
        """A TRUE grasp that MEASURED the object wider or narrower than it was
        declared: the measurement replaces the declaration in the world
        source, so Carry / Place / the scene gate plan with the real size, and
        the trace records the correction (``record.corrections``)."""
        fix = (verdict.measured or {}).get("width_correction")
        if verdict.verdict != "true" or not fix or not fix.get("jaw_axis"):
            return []
        update = getattr(self.robot, "measured_width", None)
        applied = bool(callable(update) and update(
            fix["object"], fix["jaw_axis"], fix["measured_m"]))
        record.corrections.append(dict(fix, field="width_along_jaw_axis",
                                       applied=applied))
        if not applied:
            return []
        return [f"{fix['object']!r} was declared "
                f"{fix['declared_m'] * 1000:.1f} mm across the jaws and the "
                f"grasp MEASURED {fix['measured_m'] * 1000:.1f} mm; the scene "
                f"now uses the measured width"]

    def _fold_contacts(self, after, report, verb):
        """A probe / press MEASURED where it met something: fold the run's
        contacts into the world its verifier reads (an un-folded contact
        verdict is UNKNOWN, never TRUE), and hand a surface it fitted to the
        world source so later turns plan against it.

        The contacts are MEASUREMENTS, so the world source keeps them
        across turns (``remember_contacts``) until the next ``declare_scene``
        restates the scene: three probes over three turns under one
        ``declare_as`` fit one plane.
        """
        folded = record_contacts(after, report, verb)
        remember = getattr(self.robot, "remember_contacts", None)
        if callable(remember):
            remember(folded.contacts)
        name = str(getattr(verb, "declare_as", "") or "")
        surface = folded.find(name) if name else None
        if surface is not None and getattr(self.robot, "can_declare", False):
            self.robot.declare([surface])
        return folded

    def _bookkeeping(self, primitive, side: Optional[str]) -> None:
        if side not in ("left", "right"):
            return
        verb = primitive.name()
        if verb in ("approach", "grasp"):
            self.state.aimed(side, primitive.object)
        if verb == "nudge":
            self.state.nudged(side)
        # the arm has gone somewhere no look saw
        self.state.moved(side)

    # -- the look, and its correction --------------------------------------- #
    def _servo(self, primitive, side, record, call_id) -> bool:
        """The look before a stroke, answered by the servo's judge instead of
        the model. Aligned: the look is taken and the stroke may run. Not
        aligned: the model is told why and chooses again."""
        report = self.servo.align(robot=self.robot, policy=self.policy,
                                  state=self.state, side=side,
                                  name=primitive.object, settings=self.settings,
                                  run_plan=run_plan)
        record.servo = report.to_json()
        if any(s.step_m is not None for s in report.steps):
            # the hand and the declaration moved: the record shows the world
            # the model's next choice is made in, aligned or not
            record.observation_after = self.robot.world().to_json()
        looks = [s for s in report.steps if s.kind == "look"]
        if looks:
            record.look = dict(camera=f"{side}_wrist", object=primitive.object,
                               visible=True, **{k: looks[0].look[k]
                                                for k in ("u", "v", "depth_m")})
            record.distribution = looks[-1].distribution
        if self.servo.observe_only:
            return False
        if not report.aligned:
            record.refused = [PlanError(PRECONDITION_UNMET, report.to_text(),
                                        primitive=primitive.name(),
                                        side=side).to_json()]
            _say(self.messages, call_id, report.to_text())
        return report.aligned

    def _look(self, primitive, side, world, cameras, record, call_id,
              unmet) -> None:
        name = primitive.object
        camera = f"{side}_wrist"
        model = cameras.get(camera)
        item = world.find(name)
        if model is None or item is None:
            error = PlanError(PRECONDITION_UNMET,
                              f"no {camera} camera model to look with",
                              primitive=primitive.name(), side=side,
                              unmet=tuple(unmet))
            self._refuse(record, call_id, primitive, error)
            return
        seen = model.project_object(item, world.frames)
        record.look = dict(camera=camera, object=name,
                           mount=getattr(model, "mount", None),
                           **seen.to_json())
        arm = world.arm(side)
        if seen.visible:
            self.state.aimed(side, name)
            self.state.look(side, name, arm.joints)
            text = (f"WRIST LOOK before the stroke: in the {camera} photo, "
                    f"{name!r}'s declared centre should appear at pixel "
                    f"({seen.u:.0f}, {seen.v:.0f}), {seen.depth_m:.3f} m in "
                    f"front of the lens. If it is there, call "
                    f"{primitive.name()} again. If it is not, call locate "
                    f"with camera {camera!r} on the pixel where it touches "
                    f"the table (and its size): the robot re-measures it "
                    f"there and moves the hand by the difference")
        else:
            text = (f"WRIST LOOK before the stroke: {name!r} is not in the "
                    f"{camera} frame from this posture ({seen.reason}); "
                    f"approach it first, then grasp")
        record.refused = [PlanError(PRECONDITION_UNMET, text,
                                    primitive=primitive.name(), side=side,
                                    unmet=tuple(unmet)).to_json()]
        _say(self.messages, call_id, text)

    def _correct(self, arguments, world, cameras, record) -> str:
        """``locate`` on a wrist camera: re-measure the hand's target there
        and move the hand by the difference with a kit ``Nudge``."""
        from ..perception import NoSupport, NotOnThePlane  # noqa: PLC0415
        from ..primitives import Nudge  # noqa: PLC0415
        from ..primitives.verbs import snap  # noqa: PLC0415
        camera = str(arguments.get("camera"))
        side = camera[:-len("_wrist")]
        if camera not in cameras:
            return (f"there is no {camera} camera model in this run, so a "
                    f"pixel cannot be turned into a position")
        # the object this hand is working on — else the task's object
        name = self.state.target.get(side) or self.obj
        item = world.find(name) if name else None
        if item is None:
            return apply_locate(cameras, world, arguments)
        ask = dict(arguments)
        if ask.get("size") is None:
            ask["size"] = [float(v) for v in item.size]
        try:
            located = locate_point(cameras, world, ask)
            p_old = item.pose_in_base(world.frames)[0]
        except NoSupport:
            return ("there is no table in the scene yet, so a pixel has no "
                    "plane to land on")
        except (NotOnThePlane, KeyError, TypeError, ValueError,
                LookupError) as exc:
            return f"that pixel is not usable: {exc}"
        p_new = np.array([located.p[0], located.p[1], p_old[2]], dtype=float)
        offset = p_new - np.asarray(p_old, dtype=float)
        seen = dataclasses.replace(item, p=p_new, frame_id="base",
                                   provenance="observed",
                                   stamp=float(world.stamp))
        self.robot.declare([seen])
        self.state.aimed(side, name)
        text = (f"re-measured {name!r} from the {camera}: centre "
                f"({p_new[0]:.3f}, {p_new[1]:.3f}, {p_new[2]:.3f}) m, "
                f"{offset[0] * 1000:+.0f} / {offset[1] * 1000:+.0f} mm from "
                f"where it was declared ({located.uncertainty_m * 1000:.0f} "
                f"mm uncertain)")
        limit = max(NUDGE_GRID_M)
        dx, dy = (float(np.clip(v, -limit, limit)) for v in offset[:2])
        if not (snap(dx) or snap(dy)):
            self.state.look(side, name, world.arm(side).joints)
            return text + (". That is inside the smallest correction, so the "
                           "hand stays; call grasp when ready")
        nudge = Nudge(side=side, dx=dx, dy=dy, frame="base")
        _nudge, unmet = self.policy.clamp(nudge, self.state, side=side)
        if unmet:
            return text + ". No correction: " + "; ".join(str(u) for u in unmet)
        plan = check(nudge, world, self.robot.kin)
        if not getattr(plan, "ok", False):
            return text + f". The correction {label_for(nudge)} was refused: {plan}"
        record.plan = plan.to_json()
        report = run_plan(plan, self.robot.executor, kin=self.robot.kin,
                          **self.settings)
        record.run = report.to_json()
        self.state.nudged(side)
        after = self.robot.world()
        self.state.look(side, name, after.arm(side).joints)
        verdict = nudge.verifier(world)(after)
        record.verdict = verdict.to_json()
        return (text + f". Corrected: {label_for(nudge)} — transport "
                f"{'ok' if report.completed else report.stop_reason}, "
                f"measured {verdict.verdict}. Call grasp when the photo "
                f"agrees")


def run(*, robot: Any, policy: OperatorPolicy, ask: Ask, goal: Place,
        task: str = "", system: str = "",
        trace: Optional[DecisionTrace] = None,
        observe: Optional[Observe] = None,
        on_side: Optional[Callable[[str], None]] = None,
        keep_images: int = 1, servo=None) -> DecisionTrace:
    """Run the loop until the goal is MEASURED, the model stops, or a cap.

    ``robot``    a :class:`~manipulation_kit.agent.robot.LiveRobot` (enter it
                 first: ``with robot: run(...)``)
    ``policy``   the :class:`OperatorPolicy`; its ``max_turns`` is the cap and
                 its timeouts reach every ``run()``
    ``ask``      ``(messages, tools) -> call``: the model, or a stub
    ``goal``     ``Place(object=, to=)``; ``side="auto"`` is chosen by planning
                 the whole chain for both arms
    ``system``   the prompt (the caller's; the kit adds the policy and the
                 robot facts it owns as separate messages)
    ``observe``  ``(turn, world) -> content parts`` (photos), or raise
                 :class:`ObservationError`
    ``on_side``  told which arm the planner chose (a scripted stand-in uses it)
    ``servo``    a :class:`~manipulation_kit.agent.servo.Servo`: the look
                 before a stroke is then answered by its judge (System 1) and
                 an aligned stroke runs in the same turn; ``None`` keeps the
                 look as a question to the model. A servo built with
                 ``observe_only=True`` only records its judge's answer
                 (``record.servo``) and the look stays the model's question
    """
    return _Loop(robot=robot, policy=policy, ask=ask, goal=goal, task=task,
                 system=system, trace=trace if trace is not None
                 else DecisionTrace(), observe=observe, on_side=on_side,
                 keep_images=keep_images, servo=servo).run()


__all__ = ["Ask", "ObservationError", "Observe", "STOP_REASONS", "Stop",
           "robot_facts", "run"]
