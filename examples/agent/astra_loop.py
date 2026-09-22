"""observe -> offer -> tool call -> execute -> VERIFY, as a runnable loop.

The shape of the thing, in one file: a function-calling model is given the
TASK, the world as text and the kit's primitives as JSON Schema tools; it names
one verb with arguments; the kit decodes and re-checks the BOUND call (a model
may ask for anything, it does not get to skip the guard); the plan runs on an
executor, and the turn ends with a MEASURED verdict from a verifier rather than
with the model's opinion.

Everything checkable comes from the WHEEL —
``manipulation_kit.primitives.schema.tool_schemas`` / ``decode``,
``manipulation_kit.primitives.offer.check``,
``manipulation_kit.primitives.reach.choose_side``,
``manipulation_kit.executor.run``. What is in this file is the part that knows
a model exists: the provider clients, the prompt, the scripted stand-in, and
the message bookkeeping.

THE EXECUTOR IS AN ARGUMENT. ``--executor kinematic`` mirrors the plan onto a
model of the robot; ``--executor firmware --robot http://d1-2:4750`` runs it on
a real D1 through ``manipulation_kit.executors.firmware``. The loop does not
change, which is what the Executor protocol is for — and unlike the previous
version of this file, that is now true rather than claimed: the private step
walker and the block-follows-the-hand mirror are behind the same interface as
the firmware transport.

Three stop reasons, never conflated:

``goal_verified``  the TASK verifier measured TRUE
``model_stopped``  the model returned no call — recorded, and the goal is
                   still measured before believing it
``max_turns``      the cap. A loop with only the first exit runs all night.

    python examples/agent/astra_loop.py --dry-run     # scripted, no key needed
    OPENAI_API_KEY=... OPENAI_MODEL=gpt-5 python examples/agent/astra_loop.py

WHERE THE THINGS ARE, and the model is the one who says. ``--scene`` reads a
file somebody measured with a tape. ``--perceive`` runs
``examples/agent/perceive.py`` on one head frame before turn 0, which supplies
the CAMERA and the table and deliberately no things — and then the model
declares them itself, with two tools this file adds beside the motion verbs:

``declare_scene``  "the cup is here, this big", in base metres, from the
                   photographs. Same reader a hand-written scene file goes
                   through, so it cannot declare something a person could not
                   have written.
``locate``         a pixel it picked -> a base-frame point on the current
                   table plane, with the uncertainty the nominal head mount
                   carries. Exact geometry, so the model is not doing
                   projective maths in its head.

Neither moves anything, so neither goes through the motion gate. ``--scene``
and ``--perceive`` are mutually exclusive, because two answers to "where is
everything" is one too many; and perceiving happens ONCE, before turn zero —
see :func:`perceived_scene` for why re-perceiving mid-run is not a small
change.

NO PER-SCENE CALIBRATION reaches any of this: what ``perceive.py`` takes is
this robot's camera intrinsics and neck angles, and what the model gets is
that camera, its own two tool points in the same base metres, and the picture.

`openai` is NOT a dependency of this repository: ``pip install openai`` before
using ``--model openai``.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

import numpy as np


sys.path.insert(0, str(Path(__file__).resolve().parent))

from manipulation_kit.executor import run  # noqa: E402
from manipulation_kit.primitives import Place  # noqa: E402
from manipulation_kit.primitives.offer import check, label_for  # noqa: E402
from manipulation_kit.primitives.reach import choose_side  # noqa: E402
from manipulation_kit.primitives.schema import decode, tool_schemas  # noqa: E402
from mirror import MirrorRobot, SceneMirrorRobot  # noqa: E402
from scene import demo_scene  # noqa: E402
from trace import DecisionRecord, DecisionTrace  # noqa: E402

DEFAULT_TASK = "put the red block in the box"

SYSTEM = """You drive a D1 humanoid's two arms through a fixed set of verbs.
Each observation may carry three photos: the head camera (scene from above the
torso) and both wrist cameras (each looking along its hand past the jaws).
Use them to judge what the text cannot: whether the object stands or has
tipped, whether the jaws straddle it, whether it is inside the container.

YOU ARE THE DETECTOR. Nothing on this robot measures where the things are.
There is no marker on anything, nobody has measured the table, and the scene
text starts empty or provisional. Your first job is to look at the head photo
and say, in metres in the robot's base frame, where each thing the task needs
is and how big it is — `declare_scene`. Re-declare whenever a photo disagrees
with the text.

How to get metres out of a photograph, and you are given everything you need:
- the HEAD CAMERA block gives its intrinsics and where its lens is in base,
  so a pixel is a known ray;
- `locate(u, v)` walks that ray to the table plane for you and returns the
  base-frame point with its uncertainty. USE IT. Pick the pixel where the
  object TOUCHES the table — the bottom of its silhouette, in the middle —
  and let the function do the projection. Do not estimate a position by eye
  when you can measure it;
- both arms' TOOL POINTS are in the observation text in the same base metres,
  and both hands are usually somewhere in the head photo. They are your scale
  reference and your sanity check: if `locate` says an object is where you can
  see a hand is not, one of you is wrong;
- the table's height is the ONE thing a single camera cannot measure (twice as
  far and twice as big is the same picture). If the text says the height is
  `provisional`, everything derived from it is a guess in proportion — say so
  and declare a better one from what you can see: known objects have known
  sizes, and a cup you can see is about 110 mm tall.

Sizes matter as much as positions: the jaws open 60 mm and take at most
52 mm across, so `size` must be the object's tight outer dimensions —
read the footprint from the object's own base outline, never from its
shadow or its blurred edge. Over-reporting a 47 mm side as 55 mm makes
every grasp of it refused as too wide; the top face gives the truest width.

`confidence` below 1 is expected and is not a reason to refuse to answer.
Declare your best estimate, then FIX IT BY MEASURING: approach, look at the
wrist photo, and `nudge` — nudges are exactly what an uncertain declaration
is for, and a 10 mm one costs nothing.

Rules that are not negotiable, because the robot enforces them anyway:
- You never give an orientation. Name an approach (top_down, front, side_left,
  side_right) and the robot derives the wrist pose from the object.
- `nudge` translations are snapped to a 10/30/50 mm grid, and its `dyaw` is
  clamped to +-15 degrees about the hand's own approach axis. Every OTHER
  number you give is used as you write it, inside the range in the schema.
- An action the motion guard or the inverse kinematics refuses is reported back
  to you with the reason and how far short it was. Read it and choose
  differently; repeating it will not work.
- You do not decide whether the task is done. A measurement does.

Call exactly one tool per turn."""


# --------------------------------------------------------------------------- #
# the two tools that are about the OBSERVATION, not about moving
# --------------------------------------------------------------------------- #
#
# The kit owns the verbs and refuses to own the observation: nothing in
# manipulation_kit opens a camera, and the scene file was somebody with a tape
# measure. These two are the third option — the MODEL is the detector, and it
# is looking at the same frame the loop is.
#
#   declare_scene   "the cup is here, this big". The model's own measurement,
#                   in base metres, from the photograph plus the camera model
#                   plus the robot's own hands at known positions IN that
#                   photograph, which is the scale reference a single view
#                   otherwise does not have.
#   locate          the deterministic half. A pixel the model picked, turned
#                   into a base-frame point ON THE CURRENT TABLE PLANE by
#                   examples/agent/camera.py, with the uncertainty the nominal
#                   head mount actually carries. The model should not be doing
#                   projective geometry in its head when a function can.
#
# Neither moves anything, so neither goes through the motion gate; both are
# answered inside the loop and reported back correlated with the call.

SCENE_TOOLS = ("declare_scene", "locate")

#: What a declared object may say. Deliberately the SAME fields a scene file
#: carries, so "the model measured it" and "a person measured it" produce the
#: same kind of thing and are read by the same code (``live.objects_from``).
DECLARE_SCHEMA = {
    "name": "declare_scene",
    "description": (
        "Say where the things are, in the robot's base frame, from the "
        "photographs. Replaces objects of the same name; anything you do not "
        "name is left alone. Use it on the first turn, and again whenever a "
        "photo shows something is not where the text says."),
    "parameters": {
        "type": "object",
        "properties": {
            "objects": {
                "type": "array", "minItems": 1, "maxItems": 12,
                "items": {
                    "type": "object",
                    "properties": {
                        "name": {"type": "string",
                                 "description": "how you will refer to it"},
                        "kind": {"type": "string",
                                 "enum": ["object", "container", "surface"]},
                        "p": {"type": "array", "minItems": 3, "maxItems": 3,
                              "items": {"type": "number"},
                              "description": "the CENTRE in base metres "
                                             "(+x forward, +y robot-left, "
                                             "+z up), not the bottom"},
                        "size": {"type": "array", "minItems": 3, "maxItems": 3,
                                 "items": {"type": "number", "exclusiveMinimum": 0},
                                 "description": "full extent in metres along "
                                                "the object's own axes"},
                        "yaw_rad": {"type": "number"},
                        "confidence": {"type": "number", "minimum": 0.0,
                                       "maximum": 1.0,
                                       "description": "how sure you are. "
                                                      "Below 1 is expected "
                                                      "and is not a reason to "
                                                      "refuse to answer"},
                        # A container's INSIDE is a separate measurement from
                        # its outside, and `Place` refuses to drop into an
                        # interior nobody stated (it defaults to 90% of the
                        # outside and flags itself a guess). So a model that
                        # can see into a cup has to be able to say what it
                        # sees, or the verb is unreachable for every scene it
                        # declares.
                        "interior": {
                            "type": "array", "minItems": 3, "maxItems": 3,
                            "items": {"type": "number", "exclusiveMinimum": 0},
                            "description": "containers only: the usable INNER "
                                           "extent in metres. Leave it out if "
                                           "you cannot see inside — then "
                                           "`place` will refuse, which is the "
                                           "correct answer and not a bug"},
                    },
                    "required": ["name", "kind", "p", "size"],
                    "additionalProperties": False}}},
        "required": ["objects"], "additionalProperties": False}}

LOCATE_SCHEMA = {
    "name": "locate",
    "description": (
        "Turn a pixel you picked in the HEAD photo into a point in the "
        "robot's base frame, on the table plane. Exact geometry, no guessing "
        "- use it instead of estimating a position by eye, and check what it "
        "says against what you were about to declare."),
    "parameters": {
        "type": "object",
        "properties": {
            "u": {"type": "number", "description": "pixel x, 0 at the left"},
            "v": {"type": "number", "description": "pixel y, 0 at the top"},
            "camera": {"type": "string", "enum": ["head"], "default": "head"}},
        "required": ["u", "v"], "additionalProperties": False}}


def scene_tools(camera=None) -> List[Dict[str, Any]]:
    """The observation tools available right now.

    ``locate`` is offered only when a camera model exists, because without one
    it would have to invent a projection — and a tool that sometimes answers
    from geometry and sometimes from nothing is worse than a missing tool.
    """
    return [DECLARE_SCHEMA] + ([LOCATE_SCHEMA] if camera is not None else [])


def apply_declare_scene(robot, arguments: Dict[str, Any]) -> str:
    """Decode a ``declare_scene`` call and hand it to the robot adapter.

    Validation is ``live.objects_from``'s — the same reader a hand-written
    scene file goes through — so a model cannot declare something a person
    could not have written, and a zero size or a two-element position is
    refused with the reader's own message rather than crashing four verbs
    later.
    """
    from live import objects_from  # noqa: PLC0415
    if not hasattr(robot, "declare"):
        return ("this robot cannot take a declared scene (no `declare`); run "
                "with --scene or --perceive")
    items = []
    for item in arguments.get("objects") or []:
        item = dict(item)
        if "interior" in item:
            # Stated by the model = stated. The flag is what `Place` reads,
            # and the alternative is a container that can never be placed into.
            item["interior_measured"] = True
        items.append(item)
    try:
        views = objects_from({"objects": items})
    except (KeyError, ValueError, TypeError) as exc:
        return f"that scene is malformed and nothing was changed: {exc}"
    robot.declare(views)
    lines = [f"{v.name!r} ({v.kind}) at ({v.p[0]:.3f}, {v.p[1]:.3f}, "
             f"{v.p[2]:.3f}) m, {v.size[0]*1000:.0f}x{v.size[1]*1000:.0f}x"
             f"{v.size[2]*1000:.0f} mm" for v in views]
    return ("recorded, and the next observation is measured against it: "
            + "; ".join(lines))


def apply_locate(camera, world, arguments: Dict[str, Any]) -> str:
    """Answer a ``locate`` call from the camera model and the current plane."""
    from camera import NotOnThePlane  # noqa: PLC0415
    if camera is None:
        return ("there is no camera model in this run, so a pixel cannot be "
                "turned into a position")
    plane_z, source = table_plane_z(world)
    if plane_z is None:
        return ("there is no table in the scene yet, so a pixel has no plane "
                "to land on. Declare the surface first with declare_scene "
                "(kind 'surface'), then ask again")
    try:
        located = camera.locate(float(arguments["u"]), float(arguments["v"]),
                                plane_z=plane_z, plane_source=source)
    except NotOnThePlane as exc:
        return str(exc)
    except (KeyError, TypeError, ValueError) as exc:
        return f"that pixel is not usable: {exc}"
    return located.to_text()


def table_plane_z(world) -> "tuple":
    """The z of the top of the highest SURFACE in the world, and where it
    came from. ``(None, "")`` when nobody has said there is a table."""
    from manipulation_kit.world import SurfaceView  # noqa: PLC0415
    tops = [(float(o.p[2]) + float(o.size[2]) / 2.0, o)
            for o in world.objects if isinstance(o, SurfaceView)]
    if not tops:
        return None, ""
    z, surface = max(tops, key=lambda pair: pair[0])
    return z, (f"the top of {surface.name!r}, confidence "
               f"{surface.confidence:.1f}")


@dataclass
class Stop:
    reason: str
    detail: str = ""


class ScriptedModel:
    """A stand-in for a function-calling model: plays a correct pick-and-place.

    It exists so the loop, the gate and the verifier are runnable and testable
    with no key, no network and no model. It also makes one deliberate mistake
    (it claims "done" one turn early) so the trace's claimed-vs-measured column
    has something in it. It is NOT a result: see the README on what the
    simulation findings do and do not support.
    """

    def __init__(self, obj: str = "red_block", to: str = "box",
                 declare: Optional[List[Dict[str, Any]]] = None):
        #: SCRIPTED FICTION, and the only way this stand-in can play the
        #: zero-shot path: it cannot see, so when the scene has no things in
        #: it (``--perceive``'s default, where the MODEL is the detector) it
        #: declares the two it was handed and gets on with the script. A real
        #: model reads the photograph. Nothing here is a measurement.
        self.declare = declare
        self.script: List[Dict[str, Any]] = [
            {"name": "grasp", "arguments": {"object": obj, "side": "left",
                                            "approach": "top_down"}},
            # A shorter hop when the things were declared onto a perceived
            # table: that table is 0.17 m up, and 0.10 m of lift from there
            # puts the tool outside the arm's envelope — a real reach fact,
            # and not one the stub should spend its script discovering.
            {"name": "lift", "arguments": {"object": obj, "side": "left",
                                           "height_m": 0.05 if declare
                                           else 0.1}},
            {"name": "carry", "arguments": {"object": obj, "to": to,
                                            "side": "left"}},
            {"name": "place", "arguments": {"object": obj, "to": to,
                                            "side": "left"}},
        ]
        self.turn = 0

    def use_side(self, side: str) -> None:
        """Play the script with the hand the task planner chose."""
        for call in self.script:
            if "side" in call["arguments"]:
                call["arguments"]["side"] = side

    def __call__(self, messages, tools) -> Dict[str, Any]:
        if self.declare is not None:
            names = {t["name"] for t in tools}
            payload, self.declare = self.declare, None
            if "declare_scene" in names:
                return {"name": "declare_scene", "claimed": "",
                        "call_id": "scripted-declare",
                        "arguments": {"objects": payload}}
        if self.turn >= len(self.script):
            return {"name": None, "arguments": {}, "claimed": "done"}
        call = self.script[self.turn]
        self.turn += 1
        # The deliberate over-claim: it says "done" at the CARRY, one turn
        # before the block is in the box. The loop ignores it and keeps going,
        # and the trace records the disagreement — which is the whole exhibit.
        claimed = "done" if self.turn == len(self.script) - 1 else ""
        return dict(call, claimed=claimed, call_id=f"scripted-{self.turn}")


class OpenAIModel:
    """The real thing, through the Responses API. Imported only when used."""

    def __init__(self, model: str, api_key: str):
        from openai import OpenAI  # noqa: PLC0415
        self.client = OpenAI(api_key=api_key)
        self.model = model

    def __call__(self, messages, tools) -> Dict[str, Any]:
        def clean(message):
            content = message.get("content")
            if not isinstance(content, list):
                return message
            return dict(message, content=[
                {k: v for k, v in p.items() if not k.startswith("_")}
                for p in content])

        # gpt-6-astra with three photos spent its whole default output budget
        # on hidden reasoning and returned NOTHING (status=incomplete,
        # max_output_tokens; d1-2 2026-09-22). Budget explicitly.
        extra: Dict[str, Any] = {
            "max_output_tokens": int(os.environ.get("ASTRA_MAX_OUTPUT_TOKENS", "12000"))}
        effort = os.environ.get("ASTRA_REASONING", "medium")
        if effort:
            extra["reasoning"] = {"effort": effort}
        response = self.client.responses.create(
            model=self.model,
            input=[clean(m) for m in messages],
            **extra,
            tools=[{"type": "function", **t} for t in tools])
        calls = [i for i in response.output
                 if getattr(i, "type", "") == "function_call"]
        if os.environ.get("ASTRA_DEBUG"):
            kinds = [getattr(i, "type", "?") for i in response.output]
            print(f"[astra_loop] model output items: {kinds}; text: "
                  f"{(getattr(response, 'output_text', '') or '')[:600]!r}; "
                  f"status={getattr(response, 'status', '?')} "
                  f"incomplete={getattr(response, 'incomplete_details', None)} "
                  f"usage={getattr(response, 'usage', None)}",
                  file=sys.stderr)
        if not calls:
            return {"name": None, "arguments": {},
                    "claimed": getattr(response, "output_text", "")}
        if len(calls) > 1:
            # The contract says exactly one. Taking the first silently taught
            # the model that the rest were executed too.
            return {"name": None, "arguments": {}, "call_id": "",
                    "protocol_error": (
                        f"you called {len(calls)} tools; the contract is "
                        f"exactly one per turn. Choose one and call it again."),
                    "claimed": ""}
        item = calls[0]
        return {"name": item.name, "arguments": json.loads(item.arguments),
                "call_id": getattr(item, "call_id", "") or getattr(item, "id", ""),
                "claimed": ""}


def build_model(dry_run: bool, obj: str = "red_block", to: str = "box",
                declare: Optional[List[Dict[str, Any]]] = None):
    key = os.environ.get("OPENAI_API_KEY")
    if dry_run or not key:
        if not dry_run:
            print("[astra_loop] no OPENAI_API_KEY; running the scripted stub",
                  file=sys.stderr)
        return ScriptedModel(obj, to, declare=declare)
    return OpenAIModel(os.environ.get("OPENAI_MODEL", "gpt-6-astra"), key)


def two_things_on(world, *, obj: str, destination: str
                  ) -> Optional[List[Dict[str, Any]]]:
    """Two objects on the widest surface in ``world``, for the STUB only.

    ``--perceive --dry-run`` has a real table and no things, and the scripted
    stand-in cannot see. Rather than have it stop at turn zero for want of a
    detector, it is handed a plausible pair to declare so that the LOOP —
    which is the thing the stub exists to exercise — runs end to end. These
    are not measurements of anything and the trace says ``scripted-declare``.
    ``None`` when there is no surface, or when the things are already there.
    """
    from manipulation_kit.world import SurfaceView  # noqa: PLC0415
    from scene import BLOCK_P, BOX_P  # noqa: PLC0415
    surfaces = [o for o in world.objects if isinstance(o, SurfaceView)]
    if not surfaces or any(o.name in (obj, destination) for o in world.objects):
        return None
    table = max(surfaces, key=lambda s: float(s.size[0]) * float(s.size[1]))
    top = float(table.p[2]) + float(table.size[2]) / 2.0
    # The demo scene's own x/y — inside the measured reachable envelope, which
    # a point picked off an arbitrary table is not — at the height of the
    # table that WAS perceived. Fiction with a real z.
    return [
        {"name": obj, "kind": "object",
         "p": [BLOCK_P[0], BLOCK_P[1], top + 0.025],
         "size": [0.045, 0.02, 0.05], "confidence": 0.3},
        # A SHALLOW container: a 110 mm cup standing on a 0.17 m wagon puts
        # its rim at the top of this arm's envelope, and the stub would spend
        # its whole script on a reach refusal that is about the furniture
        # rather than about the loop.
        {"name": destination, "kind": "container",
         "p": [BOX_P[0], BOX_P[1], top + 0.03],
         "size": [0.12, 0.12, 0.06], "interior": [0.10, 0.10, 0.05],
         "confidence": 0.3}]


def _say(messages: List[Dict[str, Any]], call_id: str, text: str) -> None:
    """Feed a result back CORRELATED with the call that produced it.

    The old loop appended every result as anonymous user prose, so a model
    with two outstanding ideas could not tell which one the refusal was about.
    """
    messages.append({"role": "user",
                     "content": (f"[result of {call_id}] {text}" if call_id
                                 else text)})



PLANNER_ONLY_ARGS = ("jaw_turn_deg",)


def _hide_planner_args(tools):
    """Strip arguments the LOOP decides, not the model.

    ``jaw_turn_deg`` exists so the fallback below can re-plan a refused grasp
    with the wrist a quarter turn round; shown to the model (d1-2 run8,
    2026-09-22) it picked +90 on its own, which is the IK-infeasible turn, and
    spent the run on ik_fail refusals while the plain posture would have
    planned. The model asks for a grasp; which wrist stands is the kit's call.
    """
    out = []
    for tool in tools:
        props = dict(tool["parameters"].get("properties", {}))
        for name in PLANNER_ONLY_ARGS:
            props.pop(name, None)
        params = dict(tool["parameters"], properties=props)
        if "required" in params:
            params["required"] = [r for r in params["required"] if r not in PLANNER_ONLY_ARGS]
        out.append(dict(tool, parameters=params))
    return out


def _dump_messages(trace_path: Optional[Path], messages: List[Dict[str, Any]]) -> None:
    """Keep the model's whole chat history next to the trace, rewritten every
    turn so a crash mid-turn still leaves it on disk (Shu, 2026-09-22)."""
    if trace_path is None:
        return
    path = Path(trace_path).with_suffix(".messages.json")

    def slim(message):
        content = message.get("content")
        if not isinstance(content, list):
            return message
        parts = [dict(p, image_url=f"<{p.get('_file', 'image')}>")
                 if p.get("type") == "input_image" else p for p in content]
        return dict(message, content=parts)

    path.write_text(json.dumps([slim(m) for m in messages], indent=1,
                               default=str), encoding="utf-8")


def _snapshot(trace_path: Optional[Path], turn: int) -> None:
    """Optional camera record per turn: run ``$ASTRA_SNAPSHOT_CMD`` with
    ``ASTRA_TURN`` and ``ASTRA_OUT_DIR`` set. The model is NOT shown these
    frames (the loop is text-only); they are for the human reading the run."""
    cmd = os.environ.get("ASTRA_SNAPSHOT_CMD")
    if not cmd or trace_path is None:
        return []
    import subprocess  # noqa: PLC0415
    env = dict(os.environ, ASTRA_TURN=str(turn),
               ASTRA_OUT_DIR=str(Path(trace_path).parent))
    try:
        subprocess.run(cmd, shell=True, env=env, timeout=40, check=False,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception as exc:  # noqa: BLE001 - a snapshot must never stop a run
        print(f"[astra_loop] snapshot failed: {exc!r}", file=sys.stderr)
        return []
    out = Path(trace_path).parent
    # The model is shown the head and the RIGHT wrist frame of this turn (the
    # left wrist sees nothing useful while the right hand works). Attach in a
    # fixed order so the trace is comparable turn to turn.
    wanted = (f"turn{turn}_base_0_rgb.jpg", f"turn{turn}_right_wrist_0_rgb.jpg",
              f"turn{turn}_left_wrist_0_rgb.jpg")
    return [out / name for name in wanted if (out / name).exists()]


def _observation(text: str, frames) -> Any:
    """The user turn: the world as text, plus this turn's camera frames as
    images when there are any (Shu, 2026-09-22: 'Astra に画像を渡すのは必須')."""
    if not frames:
        return text
    import base64  # noqa: PLC0415
    parts: List[Dict[str, Any]] = [{"type": "input_text", "text": text}]
    for path in frames:
        data = base64.b64encode(Path(path).read_bytes()).decode("ascii")
        parts.append({"type": "input_image", "detail": "high",
                      "image_url": f"data:image/jpeg;base64,{data}",
                      "_file": Path(path).name})
    return parts


def plan_the_hand(world, kin, *, obj: str, destination: str):
    """WHICH HAND — by planning the whole chain for BOTH arms.

    (Approach, Grasp, Lift, Carry, Place), and take the one that can DELIVER;
    the near hand is only the tie-break. See
    ``manipulation_kit.primitives.reach`` for what that cost on the
    blocks-eval wagon (2026-09-19, F10). Flat or awkward objects refuse a
    top-down grasp (``object_too_flat``), so the approach directions are tried
    in order and the first reachable chain wins.

    ``None`` when the scene does not contain both names yet, which is now a
    normal state: with ``--perceive`` the model declares the things on turn 0
    and there is nothing to plan for until it has.
    """
    names = {o.name for o in world.objects}
    if obj not in names or destination not in names:
        return None
    hand = None
    for approach in ("top_down", "front", "side_right", "side_left"):
        for jaw_turn in (0.0, -90.0, 90.0):      # see Approach.jaw_turn_deg
            candidate = choose_side(world, kin, obj=obj, destination=destination,
                                    approach=approach, jaw_turn_deg=jaw_turn)
            if hand is None or (candidate.reachable and not hand.reachable):
                hand = candidate
            if hand.reachable:
                break
        if hand.reachable:
            break
    return hand


def loop(model, robot=None, *, task: str = DEFAULT_TASK, max_turns: int = 8,
         trace_path: Optional[Path] = None, goal=None, world0=None, kin=None,
         obj: str = "red_block", destination: str = "box",
         camera=None) -> DecisionTrace:
    if world0 is None or kin is None:
        demo_world, demo_kin = demo_scene()
        world0 = demo_world if world0 is None else world0
        kin = demo_kin if kin is None else kin
    robot = robot if robot is not None else MirrorRobot(kin)
    hand = plan_the_hand(world0, kin, obj=obj, destination=destination)
    if hand is not None and goal is None:
        goal = Place(object=obj, to=destination, side=hand.side)
    trace = DecisionTrace(trace_path)
    trace.task = task
    messages: List[Dict[str, Any]] = [{"role": "system", "content": SYSTEM},
                                      {"role": "user", "content": f"TASK: {task}"}]
    if camera is not None:
        messages.append({"role": "user", "content": camera.to_text()})
    if hand is None:
        # NOT AN ERROR. The scene has not been declared yet; the model is
        # about to do that from the photographs, and the arm choice is made
        # the first turn both names exist.
        messages.append({"role": "user", "content": (
            f"The scene does not contain {obj!r} and {destination!r} yet. "
            f"Look at the photographs and call declare_scene with where they "
            f"are, in base metres; use locate on the pixel where each one "
            f"touches the table rather than estimating by eye. Then act.")})
    elif not hand.reachable:
        # F10/F12: an impossible task is refused at turn zero rather than
        # discovered three verbs in. The old loop computed this and ignored it.
        record = DecisionRecord(iteration=0, world=world0.to_json())
        record.stop = "unreachable_task"
        record.refused = [c.to_json() for c in hand.chains.values()]
        trace.write(record)
        _dump_messages(trace_path, messages)
        trace.stop = Stop("unreachable_task", hand.reason).reason
        return trace

    if isinstance(model, ScriptedModel) and hand is not None:
        model.use_side(hand.side)
    stop = Stop("max_turns", f"{max_turns} turns without a measured goal")
    for turn in range(max_turns):
        world = robot.world()
        frames = _snapshot(trace_path, turn)
        messages.append({"role": "user",
                         "content": _observation(world.to_text(), frames)})
        # REFRESHED every turn: the motion verbs' name enums narrow as the
        # world changes, and the OBSERVATION tools travel with them so a model
        # that has just been told "that is not where you said" can answer with
        # a measurement instead of another guess.
        tools = _hide_planner_args(tool_schemas(world)) + scene_tools(camera)
        record = DecisionRecord(iteration=turn, world=world.to_json())
        record.task = task

        call = model(messages, tools)
        call_id = call.get("call_id") or ""
        record.choice = {"name": call["name"], "arguments": call["arguments"],
                         "call_id": call_id}
        record.claimed = call.get("claimed") or None
        if call.get("protocol_error"):
            _say(messages, call_id, call["protocol_error"])
            trace.write(record)
            _dump_messages(trace_path, messages)
            continue
        if not call["name"]:
            # The model stopped. That is a claim about the task, and the task
            # verifier is what decides — the old loop broke here without
            # checking the goal at all, contrary to its own docstring. With
            # nothing declared there is no verifier to ask, and "I stopped
            # before saying where anything was" is not a measured success.
            if goal is None:
                record.stop = "model_stopped"
                trace.write(record)
                _dump_messages(trace_path, messages)
                stop = Stop("model_stopped",
                            f"the model stopped without declaring {obj!r} and "
                            f"{destination!r}; nothing was ever planned")
                break
            goal_report = goal.verifier(world)(robot.world())
            record.goal_verdict = goal_report.to_json()
            record.stop = "model_stopped"
            trace.write(record)
            _dump_messages(trace_path, messages)
            stop = Stop("goal_verified" if goal_report.verdict == "true"
                        else "model_stopped", goal_report.reason)
            break

        if call["name"] in SCENE_TOOLS:
            # OBSERVATION, not motion: nothing moves, so there is nothing for
            # the motion gate to check. The world is re-read afterwards and
            # the arm choice is (re)made, because a scene that has just
            # appeared is the thing the whole plan depends on.
            if call["name"] == "declare_scene":
                answer = apply_declare_scene(robot, call["arguments"])
                world0 = robot.world()
                hand = plan_the_hand(world0, kin, obj=obj,
                                     destination=destination)
                if hand is not None:
                    if goal is None:
                        goal = Place(object=obj, to=destination,
                                     side=hand.side)
                    if isinstance(model, ScriptedModel):
                        model.use_side(hand.side)
                    answer += (f". Planning the whole chain says the "
                               f"{hand.side} arm"
                               + ("" if hand.reachable
                                  else f", and nothing reaches yet: "
                                       f"{hand.reason}"))
            else:
                answer = apply_locate(camera, world, call["arguments"])
            record.observation_after = {"tool": call["name"], "answer": answer}
            _say(messages, call_id, answer)
            trace.write(record)
            _dump_messages(trace_path, messages)
            continue

        if goal is None:
            _say(messages, call_id,
                 f"nothing can be planned until {obj!r} and {destination!r} "
                 f"are in the scene. Call declare_scene first.")
            trace.write(record)
            _dump_messages(trace_path, messages)
            continue

        # THE GATE, on the BOUND call. A model may ask for anything; decode
        # checks the arguments against the kit's own table and the guard
        # decides the rest. This is the step PR #17 was missing.
        # Operator cap on the grip preset (d1-2 2026-09-22: `firm` preload on a
        # rigid body wound the hold up to -4.2 Nm and faulted the motor;
        # d1-firmware #89). ASTRA_GRIP_CAP=soft rewrites firmer requests.
        cap = os.environ.get("ASTRA_GRIP_CAP")
        if cap and call["arguments"].get("grip") not in (None, cap):
            asked = call["arguments"]["grip"]
            order = ("soft", "firm", "strong")
            if asked in order and cap in order and order.index(asked) > order.index(cap):
                call["arguments"]["grip"] = cap
                _say(messages, call_id,
                     f"note: grip {asked!r} is capped to {cap!r} on this robot tonight")
        # Operator restriction on approach directions. Until the guard knows
        # the table (manipulation-kit #23), horizontal approaches from HOME
        # sweep the forearm through the wagon top (d1-2 run5, error 2 latch).
        allowed = os.environ.get("ASTRA_APPROACH_ALLOW")
        if allowed and call["name"] in ("approach", "grasp"):
            asked = call["arguments"].get("approach", "top_down")
            if asked not in allowed.split(","):
                record.refused = [{"reason": "approach_disabled",
                                   "detail": f"{asked!r} approaches are disabled on this robot "
                                             f"tonight; allowed: {allowed}"}]
                _say(messages, call_id,
                     f"{call['name']} was refused: the {asked!r} approach direction is "
                     f"disabled on this robot (the guard cannot yet see the table); "
                     f"use one of: {allowed}. If top_down is refused near the body, the "
                     f"object is closer than you declared — re-check with locate.")
                trace.write(record)
                _dump_messages(trace_path, messages)
                continue
        for name in PLANNER_ONLY_ARGS:
            call["arguments"].pop(name, None)
        primitive = decode(call["name"], call["arguments"], world)
        if not isinstance(primitive, object) or getattr(primitive, "ok", None) is False:
            record.refused = [primitive.to_json()]
            _say(messages, call_id, f"that call is malformed: {primitive}")
            trace.write(record)
            _dump_messages(trace_path, messages)
            continue
        plan = check(primitive, world, kin)
        # JAW-TURN FALLBACK. The kit squares the jaws across the object's long
        # axis; when that wrist posture is refused by the guard or IK at the
        # standoff, the same grasp a quarter turn round often stands (d1-2
        # run6: a 45x55 mm charger). Try it once; the Grasp preconditions
        # still refuse the turned grasp if the other side is too wide.
        if (not getattr(plan, "ok", False)
                and primitive.name() in ("approach", "grasp")
                and getattr(plan, "reason", "") in ("guard_reject", "ik_fail")
                and getattr(plan, "waypoint_label", "") == "standoff"):
            import dataclasses as _dc  # noqa: PLC0415
            turned, plan2 = primitive, plan
            asked = float(getattr(primitive, "jaw_turn_deg", 0.0))
            for turn in [x for x in (0.0, -90.0, 90.0) if x != asked]:
                turned = _dc.replace(primitive, jaw_turn_deg=turn)
                plan2 = check(turned, world, kin)
                if getattr(plan2, "ok", False):
                    break
            if getattr(plan2, "ok", False):
                _say(messages, call_id,
                     f"note: {label_for(primitive)} with the jaws across the long "
                     f"side was refused ({plan.reason} at the standoff); the "
                     f"hand is turned 90 deg so the jaws close across the other "
                     f"side, which fits.")
                primitive, plan = turned, plan2
        if not getattr(plan, "ok", False):
            record.refused = [plan.to_json()]
            _say(messages, call_id, f"{label_for(primitive)} was refused: {plan}")
            trace.write(record)
            _dump_messages(trace_path, messages)
            continue
        record.offered = [{"id": f"{primitive.name()}", "label": label_for(primitive)}]

        record.plan = plan.to_json()
        # ASSOCIATION half of the pickup predicate: tell the live adapter WHAT
        # the coming stroke closes on, or every later lift/place is refused
        # with gripper_unknown (d1-2 run2, 2026-09-22: two real grasps, no lift).
        if hasattr(robot, "expect") and primitive.name() in ("grasp",):
            robot.expect(getattr(plan, "side", None) or primitive.side,
                         primitive.object)
        try:
            report = run(plan, robot.executor)
        except Exception as exc:
            # The turn is written BEFORE the exception propagates: run1 on d1-2
            # (2026-09-22) lost its whole trace to an httpx timeout in here.
            record.run = {"completed": False, "stop_reason": "exception",
                          "error": repr(exc)}
            trace.write(record)
            _dump_messages(trace_path, messages)
            raise
        record.run = report.to_json()
        after = robot.world()
        verdict = primitive.verifier(world)(after)
        record.verdict = verdict.to_json()
        record.observation_after = after.to_json()
        # WHY the transport stopped, not just that it did. A barrier failure
        # now carries a typed reason and a number ("the tool point is 27 mm
        # from the grasp pose after 2 corrections"); a model told only
        # "barrier_failed" has to guess what to do differently.
        how = ("ok" if report.completed else
               f"{report.stop_reason} — {report.error}")
        _say(messages, call_id,
             f"{label_for(primitive)}: transport {how}; "
             f"measured {verdict.verdict} — {verdict.reason}")
        goal_report = goal.verifier(world)(after)
        record.goal_verdict = goal_report.to_json()
        trace.write(record)
        _dump_messages(trace_path, messages)
        if goal_report.verdict == "true":
            stop = Stop("goal_verified", goal_report.reason)
            break
    trace.stop = stop.reason
    trace.stop_detail = stop.detail
    return trace


def build_robot(kind: str, kin, robot_url: str, scene=None, world0=None,
                obj: str = "red_block"):
    if kind == "kinematic":
        if world0 is not None:
            return SceneMirrorRobot(kin, world0, obj)
        return MirrorRobot(kin)
    from manipulation_kit.executors.firmware import FirmwareExecutor  # noqa: PLC0415
    from live import LiveRobot  # noqa: PLC0415
    from manipulation_kit.executors.firmware.client import FirmwareClient  # noqa: PLC0415
    # The daemon answers /v1/gripper/{side}/set only when the stroke is done;
    # the client default of 2 s is too short for a real close (d1-2, 2026-09-22).
    client = FirmwareClient(robot_url, timeout=20.0)
    # The daemon runs the arm at vel_ratio x its full speed, but the kit's
    # trajectory timestamps assume MAX_JOINT_RATE_DEG_S (140 deg/s). At the
    # default 0.15 the arm crawls at ~20 deg/s behind a schedule seven times
    # faster, so every large move "arrives late" and the arrival barrier
    # fails while the arm is still moving (d1-2 run4, 2026-09-22: "still
    # moving after 1.5 s, worst joint 18.2 deg/s", tool 190 mm off). Time the
    # schedule at the speed the arm will actually have.
    from manipulation_kit.executors.firmware.executor import MAX_JOINT_RATE_DEG_S  # noqa: PLC0415
    vel_ratio = float(os.environ.get("ASTRA_VEL_RATIO", "0.15"))  # Shu: the moves are not slow; keep the default speed
    rate = MAX_JOINT_RATE_DEG_S * vel_ratio
    arrive_timeout = float(os.environ.get("ASTRA_ARRIVE_TIMEOUT_S", "4.0"))
    print(f"[astra_loop] firmware executor: vel_ratio {vel_ratio}, schedule "
          f"{rate:.0f} deg/s, arrive timeout {arrive_timeout}s", file=sys.stderr)
    return LiveRobot(FirmwareExecutor(base_url=robot_url, client=client,
                                      vel_ratio=vel_ratio, acc_ratio=vel_ratio,
                                      max_joint_rate_deg_s=rate,
                                      arrive_timeout_s=arrive_timeout),
                     kin, scene)


def perceived_scene(source: str, *, trace_path: Optional[Path],
                    obj: str, destination: str,
                    options: str = "") -> Dict[str, Any]:
    """``--perceive``: MEASURE the scene from one head frame instead of
    reading one somebody measured by hand.

    ``source`` is either a path to a frame or the word ``snapshot``, which
    runs ``$ASTRA_SNAPSHOT_CMD`` once — the same hook the per-turn record
    uses, called with ``ASTRA_TURN=0`` — and perceives from the head frame it
    leaves behind. The scene is written next to the trace as
    ``scene_perceived.json``, so a run that goes wrong can be re-read against
    the picture the plan was built from.

    ONCE, BEFORE TURN ZERO. Re-perceiving every turn is out of scope here and
    is not a small change: the object moves while the loop holds it, so a
    fresh scene mid-run would have to be RECONCILED with what the gripper is
    carrying rather than replacing it, and the loop's ``held_object``
    bookkeeping (``live.LiveRobot.expect``) is what that reconciliation would
    have to agree with.

    ``options`` is a string of ``perceive.py`` flags, parsed by that file's
    OWN argument parser — the priors (table width, fx, anchor, neck angles)
    are its business and duplicating them here would be a second definition
    to keep in step.
    """
    import shlex  # noqa: PLC0415

    import perceive  # noqa: PLC0415
    if source == "snapshot":
        if trace_path is None:
            raise SystemExit("--perceive snapshot needs --trace: the frame is "
                             "written next to the trace")
        if not os.environ.get("ASTRA_SNAPSHOT_CMD"):
            raise SystemExit("--perceive snapshot needs $ASTRA_SNAPSHOT_CMD, "
                             "the same hook the per-turn frames use")
        _snapshot(trace_path, 0)
        image = Path(trace_path).parent / "turn0_base_0_rgb.jpg"
        if not image.exists():
            raise SystemExit(f"$ASTRA_SNAPSHOT_CMD left no {image.name}; "
                             f"nothing to perceive from")
    else:
        image = Path(source)
        if not image.exists():
            raise SystemExit(f"--perceive {source}: no such frame")
    tokens = shlex.split(options)
    if "--objects" not in tokens:
        tokens += ["--objects", f"{obj}:object,{destination}:container"]
    args = perceive.build_parser().parse_args(
        ["--image", str(image)] + tokens)
    scene = perceive.perceive(args)
    if trace_path is not None:
        out = Path(trace_path).parent / "scene_perceived.json"
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(scene, indent=1) + "\n", encoding="utf-8")
        print(f"[astra_loop] perceived {image.name} -> {out}", file=sys.stderr)
    return scene


def camera_from_scene(scene: Optional[Dict[str, Any]]):
    """The head-camera model a perceived scene recorded, or ``None``.

    The camera travels IN the scene file rather than being a second set of
    loop flags, so the pose ``locate`` projects through is provably the one
    the frame was perceived with — a neck that has moved since then is a
    different camera, and re-typing its angle on two command lines is how
    those two quietly stop matching.
    """
    block = (scene or {}).get("_perceive", {}).get("camera")
    if not block:
        return None
    from camera import HeadCamera  # noqa: PLC0415
    from scipy.spatial.transform import Rotation  # noqa: PLC0415
    width, height = block["image"]
    return HeadCamera(fx=block["fx"], cx=block["cx"], cy=block["cy"],
                      width=width, height=height,
                      p=np.asarray(block["p_base"], dtype=float),
                      r=Rotation.from_quat(block["quat_xyzw"]),
                      neck_pitch=block.get("neck_pitch_rad", 0.0),
                      neck_yaw=block.get("neck_yaw_rad", 0.0),
                      lift_m=block.get("lift_m"),
                      calibrated=bool(block.get("calibrated", False)),
                      notes=tuple(block.get("notes", ())))



def robot_camera_opts(robot_url: str, options: str) -> str:
    """Fill the ROBOT's own numbers into perceive's options from the daemon.

    Zero-shot means nobody types the neck angle: read ``/v1/neck/state`` and
    ``/v1/slider/state`` and append ``--neck-pitch/--neck-yaw/--lift`` unless
    the caller already gave them. SIGN: the daemon reports pitch NEGATIVE when
    the head looks down (d1-2: -0.61) while the kit URDF's neck_pitch is
    positive-down, so the value is negated here (verified 2026-09-22 on d1-2:
    -0.6117 -> +0.6117 puts the table in front of the lens; the raw value put
    the whole table above the horizon).
    """
    import json as _json  # noqa: PLC0415
    import urllib.request  # noqa: PLC0415
    have = set(options.split())
    extra = []
    try:
        if "--neck-pitch" not in have or "--neck-yaw" not in have:
            neck = _json.load(urllib.request.urlopen(
                f"{robot_url}/v1/neck/state", timeout=3))["data"]
            if "--neck-pitch" not in have:
                extra += ["--neck-pitch", f"{-float(neck['pitch']):.4f}"]
            if "--neck-yaw" not in have:
                extra += ["--neck-yaw", f"{float(neck['yaw']):.4f}"]
        if "--lift" not in have:
            lift = _json.load(urllib.request.urlopen(
                f"{robot_url}/v1/slider/state", timeout=3))["data"]
            extra += ["--lift", f"{float(lift['height_m']):.4f}"]
    except Exception as exc:  # noqa: BLE001 - say what is missing, do not guess
        print(f"[astra_loop] could not read the neck/lift from {robot_url}: "
              f"{exc!r}; pass --perceive-opts yourself", file=sys.stderr)
        return options
    if extra:
        print(f"[astra_loop] camera pose from the daemon: {' '.join(extra)}",
              file=sys.stderr)
    return (options + " " + " ".join(extra)).strip()


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--task", default=DEFAULT_TASK)
    parser.add_argument("--dry-run", action="store_true",
                        help="use the scripted stub even if a key is present")
    parser.add_argument("--executor", choices=("kinematic", "firmware"),
                        default="kinematic")
    parser.add_argument("--robot", default="http://127.0.0.1:4750")
    parser.add_argument("--max-turns", type=int, default=8)
    parser.add_argument("--trace", type=Path, default=None)
    parser.add_argument("--scene", type=Path, default=None,
                        help="a MEASURED scene file (examples/agent/scenes/*.json); "
                             "default: the built-in demo scene")
    parser.add_argument("--perceive", default=None, metavar="IMAGE|snapshot",
                        help="MEASURE the scene from one head frame with "
                             "examples/agent/perceive.py instead of reading a "
                             "hand-measured file. 'snapshot' runs "
                             "$ASTRA_SNAPSHOT_CMD once before turn 0 and "
                             "perceives from turn0_base_0_rgb.jpg. Once, "
                             "before turn zero — not per turn.")
    parser.add_argument("--perceive-opts", default="",
                        help="flags passed verbatim to perceive.py — the "
                             "ROBOT's own numbers, e.g. "
                             "\"--neck-pitch 0.52 --lift 0.205 --fx 606\"")
    parser.add_argument("--object", default="red_block",
                        help="the scene object to move (default red_block)")
    parser.add_argument("--destination", default="box",
                        help="the scene container to place it in (default box)")
    args = parser.parse_args(argv)

    if args.scene is not None and args.perceive is not None:
        # Two answers to "where is everything" is one too many, and the loop
        # would silently take the second.
        parser.error("--scene and --perceive are mutually exclusive: one "
                     "reads a scene somebody measured, the other measures it")

    _world, kin = demo_scene()
    scene = None
    world0 = None
    if args.perceive is not None:
        options = args.perceive_opts
        if args.executor == "firmware":
            options = robot_camera_opts(args.robot, options)
        scene = perceived_scene(args.perceive, trace_path=args.trace,
                                obj=args.object, destination=args.destination,
                                options=options)
    elif args.scene is not None:
        from live import load_scene  # noqa: PLC0415
        scene = load_scene(args.scene)
    if scene is not None:
        import dataclasses as _dc  # noqa: PLC0415
        import time as _time  # noqa: PLC0415
        from live import frames_from, objects_from  # noqa: PLC0415
        world0 = _dc.replace(_world, objects=tuple(objects_from(scene)),
                             frames=frames_from(scene, now=_time.time()))
    camera = camera_from_scene(scene)
    robot = build_robot(args.executor, kin, args.robot, scene,
                        world0=world0, obj=args.object)
    stub_scene = (two_things_on(world0, obj=args.object,
                                destination=args.destination)
                  if (args.dry_run or not os.environ.get("OPENAI_API_KEY"))
                  and world0 is not None else None)
    trace = loop(build_model(args.dry_run, args.object, args.destination,
                             declare=stub_scene), robot, task=args.task,
                 max_turns=args.max_turns, trace_path=args.trace,
                 world0=world0, kin=kin, obj=args.object,
                 destination=args.destination, camera=camera)
    for record in trace.records:
        name = (record.choice or {}).get("name") or "(no call)"
        verdict = (record.verdict or {}).get("verdict", "-")
        print(f"turn {record.iteration}: {name:9s} -> {verdict}"
              f"   {(record.verdict or {}).get('reason', '')[:70]}")
    print("\n" + json.dumps(trace.summary(), indent=2))
    for record in trace.disagreements():
        measured = record.goal_verdict or record.verdict or {}
        print(f"\nCLAIMED DONE, NOT MEASURED, at turn {record.iteration}: "
              f"the task verifier says {measured.get('verdict')} — "
              f"{measured.get('reason')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
