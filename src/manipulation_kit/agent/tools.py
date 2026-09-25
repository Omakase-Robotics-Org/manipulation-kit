"""The two tools that are about the OBSERVATION, not about moving.

The kit owns the verbs and refuses to own the observation — nothing in it
opens a camera — and a scene file is somebody with a tape measure. These two
are the third option, offered beside the motion verbs every turn:

``declare_scene``  "the cup is here, this big", in base metres. Validated by
                   the SAME reader a hand-written scene file goes through
                   (:func:`~manipulation_kit.agent.robot.objects_from`), so a
                   model cannot declare something a person could not have
                   written, and lifted onto its support by the kit's rule.
``locate``         a pixel the model picked in a named camera -> a base-frame
                   point on the table plane (a CONTACT point), or the object's
                   CENTRE when it gives the size, with the uncertainty the
                   nominal mount carries. On a WRIST camera it is also the
                   correction half of the look-before-stroke rule (see
                   :mod:`manipulation_kit.agent.loop`).

Neither moves anything, so neither goes through the motion gate. Moved here
from the examples (design C.8): they are loop mechanics, not model-facing
text.
"""

from __future__ import annotations

from typing import Any, Dict, List, Mapping, Optional, Sequence

SCENE_TOOLS = ("declare_scene", "locate")

#: What a declared object may say. Deliberately the SAME fields a scene file
#: carries, so "the model measured it" and "a person measured it" produce the
#: same kind of thing and are read by the same code.
DECLARE_SCHEMA: Dict[str, Any] = {
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
                        # interior nobody stated.
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


def locate_schema(cameras: Sequence[str]) -> Dict[str, Any]:
    """``locate`` over the cameras that exist right now."""
    names = list(cameras)
    return {
        "name": "locate",
        "description": (
            "Turn a pixel you picked in a photo into a point in the robot's "
            "base frame, on the table plane. Exact geometry, no guessing - "
            "use it instead of estimating a position by eye. On a WRIST "
            "camera, after the robot has looked before a grasp, it is also "
            "the correction: the object is re-measured there and the hand is "
            "nudged by the difference."),
        "parameters": {
            "type": "object",
            "properties": {
                "u": {"type": "number", "description": "pixel x, 0 at the left"},
                "v": {"type": "number", "description": "pixel y, 0 at the top"},
                "camera": {"type": "string", "enum": names,
                           "default": names[0] if names else "head"},
                "size": {"type": "array", "minItems": 3, "maxItems": 3,
                         "items": {"type": "number", "exclusiveMinimum": 0},
                         "description": "optional: the object's size in "
                                        "metres. With it, (u, v) is read as "
                                        "the BOTTOM of the object's "
                                        "silhouette and the answer is the "
                                        "object's CENTRE, ready to declare. "
                                        "Without it the answer is the "
                                        "contact point on the table"},
                "yaw_rad": {"type": "number",
                            "description": "optional, with size: the "
                                           "object's yaw"}},
            "required": ["u", "v"], "additionalProperties": False}}


#: the head-only ``locate`` schema, for callers that want the constant
LOCATE_SCHEMA = locate_schema(["head"])


def scene_tools(cameras: Any = None) -> List[Dict[str, Any]]:
    """The observation tools available right now.

    ``locate`` is offered only when a camera model exists, because without one
    it would have to invent a projection — and a tool that sometimes answers
    from geometry and sometimes from nothing is worse than a missing tool.
    ``cameras`` is a mapping of camera models, or one head camera.
    """
    models = _as_cameras(cameras)
    return [DECLARE_SCHEMA] + ([locate_schema(sorted(models))] if models
                               else [])


def _as_cameras(cameras: Any) -> Dict[str, Any]:
    if cameras is None:
        return {}
    if isinstance(cameras, Mapping):
        return dict(cameras)
    return {"head": cameras}


def apply_declare_scene(robot: Any, arguments: Dict[str, Any], *,
                        provenance: str = "declared") -> str:
    """Decode a ``declare_scene`` call and hand it to the robot.

    Validation is the scene-file reader's, so a zero size or a two-element
    position is refused with the reader's own message rather than crashing
    four verbs later. THE DESCENT FLOOR TRUSTS THE OBJECT'S BOTTOM, so a
    declared object is lifted onto the surface it stands on — the kit's rule,
    which resolves the surface's frame and rotation (design review 4).
    """
    from ..perception import lift_onto_support  # noqa: PLC0415
    from ..world import FrameGraph, SurfaceView  # noqa: PLC0415
    from .robot import objects_from  # noqa: PLC0415
    if not getattr(robot, "can_declare", hasattr(robot, "declare")):
        return ("this robot cannot take a declared scene (its world comes from "
                "somewhere else); nothing was changed")
    items = []
    for item in arguments.get("objects") or []:
        item = dict(item)
        item.setdefault("provenance", provenance)
        if "interior" in item:
            # Stated by the model = stated. The flag is what `Place` reads.
            item["interior_measured"] = True
        if item.get("kind") == "surface":
            item.setdefault("plane_source", "declared")
        items.append(item)
    try:
        views = objects_from({"objects": items})
    except (KeyError, ValueError, TypeError) as exc:
        return f"that scene is malformed and nothing was changed: {exc}"
    surfaces = [v for v in views if isinstance(v, SurfaceView)]
    frames = FrameGraph()
    try:
        current = robot.world()
        surfaces += list(current.surfaces())
        frames = current.frames
    except Exception:  # noqa: BLE001 - no world yet is fine
        pass
    views, lifted = lift_onto_support(views, surfaces, frames)
    robot.declare(list(views))
    # a restated scene: the contacts measured against the old one go with it
    forget = getattr(robot, "forget_contacts", None)
    if callable(forget):
        forget()
    lines = [f"{v.name!r} ({v.kind}) at ({v.p[0]:.3f}, {v.p[1]:.3f}, "
             f"{v.p[2]:.3f}) m, {v.size[0]*1000:.0f}x{v.size[1]*1000:.0f}x"
             f"{v.size[2]*1000:.0f} mm" for v in views]
    return ("recorded, and the next observation is measured against it: "
            + "; ".join(lines)
            + ((" | corrections: " + "; ".join(lifted)) if lifted else ""))


def locate(cameras: Any, world: Any, arguments: Dict[str, Any]):
    """A ``locate`` call -> a :class:`~manipulation_kit.perception.Located`
    (contact, or centre given a size). Raises what the perceiver raises."""
    from ..perception import ScenePerceiver, contact_to_centre  # noqa: PLC0415
    models = _as_cameras(cameras)
    name = str(arguments.get("camera") or ("head" if "head" in models
                                           else next(iter(models), "head")))
    perceiver = ScenePerceiver(models, world)
    located = perceiver.locate(name, float(arguments["u"]),
                               float(arguments["v"]))
    if arguments.get("size") is not None:
        located = contact_to_centre(
            located, size=arguments["size"], viewpoint=models[name].p,
            yaw_rad=float(arguments.get("yaw_rad", 0.0)),
            image_up=_image_up(perceiver, name, arguments, located))
    return located


def _image_up(perceiver, name, arguments, located):
    """The table-plane direction the image's up axis runs at the located
    pixel (a pixel above it, located on the same plane), or None where that
    pixel has no plane point (the horizon)."""
    from ..perception import NoSupport, NotOnThePlane  # noqa: PLC0415
    try:
        above = perceiver.locate(name, float(arguments["u"]),
                                 float(arguments["v"]) - 8.0)
    except (NoSupport, NotOnThePlane, ValueError):
        return None
    return above.p - located.p


def apply_locate(cameras: Any, world: Any, arguments: Dict[str, Any]) -> str:
    """Answer a ``locate`` call through the kit's perceiver, in words."""
    from ..perception import NoSupport, NotOnThePlane  # noqa: PLC0415
    models = _as_cameras(cameras)
    if not models:
        return ("there is no camera model in this run, so a pixel cannot be "
                "turned into a position")
    try:
        return locate(models, world, arguments).to_text()
    except NoSupport:
        return ("there is no table in the scene yet, so a pixel has no plane "
                "to land on. Declare the surface first with declare_scene "
                "(kind 'surface'), then ask again")
    except NotOnThePlane as exc:
        return str(exc)
    except (KeyError, TypeError, ValueError) as exc:
        return f"that pixel is not usable: {exc}"


__all__ = ["DECLARE_SCHEMA", "LOCATE_SCHEMA", "SCENE_TOOLS",
           "apply_declare_scene", "apply_locate", "locate", "locate_schema",
           "scene_tools"]
