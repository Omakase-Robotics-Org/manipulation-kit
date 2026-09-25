"""An empty scene must not narrow object names to an EMPTY enum.

The Responses API treats ``"enum": []`` as an unsatisfiable grammar and answers
``status=incomplete`` with zero output tokens and no error (d1-2, 2026-09-22).
"""
import dataclasses as dc

from manipulation_kit.primitives.schema import tool_schemas
from manipulation_kit.world.views import WorldView


def _empty_world() -> WorldView:
    return WorldView.of(objects=(), frames=None, arms=(), grippers=())


def test_empty_scene_leaves_object_names_free():
    for tool in tool_schemas(_empty_world()):
        for name, prop in tool["parameters"]["properties"].items():
            if name in ("object", "to", "source", "target"):
                assert prop.get("enum") != [], (tool["name"], name)
                assert "enum" not in prop or prop["enum"], (tool["name"], name)
