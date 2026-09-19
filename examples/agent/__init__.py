"""examples.agent — rendering the kit's primitives to a model, and back.

OUTSIDE THE WHEEL, on purpose. Shu, 2026-09-19: 「agent 的なのは examples フォルダ
に切り離す」 — the agent-shaped code is split out into examples.

The reason it is a good split and not merely a preference: the primitives are a
robot capability and the agent layer is one way of driving them. Scripts,
teleop assists, collection macros and learned pipelines all want
``Grasp(...).plan(world, kin)`` and none of them want a JSON tool schema. If
the schema lived in the package, every consumer of the kinematics would carry
it, and — worse — the vocabulary would start growing toward whatever the
current model happens to find easy. Here, it cannot: these files IMPORT
``manipulation_kit.primitives`` and add nothing to it.

What is here:

``offer.py``   the gate — IK + guard before a candidate becomes a word in the
               prompt, with the refusals kept and reported
``schema.py``  one definition set, two exports: JSON Schema for Astra-style
               function calling, a choice menu for Jev-style typed answers
``trace.py``   one JSONL record per decision, including the MEASURED verdict
               beside the model's claim
``astra_loop.py`` a runnable function-calling loop (scripted stub with no key)
``jev_menu.py``   the same offer rendered as a typed-choice request

Run them from a checkout with the kit installed::

    python examples/agent/astra_loop.py --dry-run
    python examples/agent/jev_menu.py
"""
