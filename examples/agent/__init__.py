"""examples.agent — the part that knows a MODEL exists. Nothing else.

OUTSIDE THE WHEEL, on purpose. Shu, 2026-09-19: 「agent 的なのは examples フォルダ
に切り離す」 — the agent-shaped code is split out into examples.

What moved INTO the wheel on 2026-09-19, after the review: the offer gate and
its result types, the canonical argument metadata and the JSON Schema export,
and the reach/arm-selection. Those are capability questions — *what can this
robot do right now*, *what may an argument be*, *which hand can deliver* — and
a script, a teleop assist or a collection macro needs all three without ever
meeting a model. They live in ``manipulation_kit.primitives.{offer,schema,
arguments,reach}`` and they add no dependency: the package still installs as
numpy + scipy.

What is left here is genuinely about a model:

``astra_loop.py`` a runnable function-calling loop — the prompt, the provider
                  client, the scripted stand-in, the message bookkeeping, and
                  the three stop reasons
``menu.py``       the Jev-style typed-choice RENDERER: ranking, the cap, the
                  wait/rescan/stop answers, the question itself
``jev_menu.py``   print one such request
``mirror.py``     the demo robot: a KinematicExecutor plus a scene the block
                  moves in
``live.py``       a real D1: the firmware transport plus a measured scene file
                  turned into a WorldView
``scene.py``      the small shared scene the examples compare against
``trace.py``      one JSONL record per decision, the model's claim beside the
                  measurement

Run them from a checkout with the kit installed::

    python examples/agent/astra_loop.py --dry-run
    python examples/agent/jev_menu.py --task "put the red block in the box"
"""
