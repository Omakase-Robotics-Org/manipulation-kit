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

What moved INTO the wheel on 2026-09-22 (redesign step 7): the loop itself,
the operator policy, the robot adapters and the decision trace —
``manipulation_kit.agent``. What is left here is genuinely about a model:

``astra_loop.py`` the prompt, the OpenAI client and ``main()`` over
                  ``manipulation_kit.agent.run`` (under 200 lines)
``scripted.py``   the scripted stand-in for a model (no key, no network)
``snapshot.py``   the camera-grab contract: fresh, labelled frames or a stop
``detector.py``   a model as the box detector for ``perceive.py``
``perceive.py``   one head frame -> a scene file, over ``manipulation_kit.perception``
``run_scene.py``  the scene a run starts from: ``--perceive`` or ``--scene``
``menu.py``       the Jev-style typed-choice RENDERER: ranking, the cap, the
                  wait/rescan/stop answers, the question itself
``jev_menu.py``   print one such request
``scene.py``      the small shared scene the examples compare against

Run them from a checkout with the kit installed::

    python examples/agent/astra_loop.py --dry-run
    python examples/agent/jev_menu.py --task "put the red block in the box"
"""
