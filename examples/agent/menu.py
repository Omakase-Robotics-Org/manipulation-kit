"""Render the kit's offer as a Jev-style typed-choice request. A RENDERER.

Everything checkable is in the wheel now
(:mod:`manipulation_kit.primitives.offer`); what is here is the part that is
about the model in front of you — how many choices it can read, in what order,
and what the question actually says.

Four things the old menu got wrong, and they are all in this file because they
are all rendering:

1. **The question did not contain the task.** It asked which action "moves the
   task forward" without ever naming the task (R, section 3). A choice among
   twenty legal actions with no goal is a choice among twenty equally good
   answers.
2. **Choices were index-only.** An index into a list changes meaning the
   moment the list is rebuilt from a newer observation, which is exactly when
   a model is answering with one. Every choice now carries
   ``Offered.id`` — a stable, bound identity — and the menu carries the
   observation's revision so a stale answer can be detected rather than
   obeyed.
3. **The cap trimmed after planning.** Every candidate was planned and then
   the tail was thrown away, so the cap bounded neither the computation nor
   what the model could see, and the right-hand action could vanish because
   the left arm was generated first. The cap is applied to a DIVERSE
   selection here, and the count of what it hid is reported.
4. **There was no way to say "not yet".** ``wait``, ``rescan`` and ``stop``
   are real answers to "which action moves the task forward" and a menu
   without them forces a move.
"""

from __future__ import annotations

from typing import Any, Dict, List, Sequence

from manipulation_kit.primitives.offer import (Offered, candidates_for,
                                               offer, why_nothing)

#: A readable menu is about twenty labels. Jev itself allows up to 255 choices,
#: but a model choosing among 200 near-identical options is not choosing.
DEFAULT_CAP = 20

#: The answers that are not a motion. A menu without them forces a move.
NON_MOTION = (
    {"id": "wait", "label": "wait and look again (nothing is ready yet)"},
    {"id": "rescan", "label": "ask for a fresh observation"},
    {"id": "stop", "label": "stop: the task is done, or cannot be done"},
)


#: verbs that advance a pick-and-place, in the order they are used. A menu
#: that spent its twenty slots on corrections has hidden the grasp.
TASK_VERBS = ("grasp", "approach", "lift", "carry", "place", "release", "pour")


def rank(offered: Sequence[Offered], *, task: str) -> List[Offered]:
    """Task verbs before corrections, and both hands before either's tail.

    Two rules, and they compose in that order. TASK FIRST, because a cap that
    trims corrections has trimmed the fine adjustments and a cap that trims
    task verbs has trimmed the point. THEN interleave the sides: the old
    candidate order was every left-arm action followed by every right-arm one,
    so a cap of 20 could remove every right-hand option as the scene grew
    (R, section 3).

    Deliberately NOT task-aware beyond the verb order — nothing here reads
    ``task``. It is passed so the signature does not change when something
    does, and so the caller cannot forget that the model is going to be asked
    about a task it has not been told.
    """
    def group(item: Offered) -> int:
        try:
            return TASK_VERBS.index(item.primitive.name())
        except ValueError:
            return len(TASK_VERBS)

    out: List[Offered] = []
    for rung in range(len(TASK_VERBS) + 1):
        tier = [i for i in offered if group(i) == rung]
        by_side: Dict[str, List[Offered]] = {}
        for item in tier:
            by_side.setdefault(getattr(item.primitive, "side", "") or "",
                               []).append(item)
        # A fixed order over the keys that are ACTUALLY present. Hard-coding
        # ("left", "right", "") looked equivalent and was not: ``go_home``
        # binds side="both", its bucket was never drained, and the loop did
        # not end.
        order = [s for s in ("left", "right") if s in by_side]
        order += [s for s in sorted(by_side) if s not in order]
        while any(by_side[s] for s in order):
            for side in order:
                if by_side[side]:
                    out.append(by_side[side].pop(0))
    return out


def choice_menu(world, kin, *, task: str, cap: int = DEFAULT_CAP,
                candidates=None) -> Dict[str, Any]:
    """A Jev-style typed-choice request: already-bound options, each planned.

    The arguments are gone by the time the model sees this — every choice is a
    concrete primitive that has already passed IK and the guard. That is the
    whole reason a typed-choice model can drive a robot at all.
    """
    offered, refused = offer(
        candidates if candidates is not None else candidates_for(world),
        world, kin)
    ordered = rank(offered, task=task)
    shown, hidden = ordered[:cap], ordered[cap:]
    return {
        "task": task,
        "observation": {"revision": world.revision, "stamp": world.stamp},
        "state": world.to_text(),
        "question": (f"TASK: {task}\n"
                     f"Which single action moves THAT task forward? Answer "
                     f"with one id."),
        "choices": [{"index": i, "id": item.id, "label": item.label,
                     "verb": item.primitive.name(),
                     "arguments": {n: getattr(item.primitive, n)
                                   for n in item.primitive.arguments()}}
                    for i, item in enumerate(shown)] + list(NON_MOTION),
        "hidden": [{"id": item.id, "label": item.label} for item in hidden],
        "tried": len(offered) + len(refused),
        "offered": len(offered),
        "nothing_offered": why_nothing(refused) if not offered else "",
        "refused": [r.to_json() for r in refused],
    }


def render(menu: Dict[str, Any]) -> str:
    lines = [menu["state"], "", menu["question"]]
    if not menu["choices"]:
        lines += ["", menu["nothing_offered"]]
        return "\n".join(lines)
    lines += [f"  {c.get('index', '  ')!s:>3}. {c['label']}"
              for c in menu["choices"]]
    if menu["hidden"]:
        lines.append(f"({len(menu['hidden'])} further checked actions did not "
                     f"fit this menu; ask for another family to see them.)")
    if menu["refused"]:
        lines += ["", f"({len(menu['refused'])} of {menu['tried']} actions were "
                      f"considered and refused by the motion guard or the IK; "
                      f"they are in the trace, not in this menu — an "
                      f"unreachable option never becomes a word in the prompt.)"]
    return "\n".join(lines)
