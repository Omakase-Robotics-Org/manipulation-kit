"""ONE jaw-turn ladder, shared by the chooser and by whoever falls back.

Astra review, finding 6: "``choose_side()`` at ``reach.py`` evaluates only the
supplied turn; the outer ``plan_the_hand()`` tries every turn and approach
regardless of the operator restriction. Live fallback is limited to standoff
IK/guard failures. **Chain planning can therefore recommend a strategy the
live gate will refuse.**"

Three copies of ``(0.0, -90.0, 90.0)`` existed and they disagreed about the
order and about whether the asked-for turn was in the list. This file pins the
generator and the two properties the copies kept getting wrong: the asked-for
turn leads, and the chooser actually walks the ladder rather than reporting a
refusal for the one turn it was handed.
"""

from __future__ import annotations

import pytest

from manipulation_kit.primitives.reach import (JAW_TURNS_DEG, choose_side,
                                               jaw_turn_candidates, plan_chain)


def test_the_asked_for_turn_always_leads():
    assert jaw_turn_candidates() == (0.0, -90.0, 90.0)
    assert jaw_turn_candidates(-90.0) == (-90.0, 0.0, 90.0)
    assert jaw_turn_candidates(90.0) == (90.0, 0.0, -90.0)


def test_every_turn_appears_exactly_once():
    for asked in JAW_TURNS_DEG:
        got = jaw_turn_candidates(asked)
        assert sorted(got) == sorted(JAW_TURNS_DEG)
        assert len(set(got)) == len(got)


def test_an_off_ladder_turn_is_tried_first_and_the_ladder_follows():
    """A caller may ask for something not in the fixed list — it still leads,
    and the fallbacks still follow, rather than the ask being dropped."""
    assert jaw_turn_candidates(45.0) == (45.0, 0.0, -90.0, 90.0)


def test_minus_90_is_offered_before_plus_90():
    """Not cosmetic. ``+90`` is the IK-infeasible wrist on this arm (d1-2
    run8, 2026-09-22: four ``ik_fail``s in a row), so trying it first costs a
    whole chain plan per side to find out what the order already knows."""
    ladder = jaw_turn_candidates()
    assert ladder.index(-90.0) < ladder.index(90.0)


# --------------------------------------------------------------------------- #
# the chooser walks it
# --------------------------------------------------------------------------- #

#: A scene where the turns genuinely disagree, which is the only kind of
#: scene this property can be shown in. Block 42x43 mm — squarish, so the
#: footprint axis is y and the default posture closes the jaws across x — at
#: a reach where that wrist is refused and the quarter-turned one is not.
#: Both widths fit the 43.96 mm driven jaws, so what separates the turns here
#: is the ARM, not the object. The same shape as d1-2's run6 charger.
SPLIT_BLOCK_P = (0.46, 0.20, 0.05)
SPLIT_BLOCK_SIZE = (0.042, 0.043, 0.05)


def test_the_chosen_chain_reports_the_turn_that_made_it_feasible(d1_arm,
                                                                 observe):
    """The finding, in one assertion: what the chooser returns has to be
    enough to REPRODUCE the chain. Side alone is not — the caller then binds
    the default turn and is refused by the gate the chooser planned around."""
    world = observe(d1_arm, block_p=(0.38, 0.25, 0.05))
    choice = choose_side(world, d1_arm, obj="red_block", destination="box")
    assert choice.chain is not None
    assert choice.jaw_turn_deg == choice.chain.jaw_turn_deg
    assert choice.jaw_turn_deg in jaw_turn_candidates()
    assert choice.to_json()["jaw_turn_deg"] == choice.jaw_turn_deg
    # and the chain that came back is reproducible from what it published
    again = plan_chain(world, d1_arm, obj="red_block", destination="box",
                       side=choice.side, approach=choice.chain.approach,
                       jaw_turn_deg=choice.chain.jaw_turn_deg)
    assert again.ok == choice.chain.ok
    assert again.planned == choice.chain.planned


def test_pinning_the_ladder_restores_the_old_single_turn_behaviour(d1_arm,
                                                                   observe):
    """``jaw_turns=(x,)`` is the escape hatch for a caller that has a reason
    to try exactly one, so the new default cannot be accused of taking a
    choice away."""
    world = observe(d1_arm, block_p=(0.38, 0.25, 0.05))
    pinned = choose_side(world, d1_arm, obj="red_block", destination="box",
                         jaw_turns=(90.0,))
    for chain in pinned.chains.values():
        assert chain.jaw_turn_deg == 90.0


def _world(d1_arm, observe):
    for side in ("left", "right"):
        d1_arm.set_joints(side, d1_arm.home(side))
    return observe(d1_arm, block_p=SPLIT_BLOCK_P, block_size=SPLIT_BLOCK_SIZE)


def test_the_default_turn_alone_cannot_do_this_task(d1_arm, observe):
    """The premise of the next test, asserted separately so a scene that
    stops being a split fails HERE, saying so, rather than making the
    interesting assertion pass for the wrong reason."""
    only_square = choose_side(_world(d1_arm, observe), d1_arm,
                              obj="red_block", destination="box",
                              jaw_turns=(0.0,))
    assert not only_square.reachable, only_square.reason


def test_the_ladder_recovers_a_task_the_single_turn_refuses(d1_arm, observe):
    """The finding's consequence, end to end.

    Before this, ``choose_side`` answered "no arm plans the whole chain" for a
    task both arms can do a quarter turn round, and the caller's own fallback
    then had to rediscover that one verb at a time, at the standoff, live.
    """
    walked = choose_side(_world(d1_arm, observe), d1_arm, obj="red_block",
                         destination="box")
    assert walked.reachable, walked.reason
    assert walked.jaw_turn_deg == -90.0, walked.reason
    assert walked.chain is not None and walked.chain.ok
    # the turn is in the sentence a caller renders, not only in the data
    assert "-90" in walked.reason


def test_plus_90_is_not_reached_when_minus_90_already_plans(d1_arm, observe):
    """The order is load-bearing: ``+90`` is the infeasible wrist, and the
    ladder must stop at the first turn that plans rather than scoring all
    three and picking by some other rule."""
    world = _world(d1_arm, observe)
    plus = choose_side(world, d1_arm, obj="red_block", destination="box",
                       jaw_turns=(90.0,))
    assert not plus.reachable
    walked = choose_side(_world(d1_arm, observe), d1_arm, obj="red_block",
                         destination="box")
    assert walked.jaw_turn_deg != 90.0
