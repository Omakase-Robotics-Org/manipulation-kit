"""``OperatorPolicy``: the one home of what this robot may do tonight (B12, L10)."""

from __future__ import annotations

import argparse
import json

import pytest

from manipulation_kit.agent import (DIRECTION_NOT_ALLOWED, NUDGE_LIMIT,
                                    OperatorPolicy, PolicyState)
from manipulation_kit.primitives import Grasp, Lift, Nudge
from manipulation_kit.primitives.types import Unmet


def test_a_policy_refusal_is_a_kit_Unmet(agent_examples):
    """B12: the approach restriction used to answer with a hand-rolled dict
    ``{"reason": "approach_disabled"}`` that looked like no other refusal. It
    is a kit ``Unmet`` now, and in the loop a kit ``PlanError``."""
    policy = OperatorPolicy(allowed_directions=("down",))
    call, unmet = policy.clamp(Grasp(object="red_block", side="left",
                                     direction="forward"))
    assert len(unmet) == 1 and isinstance(unmet[0], Unmet)
    assert unmet[0].code == DIRECTION_NOT_ALLOWED
    assert unmet[0].measured["allowed"] == ["down"]
    assert "down" in unmet[0].remedy
    # a free {axis, frame} direction is outside a named allow-list too
    _call, unmet = policy.clamp(Grasp(object="red_block",
                                      direction={"axis": [0, 0.3, -1],
                                                 "frame": "base"}))
    assert [u.code for u in unmet] == [DIRECTION_NOT_ALLOWED]
    # the allowed one, and a verb whose direction is not an approach, pass
    assert policy.clamp(Grasp(object="red_block", direction="down"))[1] == []
    assert policy.clamp(Lift(object="red_block"))[1] == []

    # ...and through the loop it is a refusal like every other refusal
    import astra_loop

    class Forward:
        def __call__(self, messages, tools):
            return {"name": "grasp", "call_id": "f", "claimed": "",
                    "arguments": {"object": "red_block", "side": "left",
                                  "direction": "forward"}}

    trace = astra_loop.loop(Forward(), max_turns=1, policy=policy)
    refusal = trace.records[0].refused[0]
    assert refusal["schema"] == "manipulation_kit.refusal/2"
    assert refusal["reason"] == "precondition_unmet"
    assert refusal["unmet"][0]["code"] == DIRECTION_NOT_ALLOWED
    assert trace.records[0].run is None


def test_the_grip_cap_lowers_the_grip_and_says_so():
    policy = OperatorPolicy(max_grip="soft")
    asked = Grasp(object="cube", grip="strong")
    call, unmet = policy.clamp(asked)
    assert call.grip == "soft" and unmet == []
    assert policy.notes(asked, call) == [
        "grip 'strong' is capped to 'soft' on this robot tonight"]
    assert policy.clamp(Grasp(object="cube", grip="soft"))[0].grip == "soft"
    # the asked-for call is a value and is not rewritten
    assert asked.grip == "strong"


def test_the_nudge_budget_is_per_hand_per_target():
    policy = OperatorPolicy(max_nudges_per_target=2)
    state = PolicyState()
    state.aimed("left", "cube")
    nudge = Nudge(side="left", dx=0.01)
    for _ in range(2):
        assert policy.clamp(nudge, state, side="left")[1] == []
        state.nudged("left")
    assert [u.code for u in policy.clamp(nudge, state, side="left")[1]] == [
        NUDGE_LIMIT]
    state.aimed("left", "cup")          # a new target, a new budget
    assert policy.clamp(nudge, state, side="left")[1] == []


def test_a_policy_is_a_file_and_a_set_of_flags(tmp_path):
    policy = OperatorPolicy(max_grip="firm", allowed_directions=("down", "forward"),
                            vel_ratio=0.2, stroke_timeout_s=30.0)
    data = policy.to_json()
    assert json.loads(json.dumps(data)) == data
    assert OperatorPolicy.from_json(data) == policy
    with pytest.raises(ValueError):
        OperatorPolicy.from_json({"grip_cap": "soft"})
    path = tmp_path / "p.json"
    path.write_text(json.dumps(data))
    parser = argparse.ArgumentParser()
    OperatorPolicy.add_arguments(parser)
    args = parser.parse_args(["--policy", str(path), "--max-grip", "soft",
                              "--no-look-before-stroke", "--max-turns", "5"])
    got = OperatorPolicy.from_args(args)
    assert got.max_grip == "soft" and got.look_before_stroke is False
    assert got.max_turns == 5 and got.vel_ratio == 0.2
    assert got.allowed_directions == ("down", "forward")


@pytest.mark.parametrize("bad", [dict(max_grip="crushing"),
                                 dict(allowed_directions=("sideways",)),
                                 dict(vel_ratio=15), dict(vel_ratio=0.0),
                                 dict(arrive_timeout_s=float("nan")),
                                 dict(max_turns=0),
                                 dict(max_nudges_per_target=-1)])
def test_a_policy_that_means_nothing_is_refused_at_construction(bad):
    with pytest.raises(ValueError):
        OperatorPolicy(**bad)


def test_the_contact_thresholds_are_capped_like_the_grip():
    """Step 4's ``Probe.contact_nm`` / ``Press.force_nm`` are torque stops a
    model chooses; the policy lowers them to the operator's cap. (A stand-in
    verb here: the contact verbs land with step 4.)"""
    from dataclasses import dataclass

    from manipulation_kit.primitives.types import Primitive

    @dataclass(frozen=True)
    class Pressing(Primitive):
        VERB = "press"
        side: str = "left"
        force_nm: float = 8.0

    policy = OperatorPolicy(max_force_nm=5.0, max_contact_nm=3.0)
    asked = Pressing()
    call, unmet = policy.clamp(asked)
    assert call.force_nm == 5.0 and unmet == []
    assert policy.notes(asked, call) == [
        "force_nm 8.0 is capped to 5.0 Nm on this robot tonight"]
    assert policy.clamp(Pressing(force_nm=2.0))[0].force_nm == 2.0
    assert OperatorPolicy.from_json(policy.to_json()) == policy
