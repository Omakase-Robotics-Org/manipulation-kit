"""A hand's retarget map as data: tables, held joints, golden vectors."""
import json
import random

import pytest

from manipulation_kit.gloves import HandPose
from manipulation_kit.gloves.v1 import parse_declaration
from manipulation_kit.hands import get_retarget
from manipulation_kit.hands.mapping import (
    GOLDEN_SCHEMA,
    TABLE_SCHEMA,
    RetargetTable,
    drivability,
    golden_inputs,
    retarget_table,
)
from manipulation_kit.hands.retarget_cli import GOLDEN_FILE, MODELS, TABLE_FILE, main, model_dir
from manipulation_kit.hands.wuji.wujihand2.axes import AXIS_NAMES
from manipulation_kit.hands.wuji.wujihand2.retarget import RetargetConfig

FINGERS = ("index", "middle", "ring", "pinky")


def test_the_committed_tables_and_golden_vectors_are_what_the_code_emits():
    assert main(["check"]) == 0, "run: mkit-retarget build"


@pytest.mark.parametrize("model", MODELS)
def test_the_map_reproduces_its_golden_vectors(model):
    golden = json.loads((model_dir(model) / GOLDEN_FILE).read_text())
    table = RetargetTable.from_json(json.loads((model_dir(model) / TABLE_FILE).read_text()))
    assert golden["schema"] == GOLDEN_SCHEMA and golden["model"] == model
    assert golden["table_digest"] == table.digest() == retarget_table(model).digest()
    assert golden["joints"] == [j.name for j in table.joints]
    assert len(golden["vectors"]) >= 3 + 2 * len(table.channels())
    for vector in golden["vectors"]:
        mapper = get_retarget(model)
        assert mapper.joint_targets(HandPose("t", vector["flex"])) == vector["targets"]
        assert table.evaluate(vector["flex"]) == vector["targets"]


@pytest.mark.parametrize("model", MODELS)
def test_a_table_round_trips_through_json(model):
    table = retarget_table(model)
    doc = table.to_json()
    assert doc["schema"] == TABLE_SCHEMA
    assert RetargetTable.from_json(json.loads(json.dumps(doc))) == table
    assert table.channels() == tuple(get_retarget(model).required_channels())
    with pytest.raises(ValueError, match="schema"):
        RetargetTable.from_json({**doc, "schema": "other"})


def test_a_table_follows_a_config_as_well_as_the_defaults():
    config = RetargetConfig(hold=(False,) * 20)
    table = retarget_table("wuji/wujihand2", config)
    assert not any(table.held())
    rng = random.Random(7)
    for flex in golden_inputs(table.channels(), seed=3, random_count=50):
        flex = {c: (None if rng.random() < 0.05 else v) for c, v in flex.items()}
        mapper = get_retarget("wuji/wujihand2", config=config)
        assert table.evaluate(flex) == mapper.joint_targets(HandPose("t", flex))


def test_held_joints_read_nothing_and_are_never_lacking():
    table = retarget_table("wuji/wujihand2")
    held = table.held()
    assert [AXIS_NAMES[i] for i, h in enumerate(held) if h] == list(AXIS_NAMES[:4])
    assert table.channels_of("thumb_S1") == ()
    assert table.channels_of("ring_S2") == ("ring_mp_swing",)
    swings = [f"{f}_mp_swing" for f in FINGERS]
    assert table.lacking(swings + ["thumb_cm_yaw"]) == {
        f"{f}_S2": (f"{f}_mp_swing",) for f in FINGERS}
    held_spread = table.holding({f"{f}_S2": 0.0 for f in FINGERS})
    assert held_spread.lacking(swings) == {}
    assert sum(held_spread.held()) == 8
    assert held_spread.evaluate({})[AXIS_NAMES.index("index_S2")] == 0.0
    assert held_spread.evaluate({})[AXIS_NAMES.index("index_S1")] is None
    with pytest.raises(KeyError):
        table.holding({"wrist": 0.0})
    with pytest.raises(ValueError, match="hold"):
        table.holding({"index_S2": 2.0})


def _declaration(swing="uncalibrated"):
    chans = {}
    for f in FINGERS:
        for j in ("mp_pitch", "pip_pitch", "dip_pitch"):
            chans[f"{f}_{j}"] = dict(kind="flex", provenance="measured", default_range=None,
                                     note="synthetic")
        chans[f"{f}_mp_swing"] = dict(kind="swing", provenance=swing, note="synthetic",
                                      default_range=None if swing == "uncalibrated" else [-0.2, 0.2])
    return parse_declaration({"type": "channel_declaration", "device_model": "test/glove",
                              "channels": chans})


def test_drivability_names_what_a_declaration_cannot_drive():
    from manipulation_kit.gloves.calibration import ChannelRange

    table = retarget_table("wuji/wujihand2")
    ranges = {f"{f}_{j}": ChannelRange(0.0, 1.5)
              for f in FINGERS for j in ("mp_pitch", "pip_pitch", "dip_pitch")}
    d = drivability(table, _declaration(), ranges)
    s2 = {AXIS_NAMES.index(f"{f}_S2") for f in FINGERS}
    assert d.held == table.held()
    assert [AXIS_NAMES[i] for i, a in enumerate(d.armed) if not a] == [
        n for i, n in enumerate(AXIS_NAMES) if i < 4 or i in s2]
    assert d.lacking == {f"{f}_S2": (f"{f}_mp_swing",) for f in FINGERS}
    assert len(d.refusals) == 4 and all("UNCALIBRATED" in r for r in d.refusals)

    # flexion with no range anywhere: those joints are lacking too, unrefused
    d = drivability(table, _declaration(swing="measured"))
    assert d.refusals == ()
    assert d.lacking["index_S1"] == ("index_mp_pitch",)
    assert d.armed[AXIS_NAMES.index("index_S2")]
    assert not d.armed[AXIS_NAMES.index("index_S1")]
