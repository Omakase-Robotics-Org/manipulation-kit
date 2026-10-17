"""The hand catalog: one index over joint tables, maps, tool configs, envelopes."""
import json
from pathlib import Path

import pytest

from manipulation_kit.hands import catalog as cat
from manipulation_kit.hands import get_retarget
from manipulation_kit.hands.cli import main
from manipulation_kit.hands.mapping import RetargetTable, retarget_table

PKG = Path(cat.__file__).resolve().parents[1]


def test_the_committed_catalog_is_what_the_code_emits():
    assert main(["catalog", "--check"]) == 0, "run: mkit-toolconfig catalog --write"
    assert cat.load() == cat.build()


def test_every_tool_config_and_hand_package_is_catalogued():
    models = {e.model for e in cat.catalog()}
    exported = {p.stem for p in (PKG / "config" / "tool_configs").glob("*.json")}
    assert exported <= {m.replace("/", "_") for m in models}, "a tool config with no model"
    assert {e.tool_config for e in cat.catalog() if e.tool_config} == {
        f"config/tool_configs/{s}.json" for s in exported}
    packages = {f"{p.parent.parent.name}/{p.parent.name}"
                for name in ("axes.py", "toolconfig.py")
                for p in (PKG / "hands").glob(f"*/*/{name}")}
    assert packages <= models, f"uncatalogued: {sorted(packages - models)}"
    for e in cat.catalog():
        if e.tool_config is not None:
            assert (PKG / e.tool_config).is_file()
        if e.retarget_table is not None:
            table = RetargetTable.from_json(json.loads((PKG / e.retarget_table).read_text()))
            assert table == retarget_table(e.model)
            assert e.joints == len(get_retarget(e.model).JOINTS) == len(table.rows)


def test_envelope_ids_name_their_shape_and_no_vendor():
    from manipulation_kit.description.d1.tools.generate_d1_urdf import HAND_ENVELOPE

    envs = cat.envelopes()
    gripper = cat.entry("d1/parallel_gripper")["envelope"]
    assert gripper["verified"] is True
    assert [(c["part"], tuple(c["a"]), tuple(c["b"]), c["radius"]) for c in gripper["capsules"]] \
        == [(p, tuple(a), tuple(b), r) for p, a, b, r in HAND_ENVELOPE]
    for model, kind, _ in cat.MODELS:
        env = cat.entry(model)["envelope"]
        assert env["id"].startswith("capsules-") and env["id"] in envs
        assert env["frame"] == "flange"
        maker, _, name = model.partition("/")
        assert maker not in env["id"] and name not in env["id"]
        if kind == "hand":
            assert env["verified"] is False, "no five-finger envelope is measured yet"
    for env_id, env in envs.items():
        assert env.id == env_id == "capsules-" + env.digest()[:16]
    moved = cat.Envelope((("hand", (0.0, 0.0, 0.04), (0.0, 0.0, 0.21), 0.07),), False, "x")
    assert moved.id != cat.FIVE_FINGER_DEFAULT.id


def test_model_ids_resolve_their_aliases_and_refuse_strangers():
    assert cat.canonical_model("wuji/hand2") == "wuji/wujihand2"
    assert cat.canonical_model("wuji/wujihand2") == "wuji/wujihand2"
    assert cat.entry("wuji/hand2")["model"] == "wuji/wujihand2"
    with pytest.raises(ValueError, match="no hand model"):
        cat.canonical_model("acme/claw")
    assert set(cat.MODEL_ALIASES.values()) <= {m for m, _, _ in cat.MODELS}
    assert not set(cat.MODEL_ALIASES) & {m for m, _, _ in cat.MODELS}
