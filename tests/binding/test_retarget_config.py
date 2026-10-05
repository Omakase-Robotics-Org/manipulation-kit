"""[bind.retarget_config]: per-binding RetargetConfig overrides, checked at load."""
import textwrap

import pytest

from manipulation_kit.binding import BindingError, build_retarget_config, load_binding
from manipulation_kit.hands.leadshine.dh116s.retarget import RetargetConfig as DhConfig
from manipulation_kit.hands.robotera.xhand1_lite.retarget import (
    RetargetConfig as LiteConfig,
    default_joint_maps,
)

HEAD = '''
schema = "omakase.binding.v1"

[[source]]
device = "glove"
family = "teleop_gloves.pose_stream.v1"
'''


def _load(tmp_path, text):
    path = tmp_path / "binding.toml"
    path.write_text(HEAD + textwrap.dedent(text))
    return load_binding(path)


def _problems(tmp_path, text):
    with pytest.raises(BindingError) as err:
        _load(tmp_path, text)
    return err.value.problems


def test_dh116s_scalar_and_channel_overrides(tmp_path):
    b = _load(tmp_path, '''
        [[bind]]
        from = "glove"
        to = "hand/right"
        retarget = "leadshine/dh116s"

        [bind.retarget_config]
        thumb_w_cm_pitch = 0
        thumb_swing_roll_blend = 0.3
        thumb_swing_blend_channel = "thumb_cm_swing"
        invert = [false, false, true, false, false, false]
    ''')
    (bind,) = b.binds
    cfg = bind.retarget_config
    assert isinstance(cfg, DhConfig)
    assert cfg.thumb_w_cm_pitch == 0.0 and isinstance(cfg.thumb_w_cm_pitch, float)
    assert cfg.thumb_swing_blend_channel == "thumb_cm_swing"
    assert cfg.invert == (False, False, True, False, False, False)
    assert cfg.w_mp == DhConfig().w_mp                      # untouched default
    hand = bind.build_retarget()
    req = hand.required_channels()
    assert "thumb_cm_pitch" not in req and "thumb_cm_swing" in req
    assert bind.retarget_overrides["thumb_w_cm_pitch"] == 0


def test_lite_joint_map_overrides_merge_onto_the_defaults(tmp_path):
    b = _load(tmp_path, '''
        [[bind]]
        from = "glove"
        to = "hand/left"
        retarget = "robotera/xhand1_lite"

        [bind.retarget_config.joints.thumb_rota1]
        sources = { thumb_mp_pitch = 1.0 }
        lo = 0.12

        [bind.retarget_config.joints.pinky_j1]
        invert = true
    ''')
    cfg = b.binds[0].retarget_config
    assert isinstance(cfg, LiteConfig)
    rota = cfg.joints["thumb_rota1"]
    assert rota.sources == (("thumb_mp_pitch", 1.0),)
    assert rota.lo == 0.12 and rota.hi == default_joint_maps()["thumb_rota1"].hi
    assert cfg.joints["pinky_j1"].invert is True
    assert cfg.joints["index_j1"] == default_joint_maps()["index_j1"]
    assert "thumb_cm_pitch" not in b.binds[0].build_retarget().required_channels()


def test_every_problem_is_listed(tmp_path):
    problems = _problems(tmp_path, '''
        [[bind]]
        from = "glove"
        to = "hand/right"
        retarget = "leadshine/dh116s"

        [bind.retarget_config]
        thumb_w_cm_pich = 0.0
        w_mp = "half"
        ema_alpha = true
        invert = [true, false]
        thumb_swing_blend_channel = "thumb_cmc_swing"
    ''')
    text = "\n".join(problems)
    assert len(problems) == 5, problems
    for needle in ("no field 'thumb_w_cm_pich'", "w_mp: expected a number",
                   "ema_alpha: expected a number", "array of 6 values",
                   "'thumb_cmc_swing' is not a glove channel"):
        assert needle in text


def test_joint_map_problems_and_the_configs_own_checks(tmp_path):
    problems = _problems(tmp_path, '''
        [[bind]]
        from = "glove"
        to = "hand/right"
        retarget = "robotera/xhand1_lite"

        [bind.retarget_config.joints.thumb_rota_joint9]
        lo = 0.1

        [bind.retarget_config.joints.index_j1]
        sources = { index_mcp = 1.0 }
        gain = 2
    ''')
    text = "\n".join(problems)
    assert "thumb_rota_joint9: no such joint" in text
    assert "'index_mcp' is not a glove channel" in text
    assert "unknown key 'gain'" in text
    # values that type-check but leave the hand: the dataclass refuses them
    (p,) = _problems(tmp_path, '''
        [[bind]]
        from = "glove"
        to = "hand/right"
        retarget = "robotera/xhand1_lite"

        [bind.retarget_config.joints.pinky_j1]
        hi = 1.9
    ''')
    assert "leaves the joint's limits" in p


def test_retarget_config_needs_a_whole_hand_and_a_retarget(tmp_path):
    (p,) = _problems(tmp_path, '''
        [[bind]]
        from = "glove"
        to = "hand/right"

        [bind.retarget_config]
        ema_alpha = 0.1
    ''')
    assert "needs a resolvable 'retarget'" in p
    problems = _problems(tmp_path, '''
        [[bind]]
        from = "glove/index_mp_pitch"
        to = "hand/right/index_flex"
        retarget_config = { ema_alpha = 0.1 }
    ''')
    assert any("whole-hand bind only" in p for p in problems)


def test_build_retarget_refuses_another_model_and_defaults_without_config(tmp_path):
    b = _load(tmp_path, '''
        [[bind]]
        from = "glove"
        to = "hand/right"
        retarget = "leadshine/dh116s"
        retarget_config = { ema_alpha = 0.1 }
    ''')
    with pytest.raises(ValueError, match="built for leadshine/dh116s"):
        b.binds[0].build_retarget("robotera/xhand1")
    plain = _load(tmp_path, '''
        [[bind]]
        from = "glove"
        to = "hand/right"
    ''').binds[0]
    assert plain.retarget_config is None
    assert len(plain.build_retarget("robotera/xhand1_lite").JOINTS) == 6


def test_build_retarget_config_directly():
    cfg, problems = build_retarget_config("leadshine/dh116s", {"w_dip": 0.2})
    assert problems == [] and cfg.w_dip == 0.2
    cfg, problems = build_retarget_config("nobody/hand", {})
    assert cfg is None and problems
