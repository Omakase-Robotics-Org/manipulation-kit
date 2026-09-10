"""The hand-agnostic retarget seam: get_retarget resolves <maker>/<model>
to that hand's flexion→wire-units mapper without the consumer importing it."""
import pytest

import manipulation_kit.hands as oh


def test_get_retarget_dh116s():
    r = oh.get_retarget("leadshine/dh116s")
    out = r(lambda name: 1.0)          # full close, all channels
    assert out.tolist() == [10000] * 6


def test_get_retarget_passes_config():
    from manipulation_kit.hands.leadshine.dh116s.retarget import RetargetConfig
    cfg = RetargetConfig(invert=(True,) * 6)
    r = oh.get_retarget("leadshine/dh116s", config=cfg)
    assert r(lambda name: 1.0).tolist() == [0] * 6


@pytest.mark.parametrize("bad", ["", "leadshine", "no-slash", "un/known"])
def test_get_retarget_rejects_malformed_and_unknown(bad):
    with pytest.raises((ValueError, NotImplementedError)):
        oh.get_retarget(bad)
