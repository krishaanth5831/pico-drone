import math

import pytest

from flight.policy import ACT_SCALE, OBS_SIZE, Policy, build_obs


def _identity_net():
    # 2 inputs -> 2 hidden (identity weights) -> 3 outputs
    return Policy([
        ([[1.0, 0.0], [0.0, 1.0]], [0.0, 0.0]),
        ([[1.0, 0.0], [0.0, 1.0], [1.0, 1.0]], [0.0, 0.0, 0.5]),
    ])


def test_forward_pass_matches_hand_maths():
    p = _identity_net()
    x = [0.3, -0.2]
    h = [math.tanh(0.3), math.tanh(-0.2)]
    expected = [math.tanh(h[0]), math.tanh(h[1]), math.tanh(h[0] + h[1] + 0.5)]
    out = p.act(x)
    for got, want in zip(out, expected, strict=True):
        assert got == pytest.approx(want * ACT_SCALE, abs=1e-9)


def test_output_is_bounded_by_act_scale():
    p = Policy([([[100.0] * OBS_SIZE] * 3, [0.0, 0.0, 0.0])])
    out = p.act([1.0] * OBS_SIZE)
    assert all(abs(v) <= ACT_SCALE + 1e-12 for v in out)


def test_build_obs_layout():
    obs = build_obs(30.0, -15.0, (300.0, 0.0, -150.0), 10.0, -5.0, 0.6, (0.1, 0.2, 0.3))
    assert len(obs) == OBS_SIZE
    assert obs == pytest.approx([1.0, -0.5, 1.0, 0.0, -0.5, 1.0, -0.5, 0.6, 0.1, 0.2, 0.3])


def test_macs():
    assert _identity_net().macs == 2 * 2 + 3 * 2
