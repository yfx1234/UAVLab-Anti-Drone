import math

import pytest

from cooperative_intercept_fake_sim.visualization import ALTITUDE_LABEL, FORMAL_FIGURES
from cooperative_intercept_fake_sim.wingman_offboard import (
    formation_layer_z, wrap_pi)


def test_formation_layer_tracks_p1_inside_safe_vertical_band():
    assert formation_layer_z(-7.0, -30.0, 15.0, 25.0) == pytest.approx(-7.0)


def test_formation_layer_clamps_to_target_below_band_in_ned():
    assert formation_layer_z(-20.0, -30.0, 15.0, 25.0) == pytest.approx(-15.0)
    assert formation_layer_z(2.0, -30.0, 15.0, 25.0) == pytest.approx(-5.0)


def test_bearing_offsets_wrap_without_discontinuity():
    assert wrap_pi(math.pi + 0.1) == pytest.approx(-math.pi + 0.1)
    assert wrap_pi(-math.pi - 0.1) == pytest.approx(math.pi - 0.1)


def test_formal_figure_contract_and_altitude_label():
    assert len(FORMAL_FIGURES) == 8
    assert [name[:2] for name in FORMAL_FIGURES] == [f'{i:02d}' for i in range(1, 9)]
    assert ALTITUDE_LABEL == '高度 h = -z_NED / m'
