"""Tester for tune-bridge (Fase 3d): Params-watcher + VDAS smooth ramp.

Selvstendige — krever ikke Params-runtime. Bruker mock for å simulere
NAPTuneOverrides.read_current() med ulike states.
"""
from __future__ import annotations

from unittest.mock import patch

import pytest

from opendbc.car.tesla.preap.nap_conf import (
  NAPTuneOverrides, TUNE_RANGES,
  _parse_float_list, _clamp_list, _clamp_scalar,
)
from opendbc.car.tesla.preap.nap_params import NAPParamKeys
from opendbc.car.tesla.preap.virtual_das import (
  VirtualDAS, TUNE_REFRESH_N, TUNE_RAMP_S,
)
from opendbc.car.tesla.preap.constants import (
  VDAS_INNER_KI_V, VDAS_INNER_KP_V,
)


# ----- _parse_float_list -----------------------------------------------------


class TestParseFloatList:
  def test_json_list(self):
    assert _parse_float_list("[0.25, 0.22, 0.18]") == [0.25, 0.22, 0.18]

  def test_comma_form(self):
    assert _parse_float_list("0.3,0.2,0.15") == [0.3, 0.2, 0.15]

  def test_with_spaces(self):
    assert _parse_float_list(" 0.3 , 0.2 , 0.15 ") == [0.3, 0.2, 0.15]

  def test_empty_returns_none(self):
    assert _parse_float_list("") is None
    assert _parse_float_list(None) is None

  def test_garbage_returns_none(self):
    assert _parse_float_list("garbage") is None
    assert _parse_float_list("{not a list}") is None

  def test_single_value(self):
    assert _parse_float_list("0.5") == [0.5]


# ----- _clamp helpers --------------------------------------------------------


class TestClampHelpers:
  def test_clamp_list_within_range(self):
    out, cl = _clamp_list([0.1, 0.2, 0.3], 0.0, 0.5)
    assert out == [0.1, 0.2, 0.3]
    assert cl is False

  def test_clamp_list_out_of_range(self):
    out, cl = _clamp_list([-0.1, 0.6, 0.3], 0.0, 0.5)
    assert out == [0.0, 0.5, 0.3]
    assert cl is True

  def test_clamp_list_none_input(self):
    out, cl = _clamp_list(None, 0.0, 1.0)
    assert out is None
    assert cl is False

  def test_clamp_scalar(self):
    assert _clamp_scalar(0.5, 0.0, 1.0) == (0.5, False)
    assert _clamp_scalar(1.5, 0.0, 1.0) == (1.0, True)
    assert _clamp_scalar(-0.5, 0.0, 1.0) == (0.0, True)
    assert _clamp_scalar(None, 0.0, 1.0) == (None, False)


# ----- NAPTuneOverrides (uten Params backend) -------------------------------


class TestNAPTuneOverridesDisabled:
  def test_default_state_is_disabled(self):
    ov = NAPTuneOverrides()
    assert not ov.tune_enabled
    assert ov.pedal_ki_v is None
    assert ov.pedal_kp_v is None
    assert ov.pedal_kd_v is None
    assert ov.pedal_long_ki_v is None
    assert ov.integral_leak is None
    assert ov.brake_factor is None
    assert not ov.any_active()

  def test_summary_disabled(self):
    ov = NAPTuneOverrides()
    assert "disabled" in ov.active_summary()

  def test_read_current_without_params_disabled(self):
    """Når openpilot Params er utilgjengelig (dev-box), read_current returner
    en disabled-instans uten å crashe."""
    ov = NAPTuneOverrides.read_current()
    assert isinstance(ov, NAPTuneOverrides)
    assert not ov.any_active()


# ----- NAPTuneOverrides med mockede Params ----------------------------------


@pytest.fixture
def mock_params():
  """Mock for openpilot Params som returnerer testbare verdier."""
  store = {}

  class _MockParams:
    def get_bool(self, key):
      return bool(store.get(key, False))

    def get(self, key, return_default=False):
      return store.get(key)

    def put(self, key, val):
      store[key] = val

    def put_bool_nonblocking(self, key, val):
      store[key] = bool(val)

  return store, _MockParams()


class TestNAPTuneOverridesEnabled:
  def test_enable_with_no_overrides_uses_base(self, mock_params):
    store, mock = mock_params
    store[NAPParamKeys.TUNE_ENABLE] = True
    with patch("opendbc.car.tesla.preap.nap_conf._params", mock), \
         patch("opendbc.car.tesla.preap.nap_conf._PARAMS_AVAILABLE", True):
      ov = NAPTuneOverrides.read_current()
    assert ov.tune_enabled is True
    assert ov.pedal_ki_v is None   # ingen verdi satt
    assert not ov.any_active()

  def test_enable_with_valid_ki(self, mock_params):
    store, mock = mock_params
    store[NAPParamKeys.TUNE_ENABLE] = True
    store[NAPParamKeys.TUNE_PEDAL_KI_V] = "[0.25, 0.22, 0.18]"
    with patch("opendbc.car.tesla.preap.nap_conf._params", mock), \
         patch("opendbc.car.tesla.preap.nap_conf._PARAMS_AVAILABLE", True):
      ov = NAPTuneOverrides.read_current()
    assert ov.tune_enabled is True
    assert ov.pedal_ki_v == [0.25, 0.22, 0.18]
    assert ov.any_active() is True
    assert ov.last_warnings == []

  def test_ki_out_of_range_clamped_with_warning(self, mock_params):
    store, mock = mock_params
    store[NAPParamKeys.TUNE_ENABLE] = True
    store[NAPParamKeys.TUNE_PEDAL_KI_V] = "[0.8, 0.22, -0.1]"   # 0.8 > 0.5, -0.1 < 0
    with patch("opendbc.car.tesla.preap.nap_conf._params", mock), \
         patch("opendbc.car.tesla.preap.nap_conf._PARAMS_AVAILABLE", True):
      ov = NAPTuneOverrides.read_current()
    assert ov.pedal_ki_v == [0.5, 0.22, 0.0]
    assert any("clamped" in w for w in ov.last_warnings)

  def test_garbage_ki_falls_back_to_none(self, mock_params):
    store, mock = mock_params
    store[NAPParamKeys.TUNE_ENABLE] = True
    store[NAPParamKeys.TUNE_PEDAL_KI_V] = "totally not a list"
    with patch("opendbc.car.tesla.preap.nap_conf._params", mock), \
         patch("opendbc.car.tesla.preap.nap_conf._PARAMS_AVAILABLE", True):
      ov = NAPTuneOverrides.read_current()
    assert ov.pedal_ki_v is None

  def test_summary_active(self, mock_params):
    store, mock = mock_params
    store[NAPParamKeys.TUNE_ENABLE] = True
    store[NAPParamKeys.TUNE_PEDAL_KI_V] = "[0.3, 0.2, 0.15]"
    with patch("opendbc.car.tesla.preap.nap_conf._params", mock), \
         patch("opendbc.car.tesla.preap.nap_conf._PARAMS_AVAILABLE", True):
      ov = NAPTuneOverrides.read_current()
    summary = ov.active_summary()
    assert "ENABLED" in summary

  def test_enable_with_valid_kd(self, mock_params):
    store, mock = mock_params
    store[NAPParamKeys.TUNE_ENABLE] = True
    store[NAPParamKeys.TUNE_PEDAL_KD_V] = "[0.05, 0.05, 0.05]"
    with patch("opendbc.car.tesla.preap.nap_conf._params", mock), \
         patch("opendbc.car.tesla.preap.nap_conf._PARAMS_AVAILABLE", True):
      ov = NAPTuneOverrides.read_current()
    assert ov.pedal_kd_v == [0.05, 0.05, 0.05]
    assert ov.any_active() is True

  def test_kd_out_of_range_clamped(self, mock_params):
    store, mock = mock_params
    store[NAPParamKeys.TUNE_ENABLE] = True
    store[NAPParamKeys.TUNE_PEDAL_KD_V] = "[0.7, 0.05, -0.05]"   # 0.7 > 0.5, -0.05 < 0
    with patch("opendbc.car.tesla.preap.nap_conf._params", mock), \
         patch("opendbc.car.tesla.preap.nap_conf._PARAMS_AVAILABLE", True):
      ov = NAPTuneOverrides.read_current()
    assert ov.pedal_kd_v == [0.5, 0.05, 0.0]
    assert any("TunePedalKdV" in w for w in ov.last_warnings)

  def test_summary_includes_kd(self, mock_params):
    store, mock = mock_params
    store[NAPParamKeys.TUNE_ENABLE] = True
    store[NAPParamKeys.TUNE_PEDAL_KD_V] = "[0.05, 0.05, 0.05]"
    with patch("opendbc.car.tesla.preap.nap_conf._params", mock), \
         patch("opendbc.car.tesla.preap.nap_conf._PARAMS_AVAILABLE", True):
      ov = NAPTuneOverrides.read_current()
    assert "kd=" in ov.active_summary()


# ----- VirtualDAS tune-bridge integration -----------------------------------


class TestVirtualDASTuneBridge:
  def test_default_no_override_keeps_base_ki(self):
    """VDAS uten Params-override skal beholde base-koeffisientene."""
    v = VirtualDAS(dt=0.02)
    base_ki = list(VDAS_INNER_KI_V)
    base_kp = list(VDAS_INNER_KP_V)
    prev_di = 0.0
    # Spin >> TUNE_REFRESH_N ticks
    for _ in range(TUNE_REFRESH_N + 50):
      di = v.update(0.5, 20.0, prev_di, a_ego=0.1, orientation_ned=[0, 0, 0])
      prev_di = di
    # Etter mange refreshes, KI/KP skal være uendret
    assert list(v.inner_pid._k_i[1]) == base_ki
    assert list(v.inner_pid._k_p[1]) == base_kp

  def test_override_triggers_ramp_to_target(self, mock_params):
    """Med override = ramp fra base til target over TUNE_RAMP_S."""
    store, mock = mock_params
    store[NAPParamKeys.TUNE_ENABLE] = True
    store[NAPParamKeys.TUNE_PEDAL_KI_V] = "[0.1, 0.1, 0.1]"
    base_ki = list(VDAS_INNER_KI_V)
    target_ki = [0.1, 0.1, 0.1]

    with patch("opendbc.car.tesla.preap.nap_conf._params", mock), \
         patch("opendbc.car.tesla.preap.nap_conf._PARAMS_AVAILABLE", True):
      v = VirtualDAS(dt=0.02)
      prev_di = 0.0
      # Spin gjennom første refresh + half ramp + complete ramp
      ramp_ticks = int(TUNE_RAMP_S / 0.02)   # 100 ticks for 2s
      total_ticks = TUNE_REFRESH_N + ramp_ticks + 20   # buffer
      for _ in range(total_ticks):
        di = v.update(0.5, 20.0, prev_di, a_ego=0.1, orientation_ned=[0, 0, 0])
        prev_di = di

      # Etter komplett ramp: effektive KI = target
      eff_ki = list(v.inner_pid._k_i[1])
      for got, want in zip(eff_ki, target_ki, strict=True):
        assert abs(got - want) < 1e-3, f"expected {target_ki}, got {eff_ki}"

  def test_ramp_is_smooth_mid_way(self, mock_params):
    """Midt i ramp skal effektive verdier være mellom base og target."""
    store, mock = mock_params
    store[NAPParamKeys.TUNE_ENABLE] = True
    store[NAPParamKeys.TUNE_PEDAL_KI_V] = "[0.1, 0.1, 0.1]"
    base_ki = list(VDAS_INNER_KI_V)

    with patch("opendbc.car.tesla.preap.nap_conf._params", mock), \
         patch("opendbc.car.tesla.preap.nap_conf._PARAMS_AVAILABLE", True):
      v = VirtualDAS(dt=0.02)
      prev_di = 0.0
      # Først driv frem til første refresh-trigger så ramp starter
      for _ in range(TUNE_REFRESH_N + 1):
        di = v.update(0.5, 20.0, prev_di, a_ego=0.1, orientation_ned=[0, 0, 0])
        prev_di = di
      # Nå er vi noen ticks inn i ramp. Verifiser progress.
      half_ramp_ticks = int(TUNE_RAMP_S / 0.02 / 2)
      for _ in range(half_ramp_ticks):
        di = v.update(0.5, 20.0, prev_di, a_ego=0.1, orientation_ned=[0, 0, 0])
        prev_di = di
      # Midt-ramp: KI skal være ca midten av base og target
      eff_ki = list(v.inner_pid._k_i[1])
      for got, base, target in zip(eff_ki, base_ki, [0.1, 0.1, 0.1], strict=True):
        # Skal være mellom base og target (ekskl. eksakt match)
        lo, hi = min(base, target), max(base, target)
        assert lo <= got <= hi, f"mid-ramp {got} ikke mellom {base} og {target}"

  def test_disable_after_enable_ramps_back(self, mock_params):
    """Hvis TUNE_ENABLE settes til False, skal effective verdier ramp tilbake til base."""
    store, mock = mock_params
    store[NAPParamKeys.TUNE_ENABLE] = True
    store[NAPParamKeys.TUNE_PEDAL_KI_V] = "[0.1, 0.1, 0.1]"
    base_ki = list(VDAS_INNER_KI_V)

    with patch("opendbc.car.tesla.preap.nap_conf._params", mock), \
         patch("opendbc.car.tesla.preap.nap_conf._PARAMS_AVAILABLE", True):
      v = VirtualDAS(dt=0.02)
      prev_di = 0.0
      # Ramp opp til target
      ramp_ticks = int(TUNE_RAMP_S / 0.02)
      for _ in range(TUNE_REFRESH_N + ramp_ticks + 20):
        di = v.update(0.5, 20.0, prev_di, a_ego=0.1, orientation_ned=[0, 0, 0])
        prev_di = di
      # Disable tune
      store[NAPParamKeys.TUNE_ENABLE] = False
      # Spin gjennom neste refresh-trigger + ramp tilbake
      for _ in range(TUNE_REFRESH_N + ramp_ticks + 20):
        di = v.update(0.5, 20.0, prev_di, a_ego=0.1, orientation_ned=[0, 0, 0])
        prev_di = di
      # Effective skal nå være tilbake til base
      eff_ki = list(v.inner_pid._k_i[1])
      for got, want in zip(eff_ki, base_ki, strict=True):
        assert abs(got - want) < 1e-3, f"after disable expected base {base_ki}, got {eff_ki}"

  def test_wrong_length_override_ignored(self, mock_params):
    """Hvis override-lista har feil lengde, skal den ignoreres (ikke crashe)."""
    store, mock = mock_params
    store[NAPParamKeys.TUNE_ENABLE] = True
    store[NAPParamKeys.TUNE_PEDAL_KI_V] = "[0.1, 0.2]"   # 2 elementer, ikke 3
    base_ki = list(VDAS_INNER_KI_V)

    with patch("opendbc.car.tesla.preap.nap_conf._params", mock), \
         patch("opendbc.car.tesla.preap.nap_conf._PARAMS_AVAILABLE", True):
      v = VirtualDAS(dt=0.02)
      prev_di = 0.0
      for _ in range(TUNE_REFRESH_N + 50):
        di = v.update(0.5, 20.0, prev_di, a_ego=0.1, orientation_ned=[0, 0, 0])
        prev_di = di
      # KI skal være uendret (override avvist pga lengde)
      assert list(v.inner_pid._k_i[1]) == base_ki

  def test_kd_override_ramps_to_target(self, mock_params):
    """Med kd-override = ramp fra base [0,0,0] til target over TUNE_RAMP_S."""
    from opendbc.car.tesla.preap.constants import VDAS_INNER_KD_V
    store, mock = mock_params
    store[NAPParamKeys.TUNE_ENABLE] = True
    store[NAPParamKeys.TUNE_PEDAL_KD_V] = "[0.1, 0.1, 0.1]"
    target_kd = [0.1, 0.1, 0.1]

    with patch("opendbc.car.tesla.preap.nap_conf._params", mock), \
         patch("opendbc.car.tesla.preap.nap_conf._PARAMS_AVAILABLE", True):
      v = VirtualDAS(dt=0.02)
      prev_di = 0.0
      ramp_ticks = int(TUNE_RAMP_S / 0.02)
      for _ in range(TUNE_REFRESH_N + ramp_ticks + 20):
        di = v.update(0.5, 20.0, prev_di, a_ego=0.1, orientation_ned=[0, 0, 0])
        prev_di = di
      eff_kd = list(v.inner_pid._k_d[1])
      for got, want in zip(eff_kd, target_kd, strict=True):
        assert abs(got - want) < 1e-3, f"kd-ramp not converged: got {eff_kd}, want {target_kd}"

  def test_kd_disable_ramps_back_to_base(self, mock_params):
    """Disable etter aktiv kd-override → ramp tilbake til VDAS_INNER_KD_V."""
    from opendbc.car.tesla.preap.constants import VDAS_INNER_KD_V
    store, mock = mock_params
    store[NAPParamKeys.TUNE_ENABLE] = True
    store[NAPParamKeys.TUNE_PEDAL_KD_V] = "[0.1, 0.1, 0.1]"
    base_kd = list(VDAS_INNER_KD_V)

    with patch("opendbc.car.tesla.preap.nap_conf._params", mock), \
         patch("opendbc.car.tesla.preap.nap_conf._PARAMS_AVAILABLE", True):
      v = VirtualDAS(dt=0.02)
      prev_di = 0.0
      ramp_ticks = int(TUNE_RAMP_S / 0.02)
      for _ in range(TUNE_REFRESH_N + ramp_ticks + 20):
        di = v.update(0.5, 20.0, prev_di, a_ego=0.1, orientation_ned=[0, 0, 0])
        prev_di = di
      store[NAPParamKeys.TUNE_ENABLE] = False
      for _ in range(TUNE_REFRESH_N + ramp_ticks + 20):
        di = v.update(0.5, 20.0, prev_di, a_ego=0.1, orientation_ned=[0, 0, 0])
        prev_di = di
      eff_kd = list(v.inner_pid._k_d[1])
      for got, want in zip(eff_kd, base_kd, strict=True):
        assert abs(got - want) < 1e-3, f"after disable kd expected {base_kd}, got {eff_kd}"

  def test_kd_independent_of_ki_kp(self, mock_params):
    """Override på kd alene skal ikke endre ki/kp."""
    store, mock = mock_params
    store[NAPParamKeys.TUNE_ENABLE] = True
    store[NAPParamKeys.TUNE_PEDAL_KD_V] = "[0.05, 0.05, 0.05]"
    base_ki = list(VDAS_INNER_KI_V)
    base_kp = list(VDAS_INNER_KP_V)

    with patch("opendbc.car.tesla.preap.nap_conf._params", mock), \
         patch("opendbc.car.tesla.preap.nap_conf._PARAMS_AVAILABLE", True):
      v = VirtualDAS(dt=0.02)
      prev_di = 0.0
      ramp_ticks = int(TUNE_RAMP_S / 0.02)
      for _ in range(TUNE_REFRESH_N + ramp_ticks + 20):
        di = v.update(0.5, 20.0, prev_di, a_ego=0.1, orientation_ned=[0, 0, 0])
        prev_di = di
      assert list(v.inner_pid._k_i[1]) == base_ki
      assert list(v.inner_pid._k_p[1]) == base_kp
      assert list(v.inner_pid._k_d[1]) == [0.05, 0.05, 0.05]
