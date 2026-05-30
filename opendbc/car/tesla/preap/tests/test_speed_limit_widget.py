#!/usr/bin/env python3
"""Tests for Tesla IC native road-sign widget (DAS_fusedSpeedLimit pipeline).

Source: UI_gpsVehicleSpeed.UI_mppSpeedLimit (msg 0x2F8 / 760, chassis bus 0).
Sink: 0x399 DAS_status.DAS_fusedSpeedLimit (consumed by Tesla IC, not Buddy).
Fallback: NAPRoadSignFallbackKph param when Tesla DI reports 0 (SNA / no GPS).

Pattern ported from Tinkla unified: selfdrive/car/tesla/carstate.py:362.
"""
import unittest
from unittest.mock import MagicMock

from opendbc.can import CANPacker, CANParser
from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP, DAS_STATUS_MSG_ID
from opendbc.car.tesla.values import CANBUS


def _make_packers():
  return {CANBUS.party: CANPacker("tesla_preap")}


def _decode_das_fused_speed_limit(frame_bytes: bytes) -> int:
  """Decode DAS_fusedSpeedLimit from 0x399 DAS_status frame.

  DBC: SG_ DAS_fusedSpeedLimit : 8|5@1+ (5,0) → bits 8-12, scale 5, units kph/mph.
  Raw 5-bit field is at byte1[0:5].
  """
  raw = frame_bytes[1] & 0x1F
  return raw * 5


class TestDasStatusFusedSpeedLimitEncoding(unittest.TestCase):
  """DAS_fusedSpeedLimit byte-level encoding into 0x399 — covers full range."""

  def setUp(self):
    self.tc = TeslaCANPreAP(_make_packers())

  def _build(self, limit_uom):
    return self.tc.create_das_status(
      DAS_op_status=2, DAS_collision_warning=0, DAS_ldwStatus=0,
      DAS_hands_on_state=2, DAS_alca_state=1,
      blindSpotLeft=False, blindSpotRight=False,
      DAS_speed_limit_kph=limit_uom, DAS_fleetSpeedState=0,
      bus=0, counter=1,
    )

  def test_zero_no_sign(self):
    """limit=0 → field=0 → Tesla IC hides road-sign widget."""
    msg = self._build(0)
    self.assertEqual(msg[0], DAS_STATUS_MSG_ID)
    self.assertEqual(_decode_das_fused_speed_limit(msg[1]), 0)

  def test_common_norwegian_limits(self):
    """Vanlige norske skiltverdier: 30/50/60/70/80/90/100/110."""
    for limit in (30, 50, 60, 70, 80, 90, 100, 110):
      with self.subTest(limit=limit):
        decoded = _decode_das_fused_speed_limit(self._build(limit)[1])
        self.assertEqual(decoded, limit,
                         f"DAS_fusedSpeedLimit roundtrip failed for {limit}")

  def test_max_supported_limit(self):
    """5-bit field max = 31*5 = 155 kph (DBC range upper bound)."""
    self.assertEqual(_decode_das_fused_speed_limit(self._build(155)[1]), 155)


class TestNapConfRoadSignFallback(unittest.TestCase):
  """nap_conf.road_sign_fallback_kph getter/setter + clipping."""

  def test_default_is_zero(self):
    """Default fallback = 0 (no sign when Tesla DI reports SNA)."""
    from opendbc.car.tesla.preap.nap_conf import NAPConf
    cfg = NAPConf.__new__(NAPConf)
    cfg._cache = {}
    # When openpilot Params unavailable, falls back to JSON cache (empty → default 0).
    self.assertEqual(int(cfg._get('road_sign_fallback_kph', 0)), 0)

  def test_clip_below_zero(self):
    """Negative values clip to 0 (display-only field cannot be negative)."""
    from opendbc.car.tesla.preap.nap_conf import nap_conf
    nap_conf.road_sign_fallback_kph = -10
    self.assertGreaterEqual(nap_conf.road_sign_fallback_kph, 0)

  def test_clip_above_150(self):
    """Values above 150 kph clip to 150 (DBC range limit)."""
    from opendbc.car.tesla.preap.nap_conf import nap_conf
    nap_conf.road_sign_fallback_kph = 999
    self.assertLessEqual(nap_conf.road_sign_fallback_kph, 150)

  def test_setter_roundtrip(self):
    """Valid value (80) round-trips through setter/getter."""
    from opendbc.car.tesla.preap.nap_conf import nap_conf
    nap_conf.road_sign_fallback_kph = 80
    self.assertEqual(nap_conf.road_sign_fallback_kph, 80)


class TestUiGpsVehicleSpeedParser(unittest.TestCase):
  """CANParser registration + UI_mppSpeedLimit decoding from packed frame.

  Verifies that the field declared in tesla_preap.dbc is actually readable
  via CANParser ("UI_gpsVehicleSpeed", math.nan). Catches DBC drift.
  """

  def test_parser_decodes_uom_value(self):
    import math
    packer = CANPacker("tesla_preap")
    parser = CANParser("tesla_preap", [("UI_gpsVehicleSpeed", math.nan)], 0)

    # Build a 0x2F8 frame with UI_mppSpeedLimit=16 (raw) → 80 UoM (scale 5)
    msg = packer.make_can_msg("UI_gpsVehicleSpeed", 0, {
      "UI_mppSpeedLimit": 80,
      "UI_mapSpeedLimitUnits": 1,  # 1 = KPH
    })
    parser.update([0, [msg]])

    self.assertEqual(int(parser.vl["UI_gpsVehicleSpeed"]["UI_mppSpeedLimit"]), 80)
    self.assertEqual(int(parser.vl["UI_gpsVehicleSpeed"]["UI_mapSpeedLimitUnits"]), 1)

  def test_parser_handles_sna_zero(self):
    """SNA / no-GPS / parking → raw=0 → UoM=0 → carstate falls back to NAP param."""
    import math
    packer = CANPacker("tesla_preap")
    parser = CANParser("tesla_preap", [("UI_gpsVehicleSpeed", math.nan)], 0)

    msg = packer.make_can_msg("UI_gpsVehicleSpeed", 0, {
      "UI_mppSpeedLimit": 0,
      "UI_mapSpeedLimitUnits": 1,
    })
    parser.update([0, [msg]])

    self.assertEqual(float(parser.vl["UI_gpsVehicleSpeed"]["UI_mppSpeedLimit"]), 0.0)


class TestHudModuleNoHardcoded80(unittest.TestCase):
  """Regression guard: hud_module must NOT silently overwrite CS.DAS_fusedSpeedLimit
  to 80 anymore (V45 diagnostic). When carstate sets it to 0 (no GPS + no fallback),
  the field must stay 0 so Tesla IC hides the widget — not show a phantom 80."""

  def test_hardcode_removed(self):
    """Scan hud_module.py source for the diagnostic hardcode block."""
    from pathlib import Path
    src = Path(__file__).resolve().parents[1] / "hud_module.py"
    text = src.read_text()
    self.assertNotIn("CS.DAS_fusedSpeedLimit = 80", text,
                     "Diagnostic hardcode still present — V45 fix-5 was supposed "
                     "to be removed once GPS-source wired up. See "
                     "project_buddy_skilt_widget_2026_05_16 memory.")


if __name__ == "__main__":
  unittest.main()
