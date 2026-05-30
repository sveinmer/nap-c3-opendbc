#!/usr/bin/env python3
"""Verify BRAKE_FACTOR speed-dep stretch + clamp-atferd.

2026-05-20: BRAKE_FACTOR_V revertert til [1.0, 1.0] (disabled) etter drive-
batch-analyse viste at Tinkla's stretch (×1.15 → ×1.45) destabiliserer vår
VDAS-FF-stack (mangler D-term/anchor/hysteresis som Tinkla's PCC-PID har).
Behold testene som regresjons-vakt:
  - speed_dep er identitet (faktor 1.0 på alle speeds)
  - brake_factor user-toggle anvendes fortsatt kun på negativ accel
  - Output clippes til [REGEN_MAX, ACCEL_MAX]
"""
import unittest
import numpy as np

from opendbc.car.tesla.preap.nap_conf import (
  BRAKE_FACTOR_BP, BRAKE_FACTOR_V, ACCEL_MAX, REGEN_MAX,
)


def _stretch(accel_request: float, v_ego: float, brake_factor: float = 1.0) -> float:
  """Speil av carcontroller.py-blokken — endre begge sammen."""
  if accel_request < 0:
    accel_request *= brake_factor
  speed_dep = float(np.interp(v_ego, BRAKE_FACTOR_BP, BRAKE_FACTOR_V))
  return float(np.clip(accel_request * speed_dep, REGEN_MAX, ACCEL_MAX))


class TestBrakeFactorStretch(unittest.TestCase):
  def test_constants_disabled(self):
    """BRAKE_FACTOR_V skal være [1.0, 1.0] (disabled) inntil Tinkla PCC-PID portes."""
    self.assertEqual(BRAKE_FACTOR_BP, [18.0, 28.0])
    self.assertEqual(BRAKE_FACTOR_V, [1.0, 1.0])

  def test_low_speed_identity(self):
    self.assertAlmostEqual(_stretch(1.0, v_ego=0.0), 1.0, places=4)
    self.assertAlmostEqual(_stretch(1.0, v_ego=10.0), 1.0, places=4)
    self.assertAlmostEqual(_stretch(1.0, v_ego=18.0), 1.0, places=4)

  def test_high_speed_identity(self):
    self.assertAlmostEqual(_stretch(1.0, v_ego=28.0), 1.0, places=4)
    self.assertAlmostEqual(_stretch(1.0, v_ego=40.0), 1.0, places=4)

  def test_mid_speed_identity(self):
    self.assertAlmostEqual(_stretch(1.0, v_ego=23.0), 1.0, places=4)

  def test_brake_factor_applies_only_to_negative(self):
    # Positiv accel: brake_factor ignoreres
    self.assertAlmostEqual(_stretch(1.0, v_ego=18.0, brake_factor=2.0), 1.0, places=4)
    # Negativ accel: brake_factor anvendes (speed_dep = 1.0 så ingen ekstra stretch)
    # -1.0 * 2.0 * 1.0 = -2.0 → clip til REGEN_MAX=-1.5
    self.assertAlmostEqual(_stretch(-1.0, v_ego=18.0, brake_factor=2.0), REGEN_MAX, places=4)

  def test_clipped_to_accel_max(self):
    # Med speed_dep=1.0: 2.5 * 1.0 = 2.5 → ACCEL_MAX (no-op clip)
    self.assertAlmostEqual(_stretch(ACCEL_MAX, v_ego=28.0), ACCEL_MAX, places=4)

  def test_clipped_to_regen_max(self):
    # Med speed_dep=1.0: -1.5 * 1.0 = -1.5 → REGEN_MAX (no-op clip)
    self.assertAlmostEqual(_stretch(REGEN_MAX, v_ego=28.0), REGEN_MAX, places=4)


if __name__ == "__main__":
  unittest.main()
