#!/usr/bin/env python3
"""Tinkla unified conditional-hysteresis pattern i compute_pedal_command.

Verifiserer at hysterese kun anvendes near set-speed-anchor når
``conditional_hysteresis=True`` (Tinkla PCC_module.py:334-335-paritet),
og at default-pathen bevarer eksisterende always-on-atferd.

Test-design: bruk accel under ACCEL_DEADBAND (0.15 m/s²) for å trigge
deadband-zero-mappingen. Resulterende pedal_di = zero_torque_di (=0 etter
reset). Med prev_pedal_di mellom 0 og HYST_GAP, blir delta < HYST_GAP og
hysterese-modusen synlig i output.
"""
import unittest

from opendbc.car.tesla.pedal.controller import compute_pedal_command, get_zero_torque


class TestConditionalHysteresis(unittest.TestCase):

  def setUp(self):
    # Reset zero-torque learning state so pedal_di base = 0 for deadband-input
    zt = get_zero_torque()
    zt.value = 0.0
    zt._best_torque = -30.0

  def _call(self, accel, v_ego, prev_di, target_kph, conditional):
    return compute_pedal_command(
      accel_request=accel, v_ego=v_ego, prev_pedal_di=prev_di,
      target_speed_kph=target_kph, conditional_hysteresis=conditional)

  def test_default_hysteresis_always_on(self):
    # accel under deadband → pedal_di_raw=0; prev=0.5 → delta=0.5 < HYST_GAP=1.0
    # Default mode freezes regardless of setpoint distance.
    _, di = self._call(0.05, v_ego=10.0, prev_di=0.5, target_kph=999.0, conditional=False)
    self.assertAlmostEqual(di, 0.5, places=4)

  def test_conditional_active_near_setpoint(self):
    # conditional=True + v_ego_kph (36) ≈ target (36) → near band → hysterese ON.
    _, di = self._call(0.05, v_ego=10.0, prev_di=0.5, target_kph=36.0, conditional=True)
    self.assertAlmostEqual(di, 0.5, places=4,
                           msg="Near setpoint conditional should freeze to prev")

  def test_conditional_bypassed_far_from_setpoint(self):
    # conditional=True + |36 - 50| = 14 km/h > 0.8 → hysterese OFF → output = 0.
    _, di = self._call(0.05, v_ego=10.0, prev_di=0.5, target_kph=50.0, conditional=True)
    self.assertAlmostEqual(di, 0.0, places=4,
                           msg="Far from setpoint conditional should pass through")

  def test_conditional_bypassed_low_speed(self):
    # conditional=True + v_ego (3 m/s) < 5 → low-speed bypass → hysterese OFF.
    # NB: at v_ego < 5 mph, zero_torque.get() returns PEDAL_DI_ZERO=0 also.
    _, di = self._call(0.05, v_ego=3.0, prev_di=0.5, target_kph=10.8, conditional=True)
    self.assertAlmostEqual(di, 0.0, places=4,
                           msg="Low-speed bypass should disable conditional hysteresis")

  def test_default_mode_ignores_target_kph(self):
    # conditional=False: target_speed_kph value irrelevant.
    _, di_near = self._call(0.05, v_ego=10.0, prev_di=0.5, target_kph=36.0, conditional=False)
    _, di_far = self._call(0.05, v_ego=10.0, prev_di=0.5, target_kph=999.0, conditional=False)
    self.assertAlmostEqual(di_near, di_far, places=4)
    self.assertAlmostEqual(di_near, 0.5, places=4)

  def test_conditional_outside_window_above(self):
    # |36 - 37| = 1.0 > 0.8 → outside near-band → hysterese OFF.
    _, di = self._call(0.05, v_ego=10.0, prev_di=0.5, target_kph=37.0, conditional=True)
    self.assertAlmostEqual(di, 0.0, places=4)

  def test_conditional_inside_window_below(self):
    # |36 - 35.3| = 0.7 < 0.8 → inside near-band → hysterese ON.
    _, di = self._call(0.05, v_ego=10.0, prev_di=0.5, target_kph=35.3, conditional=True)
    self.assertAlmostEqual(di, 0.5, places=4)


if __name__ == "__main__":
  unittest.main()
