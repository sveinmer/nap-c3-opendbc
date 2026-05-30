#!/usr/bin/env python3
"""Tests for Pre-AP engagement FSM, specifically the brake-to-disengage path.

The panda safety layer hardcodes brake_pressed=false for Pre-AP (tesla_preap.h:340).
Brake-to-disengage is handled here in the Python layer via the PreAPEngagement FSM.
This test verifies that the brake properly drops longitudinal while keeping lateral.
"""
import unittest

from opendbc.car.tesla.preap.engagement import PreAPEngagement


class TestPreAPBrakeDisengage(unittest.TestCase):
  """Verify the brake-to-disengage path that the panda safety tests reference."""

  def _make_engagement(self, double_pull=False):
    return PreAPEngagement(double_pull_enabled=double_pull, double_pull_window_ms=750)

  def _engage_single_pull(self, eng, use_pedal=True):
    """Simulate a single-pull engage with pedal mode."""
    eng.process_buttons(
      cruise_buttons=2, prev_cruise_buttons=0,  # MAIN rising edge
      curr_time_ms=1000, v_ego=10.0, speed_units="KPH",
      use_pedal=use_pedal, pedal_long_allowed=use_pedal,
      long_control_allowed=True, real_brake_pressed=False)

  def test_brake_drops_longitudinal_keeps_lateral(self):
    # This is the core invariant: brake drops pedal but keeps steering.
    eng = self._make_engagement()
    self._engage_single_pull(eng, use_pedal=True)
    self.assertTrue(eng.cruiseEnabled)
    self.assertTrue(eng.enableLongControl)

    # Brake rising edge
    eng.process_buttons(
      cruise_buttons=0, prev_cruise_buttons=0,
      curr_time_ms=2000, v_ego=10.0, speed_units="KPH",
      use_pedal=True, pedal_long_allowed=True,
      long_control_allowed=True, real_brake_pressed=True)

    # Longitudinal dropped, lateral stays
    self.assertTrue(eng.cruiseEnabled, "Lateral should stay active after brake")
    self.assertFalse(eng.enableLongControl, "Longitudinal should drop on brake")
    self.assertTrue(eng.enableJustCC, "Should transition to CC-only mode")

  def test_brake_no_effect_without_pedal(self):
    # In non-pedal mode (stock CC only), brake doesn't trigger any action
    # in the engagement FSM — stock CC handles its own brake disengage.
    eng = self._make_engagement()
    self._engage_single_pull(eng, use_pedal=False)
    self.assertTrue(eng.cruiseEnabled)

    eng.process_buttons(
      cruise_buttons=0, prev_cruise_buttons=0,
      curr_time_ms=2000, v_ego=10.0, speed_units="KPH",
      use_pedal=False, pedal_long_allowed=False,
      long_control_allowed=True, real_brake_pressed=True)

    # No change — stock CC handles brake disengage independently
    self.assertTrue(eng.cruiseEnabled)

  def test_brake_only_on_rising_edge(self):
    # Holding brake should not repeatedly disengage — only rising edge matters.
    eng = self._make_engagement()
    self._engage_single_pull(eng, use_pedal=True)

    # Brake held from previous cycle (not a rising edge)
    eng.preap_brake_pressed_prev = True
    eng.process_buttons(
      cruise_buttons=0, prev_cruise_buttons=0,
      curr_time_ms=2000, v_ego=10.0, speed_units="KPH",
      use_pedal=True, pedal_long_allowed=True,
      long_control_allowed=True, real_brake_pressed=True)

    # No disengage — brake was already pressed
    self.assertTrue(eng.enableLongControl)

  def test_brake_disengage_then_reengage(self):
    # After brake drops longitudinal, a stalk pull should re-engage everything.
    eng = self._make_engagement()
    self._engage_single_pull(eng, use_pedal=True)

    # Brake drops longitudinal
    eng.process_buttons(
      cruise_buttons=0, prev_cruise_buttons=0,
      curr_time_ms=2000, v_ego=10.0, speed_units="KPH",
      use_pedal=True, pedal_long_allowed=True,
      long_control_allowed=True, real_brake_pressed=True)
    self.assertFalse(eng.enableLongControl)
    self.assertTrue(eng.cruiseEnabled)

    # Release brake
    eng.process_buttons(
      cruise_buttons=0, prev_cruise_buttons=0,
      curr_time_ms=3000, v_ego=10.0, speed_units="KPH",
      use_pedal=True, pedal_long_allowed=True,
      long_control_allowed=True, real_brake_pressed=False)

    # Stalk pull re-engages
    eng.process_buttons(
      cruise_buttons=2, prev_cruise_buttons=0,
      curr_time_ms=4000, v_ego=10.0, speed_units="KPH",
      use_pedal=True, pedal_long_allowed=True,
      long_control_allowed=True, real_brake_pressed=False)
    self.assertTrue(eng.cruiseEnabled)
    self.assertTrue(eng.enableLongControl)


if __name__ == "__main__":
  unittest.main()
