import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.tesla.preap.nap_conf import (
  nap_conf, PEDAL_DI_MIN, PEDAL_DI_ZERO,
  ACCEL_MAX, REGEN_MAX,
  BRAKE_FACTOR_BP, BRAKE_FACTOR_V,
)
from opendbc.car.tesla.pedal.controller import get_zero_torque
from opendbc.car.tesla.preap.interface import get_preap_accel_limits
from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
from opendbc.car.tesla.preap.virtual_das import VirtualDAS
from opendbc.car.tesla.preap.debug_log import nap_debug_log
from opendbc.car.tesla.values import CANBUS, CruiseButtons
from opendbc.car.carlog import carlog


def init_preap_can(dbc_names, packers):
  packers[CANBUS.autopilot_party] = CANPacker(dbc_names[Bus.party])
  tesla_can = TeslaCANPreAP(packers)
  tesla_can.pedal_can_bus = nap_conf.pedal_can_bus
  return tesla_can


# Grace period after engage: clamp accel >= 0 to prevent PID reset regen spike
ENGAGE_GRACE_FRAMES = 50  # 0.5s at 100Hz


class PreAPLongController:
  """Pedal-mode longitudinal: pedal_command, zero-torque, gas passthrough.

  Stalk-CC spoofs (CANCEL / SET_ACCEL) live in StockCCSpoofer and run
  independently of openpilotLongitudinalControl. We communicate with the
  spoofer through CarState flags only.
  """

  def __init__(self):
    self.prev_pedal_di = 0.0
    self.prev_enable_long_control = False
    self.prev_requested_long = False
    self.prev_preap_long_active = False
    self.preap_long_engage_frame = -1000000
    # Snapshot of a_max-at-engage-speed taken on each engage rising edge.
    # Used as the deterministic ceiling for the grace-period ramp so an MPC
    # outlier on frame 1 cannot propagate into the pedal. See T2B-2 / Item 9.
    self.engage_a_max = 0.0
    # VirtualDAS — cascaded controller (jerk limiter + FF table + inner PID
    # + grade comp). Replaces open-loop compute_pedal_command. dt=0.02 since
    # preap_long.update is gated on frame % 2 == 0 inside the 100Hz tick.
    self.vdas = VirtualDAS(dt=0.02)

  def update(self, CC, CS, frame, tesla_can, can_bus_party):
    can_sends = []
    actuators = CC.actuators

    requested_long = CS.cruiseEnabled and CS.enableLongControl
    long_active = requested_long and CC.longActive
    use_pedal = nap_conf.use_pedal
    pedal_factor = float(nap_conf.pedal_factor)
    pedal_transform_valid = np.isfinite(pedal_factor) and abs(pedal_factor) > 1e-6
    pedal_long_allowed = use_pedal and pedal_transform_valid

    # For Pre-AP pedal, we use requested_long as our primary gate instead
    # of long_active. long_active is False during gas press (gasOverride
    # sets OVERRIDE_LONGITUDINAL), but we still need to track state and
    # send pedal commands for smooth transitions.
    pedal_engaged = requested_long and pedal_long_allowed

    # --- Engage transition ---
    requested_long_rising = (not self.prev_requested_long) and requested_long
    if requested_long_rising:
      self.preap_long_engage_frame = frame
      zero_torque_di = get_zero_torque().get(CS.out.vEgo)
      self.prev_pedal_di = max(CS.pedal_interceptor_value, zero_torque_di)
      _, self.engage_a_max = get_preap_accel_limits(CS.out.vEgo)
      self.vdas.reset(a_init=0.0, pedal_di_init=self.prev_pedal_di)

    engage_elapsed_frames = frame - self.preap_long_engage_frame
    in_engage_grace = engage_elapsed_frames < ENGAGE_GRACE_FRAMES

    # --- Stock CC cancel triggers from pedal mode ---
    # Engage / disengage / button-press in pedal mode all need to drop stock
    # CC if it's running. Publish the request via CarState; StockCCSpoofer
    # consumes it and TXes the CANCEL frame.
    if pedal_long_allowed:
      pedal_button_press = (CS.cruise_buttons != CS.prev_cruise_buttons
                            and CS.cruise_buttons != CruiseButtons.IDLE)
      pedal_long_falling = self.prev_requested_long and not requested_long
      if requested_long_rising or pedal_long_falling or pedal_button_press:
        CS.preap_cc_cancel_needed = True

    self.prev_requested_long = requested_long
    pedal_responding = not CS.pedal_timeout

    if frame % 2 == 0:
      self.prev_enable_long_control = CS.enableLongControl

      # Update zero-torque learning
      if use_pedal:
        get_zero_torque().update(CS.pedal.torque_level, self.prev_pedal_di, CS.out.vEgo)

      if pedal_engaged:
        try:
          if CS.out.gasPressed:
            # Gas pressed: driver is controlling. Pass through their foot
            # directly (enable=0) but track their position for smooth resume.
            self.prev_pedal_di = max(CS.pedal_interceptor_value, PEDAL_DI_ZERO)
            can_sends.append(tesla_can.create_pedal_command(0, enable=0))

          elif long_active:
            # Normal operation: planner is active, send computed pedal.
            # Tinkla unified LONG_module brake-factor pattern (LONG_module.py:69-76):
            # user-tunable brake_factor on negative accel, then speed-dep stretch.
            accel_request = float(actuators.accel)
            if accel_request < 0:
              accel_request *= float(nap_conf.brake_factor)
            speed_dep = float(np.interp(CS.out.vEgo, BRAKE_FACTOR_BP, BRAKE_FACTOR_V))
            accel_request = float(np.clip(accel_request * speed_dep, REGEN_MAX, ACCEL_MAX))
            if in_engage_grace:
              # Cap at grace_progress * engage_a_max so the ceiling is the
              # tuned accel-profile envelope, not the live MPC request. Keeps
              # MPC outliers (state corruption, solver failure) on engage from
              # propagating into the pedal. Origin: drive d0cdc986c5d023f5.
              grace_progress = engage_elapsed_frames / ENGAGE_GRACE_FRAMES
              accel_cap = grace_progress * self.engage_a_max
              accel_request = max(0.0, min(accel_request, accel_cap))

            # Fase B (2026-05-23) — pass pedal_speed_kph (Tesla CC setpoint
            # tracked in engagement.py) to VDAS for conditional hysteresis
            # ±HYSTERESIS_BAND_KPH anchor. Tinkla PCC_module.py:334-335
            # paritet. None-safe when CC not engaged yet.
            self.prev_pedal_di = self.vdas.update(
              accel_request, CS.out.vEgo, self.prev_pedal_di,
              a_ego=CS.out.aEgo, freeze_integrator=in_engage_grace,
              orientation_ned=list(CC.orientationNED),
              target_speed_kph=getattr(CS, "pedal_speed_kph", None))
            pedal_cmd = nap_conf.di_to_pedal(self.prev_pedal_di)
            can_sends.append(tesla_can.create_pedal_command(pedal_cmd, enable=1))

            # NAPDebugLog — per-tick state til /data/nap_debug.jsonl. No-op
            # når disabled. Inkluderer MPC-input pre/post BRAKE_FACTOR-stretch
            # samt alle VDAS-interne states (ff_di, pid_corr, integral...).
            debug_rec = {
              'frame': frame,
              'mpc_accel': float(actuators.accel),
              'accel_request_post_stretch': float(accel_request),
              'speed_dep': float(speed_dep),
              'pedal_cmd': float(pedal_cmd),
              'in_engage_grace': bool(in_engage_grace),
              'long_active': True,
            }
            debug_rec.update(self.vdas.last_debug)
            nap_debug_log.write(debug_rec)

            if self.prev_pedal_di <= 0.95 * PEDAL_DI_MIN and not in_engage_grace:
              CS.pccEvent = "pedalMaxRegen"

          else:
            # pedal_engaged but not long_active and not gasPressed:
            # IPC lag or transitioning. Hold at zero-torque.
            zero_torque_di = get_zero_torque().get(CS.out.vEgo)
            hold_pedal = nap_conf.di_to_pedal(zero_torque_di)
            can_sends.append(tesla_can.create_pedal_command(hold_pedal, enable=1))
            self.prev_pedal_di = zero_torque_di

        except Exception:
          carlog.exception("Pre-AP pedal command failed; sending disabled")
          idle_pedal = nap_conf.di_to_pedal(PEDAL_DI_ZERO)
          can_sends.append(tesla_can.create_pedal_command(idle_pedal, enable=0))
          self.prev_pedal_di = 0.0

      elif use_pedal and not pedal_transform_valid:
        idle_pedal = nap_conf.di_to_pedal(PEDAL_DI_ZERO)
        can_sends.append(tesla_can.create_pedal_command(idle_pedal, enable=0))
        self.prev_pedal_di = 0.0

      else:
        if use_pedal:
          idle_pedal = nap_conf.di_to_pedal(PEDAL_DI_ZERO)
          if pedal_responding:
            can_sends.append(tesla_can.create_pedal_command(idle_pedal, enable=0))
          elif frame % 100 == 0:
            can_sends.append(tesla_can.create_pedal_command(idle_pedal, enable=0))
        self.prev_pedal_di = 0.0

    self.prev_preap_long_active = long_active
    return can_sends

  def send_cancel(self, CS, tesla_can):
    if not CS.pedal_timeout:
      idle_pedal = nap_conf.di_to_pedal(PEDAL_DI_ZERO)
      return [tesla_can.create_pedal_command(idle_pedal, enable=0)]
    return []
