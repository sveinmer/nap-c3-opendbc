from numpy import interp, clip

from opendbc.car.tesla.preap.nap_conf import (
  nap_conf,
  PEDAL_DI_MIN, PEDAL_DI_ZERO,
  PEDAL_BP, PEDAL_MAX_VALUES,
  ACCEL_MAX, REGEN_MAX, REGEN_MAX_BP, REGEN_MAX_V,
)


def get_regen_floor(v_ego: float) -> float:
  """Speed-dependent regen lower bound. Softer at low speed for comfort."""
  return float(interp(v_ego, REGEN_MAX_BP, REGEN_MAX_V))
from opendbc.car.common.conversions import Conversions as CV

# Asymmetric ramp rates: MPC already jerk-constrains output, so the accel
# ramp can be fast. Decel ramp stays slower for safety.
#
# UP reduced from 5.0 → 3.5 to limit overshoot on lead-dropout transients:
# a lost radar frame momentarily opens accel, and the previous 2:1 asymmetry
# meant the regen could never catch up before the user felt the surge.
# 3.5↑ / 2.5↓ = 1.4:1, still asymmetric but no longer dramatic.
PEDAL_RAMP_RATE_UP = 3.5    # DI/step @ 50Hz = 175 DI/s
PEDAL_RAMP_RATE_DOWN = 2.5  # DI/step @ 50Hz = 125 DI/s

# Regen deadband: small accel requests near zero map to the zero-torque
# position instead of crossing the gas/regen boundary every MPC cycle.
ACCEL_DEADBAND = 0.15  # m/s²

# Pedal hysteresis: don't change pedal output unless command moved by more
# than this from the last sent value. Kills small hunting oscillations.
PEDAL_HYST_GAP = 1.0  # DI units

# Zero-torque learning: Tinkla continuously learns the pedal position where
# the DI motor produces zero torque. This makes accel=0 hold speed instead
# of applying regen. The learned value replaces DI=0 in the accel mapping.
TORQUE_LEVEL_ACC = 0.0    # upper torque bound for zero-torque detection
TORQUE_LEVEL_DECEL = -30.0  # lower bound — below this is real braking
ZERO_TORQUE_MIN_SPEED = 10.0 * CV.MPH_TO_MS  # only learn above 10 mph


class PedalZeroTorque:
  """Learns the pedal DI position that produces zero motor torque."""

  def __init__(self):
    self.value = PEDAL_DI_ZERO  # start at coast, refine from real data
    self._best_torque = TORQUE_LEVEL_DECEL

  def update(self, torque_level: float, current_pedal_di: float, v_ego: float):
    """Call every pedal frame with the current motor torque and pedal position."""
    if v_ego < ZERO_TORQUE_MIN_SPEED:
      return

    # If torque is between decel and accel thresholds and closer to zero
    # than the best we've seen, this pedal position is near zero-torque
    if (TORQUE_LEVEL_DECEL < torque_level < TORQUE_LEVEL_ACC
        and abs(torque_level) < abs(self._best_torque)):
      self.value = current_pedal_di
      self._best_torque = torque_level

  def get(self, v_ego: float) -> float:
    """Returns the zero-torque DI value. Falls back to DI=0 at low speed."""
    if v_ego < 5.0 * CV.MPH_TO_MS:
      return PEDAL_DI_ZERO
    return self.value


# Module-level singleton — persists across calls, learns over the drive
_zero_torque = PedalZeroTorque()


def get_zero_torque():
  return _zero_torque


def compute_pedal_command(accel_request: float, v_ego: float, prev_pedal_di: float,
                          target_speed_kph: float | None = None,
                          conditional_hysteresis: bool = False) -> tuple[float, float]:
  """Convert acceleration request (m/s²) to comma pedal voltage.

  Returns (pedal_voltage, updated_prev_pedal_di).

  If ``conditional_hysteresis`` is True, hysteresis applies only near the
  set-speed anchor (|v_ego_kph - target_speed_kph| < 0.8 and v_ego > 5 m/s) —
  Tinkla unified PCC_module pattern. Outside that band, MPC-output passes
  through unfiltered so lead-following stays responsive. Default False
  preserves the original always-on behavior.
  """
  if nap_conf is None:
    pedal_di = float(clip(interp(accel_request, [-1.5, 0., 2.0], [-5., 0., 100.]), -5, 100))
    pedal_di = float(clip(pedal_di, prev_pedal_di - PEDAL_RAMP_RATE_DOWN, prev_pedal_di + PEDAL_RAMP_RATE_UP))
    return _fallback_di_to_pedal(pedal_di), pedal_di

  max_pedal_value = float(interp(v_ego, PEDAL_BP, PEDAL_MAX_VALUES))

  # Zero-torque learned position: accel=0 maps here instead of DI=0
  zero_torque_di = _zero_torque.get(v_ego)

  # Deadband: treat small accel requests as zero-torque (hold speed)
  if abs(accel_request) < ACCEL_DEADBAND:
    accel_request = 0.0

  # Speed-dependent regen floor: clip the request before mapping so MPC asks
  # for more decel than we'll deliver are softened, not just clipped at PEDAL_DI_MIN.
  regen_floor = get_regen_floor(v_ego)
  accel_request = max(accel_request, regen_floor)

  # Map accel to DI using zero-torque as the midpoint
  accel_bp = [regen_floor, 0.0, ACCEL_MAX]
  accel_v = [PEDAL_DI_MIN, zero_torque_di, max_pedal_value]
  pedal_di = float(interp(accel_request, accel_bp, accel_v))

  pedal_di = float(clip(pedal_di, PEDAL_DI_MIN, max_pedal_value))

  # Asymmetric rate limiter
  pedal_di = float(clip(pedal_di, prev_pedal_di - PEDAL_RAMP_RATE_DOWN, prev_pedal_di + PEDAL_RAMP_RATE_UP))

  if conditional_hysteresis and target_speed_kph is not None:
    # Tinkla unified pattern (PCC_module.py:334-335): hysteresis only near
    # the set-speed anchor. Lets MPC accel-down responses pass through
    # unfiltered when chasing a lead.
    v_ego_kph = v_ego * 3.6
    near_setpoint = abs(v_ego_kph - float(target_speed_kph)) < 0.8 and v_ego > 5.0
    if near_setpoint and abs(pedal_di - prev_pedal_di) < PEDAL_HYST_GAP:
      pedal_di = prev_pedal_di
  else:
    # Default: always-on output hysteresis (suppresses small oscillations)
    if abs(pedal_di - prev_pedal_di) < PEDAL_HYST_GAP:
      pedal_di = prev_pedal_di

  pedal_cmd = nap_conf.di_to_pedal(pedal_di)
  return pedal_cmd, pedal_di


# Fallback constants when nap_conf unavailable
_PEDAL_CALIB_FACTOR = 1.0
_PEDAL_CALIB_ZERO = 0.0
_PEDAL_ZERO = _PEDAL_CALIB_ZERO - 1.0 / _PEDAL_CALIB_FACTOR


def _fallback_di_to_pedal(val):
  return _PEDAL_ZERO + (val - 0.0) / _PEDAL_CALIB_FACTOR
