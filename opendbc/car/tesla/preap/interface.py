import numpy as np

from opendbc.car import get_safety_config, structs, STD_CARGO_KG
from opendbc.car.carlog import carlog
from opendbc.car.tesla.preap.nap_conf import nap_conf

# Safety param flags matching tesla_preap.h (renumbered — LONG_CONTROL removed as dead code)
PREAP_FLAG_ENABLE_PEDAL = 1
PREAP_FLAG_RADAR_EMULATION = 2
PREAP_FLAG_RADAR_BEHIND_NOSECONE = 4
PREAP_FLAG_IC_INTEGRATION = 8
from opendbc.car.tesla.preap.constants import (
  PEDAL_LONG_K_BP, PEDAL_LONG_KP_V, PEDAL_LONG_KI_V, PEDAL_LONG_KD_V,
)
from opendbc.car.tesla.preap.nap_conf import (
  ACCEL_LOOKUP_BP, ACCEL_MAX_PROFILES, REGEN_MAX_BP, REGEN_MAX_V,
)

# openpilot Personality (0=aggressive, 1=standard, 2=relaxed) → NAP profile name.
# Single source of truth for accel envelopes lives in ACCEL_MAX_PROFILES.
PERSONALITY_TO_PROFILE = {0: 'MadMax', 1: 'Standard', 2: 'Chill'}

try:
  from openpilot.common.params import Params as _Params
  _params = _Params()
except ImportError:
  _params = None


def get_preap_accel_limits(current_speed):
  personality = 1
  if _params is not None:
    try:
      personality = int(_params.get("LongitudinalPersonality", return_default=True))
    except (TypeError, ValueError):
      pass
  profile_name = PERSONALITY_TO_PROFILE.get(personality, 'Standard')
  profile = ACCEL_MAX_PROFILES.get(profile_name, ACCEL_MAX_PROFILES['Standard'])
  a_max = float(np.interp(current_speed, ACCEL_LOOKUP_BP, profile))
  # Speed-dependent regen floor — mirrors pedal/controller.get_regen_floor so
  # MPC never asks for more decel than the pedal will deliver.
  a_min = float(np.interp(current_speed, REGEN_MAX_BP, REGEN_MAX_V))
  return a_min, a_max


def _read_tinkla_ic_param():
  """Read NAPTinklaICIntegration from openpilot Params if available, else False."""
  if _params is None:
    return False
  try:
    return bool(_params.get_bool("NAPTinklaICIntegration"))
  except Exception:
    return False


def get_preap_params(ret, fingerprint):
  # Build safety param flags for the standalone Pre-AP safety mode
  flags = 0
  use_pedal = nap_conf.use_pedal
  radar_enabled = nap_conf.radar_enabled
  radar_behind_nosecone = nap_conf.radar_behind_nosecone
  has_ic_integration = _read_tinkla_ic_param()
  carlog.info("Pre-AP fingerprint: use_pedal=%s radar_enabled=%s behind_nosecone=%s ic_integration=%s",
              use_pedal, radar_enabled, radar_behind_nosecone, has_ic_integration)

  if use_pedal:
    flags |= PREAP_FLAG_ENABLE_PEDAL
  if radar_enabled:
    flags |= PREAP_FLAG_RADAR_EMULATION
  if radar_behind_nosecone:
    flags |= PREAP_FLAG_RADAR_BEHIND_NOSECONE
  if has_ic_integration:
    flags |= PREAP_FLAG_IC_INTEGRATION

  ret.safetyConfigs = [
    get_safety_config(structs.CarParams.SafetyModel.teslaPreap, int(flags)),
  ]
  ret.radarUnavailable = not radar_enabled
  # T2B-3 / Item 6: bind op-long to whether Comma Pedal is present. In
  # no-pedal mode we have no TX path for accel commands, so longitudinal
  # belongs to the DI (pcmCruise=True). Safe after T2B-1 Path A — stock_cc
  # spoof + Tesla CC events run independent of op-long.
  ret.openpilotLongitudinalControl = use_pedal
  ret.steerControlType = structs.CarParams.SteerControlType.angle
  ret.pcmCruise = not use_pedal

  if use_pedal:
    ret.longitudinalTuning.kpBP = PEDAL_LONG_K_BP
    ret.longitudinalTuning.kpV = PEDAL_LONG_KP_V
    ret.longitudinalTuning.kiBP = PEDAL_LONG_K_BP
    ret.longitudinalTuning.kiV = PEDAL_LONG_KI_V
    # kf=1.0 feedforward: a_target passes through 1:1 to pedal mapping
    try:
      ret.longitudinalTuning.kf = 1.0
    except AttributeError:
      pass  # kf not available in all capnp schema versions
    # Fase B (2026-05-23): aktiver kdBP/kdV. PIDController.update får
    # error_rate = (error - last_error)/DT_CTRL fra LongControl — D-term
    # demper rate-of-change i a_target-aEgo-error, særlig
    # lead-akselerasjons-spike. Krever capnp-schema med kdBP @7 / kdV @8.
    try:
      ret.longitudinalTuning.kdBP = PEDAL_LONG_K_BP
      ret.longitudinalTuning.kdV = PEDAL_LONG_KD_V
    except AttributeError:
      pass  # kdBP not available in pre-Fase B capnp builds
    # Plant-delay korreksjon 2026-05-23 — drive 0000007f cross-correlation
    # ga acmd→aEgo lag 0.41-0.62s (mean 0.53s) over 4 engage-vinduer.
    # Tidligere 0.4s estimat underestimerte → MPC kommanderte ~0.15s for
    # tidlig → integral akkumulerte feilretning i overshoot/undershoot.
    # 0.55 matchet mean av målte verdier; konservativ vs 0.62 worst-case.
    ret.longitudinalActuatorDelay = 0.55
  else:
    ret.longitudinalTuning.kpBP = [0.0]
    ret.longitudinalTuning.kpV = [0.0]
    ret.longitudinalTuning.kiBP = [0.0]
    ret.longitudinalTuning.kiV = [0.0]

  # Legacy Model S steering and physical params
  ret.steerLimitTimer = 0.4
  ret.steerActuatorDelay = 0.1
  ret.steerAtStandstill = True
  ret.alphaLongitudinalAvailable = False
  ret.vEgoStopping = 0.1
  ret.vEgoStarting = 0.1
  ret.stoppingDecelRate = 1.0

  # Pre-AP Model S is physically the same platform as HW1/HW2/HW3 Model S.
  # Vehicle params (mass=2100, wheelbase=2.960, steerRatio=15.0) come from
  # CarSpecs in values.py — do NOT override them here to avoid double-counting
  # STD_CARGO_KG (the framework adds it automatically).
  # Confirmed by Lukas (xnor-tech): identical to HW3.
  ret.centerToFront = ret.wheelbase * 0.5

  return ret
