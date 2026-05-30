"""Tesla Pre-AP configuration: pedal calibration, radar, and control mode settings."""

import json
import os
import tempfile

from opendbc.car.carlog import carlog
from opendbc.car.tesla.preap.nap_params import NAPParamKeys

try:
  from openpilot.common.params import Params
  _params = Params()
  _PARAMS_AVAILABLE = True
except ImportError:
  _params = None    # placeholder så tester kan patche denne attributten
  _PARAMS_AVAILABLE = False

carlog.info("nap_conf: _PARAMS_AVAILABLE=%s", _PARAMS_AVAILABLE)


CONFIG_FILE = "/data/nap_params.json"

DEFAULT_CONFIG = {
  'double_pull_window_ms': 750,
  'use_pedal': False,
  'pedal_calibrated': False,
  'pedal_can_zero': False,
  'pedal_min': 0,
  'pedal_max': 1023,
  'pedal_calib_min': -3.0,
  'pedal_calib_max': 99.6,
  'pedal_calib_zero': 0.0,
  'pedal_calib_factor': 1.0,
  'radar_enabled': False,
  'radar_behind_nosecone': False,
  'radar_offset': 0.0,
  'brake_factor': 1.0,
  # Sveins overlay: PCC conditional hysterese (Tinkla unified PCC_module:334 pattern).
  # Default True → hysterese kun near pedal_speed_kph setpoint, full MPC-respons ellers.
  # Adresserer "klarer ikke følge bil foran" ved at brake-respons passerer ufiltrert.
  'long_gate_open': True,
}

# Pedal DI (Driver Intent) constants — internal representation before calibration
PEDAL_DI_MIN = -5       # Max regen (coasting hard)
PEDAL_DI_ZERO = 0       # Neutral
PEDAL_DI_PRESSED = 2    # "pedal pressed" threshold

ACCEL_MAX = 2.5         # m/s^2
REGEN_MAX = -1.5        # m/s^2 (legacy hard floor; speed-dependent floor below)
PEDAL_HYST_GAP = 1.0

# Tinkla unified LONG_module speed-dep brake/accel stretch tabell.
# Tinkla LONG_module.py:69-76 ganger denne på actuators.accel UANSETT fortegn
# (etter brake_factor-toggle for negativ accel). Det fungerer på Tinkla fordi
# deres PCC PID-stack (kp=0.75, ki=0.07, D-term, set-speed-anchor, error-side
# hysterese) demper enhver overshoot fra speed-amplifikasjonen.
#
# Vår VDAS-stack er feedforward-dominert (kf=1.0, kp=0) uten D-term/anchor/
# hysteresis. Samme speed-stretch blir destabiliserende — observert som
# progressive eskalering 1.15→1.45× MPC's a_target med speed, som lukker en
# 0.1 Hz loop-resonans mot MPC. Disablet 2026-05-20 etter drive-batch-analyse
# (drive58/5a/5b/5c hunting median 0.71 → 3.05 km/h korrelert med S85-LUT +
# stretch-amplifikasjon).
#
# Behold tabellen for fremtidig bruk hvis vi porter Tinkla's PCC PID-stack
# (Fase B i project_pedal_long_niva_2_omfang_2026_05_15).
BRAKE_FACTOR_BP = [18.0, 28.0]  # m/s
BRAKE_FACTOR_V = [1.0, 1.0]     # disabled — see comment above

# Speed-dependent regen floor (m/s² lower bound). Constant -1.5 at all speeds
# feels harsh at low speed because the relative deceleration is higher and the
# motor's torque curve makes low-speed regen "bite" more. Tinkla-style scaling:
# soft decel near standstill, full regen at highway speeds.
# mph:           0    4    16    33    67
REGEN_MAX_BP = [0., 2.0, 7.0, 15.0, 30.0]   # m/s
REGEN_MAX_V  = [-0.3, -0.6, -1.0, -1.3, -1.5]

# Speed-dependent max pedal (m/s breakpoints)
# mph:   0   11   27   44   67   90
PEDAL_BP = [0., 5., 12., 20., 30., 40.]

# Pedal-max LUT. Default fra NotAutopilot/opendbc nap-release (upstream).
# Conservative tune som ble validert å fungere på vår VDAS-stack i drive58
# (hunting median 0.71 km/h, all_med).
#
# 2026-05-20: forsøkte Tinkla S85-LUT [55, 63, 75, 90, 99, 99] for å fikse
# observert under-deliver ved 28 m/s. Resultat: drive5c hunting median ble
# 3.05 km/h (4× verre enn drive58 baseline). Tinkla S85-LUT er tunet for
# Tinkla's PCC PID-stack m/ D-term — vår VDAS-FF-stack får destabiliserende
# open-loop gain-økning med samme tabell. Revertert til upstream default.
# Se feedback_tinkla_paritet_ikke_garanti.md.
PEDAL_MAX_VALUES = [50., 58., 66., 74., 82., 90.]

# Planner acceleration envelopes by openpilot Personality. Single source of
# truth — preap.interface.PERSONALITY_TO_PROFILE maps the 0/1/2 personality
# enum to these keys, and preap.interface.get_preap_accel_limits applies them.
ACCEL_LOOKUP_BP = [0.0, 1.3, 7.5, 15.0, 25.0, 40.0]  # m/s
ACCEL_MAX_PROFILES = {
  'Chill':    [0.3, 0.7, 0.9, 0.7, 0.6, 0.5],   # personality=Relaxed
  'Standard': [0.3, 0.9, 1.2, 1.0, 0.8, 0.6],   # personality=Standard
  'MadMax':   [0.3, 1.6, 1.9, 1.5, 1.2, 1.0],   # personality=Aggressive
}



def transform_di_to_pedal(val, pedal_zero, pedal_factor):
  """DI units -> pedal voltage. pedal_zero + (val - DI_ZERO) / factor"""
  if pedal_factor == 0:
    pedal_factor = 1.0
  return pedal_zero + (val - PEDAL_DI_ZERO) / pedal_factor


def transform_pedal_to_di(val, pedal_zero, pedal_factor):
  """Pedal voltage -> DI units. DI_ZERO + (val - pedal_zero) * factor"""
  return PEDAL_DI_ZERO + (val - pedal_zero) * pedal_factor


class NAPConf:
  """Pre-AP config backed by JSON file at /data/nap_params.json.
  Uses openpilot Params when available, falls back to JSON."""

  def __init__(self):
    self._cache = {}
    self._load()

  # Storage

  def _load(self):
    try:
      if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, 'r') as f:
          loaded = json.load(f)
        self._cache = {**DEFAULT_CONFIG, **loaded}
      else:
        self._cache = DEFAULT_CONFIG.copy()
        self._save()
    except Exception:
      self._cache = DEFAULT_CONFIG.copy()

  def _save(self):
    """Atomic write: temp file + rename to prevent corruption on power loss."""
    try:
      os.makedirs(os.path.dirname(CONFIG_FILE), exist_ok=True)
      fd, tmp_path = tempfile.mkstemp(
        dir=os.path.dirname(CONFIG_FILE),
        prefix='.nap_params_', suffix='.tmp')
      try:
        with os.fdopen(fd, 'w') as f:
          json.dump(self._cache, f, indent=2)
        os.replace(tmp_path, CONFIG_FILE)
      except Exception:
        try:
          os.unlink(tmp_path)
        except Exception:
          pass
        raise
    except Exception:
      pass

  def _get(self, key, default):
    return self._cache.get(key, default)

  def _put(self, key, value):
    self._cache[key] = value
    self._save()

  # Params helpers

  def _get_param_bool(self, param_key, json_key, default=False):
    if _PARAMS_AVAILABLE:
      return _params.get_bool(param_key)
    return self._get(json_key, default)

  def _put_param_bool(self, param_key, json_key, value):
    if _PARAMS_AVAILABLE:
      _params.put_bool_nonblocking(param_key, bool(value))
    self._put(json_key, bool(value))

  def _get_param_float(self, param_key, json_key, default):
    if _PARAMS_AVAILABLE:
      val = _params.get(param_key, return_default=True)
      return float(val) if val is not None else default
    return float(self._get(json_key, default))

  def _put_param_float(self, param_key, json_key, value):
    if _PARAMS_AVAILABLE:
      _params.put(param_key, float(value))
    self._put(json_key, float(value))

  # Bool properties

  @property
  def use_pedal(self):
    return self._get_param_bool(NAPParamKeys.PEDAL_ENABLED, 'use_pedal')

  @use_pedal.setter
  def use_pedal(self, value):
    self._put_param_bool(NAPParamKeys.PEDAL_ENABLED, 'use_pedal', value)

  @property
  def radar_enabled(self):
    return self._get_param_bool(NAPParamKeys.RADAR_ENABLED, 'radar_enabled')

  @radar_enabled.setter
  def radar_enabled(self, value):
    self._put_param_bool(NAPParamKeys.RADAR_ENABLED, 'radar_enabled', value)

  @property
  def radar_behind_nosecone(self):
    return self._get_param_bool(NAPParamKeys.RADAR_BEHIND_NOSECONE, 'radar_behind_nosecone')

  @radar_behind_nosecone.setter
  def radar_behind_nosecone(self, value):
    self._put_param_bool(NAPParamKeys.RADAR_BEHIND_NOSECONE, 'radar_behind_nosecone', value)

  @property
  def pedal_calibrated(self):
    if not self._get_param_bool(NAPParamKeys.PEDAL_CALIB_DONE, 'pedal_calibrated'):
      return False
    # Defaults-sanity: the done-flag is only trustworthy if at least one calibration
    # value has moved off its DEFAULT_CONFIG value. Guards against a write-failure
    # that leaves the flag True but the values unpersisted (see upstream 2026-04
    # drive-1 incident). Ported from NotAutopilot/opendbc f867b3a4.
    if self._calib_zero_raw() == DEFAULT_CONFIG['pedal_calib_zero'] and \
       self.pedal_factor == DEFAULT_CONFIG['pedal_calib_factor']:
      return False
    return True

  @pedal_calibrated.setter
  def pedal_calibrated(self, value):
    self._put_param_bool(NAPParamKeys.PEDAL_CALIB_DONE, 'pedal_calibrated', value)

  def _calib_zero_raw(self):
    """Raw stored pedal_calib_zero (no coast-position transform applied)."""
    return self._get_param_float(NAPParamKeys.PEDAL_CALIB_ZERO, 'pedal_calib_zero', 0.0)

  @property
  def road_sign_fallback_kph(self):
    """Tesla IC road-sign widget fallback when Tesla DI reports SNA (no GPS-fix,
    no nav-DB hit, parking). Default 0 = no sign shown. Display-only, risk-tier 3.

    Fil-fallback: NAPRoadSignFallbackKph er deklarert i params_keys.h men kan
    være fraværende i pre-built params_pyx.so på enheter som ikke har re-bygd
    siden V47-deploy. UnknownKeyName fra params_pyx krasjet cardd ved
    fingerprinting tidligere — vi swallower exception og leser filen direkte."""
    if _PARAMS_AVAILABLE:
      try:
        val = _params.get(NAPParamKeys.ROAD_SIGN_FALLBACK_KPH, return_default=True)
        return max(0, min(150, int(val))) if val is not None else 0
      except (TypeError, ValueError):
        return 0
      except Exception:
        pass  # UnknownKeyName for pre-V47 params_pyx → fall through to file
      try:
        with open('/data/params/d/' + NAPParamKeys.ROAD_SIGN_FALLBACK_KPH, 'rb') as f:
          raw = f.read().strip()
          return max(0, min(150, int(raw))) if raw else 0
      except (OSError, ValueError):
        pass
    return int(self._get('road_sign_fallback_kph', 0))

  @road_sign_fallback_kph.setter
  def road_sign_fallback_kph(self, value):
    clipped = max(0, min(150, int(value)))
    if _PARAMS_AVAILABLE:
      try:
        _params.put(NAPParamKeys.ROAD_SIGN_FALLBACK_KPH, clipped)
      except Exception:
        pass
    self._put('road_sign_fallback_kph', clipped)

  @property
  def long_gate_open(self):
    """Sveins overlay: PCC conditional hysterese (Tinkla unified PCC_module:334).
    Default True → hysterese kun aktiv near pedal_speed_kph + v_ego > 5 m/s,
    så MPC's brake-respons for lead-tracking passerer ufiltrert ellers.

    Fil-fallback: NAPLongGate ble lagt til params_keys.h i V48. Inntil c3
    re-bygger params_pyx, leser vi /data/params/d/NAPLongGate direkte hvis
    Params-systemet ikke kjenner key (return_default=True gir None)."""
    if _PARAMS_AVAILABLE:
      try:
        raw = _params.get(NAPParamKeys.LONG_GATE, return_default=True)
        if raw is not None:
          return raw in (b'1', b'true', b'True', '1', 'true', 'True')
      except Exception:
        pass  # UnknownKeyName for pre-V48 params_pyx → fall through to file
      try:
        with open('/data/params/d/' + NAPParamKeys.LONG_GATE, 'rb') as f:
          return f.read().strip() in (b'1', b'true', b'True')
      except OSError:
        pass
    return bool(self._get('long_gate_open', True))

  @long_gate_open.setter
  def long_gate_open(self, value):
    self._put_param_bool(NAPParamKeys.LONG_GATE, 'long_gate_open', value)

  @property
  def pedal_can_zero(self):
    if _PARAMS_AVAILABLE:
      bus = _params.get(NAPParamKeys.PEDAL_CAN_BUS, return_default=True)
      return bus == 0
    return self._get('pedal_can_zero', False)

  @pedal_can_zero.setter
  def pedal_can_zero(self, value):
    if _PARAMS_AVAILABLE:
      _params.put(NAPParamKeys.PEDAL_CAN_BUS, 0 if value else 2)
    self._put('pedal_can_zero', bool(value))

  # Engagement

  @property
  def double_pull_enabled(self):
    return True  # always on — safety requirement

  @property
  def double_pull_window_ms(self):
    return int(self._get('double_pull_window_ms', 750))

  @double_pull_window_ms.setter
  def double_pull_window_ms(self, value):
    self._put('double_pull_window_ms', max(300, min(1500, int(value))))

  # Pedal calibration

  @property
  def pedal_min(self):
    return int(self._get('pedal_min', 0))

  @pedal_min.setter
  def pedal_min(self, value):
    self._put('pedal_min', int(value))

  @property
  def pedal_max(self):
    return int(self._get('pedal_max', 1023))

  @pedal_max.setter
  def pedal_max(self, value):
    self._put('pedal_max', int(value))

  @property
  def radar_offset(self):
    return float(self._get('radar_offset', 0.0))

  @radar_offset.setter
  def radar_offset(self, value):
    self._put('radar_offset', float(value))

  @property
  def pedal_calib_min(self):
    return self._get_param_float(NAPParamKeys.PEDAL_CALIB_MIN, 'pedal_calib_min', -3.0)

  @pedal_calib_min.setter
  def pedal_calib_min(self, value):
    self._put_param_float(NAPParamKeys.PEDAL_CALIB_MIN, 'pedal_calib_min', value)

  @property
  def pedal_calib_max(self):
    return self._get_param_float(NAPParamKeys.PEDAL_CALIB_MAX, 'pedal_calib_max', 99.6)

  @pedal_calib_max.setter
  def pedal_calib_max(self, value):
    self._put_param_float(NAPParamKeys.PEDAL_CALIB_MAX, 'pedal_calib_max', value)

  @property
  def pedal_zero(self):
    """Coast position: calib_zero - 1/factor"""
    if _PARAMS_AVAILABLE:
      calib_zero_val = _params.get(NAPParamKeys.PEDAL_CALIB_ZERO, return_default=True)
      calib_zero = float(calib_zero_val) if calib_zero_val is not None else 0.0
    else:
      calib_zero = float(self._get('pedal_calib_zero', 0.0))
    factor = self.pedal_factor
    if factor == 0:
      factor = 1.0
    return calib_zero - 1.0 / factor

  @pedal_zero.setter
  def pedal_zero(self, value):
    if _PARAMS_AVAILABLE:
      _params.put(NAPParamKeys.PEDAL_CALIB_ZERO, float(value))
    self._put('pedal_calib_zero', float(value))

  @property
  def pedal_factor(self):
    """Calibration scaling: 100.0 / (pedal_max - pedal_pressed)"""
    return self._get_param_float(NAPParamKeys.PEDAL_CALIB_FACTOR, 'pedal_calib_factor', 1.0)

  @pedal_factor.setter
  def pedal_factor(self, value):
    self._put_param_float(NAPParamKeys.PEDAL_CALIB_FACTOR, 'pedal_calib_factor', value)

  @property
  def brake_factor(self):
    """User-tunable brake-amplitude multiplier (Tinkla TinklaBrakeFactor paritet).
    Applied to negative accel before speed-dep stretch. Default 1.0."""
    return self._get_param_float(NAPParamKeys.BRAKE_FACTOR, 'brake_factor', 1.0)

  @brake_factor.setter
  def brake_factor(self, value):
    self._put_param_float(NAPParamKeys.BRAKE_FACTOR, 'brake_factor', value)


  # Utilities

  @property
  def pedal_can_bus(self):
    return 0 if self.pedal_can_zero else 2

  def di_to_pedal(self, val):
    return transform_di_to_pedal(val, self.pedal_zero, self.pedal_factor)

  def pedal_to_di(self, val):
    return transform_pedal_to_di(val, self.pedal_zero, self.pedal_factor)

  def print_config(self):
    """Diagnostic dump to stdout (for CLI tools)."""
    print("=== Tesla Pre-AP Configuration ===")
    storage = "openpilot Params" if _PARAMS_AVAILABLE else CONFIG_FILE
    print(f"    Storage: {storage}")
    print("")
    print("  [CONTROL MODES]")
    print(f"    Double-Pull Mode:     ON (always enabled)")
    print("")
    print("  [LONGITUDINAL]")
    print(f"    Pedal Enabled:        {'ON' if self.use_pedal else 'OFF'}")
    print(f"    Pedal Calibrated:     {'YES' if self.pedal_calibrated else 'NO'}")
    print(f"    Accel Profile:        (from openpilot LongitudinalPersonality)")
    print("")
    print("  [PEDAL CALIBRATION]")
    print(f"    Pedal Min (raw):      {self.pedal_min}")
    print(f"    Pedal Max (raw):      {self.pedal_max}")
    print(f"    Pedal Calib Min:      {self.pedal_calib_min:.2f}")
    print(f"    Pedal Calib Max:      {self.pedal_calib_max:.2f}")
    print(f"    Pedal Zero:           {self.pedal_zero:.3f}")
    print(f"    Pedal Factor:         {self.pedal_factor:.3f}")
    print(f"    Pedal CAN Bus:        {self.pedal_can_bus}")
    print("")
    print("  [RADAR]")
    print(f"    Radar Enabled:        {'ON' if self.radar_enabled else 'OFF'}")
    print(f"    Behind Nosecone:      {'YES' if self.radar_behind_nosecone else 'NO'}")
    print(f"    Radar Offset:         {self.radar_offset}m")
    print("")
    print("==================================")

  def get_all_params(self):
    return {
      'double_pull_window_ms': self.double_pull_window_ms,
      'use_pedal': self.use_pedal,
      'pedal_calibrated': self.pedal_calibrated,
      'pedal_min': self.pedal_min,
      'pedal_max': self.pedal_max,
      'pedal_calib_min': self.pedal_calib_min,
      'pedal_calib_max': self.pedal_calib_max,
      'pedal_zero': self.pedal_zero,
      'pedal_factor': self.pedal_factor,
      'pedal_can_bus': self.pedal_can_bus,
      'pedal_can_zero': self.pedal_can_zero,
      'radar_enabled': self.radar_enabled,
      'radar_behind_nosecone': self.radar_behind_nosecone,
      'radar_offset': self.radar_offset,
    }

  def reset_to_defaults(self):
    self._cache = DEFAULT_CONFIG.copy()
    self._save()

  def reload(self):
    self._load()


nap_conf = NAPConf()


# ----- Tune-bridge (Fase 3d 2026-05-27) -------------------------------------
#
# Live-tune-override-mekanisme. Lar `rig tune push` skrive til Params slik
# at VirtualDAS plukker opp nye PID-koeffisienter uten redeploy. Hovedmål:
# round-trip tune-iterasjon < 10s istedenfor ~5 min via SCP+reboot.
#
# Sikkerhets-arkitektur:
#   - Master-toggle NAPTuneEnable (default False). Hvis False, INGEN
#     overrides leses fra Params; nap_conf returnerer base-values fra
#     constants.py.
#   - Hver override-verdi clampes til sane range. Out-of-range → fallback
#     til base + cloudlog.warning.
#   - VirtualDAS skal ramp endringer over 2s glatt (i sin tick-loop, ikke
#     her). Vi eksponerer kun *target* values; ramping er kontroller-ansvar.
#   - Logging på engagement: cloudlog.warning hvis TUNE_ENABLE=True så
#     bruker ser i rlogs at testing-mode er på.
#
# Risk-tier 2 — KUN brukes under utvikling/tuning. Sveins ansvar å sette
# TUNE_ENABLE=0 i daglig kjøring.

# Clamp-ranges per override-key. Format: (min, max). out-of-range → ignored.
TUNE_RANGES = {
  NAPParamKeys.TUNE_PEDAL_KI_V: (0.0, 0.5),
  NAPParamKeys.TUNE_PEDAL_KP_V: (0.0, 0.5),
  NAPParamKeys.TUNE_PEDAL_KD_V: (0.0, 0.5),
  NAPParamKeys.TUNE_PEDAL_LONG_KI_V: (0.0, 0.5),
  NAPParamKeys.TUNE_INTEGRAL_LEAK: (0.95, 1.0),
  NAPParamKeys.TUNE_BRAKE_FACTOR: (0.5, 2.0),
}


def _parse_float_list(s):
  """Parse JSON-string '[0.25,0.22,0.18]' eller komma-form '0.25,0.22,0.18'.
  Returner None hvis ugyldig."""
  if s is None or s == "":
    return None
  s = s.strip()
  try:
    if s.startswith("["):
      val = json.loads(s)
    else:
      val = [float(x) for x in s.split(",") if x.strip()]
    if not isinstance(val, list) or not val:
      return None
    return [float(x) for x in val]
  except (ValueError, TypeError, json.JSONDecodeError):
    return None


def _clamp_list(lst, lo, hi):
  """Clamp hver verdi i listen til (lo, hi). Returner None hvis noen
  verdier var utenfor (advarer caller til å logge)."""
  if lst is None:
    return None, False
  out = []
  any_clamped = False
  for x in lst:
    cx = max(lo, min(hi, float(x)))
    if cx != x:
      any_clamped = True
    out.append(cx)
  return out, any_clamped


def _clamp_scalar(val, lo, hi):
  if val is None:
    return None, False
  cv = max(lo, min(hi, float(val)))
  return cv, cv != val


class NAPTuneOverrides:
  """Snapshot av live-tune-Params. Bygges via NAPTuneOverrides.read_current()
  som leser fra Params + clamper. VirtualDAS holder en instans og refresher
  hvert N tick (typisk 5s).

  Hvis tune_enabled=False, alle override-felter er None — kaller ser at
  ingen overrides er aktive.
  """

  def __init__(self):
    self.tune_enabled = False
    self.pedal_ki_v = None       # list eller None — inner VDAS_INNER_KI_V (3-element)
    self.pedal_kp_v = None       # list eller None — inner VDAS_INNER_KP_V
    self.pedal_kd_v = None       # list eller None — inner VDAS_INNER_KD_V
    self.pedal_long_ki_v = None  # list eller None — outer PEDAL_LONG_KI_V (4-element)
    self.integral_leak = None    # float eller None — outer LongControl
    self.brake_factor = None     # float eller None
    self.last_warnings = []      # liste av clamping/parse-warnings fra siste read

  @classmethod
  def read_current(cls):
    """Les Params + clamp. Returner ny instans.

    Hvis NAPTuneEnable=False eller Params utilgjengelig: alle felt None.
    """
    inst = cls()
    if not _PARAMS_AVAILABLE:
      return inst

    try:
      enabled = bool(_params.get_bool(NAPParamKeys.TUNE_ENABLE))
    except Exception:
      enabled = False
    inst.tune_enabled = enabled

    if not enabled:
      return inst

    warnings = []

    # Pedal KI
    raw_ki = _params.get(NAPParamKeys.TUNE_PEDAL_KI_V, return_default=True)
    parsed_ki = _parse_float_list(raw_ki.decode() if isinstance(raw_ki, bytes)
                                  else raw_ki)
    lo, hi = TUNE_RANGES[NAPParamKeys.TUNE_PEDAL_KI_V]
    clamped_ki, was_clamped = _clamp_list(parsed_ki, lo, hi)
    inst.pedal_ki_v = clamped_ki
    if was_clamped:
      warnings.append(f"TunePedalKiV clamped to [{lo}, {hi}]")

    # Pedal KP
    raw_kp = _params.get(NAPParamKeys.TUNE_PEDAL_KP_V, return_default=True)
    parsed_kp = _parse_float_list(raw_kp.decode() if isinstance(raw_kp, bytes)
                                  else raw_kp)
    lo, hi = TUNE_RANGES[NAPParamKeys.TUNE_PEDAL_KP_V]
    clamped_kp, was_clamped = _clamp_list(parsed_kp, lo, hi)
    inst.pedal_kp_v = clamped_kp
    if was_clamped:
      warnings.append(f"TunePedalKpV clamped to [{lo}, {hi}]")

    # Pedal KD
    raw_kd = _params.get(NAPParamKeys.TUNE_PEDAL_KD_V, return_default=True)
    parsed_kd = _parse_float_list(raw_kd.decode() if isinstance(raw_kd, bytes)
                                  else raw_kd)
    lo, hi = TUNE_RANGES[NAPParamKeys.TUNE_PEDAL_KD_V]
    clamped_kd, was_clamped = _clamp_list(parsed_kd, lo, hi)
    inst.pedal_kd_v = clamped_kd
    if was_clamped:
      warnings.append(f"TunePedalKdV clamped to [{lo}, {hi}]")

    # Pedal LONG KI (outer LongControl)
    raw_long_ki = _params.get(NAPParamKeys.TUNE_PEDAL_LONG_KI_V, return_default=True)
    parsed_long_ki = _parse_float_list(raw_long_ki.decode() if isinstance(raw_long_ki, bytes)
                                       else raw_long_ki)
    lo, hi = TUNE_RANGES[NAPParamKeys.TUNE_PEDAL_LONG_KI_V]
    clamped_long_ki, was_clamped = _clamp_list(parsed_long_ki, lo, hi)
    inst.pedal_long_ki_v = clamped_long_ki
    if was_clamped:
      warnings.append(f"TunePedalLongKiV clamped to [{lo}, {hi}]")

    # Integral leak (outer LongControl)
    try:
      raw_leak = _params.get(NAPParamKeys.TUNE_INTEGRAL_LEAK, return_default=True)
      parsed_leak = float(raw_leak) if raw_leak is not None else None
    except (TypeError, ValueError):
      parsed_leak = None
    lo, hi = TUNE_RANGES[NAPParamKeys.TUNE_INTEGRAL_LEAK]
    clamped_leak, was_clamped = _clamp_scalar(parsed_leak, lo, hi)
    inst.integral_leak = clamped_leak
    if was_clamped:
      warnings.append(f"TuneIntegralLeak clamped to [{lo}, {hi}]")

    # Brake factor
    try:
      raw_bf = _params.get(NAPParamKeys.TUNE_BRAKE_FACTOR, return_default=True)
      parsed_bf = float(raw_bf) if raw_bf is not None else None
    except (TypeError, ValueError):
      parsed_bf = None
    lo, hi = TUNE_RANGES[NAPParamKeys.TUNE_BRAKE_FACTOR]
    clamped_bf, was_clamped = _clamp_scalar(parsed_bf, lo, hi)
    inst.brake_factor = clamped_bf
    if was_clamped:
      warnings.append(f"TuneBrakeFactor clamped to [{lo}, {hi}]")

    inst.last_warnings = warnings
    return inst

  def active_summary(self):
    """Menneske-lesbar str av aktive overrides. Brukes i cloudlog."""
    if not self.tune_enabled:
      return "tune-bridge: disabled"
    parts = ["tune-bridge: ENABLED"]
    if self.pedal_ki_v is not None:
      parts.append(f"ki={self.pedal_ki_v}")
    if self.pedal_kp_v is not None:
      parts.append(f"kp={self.pedal_kp_v}")
    if self.pedal_kd_v is not None:
      parts.append(f"kd={self.pedal_kd_v}")
    if self.pedal_long_ki_v is not None:
      parts.append(f"outer_ki={self.pedal_long_ki_v}")
    if self.integral_leak is not None:
      parts.append(f"leak={self.integral_leak}")
    if self.brake_factor is not None:
      parts.append(f"brake={self.brake_factor}")
    if self.last_warnings:
      parts.append(f"warnings={self.last_warnings}")
    return " ".join(parts)

  def any_active(self):
    """True hvis minst én override er aktiv (etter clamp)."""
    if not self.tune_enabled:
      return False
    return any(x is not None for x in (
      self.pedal_ki_v, self.pedal_kp_v, self.pedal_kd_v,
      self.pedal_long_ki_v, self.integral_leak, self.brake_factor,
    ))
