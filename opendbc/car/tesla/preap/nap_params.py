"""
NAP (NotAutopilot) Parameter Keys

Single source of truth for all NAP param key names used by
the UI settings panel and Tesla Pre-AP car code.

Storage: openpilot Params system (params_keys.h)
"""


class NAPParamKeys:
  # Longitudinal Control
  ADAPTIVE_ACCEL = "NAPAdaptiveAccel"
  PEDAL_ENABLED = "NAPPedalEnabled"
  FOLLOW_DISTANCE = "NAPFollowDistance"

  # Pedal Hardware
  PEDAL_CAN_BUS = "NAPPedalCanBus"
  PEDAL_CALIB_DONE = "NAPPedalCalibDone"
  PEDAL_CALIB_MIN = "NAPPedalCalibMin"
  PEDAL_CALIB_MAX = "NAPPedalCalibMax"
  PEDAL_CALIB_FACTOR = "NAPPedalCalibFactor"
  PEDAL_CALIB_ZERO = "NAPPedalCalibZero"

  # Radar
  RADAR_ENABLED = "NAPRadarEnabled"
  RADAR_BEHIND_NOSECONE = "NAPRadarBehindNosecone"

  # iBooster / Braking
  IBOOSTER_ENABLED = "NAPiBoosterEnabled"
  BRAKE_FACTOR = "NAPBrakeFactor"

  # Advanced
  FORCE_PRE_AP = "NAPForcePreAP"

  # Sveins overlay: PCC conditional hysterese (Tinkla unified PCC_module:334).
  # Default on → hysterese frys kun aktiv near pedal_speed_kph setpoint
  # (|v_ego_kph - setpoint| < 0.8 og v_ego > 5 m/s). Utenfor band passerer
  # MPC-output ufiltrert så lead-tracking-brake-respons ikke sluker.
  # Sett False for legacy always-on output-hysterese.
  LONG_GATE = "NAPLongGate"

  # Tinkla Buddy IC integration — IC-rendering via DAS-frames on chassis bus 0.
  # Default off. Display-only, risk-tier 3 (Buddy IC does not affect engage or safety).
  TINKLA_IC_INTEGRATION = "NAPTinklaICIntegration"

  # Tesla IC native road-sign widget fallback (kph). Used when Tesla DI's
  # UI_gpsVehicleSpeed.UI_mppSpeedLimit reports 0 (no GPS-fix, no nav-DB hit).
  # Default 0 = no sign shown when no GPS data. Risk-tier 3 (display-only).
  ROAD_SIGN_FALLBACK_KPH = "NAPRoadSignFallbackKph"

  # VDAS + carcontroller per-tick debug-log til /data/nap_debug.jsonl.
  # Default off. Aktivér for tune-/diagnose-arbeid; rotér ved 50 MB.
  # Felter: t, v_ego, a_target, ff_di, pid_corr, integral, speed_dep,
  # final_di, accel_request_in/out, lead_status, dRel m.fl.
  DEBUG_LOG = "NAPDebugLog"

  # Tune-bridge (Fase 3d 2026-05-27) — live-tune-overrides for VirtualDAS-
  # controller uten redeploy. Default disabled (TUNE_ENABLE=False). Hver
  # override-key er JSON-string med liste/scalar; nap_conf parser + clamper.
  # Risk-tier 2 (rører kjørende controller — clamp + master-toggle + smooth
  # ramping er sikkerhetsbarrierer).
  TUNE_ENABLE = "NAPTuneEnable"
  TUNE_PEDAL_KI_V = "NAPTunePedalKiV"     # JSON list [a,b,c] over VDAS_INNER_K_BP (inner)
  TUNE_PEDAL_KP_V = "NAPTunePedalKpV"     # JSON list [a,b,c] (inner)
  TUNE_PEDAL_KD_V = "NAPTunePedalKdV"     # JSON list [a,b,c] (inner)
  TUNE_PEDAL_LONG_KI_V = "NAPTunePedalLongKiV"  # JSON list [a,b,c,d] over PEDAL_LONG_K_BP (outer LongControl)
  TUNE_INTEGRAL_LEAK = "NAPTuneIntegralLeak"   # float 0.95-1.0 (outer LongControl)
  TUNE_BRAKE_FACTOR = "NAPTuneBrakeFactor"     # float 0.5-2.0


# Default values matching params_keys.h declarations
DEFAULTS = {
  NAPParamKeys.ADAPTIVE_ACCEL: True,
  NAPParamKeys.PEDAL_ENABLED: False,
  NAPParamKeys.FOLLOW_DISTANCE: 4,
  NAPParamKeys.PEDAL_CAN_BUS: 2,
  NAPParamKeys.PEDAL_CALIB_DONE: False,
  NAPParamKeys.PEDAL_CALIB_MIN: -3.0,
  NAPParamKeys.PEDAL_CALIB_MAX: 99.6,
  NAPParamKeys.PEDAL_CALIB_FACTOR: 1.0,
  NAPParamKeys.PEDAL_CALIB_ZERO: 0.0,
  NAPParamKeys.RADAR_ENABLED: False,
  NAPParamKeys.RADAR_BEHIND_NOSECONE: False,
  NAPParamKeys.IBOOSTER_ENABLED: False,
  NAPParamKeys.BRAKE_FACTOR: 1.0,
  NAPParamKeys.FORCE_PRE_AP: False,
  NAPParamKeys.LONG_GATE: True,  # Sveins overlay: PCC conditional hysterese default on (Tinkla unified pattern)
  NAPParamKeys.TINKLA_IC_INTEGRATION: False,  # Tinkla Buddy IC-rendering toggle, default off
  NAPParamKeys.ROAD_SIGN_FALLBACK_KPH: 0,  # 0 = no sign on GPS-loss; sett til e.g. 80 for konstant fallback
  NAPParamKeys.DEBUG_LOG: False,  # /data/nap_debug.jsonl-skriver; aktivér for tune-arbeid
  NAPParamKeys.TUNE_ENABLE: False,
  NAPParamKeys.TUNE_PEDAL_KI_V: "",
  NAPParamKeys.TUNE_PEDAL_KP_V: "",
  NAPParamKeys.TUNE_PEDAL_KD_V: "",
  NAPParamKeys.TUNE_PEDAL_LONG_KI_V: "",
  NAPParamKeys.TUNE_INTEGRAL_LEAK: 1.0,
  NAPParamKeys.TUNE_BRAKE_FACTOR: 1.0,
}
