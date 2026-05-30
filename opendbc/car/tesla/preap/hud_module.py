"""Tinkla Buddy IC integration — HUD-module for Tesla Pre-AP.

Ported from /home/svein/repos/Tinkla/selfdrive/car/tesla/HUD_module.py
(branch tesla_unity_releaseC3) via the bit-exact verified replay-rig at
/home/svein/repos/nap-replay-rig/rig/hud_controller.py.

Function: Render openpilot path/lanes/lead-car on Tesla MCU1 IC via Tinkla
Buddy's passive EtherCAN MITM. Sends 10 DAS_* arb-IDs on chassis bus 0
that Buddy sniffs and forwards to the IC.

Stubs:
  - ALCA (CS.alca_pre_engage/engaged/direction = 0) → DAS_alca_state defaults to
    1 (UNAVAILABLE_NO_LANES) or 6/7/8 from lane-quality (no lane-change render).
  - fleet_speed (CS.fleet_speed_state = 0, CS.DAS_fusedSpeedLimit = 0) → no map
    speed-limit on IC. MagZu lacks `mapd` pipeline; future work.
  - Lead car: NOT stubbed. Stock-openpilot `radarState.leadOne` via
    radard.py `get_RadarState_from_vision()` provides vision-fallback even
    when radar disabled (see findings §3, §5.8).

Plan C 0x2B9-gate: HUD-modul sender DAS_control kun når MagZu's long-stack
IKKE er aktivt (CS.magzu_long_active=False). Når long aktivt sender MagZu's
PreAPLongController 0x2B9 separat med inkrementerende counter.

Risk-tier: 3 (display-only, ikke safety).
Gating: CS.enableICIntegration (NAPTinklaICIntegration param). Default off.
"""
import numpy as np
from numpy import clip

from opendbc.car import structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import CAR

# Bus-map: PreAP bruker chassis bus 0 for IC-rendering.
# Andre Tesla-fingerprints (AP1/AP2) inkludert for symmetri, men bare PREAP brukes.
CAN_CHASSIS = {
  CAR.TESLA_MODEL_S_PREAP: 0,
}

# IC lane render scaling. Tinkla openpilot 0.6.6 + unity_releaseC3 bruker 0.5.
# Tinkla-paritet er kritisk for å trigge Buddy MITM-pipeline (rotårsak
# 2026-05-25). V46-V64e scale/bias-iterasjoner var bygd på feilantagelsen at
# lane-encoding var bugen — rotårsak var felt-verdier i 0x659/0x2B9 som
# hindret MITM-pipeline trigger. Reverteret til Tinkla 1:1.
IC_LANE_SCALE = 0.5

VisualAlert = structs.CarControl.HUDControl.VisualAlert
AudibleAlert = structs.CarControl.HUDControl.AudibleAlert


def compute_path_pinv(l=50):
  """Precompute pseudo-inverse for cubic path polyfit (Tinkla original)."""
  deg = 3
  x = np.arange(l * 1.0)
  X = np.vstack(tuple(x**n for n in range(deg, -1, -1))).T
  pinv = np.linalg.pinv(X)
  return pinv


def model_polyfit(points, path_pinv):
  return np.dot(path_pinv, [float(x) for x in points])


class HUDController:
  """Tinkla Buddy IC orchestrator. Owns counter-state, lane-poly state,
  lead-car-state. Called from CarController.update() once per tick. Returns
  list of CAN messages (3-tuples) to extend can_sends with."""

  def __init__(self, CP, packer, tesla_can):
    self.CP = CP
    self.packer = packer
    self.tesla_can = tesla_can

    self.IC_integration_counter = 0
    self.IC_integration_warning_counter = 0
    self.IC_previous_enabled = False
    self.leftLaneQuality = 0
    self.rightLaneQuality = 0
    self._path_pinv = compute_path_pinv()
    self.leadsData = None
    self.engageable = False
    self.prev_autopilot_enabled = False

  def _can_chassis_bus(self):
    """Map CarFingerprint → chassis bus number. PreAP = 0."""
    return CAN_CHASSIS.get(self.CP.carFingerprint, 0)

  def showLeadCarOnICCanMessage(self, leadsData, curv0):
    """Build DAS_object (0x309) with lead-car data from radarState.
    Tinkla 1:1 port; lead-car is NOT stubbed (vision-fallback gives data)."""
    lead_1 = leadsData.leadOne
    lead_2 = leadsData.leadTwo
    if (lead_1 is not None) and lead_1.status:
      self.leadDx = clip(lead_1.dRel, 0, 126)
      self.leadDy = clip(curv0 - lead_1.yRel, -22.05, 22.4)
      self.leadId = 1
      self.leadClass = 2
      self.leadVx = clip(int(lead_1.vRel), -30, 26)
    else:
      self.leadDx = 0
      self.leadDy = 0.0
      self.leadClass = 0
      self.leadId = 0
      self.leadVx = 0
    if (lead_2 is not None) and lead_2.status:
      self.lead2Dx = clip(lead_2.dRel, 0, 126)
      self.lead2Dy = clip(curv0 - lead_2.yRel, -22.05, 22.4)
      self.lead2Id = 2
      self.lead2Class = 2
      self.lead2Vx = clip(int(lead_2.vRel), -30, 26)
    else:
      self.lead2Dx = 0
      self.lead2Dy = 0.0
      self.lead2Class = 0
      self.lead2Id = 0
      self.lead2Vx = 0
    return self.tesla_can.create_lead_car_object_message(
      0,  # lead vehicle objectId multiplexer (m0)
      self.leadClass, self.leadId, 0,
      self.leadDx, self.leadVx, self.leadDy,
      self.lead2Class, self.lead2Id, 0,
      self.lead2Dx, self.lead2Vx, self.lead2Dy,
      self._can_chassis_bus(),
    )

  def get_path_length_idx(self, y, distance):
    i = 0
    for val in y:
      if val < distance:
        i = i + 1
    return i

  def update(self, controls_state, enabled, CS, frame, actuators, cruise_cancel,
             hud_alert, audible_alert, left_line, right_line, lead,
             left_lane_depart, right_lane_depart, human_control,
             radar_state, lat_plan, apply_angle, model_data):
    """Main update hook. Called every tick from CarController.

    Returns: list of CAN-messages (3-tuples (addr, bytes, bus)) to TX on bus 0.
    """
    # Always increment counter and decrement warning-counter, regardless of
    # enableICIntegration — Tinkla pattern.
    self.IC_integration_counter = ((self.IC_integration_counter + 1) % 100)
    if self.IC_integration_warning_counter > 0:
      self.IC_integration_warning_counter = self.IC_integration_warning_counter - 1

    messages = []

    if controls_state is not None:
      self.engageable = controls_state.selfdriveState.engageable

    if model_data is not None:
      CS.laneWidth = 4.0  # lat_plan.lateralPlan.laneWidth — fallback constant
      CS.lProb = model_data.modelV2.laneLineProbs[1]
      CS.rProb = model_data.modelV2.laneLineProbs[2]
      CS.lLine = 1 if CS.lProb > 0.45 else 0
      CS.rLine = 1 if CS.rProb > 0.45 else 0
      self.leftLaneQuality = 1 if model_data.modelV2.laneLineProbs[0] > 0.25 else 0
      self.rightLaneQuality = 1 if model_data.modelV2.laneLineProbs[3] > 0.25 else 0
      y = np.array(model_data.modelV2.position.y)
      x = np.array(model_data.modelV2.position.x)
      max_distance = 100.0
      # V59 2026-05-21: fjernet leadsData-clamp-blokk (Agent J H1).
      # self.leadsData oppdateres på line ~336 ETTER polyfit, så på tick N+
      # brukes tick (N-1)'s radarState. Modern openpilot's vision-fallback gir
      # leadOne.status=True ofte med små dRel → max_distance saturerer mot 0
      # → max_idx=0 → polyfit-branch hoppes over → CS.curvC* fryses ved verdiene
      # fra tick 0 (eller init 0.0). Forklarer "første kurve så flat"-mønsteret.
      # Tinkla har samme kode men annen radar-status-semantikk (0.6.6 = ekte
      # radar only, ikke vision-fallback). Sikker å fjerne for NAP.
      max_idx = self.get_path_length_idx(y, max_distance)
      if (max_idx > 0) and (len(x) >= max_idx):
        order = 3
        coefs = np.polyfit(x[:max_idx], y[:max_idx], order)
        # IC shows the path 2x scaled
        f = 1 / IC_LANE_SCALE
        # V56 2026-05-20: revertert til Tinkla-paritet (True). V55-eksperimentet
        # (enable c1) var feil hypotese — rotårsak var DBC-mangel, ikke c1.
        suppress_x_coord = True
        f2 = f * f
        f3 = f2 * f
        CS.curvC0 = clip(coefs[3], -3.5, 3.5)
        CS.curvC1 = clip(coefs[2] * f * (0 if suppress_x_coord else 1), -0.2, 0.2)
        CS.curvC2 = clip(coefs[1] * f2, -0.0025, 0.0025)
        CS.curvC3 = clip(coefs[0] * f3, -0.00003, 0.00003)

    # IC warning aggregation. Warning latch: 2s after any DAS_*-warning bit.
    warnings = (
      CS.DAS_gas_to_resume + CS.DAS_025_steeringOverride + CS.DAS_202_noisyEnvironment
      + CS.DAS_206_apUnavailable + CS.DAS_207_lkasUnavailable + CS.DAS_208_rackDetected
      + CS.DAS_211_accNoSeatBelt + CS.DAS_216_driverOverriding
      + CS.DAS_219_lcTempUnavailableSpeed + CS.DAS_220_lcTempUnavailableRoad
      + CS.DAS_221_lcAborting + CS.DAS_222_accCameraBlind
      + CS.stopSignWarning + CS.stopLightWarning + CS.DAS_canErrors + CS.DAS_notInDrive
    )
    if (warnings > 0) and (self.IC_integration_warning_counter == 0):
      self.IC_integration_warning_counter = 200  # alert for 2 seconds
    if (self.IC_integration_warning_counter == 0) or (not enabled):
      # When zero, reset all warning bits.
      CS.DAS_gas_to_resume = 0
      CS.DAS_025_steeringOverride = 0
      CS.DAS_202_noisyEnvironment = 0
      CS.DAS_206_apUnavailable = 0
      CS.DAS_207_lkasUnavailable = 0
      CS.DAS_208_rackDetected = 0
      CS.DAS_211_accNoSeatBelt = 0
      CS.DAS_216_driverOverriding = 0
      CS.DAS_219_lcTempUnavailableSpeed = 0
      CS.DAS_220_lcTempUnavailableRoad = 0
      CS.DAS_221_lcAborting = 0
      CS.DAS_222_accCameraBlind = 0
      CS.stopSignWarning = 0
      CS.stopLightWarning = 0
      CS.DAS_canErrors = 0
      CS.DAS_notInDrive = 0

    # ALCA state (stubbed to 0/1 — see module docstring).
    alcaState = CS.alca_direction if (CS.alca_pre_engage or CS.alca_engaged) and CS.alca_direction > 0 else 0
    DAS_ldwStatus = 1 if left_lane_depart or right_lane_depart else 0
    DAS_hands_on_state = 2
    # Steering-required also used by ALCA.
    if (hud_alert == VisualAlert.steerRequired) and not (CS.alca_engaged or CS.alca_pre_engage):
      if audible_alert == AudibleAlert.none:
        DAS_hands_on_state = 3
      else:
        DAS_hands_on_state = 5
    # Manual steering override flashes the hands-on-icon.
    if enabled and human_control:
      DAS_hands_on_state = 3
    DAS_collision_warning = 1 if hud_alert == VisualAlert.fcw else 0

    # alcaState code:  10=ALC_IN_PROGRESS_R, 9=ALC_IN_PROGRESS_L, 8=AVAILABLE_BOTH,
    #                   7=AVAILABLE_ONLY_R, 6=AVAILABLE_ONLY_L, 1=UNAVAILABLE_NO_LANES
    DAS_alca_state = 1
    if (CS.alca_pre_engage or CS.alca_engaged) and CS.alca_direction > 0:
      DAS_alca_state = 8 + CS.alca_direction
    else:
      if self.leftLaneQuality == 1 and self.rightLaneQuality == 1:
        DAS_alca_state = 8
      elif self.leftLaneQuality == 1:
        DAS_alca_state = 6
      elif self.rightLaneQuality == 1:
        DAS_alca_state = 7
    # AP status: 0=Disabled 1=Unavailable 2=Available 3=Active_nominal,
    #            4=active_restricted 5=active_nav 8=aborting 9=aborted
    #            14=fault 15=SNA
    DAS_op_status = 5 if enabled else 2
    DAS_csaState = 2 if enabled else 1
    if not self.engageable:
      DAS_op_status = 1
      DAS_csaState = 0
    # For non-PreAP cars that have stock AP, mirror real AP-status when disabled.
    # PreAP path always uses computed DAS_op_status (no real AP).
    if (self.CP.carFingerprint != CAR.TESLA_MODEL_S_PREAP) and not enabled:
      DAS_op_status = CS.real_autopilot_status

    # PreAP cruise-speed (display-only). Sveins fork's cruiseState.speed = DI_digitalSpeed
    # (current speed), ikke CC target. Tinkla unified bruker acc_speed_kph som target-
    # tracker; vår fork har ekvivalenten i cs.engagement.pedal_speed_kph (capture'd
    # på stalk-pull i _capture_target_speed). Bruk den som target-kilde.
    target_speed_kph = float(getattr(CS, 'pedal_speed_kph', 0.0))

    # DAS_fusedSpeedLimit populated in PreAP carstate.update_preap() from Tesla DI's
    # UI_gpsVehicleSpeed.UI_mppSpeedLimit (msg 760, chassis bus 0). Tinkla unified
    # pattern. Tesla IC's native road-sign widget consumes 0x399.DAS_fusedSpeedLimit.
    # Fallback (when GPS=0) configured via NAPRoadSignFallbackKph param.
    speed_uom_kph = 1.0
    if CS.speed_units == "MPH":
      speed_uom_kph = CV.KPH_TO_MPH
    v_cruise_pcm = max(0.0, CS.out.vEgo * CV.MS_TO_KPH) * speed_uom_kph
    if CS.cruiseEnabled:
      v_cruise_pcm = max(0.0, target_speed_kph) * speed_uom_kph
    # Tinkla-paritet (HUD_module.py: `cruise_speed = CS.acc_speed_kph * speed_uom_kph`).
    # Tidligere DIAGNOSTIC hardkode 80 fjernet — ga ingen IC-effekt fordi Buddy MITM
    # ikke trigget pga DAS_accState=0 i 0x2B9. Live-sprint 2026-05-25 falsifiserte
    # hypotesen at acc_speed_limit alene styrer MAX-widget.
    cruise_speed = target_speed_kph * CV.KPH_TO_MPH
    DAS_control_speed = v_cruise_pcm
    if CS.carNotInDrive:
      DAS_control_speed = 350.0 / 3.6
    if self.engageable and (not enabled) and cruise_speed == 0:
      cruise_speed = 10

    should_send = (
      enabled or (self.IC_previous_enabled and not enabled) or CS.autopilot_disabled
      or self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP
    )

    if CS.autopilot_enabled:
      # Stock AP is in control of IC — do not interfere.
      self.IC_previous_enabled = enabled
      return messages

    chassis_bus = self._can_chassis_bus()

    # DAS_status + DAS_status2 sent at 2Hz (i.e. every 10 ticks at 100Hz).
    if (self.IC_integration_counter % 10 == 0) or (self.IC_previous_enabled and not enabled) \
       or (self.prev_autopilot_enabled and not CS.autopilot_enabled):
      if CS.enableICIntegration:
        messages.append(self.tesla_can.create_das_status(
          DAS_op_status, DAS_collision_warning, DAS_ldwStatus,
          DAS_hands_on_state, DAS_alca_state,
          CS.out.leftBlindspot, CS.out.rightBlindspot,
          CS.DAS_fusedSpeedLimit, CS.fleet_speed_state, chassis_bus, 1))
        messages.append(self.tesla_can.create_das_status2(
          DAS_csaState, cruise_speed, DAS_collision_warning, chassis_bus, 1))

    self.prev_autopilot_enabled = CS.autopilot_enabled
    if not should_send:
      self.IC_previous_enabled = enabled
      return messages

    if (enabled or CS.autopilot_disabled or self.IC_previous_enabled
        or self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP) \
       and (self.IC_integration_counter % 10 == 0):

      # DAS_bodyControls (1Hz). Ticks 20 and 70 of the 0–99 cycle, plus on disengage edge.
      if (self.IC_integration_counter in [20, 70]) or (self.IC_previous_enabled and not enabled):
        messages.append(self.tesla_can.create_body_controls_message(
          CS.alca_direction, 1 if CS.needs_hazard else 0, chassis_bus, 1))

      if CS.enableICIntegration:
        # Tinkla-paritet: counter=1 hardkodet (HUD_module.py:270).
        messages.append(self.tesla_can.create_lane_message(
          CS.laneWidth,
          1 if CS.alca_engaged else CS.rLine,
          1 if CS.alca_engaged else CS.lLine,
          50, CS.curvC0, CS.curvC1, CS.curvC2, CS.curvC3,
          self.leftLaneQuality, self.rightLaneQuality, chassis_bus, 1))

      if CS.enableICIntegration:
        messages.append(self.tesla_can.create_telemetry_road_info(
          CS.lLine, CS.rLine, self.leftLaneQuality, self.rightLaneQuality,
          alcaState, chassis_bus))

      if radar_state is not None:
        self.leadsData = radar_state.radarState
        if self.leadsData is not None and CS.enableICIntegration:
          messages.append(self.showLeadCarOnICCanMessage(
            leadsData=self.leadsData, curv0=CS.curvC0))

      # DAS_warningMatrix0 at 1Hz (tick 10)
      if (self.IC_integration_counter == 10) or (self.IC_previous_enabled and not enabled):
        if CS.enableICIntegration:
          messages.append(self.tesla_can.create_das_warningMatrix0(
            CS.DAS_canErrors, CS.DAS_025_steeringOverride, CS.DAS_notInDrive, chassis_bus))

      # DAS_warningMatrix1 at 1Hz (tick 20)
      if (self.IC_integration_counter == 20) or (self.IC_previous_enabled and not enabled):
        if CS.enableICIntegration:
          messages.append(self.tesla_can.create_das_warningMatrix1(chassis_bus))

      # DAS_warningMatrix3 at 1Hz (tick 30)
      if (self.IC_integration_counter == 30) or (self.IC_previous_enabled and not enabled):
        if CS.enableICIntegration:
          messages.append(self.tesla_can.create_das_warningMatrix3(
            CS.DAS_gas_to_resume, CS.DAS_211_accNoSeatBelt, CS.DAS_202_noisyEnvironment,
            CS.DAS_206_apUnavailable, CS.DAS_207_lkasUnavailable,
            CS.DAS_219_lcTempUnavailableSpeed, CS.DAS_220_lcTempUnavailableRoad,
            CS.DAS_221_lcAborting, CS.DAS_222_accCameraBlind,
            CS.DAS_208_rackDetected, CS.DAS_216_driverOverriding,
            CS.stopSignWarning, CS.stopLightWarning, chassis_bus))

      # Send fake-DAS (0x659) on PreAP or autopilot-disabled.
      # Plan C 0x2B9: send create_ap1_long_control ONLY when magzu_long_active is False.
      if self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP or CS.autopilot_disabled:
        # 0x659 sent always (independent of enableICIntegration — Tinkla pattern,
        # state-channel bits used by panda even when IC is off).
        #
        # Buddy IC MAX-widget gate (V46 image-grav 2026-05-17 + live-sprint 2026-05-25
        # falsifiserings-funn): Tinkla unified-paritet er kritisk for Buddy MITM-pipeline.
        # AGENT_M byte-diff (2026-05-21) viste NAP V56 sendte adaptive_cruise=0 og
        # acc_speed_kph=0 fordi `enabled`-gate aldri var True under qlog-vinduet.
        # Tinkla LONG_module.py:148-153 setter `adaptive_cruise=1` for PreAP når enten
        # ACC.adaptive eller PCC.pcc_available — i praksis ALLTID for vår fork (vi har
        # pedal). cc_state=2 settes når enabled (Tinkla LONG_module:142), ellers 0/1
        # basert på carNotInDrive.
        buddy_gate_adaptive_cruise = 1  # PreAP-paritet (Tinkla LONG_module:148)
        buddy_gate_cc_state = 2 if enabled else (0 if CS.carNotInDrive else 1)
        buddy_gate_speed_control_enabled = 1 if CS.cruiseEnabled else 0
        # Tinkla unified (HUD_module.py:323): sender `CS.out.cruiseState.speed * MS_TO_KPH`
        # UGATED på enabled, slik at Buddy MAX-widget-gate alltid får meningsfull
        # target-speed. NAP target_speed_kph = engagement.pedal_speed_kph (captured
        # ved stalk-pull, satt til 0 før første pull) — pass ungated her også.
        buddy_gate_acc_speed_kph = max(0.0, target_speed_kph)
        messages.append(self.tesla_can.create_fake_DAS_msg(
          buddy_gate_speed_control_enabled,
          CS.DAS_216_driverOverriding,
          CS.DAS_206_apUnavailable,
          DAS_collision_warning,
          DAS_op_status,
          buddy_gate_acc_speed_kph,
          CS.tap_direction,
          DAS_collision_warning,
          buddy_gate_adaptive_cruise,
          DAS_hands_on_state,
          buddy_gate_cc_state,
          1 if CS.pcc_available else 0,
          DAS_alca_state,
          v_cruise_pcm,
          int(CS.DAS_fusedSpeedLimit),
          apply_angle,
          1 if enabled else 0,
          1 if CS.enablePedal else 0,
          1 if CS.autopilot_disabled else 0,
          chassis_bus,
        ))

        # Plan C: HUD-modul sender 0x2B9 KUN når MagZu long-controller IKKE er aktivt.
        # Når long aktivt sender carcontroller_legacy.create_longitudinal_command 0x2B9
        # med inkrementerende counter på powertrain-bus. Vi sender display-only-frame
        # på chassis-bus 0 med constant counter=1, gated på enableICIntegration.
        magzu_long_active = bool(getattr(CS, 'magzu_long_active', False))
        if CS.enableICIntegration and not magzu_long_active:
          # Tinkla unified-paritet (HUD_module.py:303-313 → teslacan.py:136-157).
          # accState=4 settes når in_drive (NOT gated på openpilot-engaged), accState=3
          # når static_cruise+cruise_enabled. Caller-arg mapping matcher Tinkla:
          #   in_drive       = not CS.carNotInDrive          (Tinkla: `not CS.carNotInDrive`)
          #   static_cruise  = not buddy_gate_adaptive_cruise (Tinkla: `not CS.adaptive_cruise` — alltid False for PreAP)
          #   cruise_enabled = buddy_gate_cc_state > 1        (Tinkla: `CS.cc_state > 1`)
          # set_speed_kph = v_cruise_pcm (target når CC engaged, vEgo ellers — matcher
          # Tinkla DAS_control_speed pattern fra HUD_module.py:308).
          messages.append(self.tesla_can.create_ap1_long_control(
            in_drive=not CS.carNotInDrive,
            static_cruise=not buddy_gate_adaptive_cruise,
            cruise_enabled=buddy_gate_cc_state > 1,
            set_speed_kph=v_cruise_pcm,
            accel_limits=[-1.4, 1.8],
            jerk_limits=[-0.46, 0.476],
            bus=chassis_bus,
            counter=1,
          ))

    self.IC_previous_enabled = enabled
    return messages
