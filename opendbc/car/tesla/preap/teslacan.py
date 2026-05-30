import struct
import crcmod
from ctypes import create_string_buffer
from opendbc.car.tesla.teslacan_legacy import TeslaCANRaven
from opendbc.car.tesla.values import CANBUS

# IC-rendering arb-IDs for Tinkla Buddy integration (sendt på chassis bus 0)
DAS_LANES_MSG_ID = 0x239           # 569
DAS_OBJECT_MSG_ID = 0x309          # 777
DAS_WARNING_MATRIX_0_MSG_ID = 0x329  # raw struct.pack
DAS_WARNING_MATRIX_1_MSG_ID = 0x369  # raw struct.pack
DAS_WARNING_MATRIX_3_MSG_ID = 0x349  # raw struct.pack
DAS_STATUS2_MSG_ID = 0x389         # 905
DAS_STATUS_MSG_ID = 0x399          # 921
DAS_TELEMETRY_MSG_ID = 0x3A9       # 937
DAS_BODYCONTROLS_MSG_ID = 0x3E9    # 1001
DAS_FAKE_UDS_MSG_ID = 0x659        # raw byte-array
DAS_CONTROL_MSG_ID = 0x2B9         # AP1 long-control (Plan C: HUD gates via magzu_long_active)

# Comma Pedal protocol constants
PEDAL_M1 = 0.050796813    # Primary scaling factor
PEDAL_M2 = 0.101593626    # Secondary scaling (2x M1 for redundancy)
PEDAL_D = -22.85856576    # Offset
GAS_COMMAND_ID = 0x551

# Default STW_ACTN_RQ signal values (all signals except counter, CRC, and button).
# VSL_Enbl_Rq=1 matches what the driver's stalk module emits — the DI rejects /
# anomalously interprets frames with bit 6 of byte 0 cleared. Was previously a
# typo ("VSL_Enbl_Stat", which is not a DBC signal) — silently dropped by the
# packer, so the bit always read 0. Ported from NotAutopilot/opendbc f867b3a4.
_STW_DEFAULTS = {
  "VSL_Enbl_Rq": 1, "DTR_Dist_Rq": 0, "TurnIndLvr_Stat": 0,
  "HiBmLvr_Stat": 0, "WprWashSw_Psd": 0, "WprWash_R_Sw_Posn_V2": 0,
  "StW_Lvr_Stat": 0, "StW_Cond_Flt": 0, "StW_Cond_Psd": 0,
  "HrnSw_Psd": 0, "StW_Sw00_Psd": 0, "StW_Sw01_Psd": 0,
  "StW_Sw02_Psd": 0, "StW_Sw03_Psd": 0, "StW_Sw04_Psd": 0,
  "StW_Sw05_Psd": 0, "StW_Sw06_Psd": 0, "WprSw6Posn": 0,
}


class TeslaCANPreAP(TeslaCANRaven):
  def __init__(self, packers):
    super().__init__(packers)
    self.pedal_can_bus = 2
    # Pedal firmware watchdog requires consecutive counter values
    self.pedal_idx = 0
    # STW_ACTN_RQ uses CRC-8 (poly 0x1D), not the byte-sum checksum
    self.stw_crc = crcmod.mkCrcFun(0x11d, initCrc=0x00, rev=False, xorOut=0xff)
    # IC frame counters — Tesla IC rejects frames without rolling 4-bit counter + valid byte-sum checksum.
    self.das_status_idx = 0
    self.das_status2_idx = 0

  @staticmethod
  def pedal_checksum(msg_id, dat):
    """Comma Pedal CAN checksum: addr bytes + data bytes, truncated to 8 bits."""
    ret = (msg_id & 0xFF) + ((msg_id >> 8) & 0xFF)
    ret += sum(dat)
    return ret & 0xFF

  def create_pedal_command(self, accel_command, enable=1, pedal_can_bus=None):
    """Build GAS_COMMAND (0x551) using raw struct packing for firmware byte-compatibility."""
    if pedal_can_bus is None:
      pedal_can_bus = self.pedal_can_bus

    idx = self.pedal_idx
    self.pedal_idx = (self.pedal_idx + 1) % 16

    if enable == 1:
      int_cmd1 = max(0, min(65534, int((accel_command - PEDAL_D) / PEDAL_M1)))
      int_cmd2 = max(0, min(65534, int((accel_command - PEDAL_D) / PEDAL_M2)))
    else:
      int_cmd1 = 0
      int_cmd2 = 0

    msg = create_string_buffer(6)
    struct.pack_into("BBBBB", msg, 0,
                     (int_cmd1 >> 8) & 0xFF, int_cmd1 & 0xFF,
                     (int_cmd2 >> 8) & 0xFF, int_cmd2 & 0xFF,
                     ((enable << 7) + idx) & 0xFF)
    struct.pack_into("B", msg, 5, self.pedal_checksum(GAS_COMMAND_ID, msg.raw))

    return (GAS_COMMAND_ID, bytes(msg.raw), pedal_can_bus)

  def create_epas_control(self, counter, mode):
    values = {
      "EPB_epasEACAllow": mode,
      "EPB_epasControlCounter": counter,
      "EPB_epasControlChecksum": 0,
    }
    data = self.packers[CANBUS.party].make_can_msg("EPB_epasControl", CANBUS.party, values)[1]
    values["EPB_epasControlChecksum"] = self.checksum(0x214, data)
    return self.packers[CANBUS.party].make_can_msg("EPB_epasControl", CANBUS.party, values)

  def create_action_request(self, button_to_press, bus, counter, msg_stw=None):
    """Build STW_ACTN_RQ to simulate cruise stalk button press."""
    values = {"MC_STW_ACTN_RQ": counter, "CRC_STW_ACTN_RQ": 0, "SpdCtrlLvr_Stat": button_to_press}
    if msg_stw is not None:
      for key, default in _STW_DEFAULTS.items():
        values[key] = msg_stw.get(key, default)
    else:
      values.update(_STW_DEFAULTS)

    data = self.packers[CANBUS.party].make_can_msg("STW_ACTN_RQ", bus, values)[1]
    values["CRC_STW_ACTN_RQ"] = self.stw_crc(data[:7])
    return self.packers[CANBUS.party].make_can_msg("STW_ACTN_RQ", bus, values)

  # =================================================================
  # IC-rendering builders (Tinkla Buddy port, Sprint NAP_FIX_TINKLA_BUDDY_IC_PORT).
  # All built either via CANPacker against tesla_preap.dbc (6 DBC-defined),
  # or via raw struct.pack for the 4 not in DBC (0x329/0x349/0x369/0x659).
  # Bus: chassis bus 0 (CANBUS.party for PreAP).
  # Ported 1:1 from /home/svein/repos/Tinkla/selfdrive/car/tesla/teslacan.py
  # via /home/svein/repos/nap-replay-rig/rig/teslacan.py (bit-exact verified).
  # =================================================================

  def create_lane_message(self, lWidth, rLine, lLine, laneRange, curvC0, curvC1, curvC2, curvC3,
                          lLane2, rLane2, bus, counter):
    """DAS_lanes (0x239) — virtual lane geometry for IC path rendering, 10Hz."""
    values = {
      "DAS_leftLaneExists": lLine,
      "DAS_rightLaneExists": rLine,
      "DAS_virtualLaneWidth": lWidth,
      "DAS_virtualLaneViewRange": laneRange,
      "DAS_virtualLaneC0": curvC0,
      "DAS_virtualLaneC1": curvC1,
      "DAS_virtualLaneC2": curvC2,
      "DAS_virtualLaneC3": curvC3,
      "DAS_leftLineUsage": lLine * 2,
      "DAS_rightLineUsage": rLine * 2,
      "DAS_leftFork": lLane2,
      "DAS_rightFork": rLane2,
      "DAS_lanesCounter": counter,
    }
    return self.packers[CANBUS.party].make_can_msg("DAS_lanes", bus, values)

  def create_lead_car_object_message(self, objectId, vType1, vId1, relevant1, dx1, vxrel1, dy1,
                                      vType2, vId2, relevant2, dx2, vxrel2, dy2, bus):
    """DAS_object (0x309) — lead-car position+velocity for IC, 10Hz.

    Sender m0 (DAS_objectId=0) for lead-vehicle-frames. Tinkla bygger kun m0;
    notautopilot DBC har utvidet multiplexer for m1-m5 men vi bruker bare m0.
    """
    values = {
      "DAS_objectId": objectId,  # 0 = Lead vehicles
      "DAS_leadVehType": vType1,
      "DAS_leadVehRelevantForControl": relevant1,
      "DAS_leadVehDx": dx1,
      "DAS_leadVehVxRel": vxrel1,
      "DAS_leadVehDy": dy1,
      "DAS_leadVehId": vId1,
      "DAS_leadVeh2Type": vType2,
      "DAS_leadVeh2RelevantForControl": relevant2,
      "DAS_leadVeh2Dx": dx2,
      "DAS_leadVeh2VxRel": vxrel2,
      "DAS_leadVeh2Dy": dy2,
      "DAS_leadVeh2Id": vId2,
    }
    return self.packers[CANBUS.party].make_can_msg("DAS_object", bus, values)

  def create_body_controls_message(self, turn, hazard, bus, counter):
    """DAS_bodyControls (0x3E9) — turn signals + hazards for IC, 1Hz.

    NB: Tinkla holder `counter`-argument men hardkoder DAS_bodyControlsCounter=1
    in body (Tinkla source). Vi følger samme mønster.
    """
    values = {
      "DAS_headlightRequest": 0,
      "DAS_hazardLightRequest": 0,
      "DAS_wiperSpeed": 0,
      "DAS_turnIndicatorRequest": 0,
      "DAS_highLowBeamDecision": 3,
      "DAS_highLowBeamOffReason": 5,
      "DAS_turnIndicatorRequestReason": 0,
      "DAS_bodyControlsCounter": 1,
      "DAS_bodyControlsChecksum": 0,
    }
    values["DAS_hazardLightRequest"] = hazard
    values["DAS_turnIndicatorRequest"] = turn  # 0=off, 1=left, 2=right
    if turn > 0:
      values["DAS_turnIndicatorRequestReason"] = 1
    else:
      values["DAS_turnIndicatorRequestReason"] = 0
    return self.packers[CANBUS.party].make_can_msg("DAS_bodyControls", bus, values)

  def create_telemetry_road_info(self, lLine, rLine, lLineQualRaw, rLineQualRaw, alcaState, bus):
    """DAS_telemetry (0x3A9) — road-info marker types/colors/quality, 1Hz.

    alcaState: 0=none, 1=alca-left, 2=alca-right.
    Bygger kun m0 (DAS_telemetryMultiplexer=0).
    """
    rLineType = 1 if rLine == 1 else 7
    rLineColor = 2 if rLine == 1 else 0
    rLineQual = 3 if rLine == 1 else 0
    if rLineQualRaw == 1:
      rLineType = 3
      rLineColor = 1
    lLineType = 1 if lLine == 1 else 7
    lLineColor = 2 if lLine == 1 else 0
    lLineQual = 3 if lLine == 1 else 0
    if lLineQualRaw == 1:
      lLineType = 3
      lLineColor = 1
    values = {
      "DAS_telemetryMultiplexer": 0,
      "DAS_telLeftLaneType": lLineType,
      "DAS_telRightLaneType": rLineType,
      "DAS_telLeftMarkerQuality": lLineQual,
      "DAS_telRightMarkerQuality": rLineQual,
      "DAS_telLeftMarkerColor": lLineColor,
      "DAS_telRightMarkerColor": rLineColor,
      "DAS_telLeftLaneCrossing": 0 if alcaState != 1 else 1,
      "DAS_telRightLaneCrossing": 0 if alcaState != 2 else 1,
    }
    return self.packers[CANBUS.party].make_can_msg("DAS_telemetry", bus, values)

  def create_das_warningMatrix0(self, DAS_canErrors, DAS_025_steeringOverride, DAS_notInDrive, bus):
    """warningMatrix0 (0x329) — raw struct.pack, 1Hz. Not in DBC."""
    msg = create_string_buffer(8)
    struct.pack_into("BBBBBBBB", msg, 0,
                     0, 0, 0, DAS_025_steeringOverride + (DAS_canErrors << 7),
                     0, (DAS_notInDrive << 7), 0, 0)
    return (DAS_WARNING_MATRIX_0_MSG_ID, bytes(msg.raw), bus)

  def create_das_warningMatrix1(self, bus):
    """warningMatrix1 (0x369) — raw struct.pack, all zeros (constant), 1Hz."""
    msg = create_string_buffer(8)
    struct.pack_into("BBBBBBBB", msg, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    return (DAS_WARNING_MATRIX_1_MSG_ID, bytes(msg.raw), bus)

  def create_das_warningMatrix3(self, DAS_gas_to_resume, DAS_211_accNoSeatBelt, DAS_202_noisyEnvironment,
                                  DAS_206_apUnavailable, DAS_207_lkasUnavailable,
                                  DAS_219_lcTempUnavailableSpeed, DAS_220_lcTempUnavailableRoad,
                                  DAS_221_lcAborting, DAS_222_accCameraBlind,
                                  DAS_208_rackDetected, DAS_w216_driverOverriding,
                                  stopSignWarning, stopLightWarning, bus):
    """warningMatrix3 (0x349) — raw struct.pack, 1Hz. Not in DBC."""
    msg = create_string_buffer(8)
    struct.pack_into("BBBBBBBB", msg, 0,
                     (DAS_gas_to_resume << 1) + (stopSignWarning << 3) + (stopLightWarning << 4),
                     (DAS_202_noisyEnvironment << 1) + (DAS_206_apUnavailable << 5)
                       + (DAS_207_lkasUnavailable << 6) + (DAS_208_rackDetected << 7),
                     (DAS_211_accNoSeatBelt << 2) + (DAS_w216_driverOverriding << 7),
                     (DAS_219_lcTempUnavailableSpeed << 2) + (DAS_220_lcTempUnavailableRoad << 3)
                       + (DAS_221_lcAborting << 4) + (DAS_222_accCameraBlind << 5),
                     0, 0, 0, 0)
    return (DAS_WARNING_MATRIX_3_MSG_ID, bytes(msg.raw), bus)

  def create_das_status(self, DAS_op_status, DAS_collision_warning,
                         DAS_ldwStatus, DAS_hands_on_state, DAS_alca_state,
                         blindSpotLeft, blindSpotRight,
                         DAS_speed_limit_kph, DAS_fleetSpeedState, bus, counter):
    """DAS_status (0x399) — AP-status, blind-spot, fleet-speed for IC, 2Hz."""
    values = {
      "DAS_autopilotState": DAS_op_status,
      "DAS_blindSpotRearLeft": 1 if blindSpotLeft else 0,
      "DAS_blindSpotRearRight": 1 if blindSpotRight else 0,
      "DAS_fusedSpeedLimit": DAS_speed_limit_kph,
      "DAS_suppressSpeedWarning": 0,
      "DAS_summonObstacle": 0,
      "DAS_summonClearedGate": 0,
      "DAS_visionOnlySpeedLimit": DAS_speed_limit_kph,
      "DAS_heaterState": 0,
      "DAS_forwardCollisionWarning": DAS_collision_warning,
      "DAS_autoparkReady": 0,
      "DAS_autoParked": 0,
      "DAS_autoparkWaitingForBrake": 0,
      "DAS_summonFwdLeashReached": 0,
      "DAS_summonRvsLeashReached": 0,
      "DAS_sideCollisionAvoid": 0,
      "DAS_sideCollisionWarning": 0,
      "DAS_sideCollisionInhibit": 0,
      "DAS_lssState": 0,
      "DAS_laneDepartureWarning": DAS_ldwStatus,
      "DAS_fleetSpeedState": DAS_fleetSpeedState,
      "DAS_autopilotHandsOnState": DAS_hands_on_state,
      "DAS_autoLaneChangeState": DAS_alca_state,
      "DAS_summonAvailable": 0,
      "DAS_statusCounter": self.das_status_idx,
      "DAS_statusChecksum": 0,
    }
    self.das_status_idx = (self.das_status_idx + 1) % 16
    # V58 revert V57: IC validerer checksum, hardkodet 0 brakk hele DAS_status
    # (regresjon: skilt-widget + kantlinjer borte). Tinklas `: 0` overrides
    # antagelig av CANPacker auto-checksum via DBC-konvensjon, mens vår mangler.
    data = self.packers[CANBUS.party].make_can_msg("DAS_status", bus, values)[1]
    values["DAS_statusChecksum"] = self.checksum(DAS_STATUS_MSG_ID, data[:7])
    return self.packers[CANBUS.party].make_can_msg("DAS_status", bus, values)

  def create_das_status2(self, DAS_csaState, DAS_acc_speed_limit, fcw, bus, counter):
    """DAS_status2 (0x389) — CSA-state + ACC-speed-limit + FCW, 2Hz."""
    fcw_sig = 0x0F if fcw == 0 else 0x01
    values = {
      "DAS_accSpeedLimit": DAS_acc_speed_limit,
      "DAS_pmmObstacleSeverity": 0,
      "DAS_pmmLoggingRequest": 0,
      "DAS_activationFailureStatus": 0,
      "DAS_pmmUltrasonicsFaultReason": 0,
      "DAS_pmmRadarFaultReason": 0,
      "DAS_pmmSysFaultReason": 0,
      "DAS_pmmCameraFaultReason": 0,
      "DAS_ACC_report": 1,
      "DAS_csaState": DAS_csaState,
      "DAS_radarTelemetry": 1,
      "DAS_robState": 2,
      "DAS_driverInteractionLevel": 0,
      "DAS_ppOffsetDesiredRamp": 0x80,
      "DAS_longCollisionWarning": fcw_sig,
      "DAS_status2Counter": self.das_status2_idx,
      "DAS_status2Checksum": 0,
    }
    self.das_status2_idx = (self.das_status2_idx + 1) % 16
    # V58 revert V57: IC validerer checksum, restorerer computed checksum.
    data = self.packers[CANBUS.party].make_can_msg("DAS_status2", bus, values)[1]
    values["DAS_status2Checksum"] = self.checksum(DAS_STATUS2_MSG_ID, data[:7])
    return self.packers[CANBUS.party].make_can_msg("DAS_status2", bus, values)

  def create_fake_DAS_msg(self, speed_control_enabled, speed_override, apUnavailable,
                           collision_warning, op_status, acc_speed_kph,
                           turn_signal_needed, forward_collission_warning,
                           adaptive_cruise, hands_on_state, cc_state, pcc_available,
                           alca_state, acc_speed_limit, legal_speed_limit, apply_angle,
                           enable_steer_control, pedalEnabled, autopilot_disabled, bus):
    """fake DAS message (0x659) — Buddy-fallback + panda state-channel, 1Hz.

    Byte 5 has dual purpose: legal_speed_limit (0:5) + pedalEnabled (bit5) + autopilot_disabled (bit7).
    Tinkla sender denne uavhengig av enableICIntegration (matches Tinkla pattern).
    """
    units_included = 1
    c_apply_steer = int(
      ((int(apply_angle * 10 + 0x4000)) & 0x7FFF) + (enable_steer_control << 15)
    )
    dat = [
      int(
        (speed_control_enabled << 7)
        + (speed_override << 6)
        + (apUnavailable << 5)
        + (collision_warning << 4)
        + op_status
      ),
      int(acc_speed_kph),
      int(
        (turn_signal_needed << 6)
        + (units_included << 5)
        + (forward_collission_warning << 4)
        + (adaptive_cruise << 3)
        + hands_on_state
      ),
      int((cc_state << 6) + (pcc_available << 5) + alca_state),
      int(acc_speed_limit + 0.5),
      int(
        (legal_speed_limit & 0x1F) + ((pedalEnabled << 5) & 0x20) + ((autopilot_disabled << 7) & 0x80)
      ),
      int(c_apply_steer & 0xFF),
      int((c_apply_steer >> 8) & 0xFF)
    ]
    return (DAS_FAKE_UDS_MSG_ID, bytes(dat), bus)

  def create_ap1_long_control(self, in_drive, static_cruise, cruise_enabled,
                                set_speed_kph, accel_limits, jerk_limits,
                                bus, counter):
    """DAS_control (0x2B9) IC-display variant — Tinkla unified-paritet.

    Ported 1:1 fra `/home/svein/repos/Tinkla/selfdrive/car/tesla/teslacan.py`
    `create_ap1_long_control` (linje 136-157). Buddy MITM-pipeline trigger
    krever Tinkla-paritet på DAS_accState — accState=4 (ACC_ON) når bilen er
    i drive (NOT gated på op_engaged), accState=3 (HOLD) ved static_cruise.

    Tidligere NAP-variant (op_engaged-gated) sendte accState=0 100% av tida
    fordi NAP-rlogen V56 hadde enabled=False under hele drive. Live-sprint
    2026-05-25 bekreftet at Buddy/Tesla IC viser MCU1-konstant så lenge
    DAS_accState=0 ankommer fra panda-siden. Se docs/NAP_HANDOVER_2026_05_25_BUDDY_LIVE_SPRINT_COMPLETE.md.
    """
    accState = 0
    if in_drive:
      accState = 4
      if static_cruise and cruise_enabled:
        accState = 3
    values = {
      "DAS_setSpeed": max(0, min(set_speed_kph, 200)),
      "DAS_accState": accState,
      "DAS_aebEvent": 0,
      "DAS_jerkMin": max(-8., min(jerk_limits[0], 8.)),
      "DAS_jerkMax": max(-8., min(jerk_limits[1], 8.)),
      "DAS_accelMin": max(-12., min(accel_limits[0], 3.44)),
      "DAS_accelMax": max(-12., min(accel_limits[1], 3.44)),
      "DAS_controlCounter": counter,
    }
    data = self.packers[CANBUS.party].make_can_msg("DAS_control", bus, values)[1]
    values["DAS_controlChecksum"] = self.checksum(DAS_CONTROL_MSG_ID, data[:7])
    return self.packers[CANBUS.party].make_can_msg("DAS_control", bus, values)
