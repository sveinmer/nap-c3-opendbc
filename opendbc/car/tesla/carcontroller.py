import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.tesla.teslacan import TeslaCAN
from opendbc.car.tesla.teslacan_legacy import TeslaCANRaven
from opendbc.car.tesla.values import CarControllerParams, CANBUS, LEGACY_CARS, CAR
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.tesla.preap.carcontroller import PreAPLongController, init_preap_can
from opendbc.car.tesla.preap.stock_cc_spoofer import StockCCSpoofer
from opendbc.car.tesla.preap.hud_module import HUDController

try:
  from cereal import messaging as _messaging
except ImportError:
  _messaging = None


class _EventLikeReader:
  """Wrap a SubMaster reader so hud_module's `data.<field>` access works.

  hud_module expects Event-wrapped readers (matching Tinkla 0.6.6 sub_sock
  pattern: `model_data.modelV2.X`). SubMaster.__getitem__ returns the inner
  reader directly. This tiny wrapper bridges the API gap so we can swap the
  transport (sub_sock → SubMaster) without churning hud_module + its tests.
  """
  __slots__ = ("_field", "_reader")

  def __init__(self, field, reader):
    object.__setattr__(self, "_field", field)
    object.__setattr__(self, "_reader", reader)

  def __getattr__(self, name):
    if name == object.__getattribute__(self, "_field"):
      return object.__getattribute__(self, "_reader")
    raise AttributeError(name)


def get_safety_CP():
  # We use the TESLA_MODEL_Y platform for lateral limiting to match safety
  from opendbc.car.tesla.interface import CarInterface
  return CarInterface.get_non_essential_params("TESLA_MODEL_Y")


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.apply_angle_last = 0
    self.packer = CANPacker(dbc_names[Bus.party])
    self.tesla_can = TeslaCAN(self.packer)

    # Vehicle model used for lateral limiting
    self.VM = VehicleModel(get_safety_CP())

    if CP.carFingerprint in LEGACY_CARS:
      if CP.carFingerprint in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1, CAR.TESLA_MODEL_S_PREAP):
        CANBUS.powertrain = CANBUS.party
        CANBUS.autopilot_powertrain = CANBUS.autopilot_party

      self.packers = {CANBUS.party: CANPacker(dbc_names[Bus.party]), CANBUS.powertrain: CANPacker(dbc_names[Bus.pt])}

      if CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
        self.preap_long = PreAPLongController()
        self.stock_cc = StockCCSpoofer()
        self.tesla_can = init_preap_can(dbc_names, self.packers)
        # Tinkla Buddy IC integration — instantiated for all PreAP cars. Default
        # off (NAPTinklaICIntegration=False) so no IC TX unless toggle set.
        self.hud_controller = HUDController(CP, self.packer, self.tesla_can)
        # SubMaster for HUD payload data. Carcontroller doesn't receive SubMaster
        # from card.py, and v43 attempt using raw sub_sock returned None in 100%
        # of live drive ticks (2476 DAS_status frames TX'd, all byte0=0x01) even
        # though selfdriveState was published 17k+ times with engageable=True.
        # Reason not fully isolated, but SubMaster is card.py's proven cross-
        # process pattern and handles liveness re-binding internally.
        self._hud_sm = None
        if _messaging is not None:
          try:
            self._hud_sm = _messaging.SubMaster(['modelV2', 'radarState', 'selfdriveState'])
          except Exception:
            pass
      else:
        self.tesla_can = TeslaCANRaven(self.packers)

      from opendbc.car.tesla.interface import CarInterface
      self.VM = VehicleModel(CarInterface.get_non_essential_params("TESLA_MODEL_S_HW3"))

  def update(self, CC, CS, now_nanos):
    if self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
      return self._update_preap(CC, CS)

    actuators = CC.actuators
    can_sends = []

    # Tesla EPS enforces disabling steering on heavy lateral override force.
    lat_active = CC.latActive and CS.hands_on_level < 3

    if self.frame % 2 == 0:
      self.apply_angle_last = apply_steer_angle_limits_vm(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw, CS.out.steeringAngleDeg,
                                                          lat_active, CarControllerParams, self.VM)
      if self.CP.carFingerprint in LEGACY_CARS:
        cntr = (self.frame // 2) % 16
        can_sends.append(self.tesla_can.create_steering_control(cntr, self.apply_angle_last, lat_active))
      else:
        can_sends.append(self.tesla_can.create_steering_control(self.apply_angle_last, lat_active))

    if self.frame % 10 == 0:
      if self.CP.carFingerprint in LEGACY_CARS and self.CP.carFingerprint not in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1):
        cntr = (self.frame // 10) % 16
        can_sends.append(self.tesla_can.create_steering_allowed(cntr))
      elif self.CP.carFingerprint not in LEGACY_CARS:
        can_sends.append(self.tesla_can.create_steering_allowed())

    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      if self.frame % 4 == 0:
        state = 13 if CC.cruiseControl.cancel else 4  # ACC_ON / ACC_CANCEL_GENERIC_SILENT
        accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
        cntr = (self.frame // 4) % 8
        can_sends.append(self.tesla_can.create_longitudinal_command(state, accel, cntr, CS.out.vEgo, CC.longActive))
    else:
      if CC.cruiseControl.cancel:
        cntr = (CS.das_control["DAS_controlCounter"] + 1) % 8
        can_sends.append(self.tesla_can.create_longitudinal_command(13, 0, cntr, CS.out.vEgo, False))

    # TODO: HUD control
    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends

  def _update_preap(self, CC, CS):
    actuators = CC.actuators
    can_sends = []

    lat_active = CC.latActive and CS.hands_on_level < 3

    if self.frame % 2 == 0:
      self.apply_angle_last = apply_steer_angle_limits_vm(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw, CS.out.steeringAngleDeg,
                                                          lat_active, CarControllerParams, self.VM)
      cntr = (self.frame // 2) % 16
      can_sends.append(self.tesla_can.create_steering_control(cntr, self.apply_angle_last, lat_active))
      can_sends.append(self.tesla_can.create_epas_control(cntr, 1))

    # Reset pccEvent each tick so it expresses one-frame edge events. Without
    # this the previous frame's value sticks and teslaCC{Engaged,Disengaged}
    # alerts re-trigger indefinitely instead of fading after their alert window.
    CS.pccEvent = None

    # Pedal-mode longitudinal control. Runs only when op-long is on (Comma
    # Pedal present). Writes CS.preap_cc_cancel_needed when pedal mode wants
    # to drop a running stock CC — consumed by self.stock_cc below.
    if self.CP.openpilotLongitudinalControl:
      can_sends.extend(self.preap_long.update(CC, CS, self.frame, self.tesla_can, CANBUS.party))
    elif CC.cruiseControl.cancel:
      can_sends.extend(self.preap_long.send_cancel(CS, self.tesla_can))

    # Stock-CC stalk spoofs (CANCEL / SET_ACCEL). Independent of op-long —
    # the engagement FSM and pedal-mode publish intent through CarState flags
    # and the spoofer is the only TX path for 0x45 STW_ACTN_RQ frames.
    can_sends.extend(self.stock_cc.update(CS, self.frame, self.tesla_can, CANBUS.party))
    if self.stock_cc.pcc_event:
      CS.pccEvent = self.stock_cc.pcc_event

    # Tinkla Buddy IC integration dispatch. Builds 0–10 DAS_*-frames on chassis bus 0.
    # No-op when CS.enableICIntegration=False (default). Independent of long/lateral state.
    # Per HUD-module's own rate-gates (10/2/1Hz at counter ticks). model_data/radar_state/
    # controls_state come from a local SubMaster (see __init__); we wrap readers in
    # _EventLikeReader so hud_module's `data.<field>` access pattern stays unchanged.
    try:
      model_data = None
      radar_state = None
      controls_state = None
      if self._hud_sm is not None:
        try:
          self._hud_sm.update(0)  # non-blocking poll
          if self._hud_sm.seen.get('modelV2', False):
            model_data = _EventLikeReader('modelV2', self._hud_sm['modelV2'])
          if self._hud_sm.seen.get('radarState', False):
            radar_state = _EventLikeReader('radarState', self._hud_sm['radarState'])
          if self._hud_sm.seen.get('selfdriveState', False):
            controls_state = _EventLikeReader('selfdriveState', self._hud_sm['selfdriveState'])
        except Exception:
          # SubMaster polling failure must not propagate into safety-critical TX path.
          pass
      hud_messages = self.hud_controller.update(
        controls_state=controls_state,
        enabled=CC.enabled,
        CS=CS,
        frame=self.frame,
        actuators=actuators,
        cruise_cancel=CC.cruiseControl.cancel,
        hud_alert=CC.hudControl.visualAlert,
        audible_alert=CC.hudControl.audibleAlert,
        left_line=CC.hudControl.leftLaneVisible,
        right_line=CC.hudControl.rightLaneVisible,
        lead=CC.hudControl.leadVisible,
        left_lane_depart=CC.hudControl.leftLaneDepart,
        right_lane_depart=CC.hudControl.rightLaneDepart,
        human_control=CS.out.steeringPressed,
        radar_state=radar_state,
        lat_plan=None,
        apply_angle=self.apply_angle_last,
        model_data=model_data,
      )
      can_sends.extend(hud_messages)
    except Exception:
      # IC rendering is display-only (risk-tier 3). Never propagate exceptions
      # into the carcontroller TX path — they could disrupt the safety-critical
      # steering frames just above.
      pass

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
