from opendbc.can.parser import CANParser
from opendbc.car import structs
from opendbc.car.hyundai.values import DBC
#from cereal import car
from cereal.messaging import SubMaster
#import math

from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

ESCC_MSG = 0x2AB


class EnhancedSmartCruiseControl:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

  @property
  def enabled(self):
    return self.CP_SP.flags & HyundaiFlagsSP.ENHANCED_SCC

  @property
  def trigger_msg(self):
    return ESCC_MSG

  def update_car_state(self, car_state):
    """
      This method is invoked by the CarController to update the car state on the ESCC object.
      The updated state is then used to update SCC12 with the current car state values received through ESCC.
      :param car_state:
      :return:
    """
    self.car_state = car_state

  def update_scc12(self, values):
    """
      Update SCC12 with the current car state values received through ESCC.
      These values are sourced directly from the car's SCC radar and provide a more reliable source for AEB and FCA alerts.
      :param values: SCC12 to be sent in dictionary form before being packed
      :return: Nothing. SCC12 is updated in place.
    """
    values["AEB_CmdAct"] = self.car_state.escc_cmd_act
    values["CF_VSM_Warn"] = self.car_state.escc_aeb_warning
    values["CF_VSM_DecCmdAct"] = self.car_state.escc_aeb_dec_cmd_act
    values["CR_VSM_DecCmd"] = self.car_state.escc_aeb_dec_cmd
    # TODO-SP: we should read it from the car's settings and use that value.
    #  It may not be ideal to set this here directly.
    #  Observed flickering on the dashboard settings switching between "deactivated" and "active assistance" when sending AEB_Status = 1.
    #  These values could differ from the user's configuration from the car's settings.
    #  This indicates that SCC12 likely displays it on the dashboard, and another FCA message may also cause it to appear.
    values["AEB_Status"] = 2  # AEB enabled

  def get_radar_can_parser(self):
    lead_src, bus = "ESCC", 0
    messages = [(lead_src, 50)]
    return CANParser(DBC[self.CP.carFingerprint]['pt'], messages, bus)


class EsccCarStateBase:
  def __init__(self):
    self.escc_aeb_warning = 0
    self.escc_aeb_dec_cmd_act = 0
    self.escc_cmd_act = 0
    self.escc_aeb_dec_cmd = 0


class EsccCarController:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.ESCC = EnhancedSmartCruiseControl(CP, CP_SP)

  def update(self, car_state):
    self.ESCC.update_car_state(car_state)


class EsccRadarInterfaceBase:
  rcp: CANParser
  pts: dict[int, structs.RadarData.RadarPoint]

  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.ESCC = EnhancedSmartCruiseControl(CP, CP_SP)
    self.track_id = 0
    self.use_escc = False
    self.previous = 160
    self.prev_vRel = 0.0  # Add to store previous vRel
    self.sm = SubMaster(["deviceState", "carState", "roadCameraState", "liveCalibration", "driverMonitoringState", "carControl"])
    self.smoothing = 0.8
    self.smoothvRel = 0.0

  def update_escc(self, ret):
    for ii in range(1):
      msg_src = "ESCC"
      msg = self.rcp.vl[msg_src]

      if ii not in self.pts:
        self.pts[ii] = structs.RadarData.RadarPoint()
        self.pts[ii].trackId = self.track_id
        self.track_id += 1

      dRel = msg['ACC_ObjDist']  # Relative distance to lead car
      rSpd = msg['ACC_ObjRelSpd']  # Relative speed to lead car (km/h)

      # Fetch vehicle speed from CAN data (CLU15 message)
      try:
        self.sm.update(0)
      # Average all four wheel speeds for vEgo
        vEgo_km = self.sm["carState"].vEgo * 3.3 #hspeed.vEgo
        print(f"Car Speed: {vEgo_km}")
      except KeyError as e:
        # Fallback if signal isn’t available
        vEgo_km = 0.0
        print(f"KeyError: {e}")

      # Calculate lead car's absolute speed
      vLead = vEgo_km + rSpd


      # Validity check
      valid = False
      if msg['ACC_ObjStatus']:
        if dRel <= self.previous:  # Lead car moving > 45 km/h
          valid = True

        self.previous = dRel
      if self.smoothvRel == 0.0:
        self.smoothvRel = rSpd
      else:
        self.smoothvRel = (self.smoothing * rSpd + (1 - self.smoothing) * self.smoothvRel)


      if vEgo_km <= 20:
        valid = True

      # dRel = msg['ACC_ObjDist']

      if valid:
        #increment = 1 + math.log1p(dRel)
        increment = (dRel * 0.2)
        fn = dRel + increment
        if dRel >= 150:
          fn = 3
        self.pts[ii].measured = True if vEgo_km >= 40 else False
        self.pts[ii].dRel = fn #msg['ACC_ObjDist']
        self.pts[ii].yRel = -msg['ACC_ObjLatPos']
        self.pts[ii].vRel = self.smoothvRel #msg['ACC_ObjRelSpd']  # km/h
        # Calculate aRel
        vRel_mps = self.pts[ii].vRel #/ 3.6  # Convert km/h to m/s
        aRel = (vRel_mps - self.prev_vRel) / 0.05  # 50 Hz = 0.02 s
        self.pts[ii].aRel = aRel  # m/s²
        self.prev_vRel = vRel_mps  # Update previous vRel
        self.pts[ii].yvRel = float('nan')
        print(f"Lead Distance: {fn}")
      else:
        del self.pts[ii]

    ret.points = list(self.pts.values())
    return ret
