from opendbc.can.parser import CANParser
from opendbc.car import structs
from opendbc.car.hyundai.values import DBC

from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP
from cereal.messaging import SubMaster

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
    self.sm = SubMaster(["carState"])
    self.track_id = 0
    self.use_escc = False
    self.previous = 150

  def update_escc(self, ret):
    for ii in range(1):
      msg_src = "ESCC"
      msg = self.rcp.vl[msg_src]

      if ii not in self.pts:
        self.pts[ii] = structs.RadarData.RadarPoint()
        self.pts[ii].trackId = self.track_id
        self.track_id += 1


      # Fetch vehicle speed from CAN data (CLU15 message)
      try:
        self.sm.update()
        # Average all four wheel speeds for vEgo
        vEgo_km = self.sm["carState"].vEgo * 3.6 #hspeed.vEgo
        print(f"Car Speed: {vEgo_km}")
      except KeyError as e:
        # Fallback if signal isn’t available
        vEgo_km = 0.0
        print(f"KeyError: {e}")

      #valid = msg['ACC_ObjStatus']
      drel = msg['ACC_ObjDist']
      valid = False
      if msg['ACC_ObjStatus'] and drel <= self.previous:
      #if drel <= self.previous:
        valid = True

      # if vEgo_km <= 10 and drel <= self.previous and drel < 150:
      #   valid = True

      self.previous = drel
      if drel < 7:
        drel = drel + 1
      else:
        drel = drel + 5

      if valid:
        self.pts[ii].measured = True if vEgo_km >= 40 else False
        self.pts[ii].dRel = drel #msg['ACC_ObjDist']
        self.pts[ii].yRel = -msg['ACC_ObjLatPos']
        self.pts[ii].vRel = msg['ACC_ObjRelSpd']
        self.pts[ii].aRel = float('nan')  # TODO-SP: calculate from ACC_ObjRelSpd and with timestep 50Hz (needs to modify in interfaces.py)
        self.pts[ii].yvRel = float('nan')

      else:
        del self.pts[ii]

    ret.points = list(self.pts.values())
    return ret
