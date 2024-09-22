import math

from cereal import car
from opendbc.can.parser import CANParser
from openpilot.selfdrive.car.interfaces import RadarInterfaceBase
from openpilot.selfdrive.car.hyundai.hyundaicanfd import CanBus
from openpilot.selfdrive.car.hyundai.values import DBC, HyundaiFlagsSP, CANFD_CAR
from openpilot.common.filter_simple import StreamingMovingAverage, SpecialRangeFilter

RADAR_START_ADDR = 0x500
RADAR_MSG_COUNT = 32

# POC for parsing corner radars: https://github.com/commaai/openpilot/pull/24221/

def get_radar_can_parser(CP):
  if DBC[CP.carFingerprint]['radar'] is None:
    if CP.carFingerprint in CANFD_CAR:
      if CP.spFlags & HyundaiFlagsSP.SP_CAMERA_SCC_LEAD:
        lead_src, bus = "SCC_CONTROL", CanBus(CP).CAM
      else:
        return None
    else:
      if CP.spFlags & HyundaiFlagsSP.SP_ENHANCED_SCC:
        lead_src, bus = "ESCC", 0
      elif CP.spFlags & HyundaiFlagsSP.SP_CAMERA_SCC_LEAD:
        lead_src, bus = "SCC11", 2
      else:
        return None
    messages = [(lead_src, 50)]
    return CANParser(DBC[CP.carFingerprint]['pt'], messages, bus)

  messages = [(f"RADAR_TRACK_{addr:x}", 50) for addr in range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT)]
  return CANParser(DBC[CP.carFingerprint]['radar'], messages, 1)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.CP = CP
    self.enhanced_scc = (CP.spFlags & HyundaiFlagsSP.SP_ENHANCED_SCC) and DBC[CP.carFingerprint]['radar'] is None
    self.camera_scc = CP.spFlags & HyundaiFlagsSP.SP_CAMERA_SCC_LEAD
    self.updated_messages = set()
    self.trigger_msg = 0x2AB if self.enhanced_scc else \
                       0x1A0 if self.camera_scc and CP.carFingerprint in CANFD_CAR else \
                       0x420 if self.camera_scc else \
                       (RADAR_START_ADDR + RADAR_MSG_COUNT - 1)
    self.track_id = 0
    self.previous = 0
    self.counter = 0
    self.dRelFilter = StreamingMovingAverage(2)
    self.vRelFilter = StreamingMovingAverage(4)
    self.rangeFilter = SpecialRangeFilter(1, True)

    self.radar_off_can = CP.radarUnavailable
    self.rcp = get_radar_can_parser(CP)

    self.sp_radar_tracks = CP.spFlags & HyundaiFlagsSP.SP_RADAR_TRACKS

  def update(self, can_strings):
    if self.radar_off_can or (self.rcp is None):
      return super().update(None)

    vls = self.rcp.update_strings(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()

    radar_error = []
    if rr is not None:
      radar_error = rr.errors
    if list(radar_error) and self.sp_radar_tracks:
      return super().update(None)

    return rr

  def _update(self, updated_messages):
    ret = car.RadarData.new_message()
    if self.rcp is None:
      return ret

    errors = []

    if not self.rcp.can_valid:
      errors.append("canError")
    ret.errors = errors

    if self.enhanced_scc or self.camera_scc:
      msg_src = "ESCC" if self.enhanced_scc else \
                "SCC_CONTROL" if self.CP.carFingerprint in CANFD_CAR else \
                "SCC11"
      msg = self.rcp.vl[msg_src]
      valid = msg['ACC_ObjDist'] < 204.6 if self.CP.carFingerprint in CANFD_CAR else \
              msg['ACC_ObjStatus']
      okgo = True #False #True if -msg['ACC_ObjLatPos'] > -8 else False
      
      
      # if valid:
      #   if self.previous == 0:
      #     self.previous = msg['ACC_ObjRelSpd']
      #   elif (self.previous - msg['ACC_ObjRelSpd']) < -5 or (self.previous - msg['ACC_ObjRelSpd']) > 10:
      #     valid = False
      #     self.counter += 1
      #     if self.counter > 100:
      #       self.previous = msg['ACC_ObjRelSpd']
      #       self.counter = 0
            
      valid = msg['ACC_ObjDist'] < 9
      if msg['ACC_ObjStatus'] and ((11 < msg['ACC_ObjDist'] < 25) or (55 < msg['ACC_ObjDist'] < 100)):
          valid = True
      elif msg['ACC_ObjStatus'] and 55 < msg['ACC_ObjDist'] and 15 < msg['ACC_ObjRelSpd']:
          valid = True
      else:
          valid = False


      dRel = msg['ACC_ObjDist']
      vRel = msg['ACC_ObjRelSpd']
      #valid = msg['ACC_ObjStatus'] and ((80 < dRel < 150) or (dRel < 40 and (50 < vRel or vRel < 15))) #18 < dRel < 150
      valid = msg['ACC_ObjStatus'] and dRel < 150
      valid = self.rangeFilter.update(msg['ACC_ObjStatus'], dRel)
      # self.counter += 1
      # if self.counter % 5 == 0:
      #   valid = True
      # else:
      #   valid = False

      # if self.counter >= 50:  # After 50 iterations (1 second)
      #   self.counter = 0

      for ii in range(1):
        if valid and okgo:
          if ii not in self.pts:
            self.pts[ii] = car.RadarData.RadarPoint.new_message()
            self.pts[ii].trackId = 0 #self.track_id
            #self.track_id += 1
            #dRel = self.dRelFilter.set(dRel)
            #vRel = self.vRelFilter.set(vRel)
          #else:
            #dRel = self.dRelFilter.process(dRel)
            #vRel = self.vRelFilter.process(vRel)
          self.pts[ii].measured = True
          self.pts[ii].dRel = dRel #msg['ACC_ObjDist']
          self.pts[ii].yRel = max(-5, -msg['ACC_ObjLatPos']) #if self.enhanced_scc else float('nan')
          self.pts[ii].vRel = vRel #msg['ACC_ObjRelSpd']
          self.pts[ii].aRel = float('nan')
          self.pts[ii].yvRel = float('nan')
          # self.previous = msg['ACC_ObjRelSpd']

        else:
          if ii in self.pts:
            del self.pts[ii]
    else:
      for addr in range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT):
        msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]

        if addr not in self.pts:
          self.pts[addr] = car.RadarData.RadarPoint.new_message()
          self.pts[addr].trackId = self.track_id
          self.track_id += 1

        valid = msg['STATE'] in (3, 4)
        if valid:
          azimuth = math.radians(msg['AZIMUTH'])
          self.pts[addr].measured = True
          self.pts[addr].dRel = math.cos(azimuth) * msg['LONG_DIST']
          self.pts[addr].yRel = 0.5 * -math.sin(azimuth) * msg['LONG_DIST']
          self.pts[addr].vRel = msg['REL_SPEED']
          self.pts[addr].aRel = msg['REL_ACCEL']
          self.pts[addr].yvRel = float('nan')

        else:
          del self.pts[addr]

    ret.points = list(self.pts.values())
    return ret
