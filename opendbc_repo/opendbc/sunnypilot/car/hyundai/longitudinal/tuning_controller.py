"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import math
import numpy as np
from dataclasses import dataclass

from opendbc.car import structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.hyundai.values import CarControllerParams

from opendbc.sunnypilot.car.hyundai.longitudinal.helpers import get_car_config
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

LongCtrlState = structs.CarControl.Actuators.LongControlState


@dataclass
class LongitudinalTuningState:
  accel_last: float = 0.0
  accel_last_jerk: float = 0.0
  jerk: float = 0.0


class LongitudinalTuningController:
  """Longitudinal tuning methodology for HKG"""

  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> None:
    self.CP_SP = CP_SP

    self.state = LongitudinalTuningState()
    self.car_config = get_car_config(CP)
    self.accel_raw = 0.0
    self.accel_value = 0.0
    self.jerk_upper = 0.0
    self.jerk_lower = 0.0

  def reset(self) -> None:
    self.accel_raw = 0.0
    self.accel_value = 0.0
    self.state.accel_last = 0.0
    self.state.accel_last_jerk = 0.0
    self.jerk_upper = 0.0
    self.jerk_lower = 0.0

  def make_jerk(self, CC: structs.CarControl, CS: CarStateBase, long_control_state: LongCtrlState) -> None:
    if not self.CP_SP.flags & HyundaiFlagsSP.LONG_TUNING_BRAKING:
      jerk_limit = 3.0 if long_control_state == LongCtrlState.pid else 1.0

      self.jerk_upper = jerk_limit
      self.jerk_lower = jerk_limit
      return

    # Blend planned acceleration with current acceleration.
    planned_accel = CC.actuators.accel
    current_accel = CS.out.vEgo
    blended_value = 0.50 * planned_accel + 0.50 * current_accel
    delta = blended_value - self.state.accel_last_jerk

    self.state.jerk = math.copysign(delta * delta, delta)

    self.state.accel_last_jerk = blended_value

    # Jerk is limited by the following conditions imposed by ISO 15622:2018
    velocity = CS.out.vEgo
    if velocity < 5.0:
      decel_jerk_max = self.car_config.jerk_limits[1]
      accel_jerk_max = 1.0
    elif velocity > 20.0:
      decel_jerk_max = 2.5
      accel_jerk_max = 1.65
    else:   # Between 5 m/s and 20 m/s
      decel_jerk_max = 3.64284 - (0.05714 * velocity)
      accel_jerk_max = self.car_config.jerk_limits[2]

    accel_jerk = accel_jerk_max if LongCtrlState == LongCtrlState.pid else 1.0
    min_upper_jerk = self.car_config.jerk_limits[0] if (velocity > 3.611) else 0.60
    min_lower_jerk = self.car_config.jerk_limits[0]

    self.jerk_upper = min(max(min_upper_jerk, self.state.jerk), accel_jerk)
    self.jerk_lower = min(max(min_lower_jerk, -self.state.jerk), decel_jerk_max)

  def calculate_accel(self, CC: structs.CarControl) -> float:
    """Calculate acceleration with cruise control status handling."""
    if not CC.enabled:
      self.reset()
      return 0.0

    accel = CC.actuators.accel

    return float(np.clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

  def calculate_a_value(self, CC: structs.CarControl) -> float:
    jerk = 5
    jerk_number = jerk / 50   # This equals 0.1, but why not just 0.1? Maybe we can try self.jerk_lower?
    if not CC.enabled:
      self.reset()
      return 0.0

    self.accel_raw = CC.actuators.accel

    self.accel_value = np.clip(self.accel_raw, self.state.accel_last - jerk_number, self.state.accel_last + jerk_number)

    self.state.accel_last = self.accel_value

    return self.accel_value