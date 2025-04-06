"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from dataclasses import dataclass

from opendbc.car.hyundai.values import CAR


@dataclass
class CarTuningConfig:
  v_ego_stopping: float
  v_ego_starting: float
  stopping_decel_rate: float
  start_accel: float
  jerk_limits: tuple[float, float, float]  # (min jerk, max lower jerk, max upper jerk)


# Default configurations for different car types
# Min jerk is set to 0.53 per (Horn et al., 2024)
TUNING_CONFIGS = {
  "EV": CarTuningConfig(
    v_ego_stopping=0.25,
    v_ego_starting=0.10,
    stopping_decel_rate=0.30,
    start_accel=1.0,
    jerk_limits=(0.53, 3.3, 2.2),
  ),
  "HYBRID": CarTuningConfig(
    v_ego_stopping=0.25,
    v_ego_starting=0.12,
    stopping_decel_rate=0.35,
    start_accel=1.0,
    jerk_limits=(0.53, 3.3, 2.2),
  ),
  "DEFAULT": CarTuningConfig(
    v_ego_stopping=0.25,
    v_ego_starting=0.10,
    stopping_decel_rate=0.30,
    start_accel=1.6,
    jerk_limits=(0.53, 3.3, 2.2),
  )
}

# Car-specific configs
CAR_SPECIFIC_CONFIGS = {
  CAR.KIA_NIRO_EV: CarTuningConfig(
    v_ego_stopping=0.25,
    v_ego_starting=0.10,
    stopping_decel_rate=0.05,
    start_accel=1.0,
    jerk_limits=(0.53, 3.3, 2.2),
  )
}
