#!/usr/bin/env python3
from typing import List, Tuple, Optional

from cereal import car
from opendbc.can.packer import CANPacker
from common.swaglog import cloudlog

DBC_NAME = "ARS408"
RADAR_BUS_DEFAULT = 1
STANDSTILL_THRESHOLD = 0.01  # m/s
DEG_PER_RAD = 57.29577951308232


def _clamp(val: float, lo: float, hi: float) -> float:
  return max(lo, min(hi, val))


class RadarInfoTx:
  """Build ARS408 motion info frames for speed and yaw rate.

  Inputs:
    - speed_mps: vehicle speed in m/s (signed; +forward, -reverse)
    - yaw_rate_radps: yaw rate in rad/s (signed; +CCW)

  Outputs:
    - List of CAN messages for SpeedInformation (0x300) and YawRateInformation (0x301)
  """

  def __init__(self, bus: int = RADAR_BUS_DEFAULT):
    self.bus = bus
    self.packer = CANPacker(DBC_NAME)

  def make(self, speed_mps: float, yaw_rate_radps: float, shiftgear: Optional[car.CarState.GearShifter] = None, direction_override: Optional[int] = None) -> List[Tuple[int, bytes, int]]:
    # 方向枚举: 0=静止, 1=前进, 2=倒车
    if direction_override is not None:
      direction = direction_override
    elif abs(speed_mps) < STANDSTILL_THRESHOLD:
      direction = 0
    else:
      # 档位+速度符号一致性：优先用实际运动方向编码
      speed_forward = (speed_mps >= 0.0)
      
      if shiftgear == car.CarState.GearShifter.reverse:
        expected_direction = 2
        actual_direction = 2 if not speed_forward else 1
        if expected_direction != actual_direction:
          gear_str = str(shiftgear) if shiftgear is not None else "None"
          cloudlog.warning(f"RadarInfoTx: Gear and speed direction mismatch! Gear=R, speed direction={speed_forward}, expected direction=2, actual direction={actual_direction}")
        direction = actual_direction
      elif shiftgear in (car.CarState.GearShifter.drive,
                         car.CarState.GearShifter.low,
                         car.CarState.GearShifter.sport,
                         car.CarState.GearShifter.brake):
        expected_direction = 1
        actual_direction = 1 if speed_forward else 2
        if expected_direction != actual_direction:
          gear_str = str(shiftgear) if shiftgear is not None else "None"
          cloudlog.warning(f"RadarInfoTx: Gear and speed direction mismatch! Gear=D/L/S/B, speed direction={speed_forward}, expected direction=1, actual direction={actual_direction}")
        direction = actual_direction
      elif shiftgear in (car.CarState.GearShifter.park, car.CarState.GearShifter.neutral):
        direction = 1 if speed_forward else 2
      else:
        direction = 1 if speed_forward else 2

    # Clamp to DBC ranges
    spd = _clamp(abs(speed_mps), 0.0, 163.8)  # m/s
    yaw_deg = _clamp(yaw_rate_radps * DEG_PER_RAD, -327.68, 327.66)  # deg/s

    vals_speed = {
      "RadarDevice_Speed": spd,
      "RadarDevice_SpeedDirection": direction,
    }
    vals_yaw = {
      "RadarDevice_YawRate": yaw_deg,
    }

    cloudlog.debug(f"RadarInfoTx: 最终方向={direction}, 速度={spd:.2f}m/s, 横摆角速度={yaw_deg:.2f}deg/s")
    
    # 示例：如果需要将档位信息输出到openpilot的控制台，可以取消下面一行的注释
    # print(f"Gear Information: {shiftgear}")
    # 带颜色的输出示例（黄色）：
    # print("\033[33m" + f"Gear Information: {shiftgear}" + "\033[0m")
    
    return [
      self.packer.make_can_msg("SpeedInformation", self.bus, vals_speed),
      self.packer.make_can_msg("YawRateInformation", self.bus, vals_yaw),
    ]

  def make_from_cs(self, CS) -> List[Tuple[int, bytes, int]]:
    """Convenience wrapper using common CarState fields.
    Expects CS.vEgo (m/s), CS.yawRate (rad/s), and CS.gearShifter.
    """
    try:
      speed = float(getattr(CS, "vEgo"))
    except Exception:
      speed = 0.0
    try:
      yaw_rate = float(getattr(CS, "yawRate"))
    except Exception:
      yaw_rate = 0.0

    try:
      gear = getattr(CS, "gearShifter", None)
    except Exception:
      gear = None

    return self.make(speed, yaw_rate, shiftgear=gear)