#!/usr/bin/env python3
from typing import List, Tuple

from opendbc.can.packer import CANPacker

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

  def make(self, speed_mps: float, yaw_rate_radps: float) -> List[Tuple[int, bytes, int]]:
    # Direction: 0=standstill, 1=forward, 2=backward
    direction = 0 if abs(speed_mps) < STANDSTILL_THRESHOLD else (1 if speed_mps >= 0.0 else 2)

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

    return [
      self.packer.make_can_msg("SpeedInformation", self.bus, vals_speed),
      self.packer.make_can_msg("YawRateInformation", self.bus, vals_yaw),
    ]

  def make_from_cs(self, CS) -> List[Tuple[int, bytes, int]]:
    """Convenience wrapper using common CarState fields.
    Expects CS.vEgo (m/s) and CS.yawRate (rad/s).
    """
    try:
      speed = float(getattr(CS, "vEgo"))
    except Exception:
      speed = 0.0
    try:
      yaw_rate = float(getattr(CS, "yawRate"))
    except Exception:
      yaw_rate = 0.0
    return self.make(speed, yaw_rate)