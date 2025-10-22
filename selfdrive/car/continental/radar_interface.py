#!/usr/bin/env python3
from math import isnan
from typing import Set

from cereal import car
from opendbc.can.parser import CANParser
from openpilot.selfdrive.car.interfaces import RadarInterfaceBase
from selfdrive.car.continental.radar_info_tx import RadarInfoTx  # ARS408 速度/偏航率下发辅助

# Continental ARS408
DBC_NAME = "ARS408"
RADAR_BUS = 1

# Use Obj_0_Status as the cycle trigger (end-of-cycle marker in ARS408)
TRIGGER_MSG_ADDR = 1546  # Obj_0_Status


def _create_radar_can_parser():
  # message name with expected frequency (Hz)
  # ARS408 object list cycles every ~70-80 ms => ~13-14 Hz
  messages = [
    ("Obj_0_Status", 14),
    ("Obj_1_General", 14),
    ("Obj_2_Quality", 14),
    ("Obj_3_Extended", 14),
    ("Obj_4_Warning", 14),
    ("RadarState", 1),  # Spec: 1 Hz (0x201)
  ]

  return CANParser(DBC_NAME, messages, RADAR_BUS)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)

    self.rcp = None if CP.radarUnavailable else _create_radar_can_parser()
    self.updated_messages: Set[int] = set()
    self.trigger_msg = TRIGGER_MSG_ADDR
    self.track_id = 0
    self.motion_tx = RadarInfoTx(bus=RADAR_BUS)  # 用于构建 Speed/YawRate 下行帧

  def update(self, can_strings):
    if self.rcp is None:
      return super().update(None)

    vls = self.rcp.update_strings(can_strings)
    self.updated_messages.update(vls)

    # Wait until the trigger message is observed in the current batch
    if self.trigger_msg not in self.updated_messages:
      return None

    ret = car.RadarData.new_message()

    # Errors
    errors = []
    if not self.rcp.can_valid:
      errors.append("canError")

    # Basic fault detection from RadarState (513)
    radar_state = self.rcp.vl.get("RadarState", {})
    try:
      if radar_state.get("RadarState_Interference") or radar_state.get("RadarState_Voltage_Error"):
        errors.append("fault")
    except Exception:
      pass

    ret.errors = errors

    # Assemble radar points from object messages.
    # ARS408 broadcasts multiple objects using the same message name. Use vl_all to capture all values.
    obj_gen = self.rcp.vl_all.get("Obj_1_General", {})
    obj_ext = self.rcp.vl_all.get("Obj_3_Extended", {})

    # 当前周期目标数（来自 Obj_0_Status），仅用于一致性校验与注释说明
    obj_status = self.rcp.vl.get("Obj_0_Status", {})
    n_objects_reported = int(obj_status.get("Obj_NofObjects", 0)) if obj_status else 0

    ids = obj_gen.get("Obj_ID", [])
    dist_long = obj_gen.get("Obj_DistLong", [])
    dist_lat = obj_gen.get("Obj_DistLat", [])
    vrel_long = obj_gen.get("Obj_VrelLong", [])
    vrel_lat = obj_gen.get("Obj_VrelLat", [])

    arel_long = obj_ext.get("Obj_ArelLong", [])

    current_ids = set()

    n = min(len(ids), len(dist_long), len(dist_lat), len(vrel_long), len(vrel_lat))
    # 若雷达状态中报告了目标数量，则收敛到报告值，避免尾部未刷新导致的长度不一致
    n = min(n, n_objects_reported) if n_objects_reported > 0 else n
    for i in range(n):
      obj_id = int(ids[i])
      current_ids.add(obj_id)

      # Create point on first sight
      if obj_id not in self.pts:
        self.pts[obj_id] = car.RadarData.RadarPoint.new_message()
        # 使用雷达提供的 Obj_ID 作为轨迹ID，支持同时跟踪多个目标
        self.pts[obj_id].trackId = obj_id

        self.track_id += 1

      # Parse track data
      self.pts[obj_id].dRel = float(dist_long[i])
      self.pts[obj_id].yRel = float(dist_lat[i])
      self.pts[obj_id].vRel = float(vrel_long[i])
      self.pts[obj_id].yvRel = float(vrel_lat[i])
      self.pts[obj_id].aRel = float(arel_long[i]) if i < len(arel_long) else float('nan')
      self.pts[obj_id].measured = True

    # prune tracks that were not reported in this cycle
    for old_id in list(self.pts.keys()):
      if old_id not in current_ids:
        del self.pts[old_id]

    ret.points = list(self.pts.values())

    self.updated_messages.clear()
    return ret

  def build_motion_info(self, speed_mps: float, yaw_rate_radps: float):
    """生成 ARS408 运动信息帧供外部 sendcan 下发。

    参数:
      - speed_mps: 车辆速度 m/s（前进为正、倒车为负）
      - yaw_rate_radps: 偏航率 rad/s（逆时针为正）
    返回:
      - CAN 帧列表: [(地址, 数据, 总线), ...]
    说明:
      - RadarInterface 仅负责解析，不直接发送 CAN。
      - 实际发送应在 carcontroller 或车辆接口处调用 sendcan。
      - 缩放规则已在 radar_info_tx.py 中实现：
        - SpeedInformation: RadarDevice_Speed 0.02 m/s，方向枚举 0/1/2
        - YawRateInformation: RadarDevice_YawRate 0.01 deg/s，偏移 -327.68
    """
    return self.motion_tx.make(speed_mps, yaw_rate_radps)

  def build_motion_info_from_cs(self, CS):
    """使用 CarState 字段生成 ARS408 运动信息帧。
    读取 CS.vEgo (m/s) 与 CS.yawRate (rad/s)，并返回下行帧列表。
    """
    return self.motion_tx.make_from_cs(CS)