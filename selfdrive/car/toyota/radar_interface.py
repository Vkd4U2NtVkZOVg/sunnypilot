#!/usr/bin/env python3
"""
Toyota 雷达接口（TSS/TSS2），解析雷达 CAN 消息并生成 RadarData。
- 根据车型选择不同的消息 ID 范围（TSS vs TSS2）
- 使用 CANParser 订阅 A/B 两类消息，以 B 类最后一帧作为周期触发
- 将有效 A 类消息转换为 RadarPoint，并维护 trackId 与点的有效性
"""
from opendbc.can.parser import CANParser
from cereal import car
from openpilot.selfdrive.car.toyota.values import DBC, TSS2_CAR
from openpilot.selfdrive.car.interfaces import RadarInterfaceBase


def _create_radar_can_parser(car_fingerprint):
  """根据车型 fingerprint 构建订阅的雷达消息集合与 CANParser。
  - TSS2：A 类 0x180~0x18F，B 类 0x190~0x19F
  - 旧款：A 类 0x210~0x21F，B 类 0x220~0x22F
  返回的解析器订阅所有 A/B 消息，周期通过 B 类最后一帧触发。
  """
  if car_fingerprint in TSS2_CAR:
    RADAR_A_MSGS = list(range(0x180, 0x190))
    RADAR_B_MSGS = list(range(0x190, 0x1a0))
  else:
    RADAR_A_MSGS = list(range(0x210, 0x220))
    RADAR_B_MSGS = list(range(0x220, 0x230))

  msg_a_n = len(RADAR_A_MSGS)
  msg_b_n = len(RADAR_B_MSGS)
  # 将 A/B 类消息 ID 与频率 20Hz 配对；strict=True 强制一一对应，防止长度不匹配
  messages = list(zip(RADAR_A_MSGS + RADAR_B_MSGS, [20] * (msg_a_n + msg_b_n), strict=True))

  return CANParser(DBC[car_fingerprint]['radar'], messages, 1)

class RadarInterface(RadarInterfaceBase):
  """Toyota 雷达接口实现：管理订阅的消息集、触发帧与雷达点状态"""
  def __init__(self, CP):
    """初始化雷达解析器与状态：
    - 根据车型选择 A/B 两类消息范围
    - 若雷达不可用则禁用解析器
    - 设置触发帧为 B 类消息最后一个 ID
    """
    super().__init__(CP)
    self.track_id = 0  # 递增的轨迹 ID，用于为新目标分配唯一ID
    self.radar_ts = CP.radarTimeStep  # 雷达时间步长（来自 CarParams）

    if CP.carFingerprint in TSS2_CAR:
      self.RADAR_A_MSGS = list(range(0x180, 0x190))
      self.RADAR_B_MSGS = list(range(0x190, 0x1a0))
    else:
      self.RADAR_A_MSGS = list(range(0x210, 0x220))
      self.RADAR_B_MSGS = list(range(0x220, 0x230))

    self.valid_cnt = {key: 0 for key in self.RADAR_A_MSGS}

    self.rcp = None if CP.radarUnavailable else _create_radar_can_parser(CP.carFingerprint)
    self.trigger_msg = self.RADAR_B_MSGS[-1]
    self.updated_messages = set()

  def update(self, can_strings):
    """处理一次 CAN 输入：
    - 累计更新收到的帧 ID
    - 遇到触发帧（最后一个 B 类消息）时，调用 _update 生成 RadarData
    - 若雷达不可用，直接返回基类结果
    """
    if self.rcp is None:
      return super().update(None)

    # 解析本次传入的原始 CAN 字符串，返回已更新的消息ID列表（帧地址集合），用于周期触发与后续点解析
    vls = self.rcp.update_strings(can_strings)
    self.updated_messages.update(vls)

    # 仅在收到周期触发帧（B 类最后一帧）时才进行一次周期解码；
    # 该判断表示到达一个雷达周期的边界，并不绝对保证该周期内所有 A/B 帧都齐全（可能存在丢帧/延迟）。
    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()

    return rr

  def _update(self, updated_messages):
    """在一个雷达周期结束时，将已更新的消息解析为 RadarData：
    - 校验 CAN 有效性并记录错误
    - 遍历 A 类消息，依据 VALID 与 SCORE 判断目标有效性
    - 维护 self.valid_cnt 计数以平滑目标的出现/消失
    - 为新目标分配 trackId，并更新 dRel/yRel/vRel 等字段
    """
    ret = car.RadarData.new_message()
    errors = []
    if not self.rcp.can_valid:
      errors.append("canError")
    ret.errors = errors

    for ii in sorted(updated_messages):
      if ii in self.RADAR_A_MSGS:
        cpt = self.rcp.vl[ii]

        if cpt['LONG_DIST'] >= 255 or cpt['NEW_TRACK']:
          self.valid_cnt[ii] = 0  # 距离无效或新目标出现时重置计数
        if cpt['VALID'] and cpt['LONG_DIST'] < 255:
          self.valid_cnt[ii] += 1  # 有效测量计数增加
        else:
          self.valid_cnt[ii] = max(self.valid_cnt[ii] - 1, 0)  # 无效时缓慢衰减

        score = self.rcp.vl[ii+16]['SCORE']  # B 类对应分数（A/B 相隔 0x10）
        # print ii, self.valid_cnt[ii], score, cpt['VALID'], cpt['LONG_DIST'], cpt['LAT_DIST']

        # 有效测量或分数>50且距离有效且计数>0，认为该雷达点有效
        if cpt['VALID'] or (score > 50 and cpt['LONG_DIST'] < 255 and self.valid_cnt[ii] > 0):
          if ii not in self.pts or cpt['NEW_TRACK']:
            self.pts[ii] = car.RadarData.RadarPoint.new_message()
            self.pts[ii].trackId = self.track_id
            self.track_id += 1
          self.pts[ii].dRel = cpt['LONG_DIST']  # 车辆前方距离（m）
          self.pts[ii].yRel = -cpt['LAT_DIST']  # 车辆坐标系 y 轴，左正右负（m）
          self.pts[ii].vRel = cpt['REL_SPEED']  # 相对速度（m/s）
          self.pts[ii].aRel = float('nan')      # 未提供加速度
          self.pts[ii].yvRel = float('nan')     # 未提供横向相对速度
          self.pts[ii].measured = bool(cpt['VALID'])  # 是否为雷达直接测量
        else:
          if ii in self.pts:
            del self.pts[ii]  # 目标失效时移除

    ret.points = list(self.pts.values())
    return ret
