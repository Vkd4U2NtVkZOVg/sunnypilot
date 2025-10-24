#!/usr/bin/env python3
from math import isnan
from typing import Set

from cereal import car
from opendbc.can.parser import CANParser
from openpilot.selfdrive.car.interfaces import RadarInterfaceBase
from selfdrive.car.continental.radar_info_tx import RadarInfoTx  # ARS408 速度/偏航率下发辅助
from common.swaglog import cloudlog

# 自定义CAN日志文件路径与文件写入辅助
LOG_FILE_PATH = "/data/log/ars408_can.log"

def _write_custom_log_line(line: str):
  try:
    import os
    os.makedirs(os.path.dirname(LOG_FILE_PATH), exist_ok=True)
    with open(LOG_FILE_PATH, 'a', encoding='utf-8') as f:
      f.write(line + "\n")
  except Exception:
    pass

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
    #("Obj_4_Warning", 14),
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
    # 记录上一次 Obj_0_Status 的测量计数，用于更稳健的周期边界判定
    self.last_meas_counter = None
    # 记录上次故障集合，用于变化检测与cloudlog
    self.last_faults_active = set()
    # 跨调用缓冲：聚合同一周期内的对象数据，直到收到触发帧(0x60A)再输出
    self.cycle_objs = {}
    self.cycle_ids = set()

  def update(self, can_strings):
    """ARS408对象列表解析的主入口（按0x60A触发、跨批次缓冲、质量与类型门控）。

    参数:
    - can_strings: 本次调用收到的原始 CAN 帧字符串列表（上游增量聚合）。

    输入/输出与内部状态:
    - 解析器: 调用 `self.rcp.update_strings(can_strings)`，其会更新:
      - `self.rcp.vl`: 最近值字典（跨调用保留，适合状态/单帧信号，如RadarState/Obj_0_Status）。
      - `self.rcp.vl_all`: 当前批次聚合数组（仅代表本次调用解析到的对象列表/质量/告警等）。
      - 返回 `vls`: 本批次更新的消息地址集合（用于跨调用触发判定）。
    - 跨批次缓冲: `self.cycle_objs`/`self.cycle_ids` 累积同一周期的对象字段，触发后统一输出并清空。
    - 触发集合: `self.updated_messages` 跨调用累加更新过的地址，直到包含 `0x60A` 才输出一个周期。
    - 轨迹缓存: `self.pts` 存活跃轨迹点；触发输出后会按门控结果剪枝未出现/不合格的旧点。
    - 故障状态: `self.last_faults_active` 存最近故障集合；如故障变化立刻返回仅含错误的帧。
    - 诊断: `self.last_meas_counter` 记录 `Obj_MeasCounter`，用于日志提示未递增/跳变；并在触发处对比 `Obj_NofObjects` 与缓冲的对象数。

    处理流程概述:
    1) 解析批次并累加触发集合: `vls = rcp.update_strings(...)`; `updated_messages.update(vls)`。
    2) 提取RadarState故障（最近值），若故障集合由无到有或发生变化，立刻返回错误帧并清空周期缓冲。
    3) 批次对象追加到跨批次缓冲:
       - General(0x60B): 按索引对齐写入 dRel/yRel/vRel/yvRel。
       - Extended(0x60D): 按 Obj_ID 对齐补充 aRel 与 Obj_Class（兼容跨批次到达）。
       - Quality(0x60C): 按 Obj_ID 对齐补充 ProbOfExist/MeasState。
       - Warning(0x60E): 按 Obj_ID 对齐补充 CollDetRegionBitfield。
    4) 触发判定: 若本周期尚未收到 `0x60A`（Obj_0_Status），返回 None；否则继续。
    5) 读取 Obj_0_Status 最近值，记录 MeasCounter 增量并输出周期完整性日志（NofObjects vs 缓冲对象数）。
    6) 门控与点云组装:
       - 忽略点目标: `Obj_Class == 0` 不上报。
       - 概率阈值: 要求 `Obj_ProbOfExist > 1`（>25%）才上报；否则丢弃。
       - 字段缺失用 `NaN` 填充（区分未测量与有效零值）。
       - 构建/更新 `self.pts[trackId]` 并设置 `measured`。
       - 剪枝未通过门控的旧轨迹，避免幽灵目标。
    7) 周期结束清理: 清空 `updated_messages` 与 `cycle_*` 缓冲，返回 `car.RadarData`。

    返回值:
    - `car.RadarData`：在收到 `0x60A` 触发帧并完成门控与组装后返回。
    - `None`：未到触发帧，继续缓冲对象数据，等待后续批次触发。

    设计要点与边界:
    - 触发后输出、期间累积地址：兼容对象帧与触发帧跨批次到达，防止丢点。
    - 门控为轻量策略：默认忽略点目标且要求存在概率>25%，可按需调整阈值或加入 MeasState 进一步筛选。
    - 质量和告警仅用于标注与门控，不改变触发与缓冲机制。
    - 线程安全假设：在调用层面为串行执行；如需并发处理需加锁保护内部状态。
    - RadarState 为 1Hz 最近值读取；故障变化即时上报并清空缓冲，故障恢复不强制输出一帧。
    """
    if self.rcp is None:
      return super().update(None)

    vls = self.rcp.update_strings(can_strings)
    # 记录本批次原始 CAN 字符串与解析出的消息地址，便于问题定位
    try:
      msgs_hex = []
      for m in can_strings:
        if isinstance(m, (bytes, bytearray, memoryview)):
          msgs_hex.append(" ".join(f"{b:02X}" for b in m))
        elif isinstance(m, (list, tuple)):
          # 可能是 (addr, data, bus) 或 [timestamp, [[addr, data, bus], ...]]
          if len(m) == 3 and isinstance(m[1], (bytes, bytearray, memoryview)):
            msgs_hex.append(" ".join(f"{b:02X}" for b in m[1]))
          elif len(m) == 2 and isinstance(m[1], (list, tuple)):
            for sub in m[1]:
              if isinstance(sub, (list, tuple)) and len(sub) >= 2 and isinstance(sub[1], (bytes, bytearray, memoryview)):
                msgs_hex.append(" ".join(f"{b:02X}" for b in sub[1]))
              else:
                msgs_hex.append(str(sub))
          else:
            msgs_hex.append(str(m))
      addrs_hex = [f"0x{addr:03X}" for addr in sorted(list(vls))]
      line1 = f"ARS408 CAN raw batch: count={len(msgs_hex)}; msgs_hex={msgs_hex}"
      line2 = f"ARS408 CAN parsed addrs this batch: addrs={addrs_hex}"
      cloudlog.info(line1)
      cloudlog.info(line2)
      _write_custom_log_line(line1)
      _write_custom_log_line(line2)
    except Exception:
      pass
    # `update_strings` 会重建 `vl_all` 为当前批次，并覆盖 `vl` 为最近值；
    # 同时返回本批次更新过的消息地址集合 `vls`，用于跨调用的触发判定。
    # 例如: {0x60A, 0x60B, 0x60D} => {1546, 1547, 1549}
    self.updated_messages.update(vls)

    # RadarState 为 1Hz，包含关键故障位；此处无论本周期是否更新，都读取“最近值”。
    # 若需要判断是否在本周期更新，可检查其地址（0x201=513）是否在 `updated_messages` 中。
    def _extract_faults(rs: dict) -> set:
      """从 RadarState 最近值字典中提取故障集合。
      - 典型字段: 干扰、温度错误、瞬时/持久错误、电压错误等。
      - 解析失败时返回空集合，不抛异常。
      """
      faults = set()
      try:
        if int(rs.get("RadarState_Interference", 0)):
          faults.add("interference")
        if int(rs.get("RadarState_Temperature_Error", 0)):
          faults.add("temperatureError")
        if int(rs.get("RadarState_Temporary_Error", 0)):
          faults.add("temporaryError")
        if int(rs.get("RadarState_Persistent_Error", 0)):
          faults.add("persistentError")
        if int(rs.get("RadarState_Voltage_Error", 0)):
          faults.add("voltageError")
      except Exception:
        pass
      return faults

    radar_state = self.rcp.vl.get("RadarState", {})
    faults_now = _extract_faults(radar_state)
    # 若故障集合从无到有或发生变化：立即上报一个仅含错误的 RadarData，并清空周期缓冲与触发集合；
    # 若故障清除，仅记录日志，不影响正常输出节奏。
    if faults_now != self.last_faults_active and len(faults_now) > 0:
      cloudlog.error(f"ARS408 RadarState faults: {sorted(list(faults_now))}; raw={radar_state}")
      self.last_faults_active = faults_now.copy()
      ret = car.RadarData.new_message()
      ret.errors = sorted(list(faults_now))
      ret.points = []
      # 清空当前周期的触发判定与对象缓冲，避免跨周期污染
      self.updated_messages.clear()
      self.cycle_objs.clear()
      self.cycle_ids.clear()
      return ret
    elif len(faults_now) == 0 and len(self.last_faults_active) > 0:
      # 故障从有到无，记录日志并恢复正常；不强制输出一帧。
      cloudlog.info("ARS408 RadarState faults cleared")
      self.last_faults_active = set()

    # 跨调用缓冲：把当前批次的对象帧追加进 `self.cycle_objs`/`self.cycle_ids`。
    # - General(0x60B) 按索引写入基础字段（dRel/yRel/vRel/yvRel），索引由同批次的数组对齐。
    # - Extended(0x60D) 按 Obj_ID 显式对齐补充 aRel，允许跨批次到达。
    # 缓冲不依赖 `updated_messages`，每次调用都会尝试追加。
    #TODO: 需要解析60C和60D以增强目标判断的可靠性
    #TODO: 60C中需要判断目标存在的可能性
    #TODO: 60D中需要判断目标类型
    #TODO: 还需要解析60E中碰撞的相关信息以提醒Driver
    # ... Object_1_General：索引对齐的基础几何/速度（单位遵循规格）
    # - Obj_DistLong/Obj_DistLat: m（雷达坐标系：前为+，左为+）
    # - Obj_VrelLong/Obj_VrelLat: m/s（相对速度；前向为+，左向为+）
    obj_gen = self.rcp.vl_all.get("Obj_1_General", {})
    # 0x60D Object_3_Extended：按 Obj_ID 显式对齐的扩展属性
    # - Obj_ArelLong: 纵向加速度，m/s^2（相对加速度）
    # - Obj_Class: 目标类别枚举（0: 点目标，1: 小汽车，2: 卡车/公交，3: 行人，4: 摩托/踏板，5: 自行车，6: 宽目标/墙面等，7: 保留）
    obj_ext = self.rcp.vl_all.get("Obj_3_Extended", {})

    ids = obj_gen.get("Obj_ID", [])
    dist_long = obj_gen.get("Obj_DistLong", [])
    dist_lat = obj_gen.get("Obj_DistLat", [])
    vrel_long = obj_gen.get("Obj_VrelLong", [])
    vrel_lat = obj_gen.get("Obj_VrelLat", [])

    ids_ext = obj_ext.get("Obj_ID", [])
    arel_long = obj_ext.get("Obj_ArelLong", [])
    obj_class = obj_ext.get("Obj_Class", [])

    # 辅助：将浮点数组统一格式化为三位小数的字符串，避免日志小数位过长
    def _fmt3_list(arr):
      out = []
      for x in arr:
        try:
          if isinstance(x, (int, float)):
            out.append(f"{float(x):.3f}")
          else:
            out.append(str(x))
        except Exception:
          out.append(str(x))
      return out

    # 记录原始解析数组到日志，便于离线比对与调试（可能较为冗长）
    cloudlog.info(f"ARS408 Obj_1_General raw: count={len(ids)}; ids={list(ids)}; dist_long={_fmt3_list(dist_long)}; dist_lat={_fmt3_list(dist_lat)}; vrel_long={_fmt3_list(vrel_long)}; vrel_lat={_fmt3_list(vrel_lat)}")
    cloudlog.info(f"ARS408 Obj_3_Extended raw: count={len(ids_ext)}; ids={list(ids_ext)}; arel_long={_fmt3_list(arel_long)}; obj_class={list(obj_class)}")


    # General：按索引写入基础字段；数组长度可能不同，使用最小长度保证安全
    n_general = min(len(ids), len(dist_long), len(dist_lat), len(vrel_long), len(vrel_lat))
    for i in range(n_general):
      obj_id = int(ids[i])
      entry = self.cycle_objs.get(obj_id)
      if entry is None:
        entry = {}
        self.cycle_objs[obj_id] = entry
      # 基础几何/速度字段（单位：m / m/s）
      entry["dRel"] = round(float(dist_long[i]), 3)
      entry["yRel"] = round(float(dist_lat[i]), 3)
      entry["vRel"] = round(float(vrel_long[i]), 3)
      entry["yvRel"] = round(float(vrel_lat[i]), 3)
      entry["measured"] = True
      self.cycle_ids.add(obj_id)

    # Extended：用 Obj_ID 显式对齐补充加速度与类型（可能跨批次到达）
    n_ext = len(ids_ext)
    for j in range(n_ext):
      obj_id_ext = int(ids_ext[j])
      entry = self.cycle_objs.get(obj_id_ext)
      if entry is None:
        entry = {}
        self.cycle_objs[obj_id_ext] = entry
      # ArelLong 单位 m/s^2；若缺失保留为空以便后续用 NaN 填充
      if j < len(arel_long):
        entry["aRel"] = round(float(arel_long[j]), 3)
      # Obj_Class 为枚举；解析失败时忽略，不影响其他字段
      if j < len(obj_class):
        try:
          entry["objClass"] = int(obj_class[j])
        except Exception:
          pass
      entry["measured"] = True
      self.cycle_ids.add(obj_id_ext)

    # 0x60C Object_2_Quality：质量与存在概率（按 Obj_ID 对齐）
    # - Obj_ProbOfExist: 概率枚举（0: 无效，1: >25%，2: >75%，3: >90%，4: >99%，5: >99.9%，6: ≈100%）
    # - Obj_MeasState: 测量状态（0: 已删除，1: 新创建，2: 测量，3: 预测，4/5: 由聚类/融合产生等；不同版本可能存在差异）
    obj_qual = self.rcp.vl_all.get("Obj_2_Quality", {})
    ids_qual = obj_qual.get("Obj_ID", [])
    prob_exist = obj_qual.get("Obj_ProbOfExist", [])
    meas_state = obj_qual.get("Obj_MeasState", [])
    n_qual = min(len(ids_qual), len(prob_exist), len(meas_state))
    for k in range(n_qual):
      oid_q = int(ids_qual[k])
      entry_q = self.cycle_objs.get(oid_q)
      if entry_q is None:
        entry_q = {}
        self.cycle_objs[oid_q] = entry_q
      # 将质量属性保存在条目中，供门控/调试使用
      try:
        entry_q["probExist"] = int(prob_exist[k])
      except Exception:
        pass
      try:
        entry_q["measState"] = int(meas_state[k])
      except Exception:
        pass
      entry_q["measured"] = True
      self.cycle_ids.add(oid_q)

    # 0x60E Object_4_Warning：碰撞检测区域位图（bitfield）；具体位义由雷达端定义
    # 这里只做透传以便上层根据需求进行提示或进一步筛选
    # obj_warn = self.rcp.vl_all.get("Obj_4_Warning", {})
    # ids_warn = obj_warn.get("Obj_ID", [])
    # warn_bits = obj_warn.get("Obj_CollDetRegionBitfield", [])
    # n_warn = min(len(ids_warn), len(warn_bits))
    # for w in range(n_warn):
    #   oid_w = int(ids_warn[w])
    #   entry_w = self.cycle_objs.get(oid_w)
    #   if entry_w is None:
    #     entry_w = {}
    #     self.cycle_objs[oid_w] = entry_w
    #   try:
    #     entry_w["collDetRegionBits"] = int(warn_bits[w])
    #   except Exception:
    #     pass
    #   entry_w["measured"] = True
    #   self.cycle_ids.add(oid_w)

    # 触发判定：仅在收到触发帧（Obj_0_Status / 0x60A）时输出一个周期；
    # 若尚未触发，返回 None，但缓冲继续累积对象数据，等待后续批次触发。
    if self.trigger_msg not in self.updated_messages:
      return None

    # 从 Obj_0_Status 读取滚动计数与对象数（诊断用途），周期边界以触发帧到达为准。
    obj_status = self.rcp.vl.get("Obj_0_Status", {})
    try:
      meas_counter = int(obj_status.get("Obj_MeasCounter"))
    except Exception:
      meas_counter = None

    # 记录所有 0x60A 的 measurement counter 到自定义日志
    if meas_counter is not None:
      _write_custom_log_line(f"ARS408 Obj_0_Status MeasCounter={meas_counter}")
    # 诊断：判断滚动码是否相对上次有增加（考虑65536回绕），用于发现漏周期或重复帧现象（仅日志，不影响输出）。
    if meas_counter is not None:
      if self.last_meas_counter is not None:
        counter_delta = (meas_counter - self.last_meas_counter) & 0xFFFF
        if counter_delta == 0:
          cloudlog.warning(f"ARS408 Obj_MeasCounter did not increment: prev={self.last_meas_counter}, curr={meas_counter}")
        #ARS408 Obj_MeasCounter跳变超过2，可能表示丢帧或重复帧, 大陆雷达自增单位原本就是2
        elif counter_delta > 2:
          cloudlog.info(f"ARS408 Obj_MeasCounter jumped by {counter_delta} (missed cycles?): prev={self.last_meas_counter}, curr={meas_counter}")
      self.last_meas_counter = meas_counter

    # 周期完整性日志：对比 Obj_NofObjects 与缓冲中对象数（仅提示）。
    try:
      nof_objects = int(obj_status.get("Obj_NofObjects"))
    except Exception:
      nof_objects = None
    if nof_objects is not None:
      num_ids = len(self.cycle_ids)
      if num_ids < nof_objects:
        cloudlog.info(f"ARS408 cycle completeness: expected={nof_objects}, seen={num_ids}, missing={nof_objects - num_ids}, meas_counter={meas_counter}")
      elif num_ids > nof_objects:
        cloudlog.info(f"ARS408 cycle over-complete: expected={nof_objects}, seen={num_ids}, extra={num_ids - nof_objects}, meas_counter={meas_counter}")
      else:
        cloudlog.debug(f"ARS408 cycle complete: expected={nof_objects}, seen={num_ids}, meas_counter={meas_counter}")

    # 初始化返回对象与错误列表（包含 RadarState 的故障位与 CAN 有效性）
    ret = car.RadarData.new_message()
    errors = []
    if len(faults_now) > 0:
      errors.extend(sorted(list(faults_now)))
    if not self.rcp.can_valid:
      errors.append("canError")
    try:
      if radar_state.get("RadarState_Interference") or radar_state.get("RadarState_Voltage_Error"):
        if "interference" not in errors and int(radar_state.get("RadarState_Interference", 0)):
          errors.append("interference")
        if "voltageError" not in errors and int(radar_state.get("RadarState_Voltage_Error", 0)):
          errors.append("voltageError")
    except Exception:
      pass

    # 门控：
    # - 忽略点目标（Obj_Class == 0）
    # - 需要存在概率 >25%（ProbOfExist > 1）；不足则丢弃以降低噪声
    current_ids = set(self.cycle_ids)
    gated_ids = set()
    for obj_id in current_ids:
      entry = self.cycle_objs.get(obj_id, {})
      cls_val = entry.get("objClass")
      prob_val = entry.get("probExist")
      if cls_val is not None:
        try:
          if int(cls_val) == 0:
            continue
        except Exception:
          pass
      try:
        if prob_val is None or int(prob_val) <= 1:
          continue
      except Exception:
        continue

      gated_ids.add(obj_id)
      if obj_id not in self.pts:
        self.pts[obj_id] = car.RadarData.RadarPoint.new_message()
        self.pts[obj_id].trackId = obj_id
      # 轨迹字段单位说明：
      # - dRel/yRel: m；vRel/yvRel: m/s；aRel: m/s^2
      # 缺失字段使用 NaN，以区分“未测量”与有效零值
      self.pts[obj_id].dRel = round(float(entry.get("dRel", float('nan'))), 3)
      self.pts[obj_id].yRel = round(float(entry.get("yRel", float('nan'))), 3)
      self.pts[obj_id].vRel = round(float(entry.get("vRel", float('nan'))), 3)
      self.pts[obj_id].yvRel = round(float(entry.get("yvRel", float('nan'))), 3)
      self.pts[obj_id].aRel = round(float(entry.get("aRel", float('nan'))), 3)
      self.pts[obj_id].measured = bool(entry.get("measured", False))
      _write_custom_log_line(
        f"ARS408 RadarPoint trackId={obj_id}; "
        f"dRel={self.pts[obj_id].dRel:.3f}; "
        f"yRel={self.pts[obj_id].yRel:.3f}; "
        f"vRel={self.pts[obj_id].vRel:.3f}; "
        f"yvRel={self.pts[obj_id].yvRel:.3f}; "
        f"aRel={self.pts[obj_id].aRel:.3f}; "
        f"measured={self.pts[obj_id].measured}"
      )
    # 剪枝：删除未通过门控的旧轨迹，避免幽灵目标残留
    for old_id in list(self.pts.keys()):
      if old_id not in gated_ids:
        del self.pts[old_id]

    ret.errors = errors
    ret.points = list(self.pts.values())

    # 周期结束：清空触发集合与跨调用缓冲，准备下一周期；返回解析结果。
    self.updated_messages.clear()
    self.cycle_objs.clear()
    self.cycle_ids.clear()
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