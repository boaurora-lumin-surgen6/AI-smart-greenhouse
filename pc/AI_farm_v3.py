# -*- coding: utf-8 -*-
"""
================================================================================
 AI 智能农业仓储系统 · 第三版（端云协同鲁棒边缘智能温室系统）
 上位机主程序：AI_farm_v3.py

 在第二版 AI_farm_final.py 基础上升级，保留原有 GUI、串口连接、实时温湿度曲线、
 手动目标设置、设备状态显示，新增：
   1. 通信层：SEQ / TS / CHK / ACK / ERR 增强协议（默认关闭，兼容旧硬件）
   2. 控制层：滑动平均 + 标准差 + 斜率方向 + 趋势置信度
   3. 执行层：threshold / trend_assisted / abnormal_protection 三模式
   4. 保护层：死区、滞回、最小开关时间、设备互斥
   5. AI 层：DeepSeek → Qwen(阿里云百炼)，低频辅助，不参与 2 秒闭环
   6. 安全层：LLM 输出本地安全校验 + 人工确认，绝不直接控硬件
   7. 实验层：CSV 实验日志

 模块划分：Protocol / TrendPredictor / RobustController /
          QwenAdvisor / ExperimentLogger / EnvMonitorGUI

 运行：python AI_farm_v3.py
================================================================================
"""

import os
import re
import sys
import csv
import json
import time
import queue
import threading
from collections import deque


# ==============================================================================
#  0. 依赖检查：缺库给出清晰提示，而不是抛一堆红色异常
# ==============================================================================

def check_dependencies():
    """必需依赖缺失时给出可复制的安装命令。"""
    required = {
        "serial": "pyserial",
        "requests": "requests",
        "numpy": "numpy",
        "matplotlib": "matplotlib",
    }
    missing = []
    for mod, pkg in required.items():
        try:
            __import__(mod)
        except ImportError:
            missing.append(pkg)
    if missing:
        print("=" * 70)
        print("缺少必需依赖：%s" % ", ".join(missing))
        print("请执行：pip install -r requirements.txt")
        print("或：pip install %s" % " ".join(missing))
        print("=" * 70)
        sys.exit(1)
    return True


check_dependencies()

import serial
import serial.tools.list_ports
import numpy as np
import requests

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext

import matplotlib
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib import rcParams

rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "Arial Unicode MS",
                               "Segoe UI Emoji", "Segoe UI Symbol"]
rcParams["axes.unicode_minus"] = False


# ==============================================================================
#  1. 全局配置
# ==============================================================================

# ---- 协议开关：默认 False，保证现有硬件（第二版固件）能直接跑 ----
USE_ENHANCED_PROTOCOL = os.getenv("USE_ENHANCED_PROTOCOL", "0") == "1"

# ---- 第四版通信层开关（默认开：CRC 可切 + 自动重传 + 心跳状态机）----
# USE_V4_CRC8=1 时把帧校验从累加和换成 CRC8，**必须同步烧第四版固件**，
# 否则两端校验算法不一致会一直 ACK 超时。默认 0 = 继续用累加和。
USE_V4_CRC8 = os.getenv("USE_V4_CRC8", "0") == "1"
USE_V4_RETRY = os.getenv("USE_V4_RETRY", "1") == "1"

# ---- 第四版预测层开关（默认开：MAD 离群剔除 + EMA + 区间预测）----
# 纯软件、零硬件依赖，接口字段名与第三版完全一致，设 0 可一键退回。
USE_V4_PREDICT = os.getenv("USE_V4_PREDICT", "1") == "1"

# ---- 第四版控制层开关（默认关）----
# 增量式 PID + 时间比例占空比需要能承受周期性通断的执行器。
# 机械继电器建议先不打开；换 SSR（固态继电器）后再开，并把周期设到 30 秒以上。
USE_V4_DUTY = os.getenv("USE_V4_DUTY", "0") == "1"
DUTY_PERIOD_SEC = float(os.getenv("DUTY_PERIOD_SEC", "30"))
FF_MARGIN = float(os.getenv("FF_MARGIN", "0.3"))   # 热惯性前馈余量 ℃

# ---- ACK ---
ACK_TIMEOUT_SEC = 3.0

# ---- 采样 / 控制周期 ----
SAMPLE_INTERVAL_SEC = 2.0
CONTROL_INTERVAL_SEC = 2.0

# ---- 鲁棒控制参数（prompt 第五节指定值）----
TEMP_DEADBAND = 0.5        # 温度死区 ℃
HUMI_DEADBAND = 3.0        # 湿度死区 %
TEMP_HYSTERESIS = 0.8      # 温度滞回 ℃
HUMI_HYSTERESIS = 5.0      # 湿度滞回 %
MIN_SWITCH_INTERVAL = 10.0 # 最小开关间隔 秒

# ---- 趋势判断阈值（prompt 第四节指定值）----
STD_TEMP_MAX = 0.8         # 温度标准差上限，超过判为低可信度
STD_HUMI_MAX = 5.0         # 湿度标准差上限
SLOPE_TEMP_DEAD = 0.03     # 温度斜率死区，绝对值小于此判 stable
SLOPE_HUMI_DEAD = 0.2      # 湿度斜率死区
STD_TEMP_GOOD = 0.3        # 判 high 的温度标准差上限
STD_HUMI_GOOD = 2.0        # 判 high 的湿度标准差上限

# ---- 设备无效果检测 ----
DEVICE_EFFECT_WINDOW_SEC = 90.0   # 设备开启后观察窗口
DEVICE_EFFECT_MIN_DELTA = 1.0     # 窗口内环境变化低于此值 → 判为无效果

# ---- Qwen（阿里云百炼）----
QWEN_MODEL = os.getenv("QWEN_MODEL", "qwen-plus")
QWEN_BASE_URL = os.getenv(
    "QWEN_BASE_URL", "https://dashscope.aliyuncs.com/compatible-mode/v1"
)
QWEN_DISABLE_PROXY = os.getenv("QWEN_DISABLE_PROXY", "1") == "1"
QWEN_COOLDOWN_SEC = float(os.getenv("QWEN_COOLDOWN_SEC", "60"))

# ---- 数据缓存长度 ----
MAX_POINTS = 50

# ---- 实验日志 ----
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data")
LOG_PATH = os.path.join(LOG_DIR, "experiment_log.csv")


# ==============================================================================
#  2. Protocol —— 增强型串口协议（兼容旧协议）
# ==============================================================================

class Protocol:
    """
    旧协议（USE_ENHANCED_PROTOCOL=False）：
        DATA:T=25.6,H=62.3
        CTRL:HEAT=1,FAN=0,HUM=0
        PRED:T=25.2,H=60.1
        TARGET:T=18.0-24.0,H=55.0-75.0
        AI_ON

    增强协议（=True）：
        DATA:SEQ=12,TS=1730000000,T=25.6,H=62.3,HEAT=0,FAN=1,HUM=0,CHK=85
        CTRL:SEQ=33,TS=1730000002,HEAT=0,FAN=1,HUM=0,CHK=91
        PRED:SEQ=34,TS=1730000002,T=26.2,H=60.8,CONF=0.82,CHK=77
        TARGET:SEQ=35,TS=1730000003,TMIN=18,TMAX=24,HMIN=55,HMAX=75,CHK=66
        ACK:SEQ=33,OK=1
        ERR:SEQ=33,REASON=BAD_CHECKSUM

    CHK = 各字段字符串（不含 "TYPE:" 前缀、不含 ",CHK=" 部分）的 ASCII 求和 % 256
    """

    def __init__(self, enhanced=None):
        self.enhanced = USE_ENHANCED_PROTOCOL if enhanced is None else enhanced
        self._seq = 0
        self._lock = threading.Lock()
        # seq -> 发送时间戳，用于 ACK 超时检测
        self.ack_pending = {}
        self.ack_stats = {"sent": 0, "acked": 0, "timeout": 0, "err": 0,
                          "latency_sum": 0.0}

    # ---------- 序号 ----------
    def next_seq(self):
        with self._lock:
            self._seq = (self._seq + 1) % 100000
            return self._seq

    # ---------- 校验 ----------
    @staticmethod
    def checksum(body):
        """字段字符串 ASCII 求和 % 256。与 ESP32 端实现必须完全一致。"""
        return sum(ord(c) for c in body) % 256

    @staticmethod
    def mask_key(key):
        """只显示前后缀，绝不打印完整 API Key。"""
        if not key:
            return "<未设置>"
        if len(key) <= 10:
            return key[:3] + "***"
        return key[:6] + "***" + key[-4:]

    # ---------- 构造报文 ----------
    def build_ctrl(self, heater, fan, hum):
        if not self.enhanced:
            return "CTRL:HEAT=%d,FAN=%d,HUM=%d" % (heater, fan, hum)
        seq = self.next_seq()
        body = "SEQ=%d,TS=%d,HEAT=%d,FAN=%d,HUM=%d" % (
            seq, int(time.time()), heater, fan, hum)
        return "CTRL:%s,CHK=%d" % (body, self.checksum(body))

    def build_pred(self, pred_t, pred_h, confidence="medium"):
        conf_map = {"low": 0.35, "medium": 0.65, "high": 0.90}
        conf_val = conf_map.get(confidence, 0.65)
        if not self.enhanced:
            return "PRED:T=%.2f,H=%.2f" % (pred_t, pred_h)
        seq = self.next_seq()
        body = "SEQ=%d,TS=%d,T=%.2f,H=%.2f,CONF=%.2f" % (
            seq, int(time.time()), pred_t, pred_h, conf_val)
        return "PRED:%s,CHK=%d" % (body, self.checksum(body))

    def build_target(self, t_min, t_max, h_min, h_max):
        if not self.enhanced:
            return "TARGET:T=%.1f-%.1f,H=%.1f-%.1f" % (t_min, t_max, h_min, h_max)
        seq = self.next_seq()
        body = "SEQ=%d,TS=%d,TMIN=%.1f,TMAX=%.1f,HMIN=%.1f,HMAX=%.1f" % (
            seq, int(time.time()), t_min, t_max, h_min, h_max)
        return "TARGET:%s,CHK=%d" % (body, self.checksum(body))

    # ---------- 发送登记（增强协议）----------
    def register_pending(self, msg):
        """下发增强协议命令时登记 seq，供 ACK 超时检测使用。"""
        m = re.search(r"SEQ=(\d+)", msg)
        if m and self.enhanced:
            seq = int(m.group(1))
            self.ack_pending[seq] = time.time()
            self.ack_stats["sent"] += 1
            return seq
        return None

    # ---------- 解析 ----------
    def parse_line(self, line):
        """
        返回 dict：
          ok / type / reason / legacy / seq / ts / temp / humi /
          heater / fan / hum / pred_t / pred_h / confidence /
          t_min / t_max / h_min / h_max / ack_ok / err_reason
        解析失败不抛异常，ok=False 并给出 reason。
        """
        res = {
            "ok": False, "type": "UNKNOWN", "reason": "", "legacy": False,
            "seq": None, "ts": None, "temp": None, "humi": None,
            "heater": None, "fan": None, "hum": None,
            "pred_t": None, "pred_h": None, "confidence": None,
            "t_min": None, "t_max": None, "h_min": None, "h_max": None,
            "ack_ok": None, "err_reason": None, "raw": line,
        }
        if not line:
            res["reason"] = "EMPTY_LINE"
            return res

        s = line.strip()
        try:
            # ---------- ACK ----------
            if s.startswith("ACK:"):
                res["type"] = "ACK"
                m = re.search(r"ACK:SEQ=(\d+),OK=(\d)", s)
                if not m:
                    res["reason"] = "BAD_ACK_FORMAT"
                    return res
                res["seq"] = int(m.group(1))
                res["ack_ok"] = (m.group(2) == "1")
                res["ok"] = True
                return res

            # ---------- ERR ----------
            if s.startswith("ERR:"):
                res["type"] = "ERR"
                m = re.search(r"ERR:SEQ=(\d+),REASON=([A-Z_]+)", s)
                if not m:
                    res["reason"] = "BAD_ERR_FORMAT"
                    return res
                res["seq"] = int(m.group(1))
                res["err_reason"] = m.group(2)
                res["ok"] = True
                return res

            # ---------- DATA ----------
            if s.startswith("DATA:"):
                res["type"] = "DATA"
                if s.startswith("DATA:ERROR"):
                    res["reason"] = "SENSOR_ERROR"
                    return res
                # 增强 DATA
                m = re.match(
                    r"^DATA:SEQ=(\d+),TS=(\d+),T=(-?[\d.]+),H=(-?[\d.]+),"
                    r"HEAT=(\d),FAN=(\d),HUM=(\d),CHK=(\d+)$", s)
                if m:
                    body = s[len("DATA:"):s.rindex(",CHK=")]
                    if self.checksum(body) != int(m.group(8)):
                        res["reason"] = "BAD_CHECKSUM"
                        return res
                    res.update({
                        "ok": True, "legacy": False,
                        "seq": int(m.group(1)), "ts": int(m.group(2)),
                        "temp": float(m.group(3)), "humi": float(m.group(4)),
                        "heater": int(m.group(5)), "fan": int(m.group(6)),
                        "hum": int(m.group(7)),
                    })
                    return res
                # 旧 DATA
                m2 = re.search(r"DATA:T=(-?[\d.]+),H=(-?[\d.]+)", s)
                if m2:
                    res.update({
                        "ok": True, "legacy": True,
                        "temp": float(m2.group(1)), "humi": float(m2.group(2)),
                    })
                    return res
                res["reason"] = "UNPARSEABLE_DATA"
                return res

            # ---------- PRED（回环，上位机自发自收，仅作记录）----------
            if s.startswith("PRED:"):
                res["type"] = "PRED"
                m = re.search(r"T=(-?[\d.]+),H=(-?[\d.]+)", s)
                if m:
                    res.update({"ok": True,
                                "pred_t": float(m.group(1)),
                                "pred_h": float(m.group(2))})
                    c = re.search(r"CONF=([\d.]+)", s)
                    if c:
                        v = float(c.group(1))
                        res["confidence"] = ("high" if v >= 0.8
                                             else "low" if v < 0.5 else "medium")
                    return res
                res["reason"] = "UNPARSEABLE_PRED"
                return res

            # ---------- TARGET ----------
            if s.startswith("TARGET:"):
                res["type"] = "TARGET"
                m = re.search(r"TMIN=(-?[\d.]+),TMAX=(-?[\d.]+),"
                              r"HMIN=(-?[\d.]+),HMAX=(-?[\d.]+)", s)
                if m:
                    res.update({
                        "ok": True,
                        "t_min": float(m.group(1)), "t_max": float(m.group(2)),
                        "h_min": float(m.group(3)), "h_max": float(m.group(4)),
                    })
                    return res
                m2 = re.search(r"T=(-?[\d.]+)-(-?[\d.]+),H=(-?[\d.]+)-(-?[\d.]+)", s)
                if m2:
                    res.update({
                        "ok": True, "legacy": True,
                        "t_min": float(m2.group(1)), "t_max": float(m2.group(2)),
                        "h_min": float(m2.group(3)), "h_max": float(m2.group(4)),
                    })
                    return res
                res["reason"] = "UNPARSEABLE_TARGET"
                return res

            # ---------- CTRL（回环）----------
            if s.startswith("CTRL:"):
                res["type"] = "CTRL"
                m = re.search(r"HEAT=(\d),FAN=(\d),HUM=(\d)", s)
                if m:
                    res.update({"ok": True,
                                "heater": int(m.group(1)),
                                "fan": int(m.group(2)),
                                "hum": int(m.group(3))})
                    return res
                res["reason"] = "UNPARSEABLE_CTRL"
                return res

            if s == "AI_ON":
                res.update({"ok": True, "type": "AI_ON", "legacy": True})
                return res

            res["reason"] = "UNKNOWN_TYPE"
            return res

        except Exception as e:
            res["reason"] = "EXCEPTION:%s" % e
            return res

    # ---------- ACK 处理 / 超时检测（非阻塞）----------
    def on_ack(self, seq, ok):
        sent_ts = self.ack_pending.pop(seq, None)
        if sent_ts is None:
            return None, None
        latency = time.time() - sent_ts
        if ok:
            self.ack_stats["acked"] += 1
            self.ack_stats["latency_sum"] += latency
        else:
            self.ack_stats["err"] += 1
        return latency, ok

    def check_ack_timeout(self):
        """返回超时 seq 列表。主线程每 1 秒调用一次，不阻塞。"""
        now = time.time()
        expired = [seq for seq, ts in self.ack_pending.items()
                   if now - ts > ACK_TIMEOUT_SEC]
        for seq in expired:
            self.ack_pending.pop(seq, None)
            self.ack_stats["timeout"] += 1
        return expired

    def ack_summary(self):
        st = self.ack_stats
        sent = st["sent"]
        rate = (st["acked"] / sent * 100.0) if sent else 0.0
        avg_lat = (st["latency_sum"] / st["acked"]) if st["acked"] else 0.0
        return {"sent": sent, "acked": st["acked"], "timeout": st["timeout"],
                "err": st["err"], "rate": rate, "avg_latency": avg_lat}


# ==============================================================================
#  3. TrendPredictor —— 趋势判断（不是高精度预测）
# ==============================================================================

class TrendPredictor:
    """
    最近 10 个点算斜率 slope；最近 5 个点算滑动平均 smooth 与标准差 std。
    pred = smooth + slope（仅作趋势参考，UI 中称"趋势参考"，不称"高精度预测"）。
    """

    def __init__(self):
        self.slope_window = 10
        self.smooth_window = 5

    @staticmethod
    def _slope(values):
        """最小二乘斜率，点数不足返回 0.0。"""
        n = len(values)
        if n < 2:
            return 0.0
        x = np.arange(n, dtype=float)
        y = np.asarray(values, dtype=float)
        xm, ym = x.mean(), y.mean()
        denom = float(((x - xm) ** 2).sum())
        if denom == 0:
            return 0.0
        return float(((x - xm) * (y - ym)).sum() / denom)

    @staticmethod
    def _mean(values):
        return float(np.mean(values)) if values else 0.0

    @staticmethod
    def _std(values):
        return float(np.std(values)) if len(values) >= 2 else 0.0

    def analyze(self, temps, humis):
        temps = list(temps)
        humis = list(humis)

        smooth_t = self._mean(temps[-self.smooth_window:])
        smooth_h = self._mean(humis[-self.smooth_window:])
        std_t = self._std(temps[-self.smooth_window:])
        std_h = self._std(humis[-self.smooth_window:])

        # 数据不足 10 点：不报错，confidence=medium，用当前值做 smooth 和 pred
        if len(temps) < self.slope_window:
            cur_t = temps[-1] if temps else 0.0
            cur_h = humis[-1] if humis else 0.0
            return {
                "smooth_temp": cur_t, "smooth_humi": cur_h,
                "slope_temp": 0.0, "slope_humi": 0.0,
                "std_temp": std_t, "std_humi": std_h,
                "pred_temp": cur_t, "pred_humi": cur_h,
                "confidence": "medium",
                "temp_trend": "stable", "humi_trend": "stable",
                "insufficient": True,
            }

        slope_t = self._slope(temps[-self.slope_window:])
        slope_h = self._slope(humis[-self.slope_window:])

        # 趋势方向
        if std_t > STD_TEMP_MAX:
            temp_trend = "unstable"
        elif slope_t >= SLOPE_TEMP_DEAD:
            temp_trend = "rising"
        elif slope_t <= -SLOPE_TEMP_DEAD:
            temp_trend = "falling"
        else:
            temp_trend = "stable"

        if std_h > STD_HUMI_MAX:
            humi_trend = "unstable"
        elif slope_h >= SLOPE_HUMI_DEAD:
            humi_trend = "rising"
        elif slope_h <= -SLOPE_HUMI_DEAD:
            humi_trend = "falling"
        else:
            humi_trend = "stable"

        # 置信度分级
        noisy = (std_t > STD_TEMP_MAX) or (std_h > STD_HUMI_MAX)
        clear = (abs(slope_t) >= SLOPE_TEMP_DEAD) or (abs(slope_h) >= SLOPE_HUMI_DEAD)
        clean = (std_t < STD_TEMP_GOOD) and (std_h < STD_HUMI_GOOD)

        if noisy:
            confidence = "low"
        elif clean and clear:
            confidence = "high"
        else:
            confidence = "medium"

        return {
            "smooth_temp": smooth_t, "smooth_humi": smooth_h,
            "slope_temp": slope_t, "slope_humi": slope_h,
            "std_temp": std_t, "std_humi": std_h,
            "pred_temp": smooth_t + slope_t,
            "pred_humi": smooth_h + slope_h,
            "confidence": confidence,
            "temp_trend": temp_trend, "humi_trend": humi_trend,
            "insufficient": False,
        }


# ==============================================================================
#  4. RobustController —— 三模式鲁棒控制 + 安全约束
# ==============================================================================

class RobustController:
    """
    模式 A threshold          ：普通阈值控制
    模式 B trend_assisted     ：趋势辅助控制（仅 confidence == high 时启用）
    模式 C abnormal_protection：异常保护（confidence == low 或设备无效果）
    """

    def __init__(self):
        self.state = {"heater": 0, "fan": 0, "humidifier": 0}
        self.last_change = {"heater": 0.0, "fan": 0.0, "humidifier": 0.0}
        self.switch_count = {"heater": 0, "fan": 0, "humidifier": 0}
        # 设备开启时刻与开启时环境值，用于"开启后环境没有按预期变化"检测
        self.effect_watch = {
            "fan": {"on_at": None, "base": None},
            "heater": {"on_at": None, "base": None},
            "humidifier": {"on_at": None, "base": None},
        }
        self.mode = "threshold"
        self.reason = ""
        self.need_llm_diagnosis = False

    # ---------- 最小开关间隔 ----------
    def _can_switch(self, dev, now):
        return (now - self.last_change[dev]) >= MIN_SWITCH_INTERVAL

    # ---------- 应用状态并维护计数 ----------
    CONFLICT = {"heater": "fan", "fan": "heater", "humidifier": None}

    def _set(self, dev, value, now):
        self.state[dev] = value
        self.last_change[dev] = now
        self.switch_count[dev] += 1
        self.effect_watch[dev] = ({"on_at": now, "base": None} if value == 1
                                  else {"on_at": None, "base": None})

    def _apply(self, desired, now):
        for dev in ("heater", "fan", "humidifier"):
            want = int(desired.get(dev, 0))
            cur = self.state[dev]
            if want == cur:
                continue
            if not self._can_switch(dev, now):
                continue  # 最小开关时间未到，保持原状态

            if want == 1:
                # 互斥二次保险：冲突设备若还开着，必须先关掉；
                # 若它也卡在最小开关时间内，则本次干脆不开，绝不出现两者同时 ON
                other = self.CONFLICT.get(dev)
                if other and self.state[other] == 1:
                    if (int(desired.get(other, 0)) == 1
                            or not self._can_switch(other, now)):
                        continue
                    self._set(other, 0, now)
            self._set(dev, want, now)

    # ---------- 设备无效果检测 ----------
    def check_device_effect(self, cur_t, cur_h, now):
        """
        设备开启 DEVICE_EFFECT_WINDOW_SEC 后，环境变化低于阈值 → 判为无效果。
        返回 (有异常?, 描述)
        """
        problems = []
        watch = self.effect_watch
        for dev, key, delta_min, sign in (
                ("fan", "temp", 0.2, -1),
                ("heater", "temp", 0.2, +1),
                ("humidifier", "humi", DEVICE_EFFECT_MIN_DELTA, +1)):
            w = watch[dev]
            if w["on_at"] is None:
                continue
            if w["base"] is None:
                w["base"] = cur_t if key == "temp" else cur_h
                continue
            if now - w["on_at"] < DEVICE_EFFECT_WINDOW_SEC:
                continue
            cur_val = cur_t if key == "temp" else cur_h
            delta = (cur_val - w["base"]) * sign
            if delta < delta_min:
                problems.append("%s开启后%s未按预期变化" % (dev, "温度" if key == "temp" else "湿度"))
                w["on_at"] = now   # 重置观察窗口，避免每 2 秒重复告警
                w["base"] = cur_val
        return (len(problems) > 0), ("，".join(problems))

    # ---------- 主决策 ----------
    def decide(self, cur_t, cur_h, trend, t_min, t_max, h_min, h_max, now):
        conf = trend.get("confidence", "medium")
        temp_trend = trend.get("temp_trend", "stable")
        humi_trend = trend.get("humi_trend", "stable")
        pred_t = trend.get("pred_temp", cur_t)
        pred_h = trend.get("pred_humi", cur_h)

        desired = {"heater": 0, "fan": 0, "humidifier": 0}
        reasons = []
        mode = "threshold"
        need_llm = False

        # ======== 模式 C：异常保护 ========
        no_effect, no_effect_desc = self.check_device_effect(cur_t, cur_h, now)
        if conf == "low" or no_effect:
            mode = "abnormal_protection"
            need_llm = True
            if conf == "low":
                reasons.append("数据波动大，趋势不可信，退回普通阈值控制")
            if no_effect:
                reasons.append(no_effect_desc)

        # ======== 模式 A：普通阈值控制（带死区 + 滞回）========
        # 温度上限 → 风扇
        if self.state["fan"] == 1:
            if cur_t > t_max - TEMP_HYSTERESIS:
                desired["fan"] = 1
                reasons.append("温度仍高于上限-滞回，风扇保持")
        else:
            if cur_t > t_max + TEMP_DEADBAND:
                desired["fan"] = 1
                reasons.append("温度高于上限，开风扇")
            elif cur_t > t_max:
                reasons.append("温度略超上限但在死区内，不动")

        # 温度下限 → 加热
        if self.state["heater"] == 1:
            if cur_t < t_min + TEMP_HYSTERESIS:
                desired["heater"] = 1
                reasons.append("温度仍低于下限+滞回，加热保持")
        else:
            if cur_t < t_min - TEMP_DEADBAND:
                desired["heater"] = 1
                reasons.append("温度低于下限，开加热")
            elif cur_t < t_min:
                reasons.append("温度略低于下限但在死区内，不动")

        # 湿度下限 → 加湿
        if self.state["humidifier"] == 1:
            if cur_h < h_min + HUMI_HYSTERESIS:
                desired["humidifier"] = 1
                reasons.append("湿度仍低于下限+滞回，加湿保持")
        else:
            if cur_h < h_min - HUMI_DEADBAND:
                desired["humidifier"] = 1
                reasons.append("湿度低于下限，开加湿")
            elif cur_h < h_min:
                reasons.append("湿度略低于下限但在死区内，不动")

        # 湿度高于上限：本系统无除湿装置，只提示
        if cur_h > h_max + HUMI_DEADBAND:
            reasons.append("湿度高于上限，本系统无除湿装置，建议人工通风")

        # ======== 模式 B：趋势辅助控制（仅 high 置信度）========
        if conf == "high" and mode == "threshold":
            assisted = False
            if (temp_trend == "rising" and cur_t >= t_max - TEMP_DEADBAND
                    and desired["fan"] == 0):
                desired["fan"] = 1
                reasons.append("接近上限且升温中，趋势辅助提前开风扇")
                assisted = True
            if (temp_trend == "falling" and cur_t <= t_min + TEMP_DEADBAND
                    and desired["heater"] == 0):
                desired["heater"] = 1
                reasons.append("接近下限且降温中，趋势辅助提前开加热")
                assisted = True
            if (humi_trend == "falling" and cur_h <= h_min + HUMI_DEADBAND
                    and desired["humidifier"] == 0):
                desired["humidifier"] = 1
                reasons.append("接近湿度下限且下降中，趋势辅助提前加湿")
                assisted = True
            if assisted:
                mode = "trend_assisted"

        # ======== 安全约束 ========
        # 1. 加热与风扇禁止同时开启
        if desired["heater"] == 1 and desired["fan"] == 1:
            # 2/3. 高于上限优先风扇，低于下限优先加热，否则按偏离量决定
            over_high = cur_t > t_max
            under_low = cur_t < t_min
            if over_high and not under_low:
                desired["heater"] = 0
                reasons.append("互斥约束：温度超限，保留风扇")
            elif under_low and not over_high:
                desired["fan"] = 0
                reasons.append("互斥约束：温度偏低，保留加热")
            else:
                d_high = abs(cur_t - t_max)
                d_low = abs(cur_t - t_min)
                if d_high >= d_low:
                    desired["heater"] = 0
                    reasons.append("互斥约束：偏离上限更远，保留风扇")
                else:
                    desired["fan"] = 0
                    reasons.append("互斥约束：偏离下限更远，保留加热")

        # 4. 最小开关时间保护（_apply 内部处理）
        blocked = []
        for dev in ("heater", "fan", "humidifier"):
            if desired[dev] != self.state[dev] and not self._can_switch(dev, now):
                blocked.append(dev)
        if blocked:
            reasons.append("最小开关时间未到，%s 保持原状态" % "/".join(blocked))
        self._apply(desired, now)

        if not reasons:
            reasons.append("温湿度均在目标区间内，待机")

        self.mode = mode
        self.reason = "；".join(reasons)
        self.need_llm_diagnosis = need_llm

        return {
            "heater": self.state["heater"],
            "fan": self.state["fan"],
            "humidifier": self.state["humidifier"],
            "mode": mode,
            "reason": self.reason,
            "should_call_llm_diagnosis": need_llm,
            "pred_temp": pred_t,
            "pred_humi": pred_h,
        }


# ==============================================================================
#  5. QwenAdvisor —— 阿里云百炼低频农业辅助（不直接控硬件）
# ==============================================================================

class QwenAdvisor:
    """只做：作物参数推荐 / 策略分析 / 异常诊断。输出必须本地安全校验 + 人工确认。"""

    JSON_RULE = "请只返回合法 JSON，不要输出 Markdown，不要输出解释性文字。"

    def __init__(self):
        self.api_key, self.key_source = self._resolve_key()
        self.model = QWEN_MODEL
        self.base_url = QWEN_BASE_URL
        self.disable_proxy = QWEN_DISABLE_PROXY
        self.cooldown = QWEN_COOLDOWN_SEC
        self.last_call_ts = 0.0
        self.call_count = 0
        self._session = None

    # ---------- Key 解析 ----------
    @staticmethod
    def _key_path():
        """本地 Key 文件放在项目根目录，文件名为 .api_key（不进 git）。"""
        return os.path.abspath(os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "..", ".api_key"))

    @staticmethod
    def _read_key_file():
        try:
            p = QwenAdvisor._key_path()
            if os.path.exists(p):
                with open(p, "r", encoding="utf-8") as f:
                    v = f.read().strip()
                    if v:
                        return v
        except Exception:
            pass
        return None

    @staticmethod
    def _resolve_key():
        # 优先级：环境变量 > 本地文件
        for name in ("DASHSCOPE_API_KEY", "QWEN_API_KEY", "OPENAI_API_KEY"):
            v = os.getenv(name)
            if v and v.strip():
                return v.strip(), name
        v = QwenAdvisor._read_key_file()
        if v:
            return v, "本地文件 .api_key"
        return None, None

    def set_api_key(self, key, save=True):
        """运行时更换 Key。返回 (ok, 说明文字)。"""
        key = (key or "").strip()
        self.api_key = key or None
        self._session = None
        if not key:
            self.key_source = None
            return True, "已清空 API Key"
        if not save:
            self.key_source = "临时（未保存）"
            return True, "Key 已生效，重启后失效"
        try:
            p = self._key_path()
            with open(p, "w", encoding="utf-8") as f:
                f.write(key)
            try:
                os.chmod(p, 0o600)
            except Exception:
                pass
            self.key_source = "本地文件 .api_key"
            return True, "已保存到 %s" % p
        except Exception as e:
            self.key_source = "临时（保存失败）"
            return False, "保存失败：%s" % e

    def check_qwen_env(self):
        """返回 (ok, 说明文字)。"""
        if not self.api_key:
            return False, ("未找到 API Key。请在左侧「API Key」框填入 Key 点保存，"
                           "或设置环境变量 DASHSCOPE_API_KEY"
                           "（或 QWEN_API_KEY / OPENAI_API_KEY）")
        return True, ("Key 来源：%s，值：%s；模型：%s；代理：%s" % (
            self.key_source, Protocol.mask_key(self.api_key), self.model,
            "已禁用系统代理" if self.disable_proxy else "允许系统代理"))

    # ---------- 冷却 ----------
    def check_cooldown(self):
        left = self.cooldown - (time.time() - self.last_call_ts)
        if left > 0:
            return False, "LLM_COOLDOWN（还需等待 %.0f 秒）" % left
        return True, ""

    def _mark_called(self):
        self.last_call_ts = time.time()
        self.call_count += 1

    # ---------- HTTP ----------
    def _get_session(self):
        if self._session is None:
            self._session = requests.Session()
            if self.disable_proxy:
                self._session.trust_env = False
        return self._session

    def _chat(self, system_prompt, user_prompt):
        """返回 dict。优先 openai SDK，失败回退 requests。"""
        messages = [{"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}]
        try:
            from openai import OpenAI
            import httpx
        except ImportError:
            return self._chat_requests(messages)

        try:
            client = OpenAI(
                api_key=self.api_key,
                base_url=self.base_url,
                http_client=httpx.Client(timeout=30.0,
                                         trust_env=not self.disable_proxy),
            )
            resp = client.chat.completions.create(
                model=self.model, messages=messages, temperature=0.2,
                response_format={"type": "json_object"},
            )
            return self._to_json(resp.choices[0].message.content)
        except Exception as e:
            # SDK 调用失败不再二次请求，避免重复计费，直接把原因抛上去
            raise RuntimeError("OpenAI SDK 调用失败：%s" % e)

    def _chat_requests(self, messages):
        """openai/httpx 不可用时的 requests 回退通道。"""
        url = self.base_url.rstrip("/") + "/chat/completions"
        headers = {"Authorization": "Bearer %s" % self.api_key,
                   "Content-Type": "application/json"}
        payload = {"model": self.model, "messages": messages,
                   "temperature": 0.2,
                   "response_format": {"type": "json_object"}}
        s = self._get_session()
        kwargs = {"timeout": 30}
        if self.disable_proxy:
            kwargs["proxies"] = {"http": None, "https": None}
        r = s.post(url, headers=headers, json=payload, **kwargs)
        if r.status_code != 200:
            raise RuntimeError("API 错误 %s：%s" % (r.status_code, r.text[:200]))
        data = r.json()
        return self._to_json(data["choices"][0]["message"]["content"])

    @staticmethod
    def _to_json(content):
        """模型偶尔会带 ```json 围栏，这里做健壮提取。"""
        if content is None:
            raise RuntimeError("模型返回空内容")
        content = content.strip()
        if content.startswith("```"):
            content = re.sub(r"^```[a-zA-Z]*\s*", "", content)
            content = re.sub(r"\s*```$", "", content)
        try:
            return json.loads(content)
        except Exception:
            m = re.search(r"\{.*\}", content, re.DOTALL)
            if not m:
                raise RuntimeError("模型返回的不是 JSON：%s" % content[:200])
            return json.loads(m.group())

    # ---------- 三个功能 ----------
    def ask_crop_params(self, crop, stage, cultivation):
        sys_p = "你是一个专业的农业专家助手。" + self.JSON_RULE
        usr_p = (
            '请为作物"%s"（生长阶段：%s，种植方式：%s）推荐温室种植的适宜环境参数。\n'
            '严格按以下 JSON 返回：\n'
            '{"crop":"作物名","stage":"阶段","cultivation":"种植方式",'
            '"temp_range":[最低温,最高温],"humi_range":[最低湿,最高湿],'
            '"control_priority":["优先级1","优先级2"],'
            '"risk":"风险描述","need_human_confirm":true}'
            % (crop, stage, cultivation))
        return self._chat(sys_p, usr_p)

    def ask_strategy(self, ctx):
        sys_p = "你是一个专业的温室控制策略顾问。" + self.JSON_RULE
        usr_p = (
            "当前温室状态如下：\n"
            "作物=%s，阶段=%s，种植方式=%s\n"
            "当前温度=%.1f℃，当前湿度=%.1f%%\n"
            "平滑温度=%.1f℃，平滑湿度=%.1f%%\n"
            "温度斜率=%.3f，湿度斜率=%.3f\n"
            "温度标准差=%.3f，湿度标准差=%.3f\n"
            "趋势置信度=%s，温度趋势=%s，湿度趋势=%s\n"
            "设备状态：加热=%d，风扇=%d，加湿=%d\n"
            "目标区间：温度 %.1f~%.1f℃，湿度 %.1f~%.1f%%\n\n"
            "请给出控制策略建议，严格按以下 JSON 返回：\n"
            '{"strategy":"策略描述","suggested_rules":'
            '{"fan_on_when_pred_temp_above":25.5,'
            '"heater_on_when_pred_temp_below":18.0,'
            '"humidifier_on_when_pred_humi_below":55},'
            '"human_message":"给操作人员的建议",'
            '"safety_warning":"安全提醒","need_human_confirm":true}'
            % (ctx.get("crop"), ctx.get("stage"), ctx.get("cultivation"),
               ctx.get("cur_t", 0), ctx.get("cur_h", 0),
               ctx.get("smooth_t", 0), ctx.get("smooth_h", 0),
               ctx.get("slope_t", 0), ctx.get("slope_h", 0),
               ctx.get("std_t", 0), ctx.get("std_h", 0),
               ctx.get("confidence"), ctx.get("temp_trend"), ctx.get("humi_trend"),
               ctx.get("heater", 0), ctx.get("fan", 0), ctx.get("humidifier", 0),
               ctx.get("t_min", 0), ctx.get("t_max", 0),
               ctx.get("h_min", 0), ctx.get("h_max", 0)))
        return self._chat(sys_p, usr_p)

    def ask_diagnosis(self, ctx):
        sys_p = "你是一个农业设备故障诊断专家。" + self.JSON_RULE
        usr_p = (
            "温室出现异常，信息如下：\n"
            "异常类型=%s\n"
            "最近温度序列=%s\n"
            "最近湿度序列=%s\n"
            "设备状态：加热=%d，风扇=%d，加湿=%d\n"
            "目标区间：温度 %.1f~%.1f℃，湿度 %.1f~%.1f%%\n\n"
            "请给出诊断，严格按以下 JSON 返回：\n"
            '{"abnormal_type":"异常类型","possible_causes":["原因1","原因2"],'
            '"check_steps":["步骤1","步骤2"],'
            '"safe_action":"安全处置建议","need_human_confirm":true}'
            % (ctx.get("abnormal", "unknown"),
               ctx.get("temp_series", []), ctx.get("humi_series", []),
               ctx.get("heater", 0), ctx.get("fan", 0), ctx.get("humidifier", 0),
               ctx.get("t_min", 0), ctx.get("t_max", 0),
               ctx.get("h_min", 0), ctx.get("h_max", 0)))
        return self._chat(sys_p, usr_p)

    def test_connection(self):
        sys_p = "你是一个连通性测试助手。" + self.JSON_RULE
        usr_p = '请返回：{"ok":true,"message":"pong"}'
        return self._chat(sys_p, usr_p)

    # ---------- 本地安全校验 ----------
    def validate_llm_advice(self, data, advice_type, cooldown_ok=True):
        """返回 (valid: bool, reason: str, 已清洗的数据 or None)"""
        if not isinstance(data, dict):
            return False, "LLM_RETURN_NOT_JSON", None
        if not cooldown_ok:
            return False, "LLM_COOLDOWN", None
        if not data.get("need_human_confirm", False):
            return False, "MISSING_HUMAN_CONFIRM_FLAG", None

        if advice_type == "crop":
            for k in ("crop", "stage", "cultivation", "temp_range",
                      "humi_range", "control_priority", "risk"):
                if k not in data:
                    return False, "MISSING_FIELD:%s" % k, None
            tr, hr = data["temp_range"], data["humi_range"]
            if not (isinstance(tr, list) and len(tr) == 2):
                return False, "BAD_TEMP_RANGE", None
            if not (isinstance(hr, list) and len(hr) == 2):
                return False, "BAD_HUMI_RANGE", None
            if not (5 <= float(tr[0]) <= 45 and 5 <= float(tr[1]) <= 45):
                return False, "TEMP_OUT_OF_SAFE_RANGE(5~45℃)", None
            if not (20 <= float(hr[0]) <= 95 and 20 <= float(hr[1]) <= 95):
                return False, "HUMI_OUT_OF_SAFE_RANGE(20~95%)", None
            if float(tr[0]) >= float(tr[1]):
                return False, "TEMP_RANGE_REVERSED", None
            if float(hr[0]) >= float(hr[1]):
                return False, "HUMI_RANGE_REVERSED", None

        elif advice_type == "strategy":
            for k in ("strategy", "suggested_rules", "human_message",
                      "safety_warning"):
                if k not in data:
                    return False, "MISSING_FIELD:%s" % k, None
            rules = data["suggested_rules"]
            if not isinstance(rules, dict):
                return False, "BAD_SUGGESTED_RULES", None
            for key, lo, hi in (("fan_on_when_pred_temp_above", 5, 45),
                                ("heater_on_when_pred_temp_below", 5, 45),
                                ("humidifier_on_when_pred_humi_below", 20, 95)):
                if key in rules:
                    try:
                        v = float(rules[key])
                    except Exception:
                        return False, "BAD_RULE_VALUE:%s" % key, None
                    if not (lo <= v <= hi):
                        return False, "RULE_OUT_OF_RANGE:%s" % key, None

        elif advice_type == "diagnosis":
            for k in ("abnormal_type", "possible_causes", "check_steps",
                      "safe_action"):
                if k not in data:
                    return False, "MISSING_FIELD:%s" % k, None
            if not isinstance(data["possible_causes"], list):
                return False, "BAD_POSSIBLE_CAUSES", None
            if not isinstance(data["check_steps"], list):
                return False, "BAD_CHECK_STEPS", None

        else:
            return False, "UNKNOWN_ADVICE_TYPE", None

        # 通用：不允许 0/1 以外的值，不允许同时加热 + 风扇
        for key in ("heater", "fan", "humidifier"):
            if key in data and data[key] not in (0, 1, True, False, None):
                return False, "BAD_DEVICE_VALUE:%s" % key, None
        if data.get("heater") in (1, True) and data.get("fan") in (1, True):
            return False, "CONFLICT_HEATER_AND_FAN", None

        return True, "", data


# ==============================================================================
#  6. ExperimentLogger —— CSV 实验日志
# ==============================================================================

CSV_FIELDS = [
    "timestamp", "elapsed", "row_type", "raw_line", "parse_ok", "reason",
    "seq", "temp", "humi",
    "smooth_temp", "smooth_humi", "slope_temp", "slope_humi",
    "std_temp", "std_humi", "pred_temp", "pred_humi",
    "confidence", "temp_trend", "humi_trend",
    "t_min", "t_max", "h_min", "h_max",
    "heater", "fan", "humidifier", "mode", "action_reason",
    "switch_count_heater", "switch_count_fan", "switch_count_humidifier",
    "llm_called", "llm_event", "llm_valid", "llm_blocked_reason",
    "ack_status",
    # ---- 第四版新增列（设在最后一律追加，老 CSV 不会错位）----
    "pred_temp_lo", "pred_temp_hi", "pred_humi_lo", "pred_humi_hi",
    "outlier_temp", "outlier_humi", "clean_temp", "clean_humi",
    "std_raw_temp", "std_raw_humi", "window_temp", "link_state",
    "retransmit", "ack_rate",
]


class ExperimentLogger:
    """写入 data/experiment_log.csv，目录不存在自动创建。"""

    def __init__(self, path=None):
        self.path = os.path.abspath(path or LOG_PATH)
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        self._lock = threading.Lock()
        self.row_count = 0
        # 第四版通信层的每轮快照（link_state / retransmit / ack_rate 三个列）
        self.link_v4_extra = {}
        self._prepare_file()
        if not os.path.exists(self.path) or os.path.getsize(self.path) == 0:
            # 只在新建文件时写 BOM（Excel 中文不乱码）
            with open(self.path, "w", newline="", encoding="utf-8-sig") as f:
                csv.writer(f).writerow(CSV_FIELDS)

    def _prepare_file(self):
        """
        列数变化时的迁移。

        第四版新增了 14 列（预测区间、离群标记、链路状态等）。如果旧 CSV 已经存在，
        直接按新列表追加会让每一行都比表头多出十几个格子，Excel 里从第 38 列开始全错位。
        所以检测到表头不一致时，把旧文件改名存档（数据不丢），再用新表头重新开始。
        """
        if not os.path.exists(self.path) or os.path.getsize(self.path) == 0:
            return
        try:
            with open(self.path, "r", encoding="utf-8-sig", newline="") as f:
                first = f.readline()
            header = next(csv.reader([first]), [])
            if header == CSV_FIELDS:
                return
            stamp = time.strftime("%Y%m%d_%H%M%S")
            base, ext = os.path.splitext(self.path)
            archive = "%s_v3_%s%s" % (base, stamp, ext)
            os.rename(self.path, archive)
            print("[CSV] 检测到旧格式（%d 列），已存档为：%s" % (len(header), archive))
        except Exception as e:
            print("[CSV] 迁移检查失败：%s" % e)

    def _blank(self):
        return {k: "" for k in CSV_FIELDS}

    def log(self, **kw):
        row = self._blank()
        row["timestamp"] = time.strftime("%Y-%m-%d %H:%M:%S")
        for k, v in kw.items():
            if k in row:
                row[k] = v
        with self._lock:
            try:
                # 追加时用 utf-8（不能用 utf-8-sig，否则每次 open 都会再写一个 BOM）
                with open(self.path, "a", newline="", encoding="utf-8") as f:
                    csv.writer(f).writerow([row[k] for k in CSV_FIELDS])
                self.row_count += 1
            except Exception as e:
                print("[CSV] 写入失败：%s" % e)

    def log_data(self, parsed, elapsed, trend=None, ctrl_state=None):
        row = {
            "elapsed": round(elapsed, 2), "row_type": "DATA",
            "raw_line": parsed.get("raw", ""),
            "parse_ok": 1 if parsed.get("ok") else 0,
            "reason": parsed.get("reason", ""),
            "seq": parsed.get("seq") if parsed.get("seq") is not None else "",
            "temp": parsed.get("temp") if parsed.get("temp") is not None else "",
            "humi": parsed.get("humi") if parsed.get("humi") is not None else "",
        }
        if trend:
            row.update({
                "smooth_temp": round(trend["smooth_temp"], 3),
                "smooth_humi": round(trend["smooth_humi"], 3),
                "slope_temp": round(trend["slope_temp"], 4),
                "slope_humi": round(trend["slope_humi"], 4),
                "std_temp": round(trend["std_temp"], 4),
                "std_humi": round(trend["std_humi"], 4),
                "pred_temp": round(trend["pred_temp"], 3),
                "pred_humi": round(trend["pred_humi"], 3),
                "confidence": trend["confidence"],
                "temp_trend": trend["temp_trend"],
                "humi_trend": trend["humi_trend"],
            })
            for k in ("pred_temp_lo", "pred_temp_hi", "pred_humi_lo",
                      "pred_humi_hi", "outlier_temp", "outlier_humi",
                      "clean_temp", "clean_humi", "std_raw_temp",
                      "std_raw_humi", "window_temp"):
                if k in trend:
                    row[k] = trend[k]
            if self.link_v4_extra:
                row.update(self.link_v4_extra)
        if ctrl_state:
            row.update(ctrl_state)
        self.log(**row)

    def log_decision(self, elapsed, cur_t, cur_h, trend, decision,
                     targets, switch_counts, ack_status="", llm=None):
        row = {
            "elapsed": round(elapsed, 2), "row_type": "CONTROL",
            "parse_ok": 1, "temp": round(cur_t, 2), "humi": round(cur_h, 2),
            "smooth_temp": round(trend["smooth_temp"], 3),
            "smooth_humi": round(trend["smooth_humi"], 3),
            "slope_temp": round(trend["slope_temp"], 4),
            "slope_humi": round(trend["slope_humi"], 4),
            "std_temp": round(trend["std_temp"], 4),
            "std_humi": round(trend["std_humi"], 4),
            "pred_temp": round(decision["pred_temp"], 3),
            "pred_humi": round(decision["pred_humi"], 3),
            "confidence": trend["confidence"],
            "temp_trend": trend["temp_trend"],
            "humi_trend": trend["humi_trend"],
            "t_min": targets[0], "t_max": targets[1],
            "h_min": targets[2], "h_max": targets[3],
            "heater": decision["heater"], "fan": decision["fan"],
            "humidifier": decision["humidifier"],
            "mode": decision["mode"], "action_reason": decision["reason"],
            "switch_count_heater": switch_counts["heater"],
            "switch_count_fan": switch_counts["fan"],
            "switch_count_humidifier": switch_counts["humidifier"],
            "ack_status": ack_status,
        }
        # 第四版附加列：键不存在时留空，保证第三版逻辑下也能写
        for k in ("pred_temp_lo", "pred_temp_hi", "pred_humi_lo", "pred_humi_hi",
                  "outlier_temp", "outlier_humi", "clean_temp", "clean_humi",
                  "std_raw_temp", "std_raw_humi", "window_temp"):
            if k in trend:
                row[k] = trend[k]
        if self.link_v4_extra:
            row.update(self.link_v4_extra)
        if llm:
            row.update({"llm_called": llm.get("called", ""),
                        "llm_event": llm.get("event", ""),
                        "llm_valid": llm.get("valid", ""),
                        "llm_blocked_reason": llm.get("blocked_reason", "")})
        self.log(**row)


# ==============================================================================
#  7. EnvMonitorGUI —— 界面（保留第二版全部元素 + 第三版新增）
# ==============================================================================

class EnvMonitorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("AI 智能农场 · 第三版 端云协同鲁棒温室系统")
        self.root.geometry("1420x880")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        # ---- 串口 ----
        self.ser = None
        self.connected = False
        self.stop_flag = False

        # ---- 线程安全队列：所有子线程只往队列里放，主线程统一处理 ----
        self.data_queue = queue.Queue()
        self.log_queue = queue.Queue()
        self.ui_queue = queue.Queue()

        # ---- 数据缓存 ----
        self.temp_data = deque(maxlen=MAX_POINTS)
        self.humi_data = deque(maxlen=MAX_POINTS)
        self.time_data = deque(maxlen=MAX_POINTS)
        self.current_temp = 0.0
        self.current_humi = 0.0
        self.start_time = time.time()

        # ---- 目标区间 ----
        self.t_min, self.t_max = 18.0, 25.0
        self.h_min, self.h_max = 40.0, 70.0

        # ---- 模块实例 ----
        self.protocol = Protocol()
        # 第四版预测层：MAD 离群剔除 + EMA + 区间预测 + 自适应窗口。
        # 接口字段名与第三版完全一致，设 USE_V4_PREDICT=0 一键退回。
        if USE_V4_PREDICT:
            try:
                from predict_v4 import TrendPredictorV4
                self.predictor = TrendPredictorV4(dt=CONTROL_INTERVAL_SEC)
                self.v4_predict = True
            except Exception as e:      # 依赖缺失时不允许整个程序起不来
                self.predictor = TrendPredictor()
                self.v4_predict = False
                self._v4_fallback_reason = str(e)
        else:
            self.predictor = TrendPredictor()
            self.v4_predict = False
        self.controller = RobustController()
        self.advisor = QwenAdvisor()
        self.logger = ExperimentLogger()

        # ---- 状态 ----
        self.trend = None
        self.decision = None
        self.ai_active = False
        self.last_action = "无"
        self.last_logged_action = ""
        self.last_advice = "（暂无）"
        self.pending_advice = None      # 待人工确认的建议
        self.llm_event = ""
        # ---- 第四版通信层：自动重传 + 心跳状态机 ----
        self.use_v4_retry = USE_V4_RETRY
        self.retransmit = None
        self.heartbeat = None
        self.link_state = "OFFLINE"
        if USE_V4_RETRY:
            try:
                from link_v4 import RetransmitManager, HeartbeatMonitor
                self.retransmit = RetransmitManager()
                self.heartbeat = HeartbeatMonitor()
            except Exception:
                self.retransmit = None
                self.heartbeat = None
        # ---- 第四版控制层：热惯性前馈（默认开）+ PID 占空比（默认关）----
        self.ff_enabled = True
        self.ff_handler = None
        self.duty_enabled = USE_V4_DUTY
        self._duty_controllers = None
        try:
            from duty_pid import feedforward_shutoff
            self.ff_handler = feedforward_shutoff
        except Exception:
            self.ff_enabled = False
        if self.duty_enabled:
            try:
                from duty_pid import IncrementalPID, TimeProportionalDuty
                self._duty_controllers = {
                    "heater": {"pid": IncrementalPID(dt=CONTROL_INTERVAL_SEC),
                               "tp": TimeProportionalDuty(DUTY_PERIOD_SEC)},
                }
            except Exception:
                self.duty_enabled = False
        self.ack_status = ""
        self.comm_stats = {"total": 0, "ok": 0, "fail": 0}

        # ---- UI ----
        self.build_ui()

        # ---- 定时器 ----
        self.root.after(100, self.process_queues)
        self.root.after(200, self.update_ui)
        self.root.after(1000, self.check_ack_loop)

        # ---- 启动日志 ----
        ok, msg = self.advisor.check_qwen_env()
        self.log(("✅ Qwen 环境：%s" % msg) if ok else ("⚠️ %s" % msg))
        self.log("Qwen 请求代理策略：%s" %
                 ("已禁用系统代理" if self.advisor.disable_proxy else "允许使用系统代理"))
        self.log("串口协议模式：%s（可在左侧勾选切换）" %
                 ("增强协议 SEQ/TS/CHK/ACK/ERR" if self.protocol.enhanced else "旧协议（兼容）"))
        self.log("实验日志：%s" % self.logger.path)

    # =================== UI 构建 ===================
    def build_ui(self):
        main_frame = ttk.Frame(self.root, padding=5)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # ---------- 底部日志（必须最先 pack：通栏，窗口再矮也不会被挤掉）----------
        log_frame = ttk.LabelFrame(main_frame, text="系统日志（所有收发与决策都在这里）")
        log_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=5, pady=(0, 5))
        self.log_text = scrolledtext.ScrolledText(log_frame, height=7,
                                                  state="disabled", wrap=tk.WORD)
        self.log_text.pack(fill=tk.BOTH, expand=True)

        # ---------- 左侧 ----------
        left_frame = ttk.LabelFrame(main_frame, text="控制中心", width=400)
        left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=5)
        left_frame.pack_propagate(False)

        # 串口设置
        ttk.Label(left_frame, text="串口设置", font=("Arial", 10, "bold")).pack(
            anchor=tk.W, pady=(5, 0))
        port_frame = ttk.Frame(left_frame)
        port_frame.pack(fill=tk.X, pady=2)
        ttk.Label(port_frame, text="端口:").pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(port_frame, width=12)
        self.port_combo.pack(side=tk.RIGHT)
        self.refresh_ports()
        ttk.Button(port_frame, text="刷新", command=self.refresh_ports,
                   width=5).pack(side=tk.RIGHT, padx=2)

        baud_frame = ttk.Frame(left_frame)
        baud_frame.pack(fill=tk.X, pady=2)
        ttk.Label(baud_frame, text="波特率:").pack(side=tk.LEFT)
        self.baud_combo = ttk.Combobox(
            baud_frame, width=12, values=["9600", "115200", "57600", "38400"])
        self.baud_combo.set("115200")
        self.baud_combo.pack(side=tk.RIGHT)

        self.connect_btn = ttk.Button(left_frame, text="连接设备",
                                      command=self.toggle_connection)
        self.connect_btn.pack(fill=tk.X, pady=5)

        self.enhanced_var = tk.BooleanVar(value=self.protocol.enhanced)
        ttk.Checkbutton(left_frame, text="启用增强协议 (SEQ/TS/CHK/ACK/ERR)",
                        variable=self.enhanced_var,
                        command=self.toggle_protocol).pack(anchor=tk.W)

        ttk.Separator(left_frame, orient="horizontal").pack(fill=tk.X, pady=5)

        # 作物信息
        ttk.Label(left_frame, text="AI 智能种植（Qwen 低频辅助）",
                  font=("Arial", 10, "bold")).pack(anchor=tk.W)

        # API Key 输入（优先级低于环境变量，高于空白；可保存到本地 .api_key）
        key_frame = ttk.Frame(left_frame)
        key_frame.pack(fill=tk.X, pady=2)
        ttk.Label(key_frame, text="API Key:").pack(side=tk.LEFT)
        self.key_entry = ttk.Entry(key_frame, width=12, show="*")
        self.key_entry.pack(side=tk.RIGHT)
        if self.advisor.api_key:
            self.key_entry.insert(0, self.advisor.api_key)

        key_btn_frame = ttk.Frame(left_frame)
        key_btn_frame.pack(fill=tk.X, pady=1)
        ttk.Button(key_btn_frame, text="💾 保存Key",
                   command=self.save_api_key).pack(side=tk.LEFT, expand=True,
                                                   fill=tk.X)
        ttk.Button(key_btn_frame, text="👁 显示/隐藏",
                   command=self.toggle_key_visibility).pack(
                       side=tk.LEFT, expand=True, fill=tk.X)

        crop_frame = ttk.Frame(left_frame)
        crop_frame.pack(fill=tk.X, pady=2)
        ttk.Label(crop_frame, text="作物:").pack(side=tk.LEFT)
        self.crop_entry = ttk.Entry(crop_frame, width=12)
        self.crop_entry.pack(side=tk.RIGHT)
        self.crop_entry.insert(0, "生菜")

        stage_frame = ttk.Frame(left_frame)
        stage_frame.pack(fill=tk.X, pady=2)
        ttk.Label(stage_frame, text="阶段:").pack(side=tk.LEFT)
        self.stage_combo = ttk.Combobox(
            stage_frame, width=10,
            values=["幼苗期", "生长期", "开花期", "结果期"])
        self.stage_combo.set("幼苗期")
        self.stage_combo.pack(side=tk.RIGHT)

        cult_frame = ttk.Frame(left_frame)
        cult_frame.pack(fill=tk.X, pady=2)
        ttk.Label(cult_frame, text="方式:").pack(side=tk.LEFT)
        self.cult_combo = ttk.Combobox(
            cult_frame, width=10, values=["土培", "水培", "基质栽培"])
        self.cult_combo.set("水培")
        self.cult_combo.pack(side=tk.RIGHT)

        self.crop_btn = ttk.Button(left_frame, text="🌱 AI作物参数推荐",
                                   command=lambda: self.run_qwen("crop"))
        self.crop_btn.pack(fill=tk.X, pady=1)
        self.strategy_btn = ttk.Button(left_frame, text="📊 AI策略分析",
                                       command=lambda: self.run_qwen("strategy"))
        self.strategy_btn.pack(fill=tk.X, pady=1)
        self.diag_btn = ttk.Button(left_frame, text="🩺 AI异常诊断",
                                   command=lambda: self.run_qwen("diagnosis"))
        self.diag_btn.pack(fill=tk.X, pady=1)
        self.test_btn = ttk.Button(left_frame, text="🔌 测试Qwen连接",
                                   command=lambda: self.run_qwen("test"))
        self.test_btn.pack(fill=tk.X, pady=1)

        # 待确认建议 + 应用按钮紧跟 AI 按钮组：人工确认动作必须一眼可见
        self.advice_label = ttk.Label(left_frame, text="🗒 最近Qwen建议: （暂无）",
                                      wraplength=380, justify=tk.LEFT,
                                      font=("Arial", 9))
        self.advice_label.pack(anchor=tk.W, pady=(4, 0))
        self.apply_btn = ttk.Button(left_frame, text="✅ 应用上一条 Qwen 建议",
                                    command=self.apply_pending_advice,
                                    state="disabled")
        self.apply_btn.pack(fill=tk.X, pady=2)

        # 手动目标
        manual_frame = ttk.LabelFrame(left_frame, text="手动设定区间")
        manual_frame.pack(fill=tk.X, pady=5)
        ttk.Label(manual_frame, text="温度(℃):").grid(row=0, column=0, sticky=tk.W)
        self.t_min_var = tk.DoubleVar(value=18)
        self.t_max_var = tk.DoubleVar(value=25)
        ttk.Entry(manual_frame, textvariable=self.t_min_var, width=5).grid(
            row=0, column=1)
        ttk.Label(manual_frame, text="~").grid(row=0, column=2)
        ttk.Entry(manual_frame, textvariable=self.t_max_var, width=5).grid(
            row=0, column=3)
        ttk.Button(manual_frame, text="应用",
                   command=self.apply_manual_target).grid(row=0, column=4, padx=5)
        ttk.Label(manual_frame, text="湿度(%):").grid(row=1, column=0, sticky=tk.W)
        self.h_min_var = tk.DoubleVar(value=40)
        self.h_max_var = tk.DoubleVar(value=70)
        ttk.Entry(manual_frame, textvariable=self.h_min_var, width=5).grid(
            row=1, column=1)
        ttk.Label(manual_frame, text="~").grid(row=1, column=2)
        ttk.Entry(manual_frame, textvariable=self.h_max_var, width=5).grid(
            row=1, column=3)

        ttk.Separator(left_frame, orient="horizontal").pack(fill=tk.X, pady=5)

        # 实时数据
        ttk.Label(left_frame, text="实时数据",
                  font=("Arial", 10, "bold")).pack(anchor=tk.W)
        f_small = ("Arial", 9)
        self.temp_label = ttk.Label(left_frame, text="🌡 温度: -- °C",
                                    font=("Arial", 11))
        self.temp_label.pack(anchor=tk.W)
        self.humi_label = ttk.Label(left_frame, text="💧 湿度: -- %",
                                    font=("Arial", 11))
        self.humi_label.pack(anchor=tk.W)
        self.pred_label = ttk.Label(left_frame, text="📈 趋势参考: -- °C / -- %",
                                    foreground="blue")
        self.pred_label.pack(anchor=tk.W)
        self.range_label = ttk.Label(left_frame, text="🎯 目标区间: --",
                                     foreground="green")
        self.range_label.pack(anchor=tk.W)

        self.smooth_label = ttk.Label(left_frame, text="平滑: -- / --", font=f_small)
        self.smooth_label.pack(anchor=tk.W)
        self.std_label = ttk.Label(left_frame, text="标准差: -- / --", font=f_small)
        self.std_label.pack(anchor=tk.W)
        self.slope_label = ttk.Label(left_frame, text="斜率: -- / --", font=f_small)
        self.slope_label.pack(anchor=tk.W)

        self.conf_label = ttk.Label(left_frame, text="🔎 趋势置信度: --")
        self.conf_label.pack(anchor=tk.W)
        self.trend_label = ttk.Label(left_frame, text="📉 趋势: -- / --")
        self.trend_label.pack(anchor=tk.W)
        self.mode_label = ttk.Label(left_frame, text="🎛 控制模式: --",
                                    foreground="purple")
        self.mode_label.pack(anchor=tk.W)
        self.reason_label = ttk.Label(left_frame, text="💬 原因: --",
                                      wraplength=380, justify=tk.LEFT, font=f_small)
        self.reason_label.pack(anchor=tk.W)
        self.switch_label = ttk.Label(left_frame, text="🔁 开关次数: H0 F0 U0",
                                      font=f_small)
        self.switch_label.pack(anchor=tk.W)
        self.ack_label = ttk.Label(left_frame, text="📡 ACK: --", font=f_small)
        self.ack_label.pack(anchor=tk.W)

        self.ai_status_label = ttk.Label(left_frame, text="🧠 AI 状态: 待机",
                                         foreground="gray")
        self.ai_status_label.pack(anchor=tk.W, pady=(5, 0))
        self.action_label = ttk.Label(left_frame, text="⚙ 当前动作: 等待数据...")
        self.action_label.pack(anchor=tk.W)

        # 日志区已挪到窗口底部通栏（见 build_ui 开头），这里不再创建。

        # ---------- 右侧 ----------
        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.fig = Figure(figsize=(8, 6), dpi=100)
        self.ax1 = self.fig.add_subplot(211)
        self.ax2 = self.fig.add_subplot(212)
        self.fig.suptitle("AI 智能农场 · 环境趋势监控（第三版）",
                          fontsize=15, fontweight="bold", y=0.98)

        self.ax1.set_title("实时温度曲线 (红线实际 / 蓝点趋势参考)")
        self.ax1.set_ylabel("温度 (°C)")
        self.ax1.grid(True, linestyle="--", alpha=0.6)
        self.line_temp, = self.ax1.plot([], [], "r-", linewidth=2, label="实际温度")
        self.pred_scatter_temp = self.ax1.scatter(
            [], [], c="blue", marker="o", s=30, label="趋势参考点")
        self.ax1.legend(loc="upper right")
        self.t_min_line = self.ax1.axhline(y=self.t_min, color="green",
                                           linestyle="--", alpha=0.7, label="目标下限")
        self.t_max_line = self.ax1.axhline(y=self.t_max, color="orange",
                                           linestyle="--", alpha=0.7, label="目标上限")

        self.ax2.set_title("实时湿度曲线 (蓝线实际 / 红点趋势参考)")
        self.ax2.set_ylabel("湿度 (%)")
        self.ax2.set_xlabel("时间 (秒)")
        self.ax2.grid(True, linestyle="--", alpha=0.6)
        self.line_humi, = self.ax2.plot([], [], "b-", linewidth=2, label="实际湿度")
        self.pred_scatter_humi = self.ax2.scatter(
            [], [], c="red", marker="o", s=30, label="趋势参考点")
        self.ax2.legend(loc="upper right")
        self.h_min_line = self.ax2.axhline(y=self.h_min, color="green",
                                           linestyle="--", alpha=0.7)
        self.h_max_line = self.ax2.axhline(y=self.h_max, color="orange",
                                           linestyle="--", alpha=0.7)

        self.fig.tight_layout(rect=[0, 0, 1, 0.95])
        self.canvas = FigureCanvasTkAgg(self.fig, master=right_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # =================== 协议开关 ===================
    def toggle_protocol(self):
        self.protocol.enhanced = self.enhanced_var.get()
        self.log("串口协议模式切换为：%s" %
                 ("增强协议" if self.protocol.enhanced else "旧协议（兼容）"))

    # =================== API Key ===================
    def save_api_key(self):
        key = self.key_entry.get().strip()
        if not key:
            self.advisor.set_api_key("")
            self.log("⚠ API Key 已清空，AI 按钮不可用（采集与控制不受影响）")
            return
        ok, msg = self.advisor.set_api_key(key, save=True)
        self.log(("✅ %s" % msg) if ok else ("❌ %s" % msg))
        if ok:
            self.log("下一步：点「🔌 测试Qwen连接」验证是否可用")

    def toggle_key_visibility(self):
        cur = self.key_entry.cget("show")
        self.key_entry.config(show="" if cur == "*" else "*")

    # =================== 端口 ===================
    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports:
            self.port_combo.set(ports[0])

    def toggle_connection(self):
        if not self.connected:
            self.connect()
        else:
            self.disconnect()

    def connect(self):
        port = self.port_combo.get()
        try:
            baud = int(self.baud_combo.get())
        except Exception:
            baud = 115200
        if not port:
            messagebox.showwarning("警告", "没有可用串口，请检查设备是否插好")
            return
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.connected = True
            self.stop_flag = False
            self.connect_btn.config(text="断开连接")
            self.log("✅ 已连接到 %s @ %d" % (port, baud))
            threading.Thread(target=self.read_serial, daemon=True).start()
            threading.Thread(target=self.local_control_loop, daemon=True).start()
        except Exception as e:
            messagebox.showerror("连接失败", str(e))
            self.log("❌ 连接失败：%s" % e)

    def disconnect(self):
        self.stop_flag = True
        self.connected = False
        try:
            if self.ser and self.ser.is_open:
                # 断开前把执行器全部关掉，避免掉线后一直加热
                self.protocol.enhanced = self.enhanced_var.get()
                try:
                    self.ser.write((self.protocol.build_ctrl(0, 0, 0) + "\n").encode())
                except Exception:
                    pass
                self.ser.close()
        except Exception:
            pass
        self.connect_btn.config(text="连接设备")
        self.log("🔌 已断开连接，已下发关闭指令")

    def on_close(self):
        try:
            self.disconnect()
        except Exception:
            pass
        try:
            self.root.destroy()
        except Exception:
            pass

    # =================== 串口线程 ===================
    def read_serial(self):
        while not self.stop_flag and self.connected and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    self.data_queue.put(("RAW", line))
            except Exception as e:
                self.log_queue.put("⚠️ 串口读取错误：%s" % e)
                time.sleep(0.5)

    # =================== 本地控制循环（绝不调用 Qwen）===================
    def local_control_loop(self):
        while not self.stop_flag and self.connected:
            try:
                if len(self.temp_data) >= 2:
                    self.local_control_once()
            except Exception as e:
                self.log_queue.put("❌ 本地控制错误：%s" % e)
            time.sleep(CONTROL_INTERVAL_SEC)

    def _log_once(self, msg, cooldown=30.0):
        """同一条前馈提示最多 30 秒打一次，避免日志被刷满。"""
        now = time.time()
        if getattr(self, "_last_ff_log", 0.0) and now - self._last_ff_log < cooldown:
            return
        self._last_ff_log = now
        self.log_queue.put(msg)

    def local_control_once(self):
        trend = self.predictor.analyze(self.temp_data, self.humi_data)
        decision = self.controller.decide(
            self.current_temp, self.current_humi, trend,
            self.t_min, self.t_max, self.h_min, self.h_max, time.time())

        # ---- 第四版热惯性前馈：只"关"不"开"，所以始终启用 ----
        # 预测区间上界已经逼近目标上限时提前断电，对冲断电后的温度上冲。
        if self.ff_enabled:
            desired, changed, why = self.ff_handler(
                {"heater": decision["heater"], "fan": decision["fan"],
                 "humidifier": decision["humidifier"]},
                trend, (self.t_min, self.t_max, self.h_min, self.h_max))
            if changed:
                decision = dict(decision)
                decision.update(desired)
                decision["mode"] = decision.get("mode", "") + "+feedforward"
                decision["reason"] = why
                self._log_once("⚡ " + why)

        # 发 PRED（带置信度）
        self.send_cmd(self.protocol.build_pred(
            trend["pred_temp"], trend["pred_humi"], trend["confidence"]),
            track_ack=False)
        # 发 CTRL
        self.send_cmd(self.protocol.build_ctrl(
            decision["heater"], decision["fan"], decision["humidifier"]),
            track_ack=True)
        # 兼容旧下位机：AI_ON 点亮 OLED
        self.send_cmd("AI_ON", track_ack=False)

        self.logger.log_decision(
            time.time() - self.start_time, self.current_temp, self.current_humi,
            trend, decision, (self.t_min, self.t_max, self.h_min, self.h_max),
            self.controller.switch_count, self.ack_status)

        self.ui_queue.put(("CONTROL", {"trend": trend, "decision": decision}))
        self.ai_active = True

    def send_cmd(self, msg, track_ack=False):
        if not (self.ser and self.connected):
            return
        try:
            self.ser.write((msg + "\n").encode())
            if track_ack:
                self.protocol.register_pending(msg)
                if self.retransmit is not None:
                    m = re.search(r"SEQ=(\d+)", msg)
                    if m and self.protocol.enhanced:
                        self.retransmit.register(int(m.group(1)), msg)
        except Exception as e:
            self.log_queue.put("⚠️ 发送失败：%s" % e)

    # =================== 主线程队列处理 ===================
    def process_queues(self):
        # 串口数据
        try:
            while True:
                mtype, content = self.data_queue.get_nowait()
                if mtype == "RAW":
                    self.handle_line(content)
        except queue.Empty:
            pass
        # 日志
        try:
            while True:
                self.log(self.log_queue.get_nowait())
        except queue.Empty:
            pass
        # UI 事件（子线程绝不直接碰 Tk 控件）
        try:
            while True:
                evt, payload = self.ui_queue.get_nowait()
                self.handle_ui_event(evt, payload)
        except queue.Empty:
            pass

        self._reschedule(100, self.process_queues)

    def _reschedule(self, ms, func):
        """窗口已销毁时不再续命，避免关闭时报 TclError。"""
        try:
            if self.root.winfo_exists():
                self.root.after(ms, func)
        except Exception:
            pass

    def handle_line(self, line):
        p = self.protocol.parse_line(line)
        self.comm_stats["total"] += 1
        if p["ok"]:
            self.comm_stats["ok"] += 1
        else:
            self.comm_stats["fail"] += 1

        if p["type"] == "ACK":
            latency, ok = self.protocol.on_ack(p["seq"], p["ack_ok"])
            if self.retransmit is not None:
                self.retransmit.on_ack(p["seq"], p["ack_ok"])
            if latency is not None:
                self.ack_status = "OK %.0fms" % (latency * 1000)
                self.log("✅ ACK SEQ=%d 延迟 %.0f ms" % (p["seq"], latency * 1000))
            return

        if p["type"] == "ERR":
            self.protocol.on_ack(p["seq"], False)
            if self.retransmit is not None:
                self.retransmit.on_ack(p["seq"], False)
            self.ack_status = "ERR:%s" % p["err_reason"]
            self.log("❌ ERR SEQ=%d 原因=%s" % (p["seq"], p["err_reason"]))
            return

        if p["type"] == "DATA":
            # DATA 每 2 秒一条，天然就是心跳，不需要额外的 HEARTBEAT 帧
            if self.heartbeat is not None and p["ok"]:
                if self.heartbeat.on_frame():
                    self.link_state = self.heartbeat.state
                    self.log("✅ 链路 ONLINE（下位机恢复上报）")
            if not p["ok"]:
                self.logger.log_data(p, time.time() - self.start_time)
                if p["reason"] == "SENSOR_ERROR":
                    self.log("⚠️ 下位机传感器读取失败 (DATA:ERROR)")
                elif p["reason"]:
                    self.log("⚠️ 解析失败：%s | %s" % (p["reason"], line[:60]))
                return

            self.current_temp = p["temp"]
            self.current_humi = p["humi"]
            elapsed = time.time() - self.start_time
            self.time_data.append(elapsed)
            self.temp_data.append(p["temp"])
            self.humi_data.append(p["humi"])

            trend = self.predictor.analyze(self.temp_data, self.humi_data)
            self.trend = trend
            self.logger.log_data(p, elapsed, trend=trend, ctrl_state={
                "heater": self.controller.state["heater"],
                "fan": self.controller.state["fan"],
                "humidifier": self.controller.state["humidifier"],
                "mode": self.controller.mode,
                "switch_count_heater": self.controller.switch_count["heater"],
                "switch_count_fan": self.controller.switch_count["fan"],
                "switch_count_humidifier": self.controller.switch_count["humidifier"],
                "ack_status": self.ack_status,
            })
            return

        if p["type"] in ("PRED", "TARGET", "CTRL", "AI_ON"):
            return  # 自发自收的回环，不刷屏

    def handle_ui_event(self, evt, payload):
        if evt == "CONTROL":
            self.trend = payload["trend"]
            self.decision = payload["decision"]
            d = payload["decision"]
            parts = []
            if d["heater"]:
                parts.append("🔥加热")
            if d["fan"]:
                parts.append("💨风扇")
            if d["humidifier"]:
                parts.append("💦加湿")
            self.last_action = " + ".join(parts) if parts else "待机"
            key = "%s|%s" % (self.last_action, d["mode"])
            if key != self.last_logged_action:
                self.log("🎛 [%s] %s → %s" %
                         (d["mode"], self.last_action, d["reason"]))
                self.last_logged_action = key
            if d["should_call_llm_diagnosis"]:
                self.ui_queue.put(("LLM_HINT", "检测到异常：可点击「AI异常诊断」"
                                               "获取排查建议（不会自动调用）"))

        elif evt == "LLM_HINT":
            self.log("💡 %s" % payload)

        elif evt == "LLM_RESULT":
            self.on_qwen_result(payload)

        elif evt == "AI_BUSY":
            busy, name = payload
            state = "disabled" if busy else "normal"
            btn = {"crop": self.crop_btn, "strategy": self.strategy_btn,
                   "diagnosis": self.diag_btn, "test": self.test_btn}.get(name)
            if btn:
                btn.config(state=state)

        elif evt == "DIALOG":
            kind, title, msg = payload
            try:
                if kind == "error":
                    messagebox.showerror(title, msg)
                elif kind == "warning":
                    messagebox.showwarning(title, msg)
                else:
                    messagebox.showinfo(title, msg)
            except Exception:
                pass

    def check_ack_loop(self):
        expired = self.protocol.check_ack_timeout()
        for seq in expired:
            self.ack_status = "TIMEOUT"
            self.log("⏱ ACK_TIMEOUT SEQ=%d（%.0f 秒未确认，不阻塞控制）"
                     % (seq, ACK_TIMEOUT_SEC))

        # ---- 第四版：超时自动重传 + 心跳状态机 ----
        if self.retransmit is not None and self.connected:
            due, gave_up = self.retransmit.due_frames()
            for frame in due:
                self.retransmit.mark_retransmit(frame.seq)
                if self.ser and self.ser.is_open:
                    try:
                        self.ser.write((frame.msg + "\n").encode())
                        self.log("🔁 重传 SEQ=%d（第 %d 次）"
                                 % (frame.seq, frame.retries))
                    except Exception as e:
                        self.log("⚠️ 重传失败：%s" % e)
            for seq in gave_up:
                self.log("❌ SEQ=%d 重传 %d 次仍无 ACK，已放弃"
                         % (seq, self.retransmit.max_retry))

        if self.heartbeat is not None:
            if self.heartbeat.update():
                self.link_state = self.heartbeat.state
                if self.link_state == "OFFLINE":
                    self.log("📴 链路 OFFLINE：%.1f 秒未收到下位机帧，暂停下发控制命令"
                             % self.heartbeat.gap())
                elif self.link_state == "DEGRADED":
                    self.log("⚠️ 链路 DEGRADED：延迟偏高")
                else:
                    self.log("✅ 链路恢复 ONLINE")

        # 每轮把通信层健康度快照交给 logger，写进 CSV 的三列
        if self.logger is not None and self.retransmit is not None:
            s = self.retransmit.summary()
            self.logger.link_v4_extra = {
                "link_state": self.heartbeat.state if self.heartbeat else "",
                "retransmit": s["retransmit"],
                "ack_rate": round(s["ack_rate"], 3),
            }

        self._reschedule(1000, self.check_ack_loop)

    # =================== Qwen 调用（子线程）===================
    def run_qwen(self, kind):
        ok, msg = self.advisor.check_qwen_env()
        if not ok:
            messagebox.showwarning("Qwen 未配置", msg)
            self.log("⚠️ %s" % msg)
            return
        cok, creason = self.advisor.check_cooldown()
        if not cok:
            self.log("⏳ Qwen 冷却中：%s（低频辅助，防止高频调用）" % creason)
            return
        # 主线程里把控件值一次性取出来，子线程绝不碰 Tk 控件
        ui = {
            "crop": self.crop_entry.get().strip() or "生菜",
            "stage": self.stage_combo.get(),
            "cultivation": self.cult_combo.get(),
        }
        self.ui_queue.put(("AI_BUSY", (True, kind)))
        threading.Thread(target=self._qwen_worker, args=(kind, ui),
                         daemon=True).start()

    def _qwen_worker(self, kind, ui):
        """子线程：只做网络请求，结果全部塞回队列。"""
        result = {"kind": kind, "ok": False, "data": None,
                  "error": "", "raw": "", "event": kind}
        try:
            if kind == "crop":
                data = self.advisor.ask_crop_params(
                    ui["crop"], ui["stage"], ui["cultivation"])
                result["data"] = data
                result["advice_type"] = "crop"
            elif kind == "strategy":
                t = self.trend or self.predictor.analyze(
                    self.temp_data, self.humi_data)
                ctx = {
                    "crop": ui["crop"], "stage": ui["stage"],
                    "cultivation": ui["cultivation"],
                    "cur_t": self.current_temp, "cur_h": self.current_humi,
                    "smooth_t": t["smooth_temp"], "smooth_h": t["smooth_humi"],
                    "slope_t": t["slope_temp"], "slope_h": t["slope_humi"],
                    "std_t": t["std_temp"], "std_h": t["std_humi"],
                    "confidence": t["confidence"], "temp_trend": t["temp_trend"],
                    "humi_trend": t["humi_trend"],
                    "heater": self.controller.state["heater"],
                    "fan": self.controller.state["fan"],
                    "humidifier": self.controller.state["humidifier"],
                    "t_min": self.t_min, "t_max": self.t_max,
                    "h_min": self.h_min, "h_max": self.h_max,
                }
                result["data"] = self.advisor.ask_strategy(ctx)
                result["advice_type"] = "strategy"
            elif kind == "diagnosis":
                t = self.trend or {}
                abnormal = t.get("confidence", "") == "low" and "趋势不可信" or "manual_request"
                ctx = {
                    "abnormal": abnormal,
                    "temp_series": list(self.temp_data)[-10:],
                    "humi_series": list(self.humi_data)[-10:],
                    "heater": self.controller.state["heater"],
                    "fan": self.controller.state["fan"],
                    "humidifier": self.controller.state["humidifier"],
                    "t_min": self.t_min, "t_max": self.t_max,
                    "h_min": self.h_min, "h_max": self.h_max,
                }
                result["data"] = self.advisor.ask_diagnosis(ctx)
                result["advice_type"] = "diagnosis"
            elif kind == "test":
                result["data"] = self.advisor.test_connection()
                result["advice_type"] = None
            result["ok"] = True
            self.advisor._mark_called()
        except Exception as e:
            result["error"] = str(e)
        self.ui_queue.put(("LLM_RESULT", result))
        self.ui_queue.put(("AI_BUSY", (False, kind)))

    def on_qwen_result(self, r):
        """主线程：安全校验 + 展示 + 等待人工确认。"""
        if not r["ok"]:
            self.log("❌ Qwen 调用失败：%s" % r["error"])
            self.logger.log(llm_called=1, llm_event=r["kind"], llm_valid=0,
                            llm_blocked_reason="CALL_FAILED:%s" % r["error"][:80])
            self.ui_queue.put(("DIALOG", ("error", "Qwen 调用失败", r["error"][:400])))
            return

        advice_type = r.get("advice_type")
        if advice_type is None:      # 连通性测试
            self.log("✅ Qwen 连通正常：%s" % json.dumps(r["data"], ensure_ascii=False)[:150])
            self.last_advice = "连通性测试通过"
            self.advice_label.config(text="🗒 最近Qwen建议: 连通性测试通过")
            self.logger.log(llm_called=1, llm_event="test", llm_valid=1)
            return

        valid, reason, data = self.advisor.validate_llm_advice(
            r["data"], advice_type, cooldown_ok=True)
        if not valid:
            self.log("🛡 Qwen建议被安全函数拦截：%s" % reason)
            self.last_advice = "被拦截：%s" % reason
            self.advice_label.config(text="🗒 最近Qwen建议: 被安全函数拦截（%s）" % reason)
            self.logger.log(llm_called=1, llm_event=advice_type, llm_valid=0,
                            llm_blocked_reason=reason)
            return

        self.last_advice = json.dumps(data, ensure_ascii=False)
        self.pending_advice = {"type": advice_type, "data": data}
        self.apply_btn.config(state="normal")

        if advice_type == "crop":
            tr, hr = data["temp_range"], data["humi_range"]
            txt = ("%s/%s/%s 推荐 T %.1f~%.1f℃  H %.1f~%.1f%%；风险：%s"
                   % (data.get("crop"), data.get("stage"), data.get("cultivation"),
                      float(tr[0]), float(tr[1]), float(hr[0]), float(hr[1]),
                      data.get("risk", "")))
        elif advice_type == "strategy":
            txt = "%s ｜ 提醒：%s" % (data.get("strategy", ""),
                                   data.get("safety_warning", ""))
        else:
            txt = "%s ｜ 可能原因：%s ｜ 建议：%s" % (
                data.get("abnormal_type", ""),
                "/".join(map(str, data.get("possible_causes", []))[:3]),
                data.get("safe_action", ""))

        self.advice_label.config(text="🗒 最近Qwen建议: %s" % txt)
        self.log("✅ Qwen[%s] 通过安全校验，等待人工确认后才生效" % advice_type)
        self.logger.log(llm_called=1, llm_event=advice_type, llm_valid=1)

    def apply_pending_advice(self):
        """人工确认后才应用 —— LLM 不能直接控硬件。"""
        if not self.pending_advice:
            return
        pa = self.pending_advice
        if pa["type"] == "crop":
            tr, hr = pa["data"]["temp_range"], pa["data"]["humi_range"]
            self.t_min, self.t_max = float(tr[0]), float(tr[1])
            self.h_min, self.h_max = float(hr[0]), float(hr[1])
            self.t_min_var.set(self.t_min)
            self.t_max_var.set(self.t_max)
            self.h_min_var.set(self.h_min)
            self.h_max_var.set(self.h_max)
            self.send_cmd(self.protocol.build_target(
                self.t_min, self.t_max, self.h_min, self.h_max), track_ack=True)
            self.send_cmd("AI_ON")
            self.log("📌 已人工确认并应用目标区间：T %.1f~%.1f℃  H %.1f~%.1f%%"
                     % (self.t_min, self.t_max, self.h_min, self.h_max))
        elif pa["type"] == "strategy":
            rules = pa["data"].get("suggested_rules", {})
            self.log("📌 策略建议（仅供参考，未修改控制参数）：%s"
                     % json.dumps(rules, ensure_ascii=False))
            self.log("💡 操作建议：%s" % pa["data"].get("human_message", ""))
        else:
            steps = pa["data"].get("check_steps", [])
            self.log("📌 排查步骤：" + " → ".join(map(str, steps)))
            self.log("💡 安全处置：%s" % pa["data"].get("safe_action", ""))
        self.apply_btn.config(state="disabled")
        self.pending_advice = None

    # =================== 手动目标 ===================
    def apply_manual_target(self):
        try:
            self.t_min = float(self.t_min_var.get())
            self.t_max = float(self.t_max_var.get())
            self.h_min = float(self.h_min_var.get())
            self.h_max = float(self.h_max_var.get())
        except Exception:
            messagebox.showwarning("输入错误", "目标区间必须是数字")
            return
        if self.t_min >= self.t_max or self.h_min >= self.h_max:
            messagebox.showwarning("输入错误", "下限必须小于上限")
            return
        self.send_cmd(self.protocol.build_target(
            self.t_min, self.t_max, self.h_min, self.h_max), track_ack=True)
        self.log("📌 手动设定目标：T %.1f~%.1f℃  H %.1f~%.1f%%"
                 % (self.t_min, self.t_max, self.h_min, self.h_max))

    # =================== 日志 ===================
    def log(self, msg):
        try:
            self.log_text.config(state="normal")
            self.log_text.insert(tk.END, "%s  %s\n"
                                 % (time.strftime("%H:%M:%S"), msg))
            self.log_text.see(tk.END)
            self.log_text.config(state="disabled")
        except Exception:
            pass

    # =================== UI 刷新 ===================
    def update_ui(self):
        try:
            conf_colors = {"high": "green", "medium": "#B8860B", "low": "red"}
            t = self.trend
            self.temp_label.config(text="🌡 温度: %.1f °C" % self.current_temp)
            self.humi_label.config(text="💧 湿度: %.1f %%" % self.current_humi)
            self.range_label.config(
                text="🎯 目标: T %.1f~%.1f℃  H %.1f~%.1f%%"
                     % (self.t_min, self.t_max, self.h_min, self.h_max))

            if t:
                self.pred_label.config(
                    text="📈 趋势参考: %.1f °C / %.1f %%"
                         % (t["pred_temp"], t["pred_humi"]))
                self.smooth_label.config(
                    text="平滑: %.2f / %.2f" % (t["smooth_temp"], t["smooth_humi"]))
                self.std_label.config(
                    text="标准差: %.3f / %.3f" % (t["std_temp"], t["std_humi"]))
                self.slope_label.config(
                    text="斜率: %+.3f / %+.3f" % (t["slope_temp"], t["slope_humi"]))
                self.conf_label.config(
                    text="🔎 趋势置信度: %s" % t["confidence"],
                    foreground=conf_colors.get(t["confidence"], "black"))
                self.trend_label.config(
                    text="📉 趋势: 温度 %s / 湿度 %s"
                         % (t["temp_trend"], t["humi_trend"]))

            if self.decision:
                d = self.decision
                mode_cn = {"threshold": "A 普通阈值",
                           "trend_assisted": "B 趋势辅助",
                           "abnormal_protection": "C 异常保护"}.get(
                               d["mode"], d["mode"])
                self.mode_label.config(text="🎛 控制模式: %s" % mode_cn)
                self.reason_label.config(text="💬 原因: %s" % d["reason"])
                self.pred_scatter_temp.set_offsets(
                    [[(self.time_data[-1] if self.time_data else 0) + 2.0,
                      d["pred_temp"]]])
                self.pred_scatter_humi.set_offsets(
                    [[(self.time_data[-1] if self.time_data else 0) + 2.0,
                      d["pred_humi"]]])

            sc = self.controller.switch_count
            self.switch_label.config(
                text="🔁 开关次数: 加热%d 风扇%d 加湿%d"
                     % (sc["heater"], sc["fan"], sc["humidifier"]))

            summ = self.protocol.ack_summary()
            self.ack_label.config(
                text="📡 ACK 成功率 %.0f%% (发%d/回%d/超时%d) ｜ 解析成功 %.0f%%"
                     % (summ["rate"], summ["sent"], summ["acked"], summ["timeout"],
                        (self.comm_stats["ok"] / self.comm_stats["total"] * 100)
                        if self.comm_stats["total"] else 0))

            self.ai_status_label.config(
                text="🧠 AI 状态: %s" % ("激活" if self.ai_active else "待机"),
                foreground="green" if self.ai_active else "gray")
            self.action_label.config(text="⚙ 当前动作: %s" % self.last_action)

            if self.time_data and self.temp_data and self.humi_data:
                self.line_temp.set_data(list(self.time_data), list(self.temp_data))
                self.line_humi.set_data(list(self.time_data), list(self.humi_data))
                self.t_min_line.set_ydata([self.t_min, self.t_min])
                self.t_max_line.set_ydata([self.t_max, self.t_max])
                self.h_min_line.set_ydata([self.h_min, self.h_min])
                self.h_max_line.set_ydata([self.h_max, self.h_max])
                self.ax1.relim(); self.ax1.autoscale_view(scalex=True, scaley=True)
                self.ax2.relim(); self.ax2.autoscale_view(scalex=True, scaley=True)
                self.canvas.draw_idle()
        except Exception:
            pass
        self._reschedule(200, self.update_ui)


# ==============================================================================
#  8. 入口
# ==============================================================================

def main():
    root = tk.Tk()
    app = EnvMonitorGUI(root)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
