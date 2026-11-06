# -*- coding: utf-8 -*-
"""
鲁棒预处理：EMA 低通 + MAD 离群剔除 + 自适应窗口

对应《第四版优化方向文档》第 2 节「预测方向」的三条改动：
    # 等权平均 -> 一阶低通，alpha 可调且滞后可量化
    # 裸 std   -> 中位数 + MAD 先剔离群再算散布
    # 固定窗口 -> 自适应：乱则收缩求快，稳则拉长求平

用法（温度、湿度各建一个实例）：
    temp_pp = RobustPreprocess(mad_floor=0.05)     # 温度，℃
    humi_pp = RobustPreprocess(mad_floor=0.5)      # 湿度，%RH

    r = temp_pp.add(23.8)
    r["smooth"]      # 平滑后的值，直接替换原来的 mean(window)
    r["clean"]       # 离群点已被中位数替换的最新读数
    r["is_outlier"]  # 本次读数是否被判为离群
    r["std_robust"]  # 剔除后的标准差，直接替换原来的 std(window)
    r["window"]      # 当前自适应窗口宽度

设计要点：
1. alpha 不是拍脑袋的常数，由物理时间常数反推：
       alpha = dt / (tau + dt)
   取 tau=10s、dt=2s -> alpha=0.167，EMA 对斜坡的稳态滞后约等于 tau。
2. MAD 判据对 5 点小窗口可能因数据过齐导致 mad=0，
   此时任何微小抖动都会被判成离群，故设 mad_floor 作下界保护。
3. MAD 的击穿点是 50%：窗口内坏点超过一半中位数就会跟坏点走。
   这是统计方法的固有上限，不是实现缺陷——下位机 MAX_FAIL_COUNT 做第一道防线。
"""

from collections import deque

# 1.4826 使 MAD 在正态分布下与标准差可比
MAD_SCALE = 1.4826


def _median(vals):
    """低中位数 lower median。

    偶数长度时不取中间两点的平均，而是取偏小那个。
    原因：窗口里混进尖峰时，跨域的两点平均会把中位数直接拽向坏点。
        vals=[23.6, 23.7, 60.0, 60.2]
        普通中位数 = (23.7+60.0)/2 = 41.85   <- 已被污染
        低中位数   = 23.7                    <- 仍然是好的中位
    数据本身近乎重合（如传感器静止读数）时，两者差异可忽略，无损日常精度。
    """
    s = sorted(vals)
    n = len(s)
    if n == 0:
        return 0.0
    return float(s[(n - 1) // 2])


def _mean(vals):
    return sum(vals) / len(vals) if vals else 0.0


def _std(vals):
    n = len(vals)
    if n < 2:
        return 0.0
    m = _mean(vals)
    # 样本标准差（n-1），与第三版保持一致
    return (sum((v - m) ** 2 for v in vals) / (n - 1)) ** 0.5


class RobustPreprocess:
    def __init__(self, dt=2.0, tau=10.0,
                 base_window=5, min_window=3, max_window=9,
                 mad_k=3.0, mad_floor=0.5, std_ref=0.3):
        """
        dt          采样周期（秒），与下位机 loop 里的 delay(2000) 对应
        tau         期望的时间常数（秒）。越大越平滑，滞后也越大
        base_window 基准窗口宽度，第三版写死的 5
        mad_k       MAD 判据倍数，3.0 对应约 99% 正态覆盖
        mad_floor   MAD 下界，防止窗口无散布时误杀正常数据。
                    取值依据是传感器噪声水平而非任意小数：
                      温度取 0.5℃（DHT11 标称 ±2℃，实测静止抖动约 ±0.3）
                      湿度取 2.0 %RH（DHT11 标称 ±5%RH）
                    判据阈值 = 3 × 1.4826 × 0.5 ≈ 2.2℃，
                    大于正常抖动、远小于 DHT11 坏点跳变幅度，两侧都有余量。
        std_ref     自适应窗口的参考标准差
        """
        self.dt = float(dt)
        self.tau = float(tau)
        # 核心：由时间常数反推 alpha，而不是凑一个 0.2
        self.alpha = self.dt / (self.tau + self.dt)
        self.base_window = int(base_window)
        self.min_window = int(min_window)
        self.max_window = int(max_window)
        self.mad_k = float(mad_k)
        self.mad_floor = float(mad_floor)
        self.std_ref = float(std_ref)

        self._buf = deque(maxlen=self.max_window * 2)
        self._smooth = None
        self.n_outlier = 0

    # ---------- 属性：可直接写进论文的数字 ----------
    @property
    def lag_sec(self):
        """EMA 对斜坡输入的稳态滞后（秒），约等于 tau。"""
        return (1.0 - self.alpha) / self.alpha * self.dt

    # ---------- 离群剔除 ----------
    def reject_outliers(self, vals):
        """返回 (清洗后的列表, 被判为离群的下标集合)。"""
        if len(vals) < 3:
            return list(vals), set()
        med = _median(vals)
        mad = _median([abs(v - med) for v in vals])
        # 下界保护：窗口几乎无散布时，改用传感器尺度做判据
        scale = mad if mad > self.mad_floor else self.mad_floor
        thr = self.mad_k * MAD_SCALE * scale
        clean, bad = [], set()
        for i, v in enumerate(vals):
            if abs(v - med) > thr:
                bad.add(i)
                clean.append(med)      # 用中位数替换，而不是删除
            else:
                clean.append(v)
        return clean, bad

    # ---------- 自适应窗口 ----------
    def adaptive_window(self, std_val):
        if std_val <= 1e-9:
            return self.max_window
        w = self.base_window * (self.std_ref / std_val)
        return int(max(self.min_window, min(self.max_window, round(w))))

    # ---------- 主入口 ----------
    def add(self, x):
        self._buf.append(float(x))
        vals = list(self._buf)

        clean, bad = self.reject_outliers(vals)
        latest_idx = len(vals) - 1
        is_outlier = latest_idx in bad
        if is_outlier:
            self.n_outlier += 1
        latest_clean = clean[latest_idx]

        # EMA 更新：用清洗后的值，避免尖峰污染滤波器内部状态
        if self._smooth is None:
            self._smooth = latest_clean
        else:
            self._smooth = self.alpha * latest_clean + (1 - self.alpha) * self._smooth

        std_robust = _std(clean[-self.base_window:])
        window = self.adaptive_window(std_robust)

        med = _median(vals)
        mad = _median([abs(v - med) for v in vals]) if len(vals) >= 3 else 0.0

        return {
            "raw": float(x),
            "clean": latest_clean,
            "smooth": self._smooth,
            "is_outlier": is_outlier,
            "std_robust": std_robust,
            "std_raw": _std(vals[-self.base_window:]),
            "window": window,
            "median": med,
            "mad": mad,
            "alpha": self.alpha,
        }

    def reset(self):
        self._buf.clear()
        self._smooth = None
        self.n_outlier = 0
