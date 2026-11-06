# -*- coding: utf-8 -*-
"""
第四版预测层：带不确定度的状态估计

继承第三版 TrendPredictor，只改 analyze()，对外返回的字段名一个都不动，
所以 RobustController / ExperimentLogger / GUI 全都不用改。

四件事：
  1. 平滑：等权窗口均值 -> EMA，alpha 由 tau 推出，滞后可量化（写进论文）
  2. 抗差：裸 std -> 中位数 + MAD 先剔离群，再算 std（这是修 bug，不是升级）
  3. 输出：一个数 -> 数 + 95% 区间，控制器可以拿"整体越界才动作"当依据
  4. 窗口：写死 -> 自适应（乱则收缩求快，稳则拉长求平）

最关键的副作用：confidence 不再被单个尖峰打掉。
第三版里 DHT11 跳一个值 -> std 爆 -> confidence=low -> 系统退回普通阈值控制，
在最需要判断的时刻主动放弃判断。第四版用 std_robust 做判据，这条链断了。
"""

import AI_farm_v3 as V
from robust_preprocess import RobustPreprocess

# 95% 区间的 z 值
Z95 = 1.96

# 两个通道的噪声尺度。不是拍脑袋：
# 温度 mad_floor 取 DHT11 静止实测抖动（±0.3℃）再放宽一点到 0.5℃，
# 阈值 3 × 1.4826 × 0.5 ≈ 2.2℃，压得住抖动，又远小于坏点的跳变幅度。
TEMP_CFG = {"mad_floor": 0.5, "mad_k": 3.0, "std_ref": 0.3}
HUMI_CFG = {"mad_floor": 2.0, "mad_k": 3.0, "std_ref": 2.0}


class TrendPredictorV4(V.TrendPredictor):
    """第四版预测器。接口与 TrendPredictor 完全一致，额外多返回 v4 字段。"""

    def __init__(self, dt=2.0, tau_temp=10.0, tau_humi=10.0):
        V.TrendPredictor.__init__(self)
        self.pp_temp = RobustPreprocess(dt=dt, tau=tau_temp, **TEMP_CFG)
        self.pp_humi = RobustPreprocess(dt=dt, tau=tau_humi, **HUMI_CFG)
        self._n_temp = 0          # 已经喂过的点数，避免每次全量重放
        self._n_humi = 0
        self._last = None         # 最近一次成功的结果（传感器断流时复用）
        self.alpha = self.pp_temp.alpha

    # ---------- 增量同步 ----------
    def _sync(self, pp, vals, seen):
        """只喂新到的点。若历史被清空（长度变短）则自动重置。"""
        if len(vals) < seen:
            pp.reset()
            seen = 0
        latest = None
        for x in vals[seen:]:
            latest = pp.add(x)
        return latest, len(vals)

    def reset(self):
        self.pp_temp.reset()
        self.pp_humi.reset()
        self._n_temp = 0
        self._n_humi = 0
        self._last = None

    def analyze(self, temps, humis):
        """对外入口。没有新数据（传感器失败/控制环空转）时复用上次结果，
        而不是让增量同步返回 None 然后在取下标处崩掉。"""
        res = self._analyze_impl(temps, humis)
        self._last = dict(res)
        return res

    # ---------- 清洗窗口（给斜率和 std 用）----------
    @staticmethod
    def _clean_window(pp, vals, n):
        if len(vals) < 2:
            return list(vals), set()
        seg = vals[-min(n, len(vals)):]
        clean, bad = pp.reject_outliers(seg)
        return clean, bad

    def _analyze_impl(self, temps, humis):
        temps = list(temps)
        humis = list(humis)
        n = min(len(temps), len(humis))

        if n == 0:
            return dict(V.TrendPredictor.analyze(self, temps, humis),
                        **self._empty_v4_fields())

        # 无新数据（传感器失败时控制环照跑）：复用上一次估计，别让同步返回 None
        if (len(temps) == self._n_temp and len(humis) == self._n_humi
                and self._last is not None):
            return dict(self._last)

        r_t, self._n_temp = self._sync(self.pp_temp, temps, self._n_temp)
        r_h, self._n_humi = self._sync(self.pp_humi, humis, self._n_humi)

        # ---------- 数据不足：结构照旧，补 v4 字段 ----------
        if n < self.slope_window:
            cur_t = temps[-1]
            cur_h = humis[-1]
            std_t = self._std(temps[-self.smooth_window:])
            std_h = self._std(humis[-self.smooth_window:])
            return {
                "smooth_temp": cur_t, "smooth_humi": cur_h,
                "slope_temp": 0.0, "slope_humi": 0.0,
                "std_temp": std_t, "std_humi": std_h,
                "pred_temp": cur_t, "pred_humi": cur_h,
                "confidence": "medium",
                "temp_trend": "stable", "humi_trend": "stable",
                "insufficient": True,
                **self._v4_fields(r_t, r_h, std_t, std_h, cur_t, cur_h,
                                  cur_t, cur_h, 0.0, 0.0, self.smooth_window),
            }

        # ---------- 清洗后的窗口 ----------
        # 自适应窗口：std 大 -> 收缩求快，std 小 -> 拉长求平
        win_t = r_t["window"] if r_t else self.smooth_window
        win_h = r_h["window"] if r_h else self.smooth_window
        clean_t, bad_t = self._clean_window(self.pp_temp, temps, max(win_t, self.slope_window))
        clean_h, bad_h = self._clean_window(self.pp_humi, humis, max(win_h, self.slope_window))

        # 第三版的裸 std，留作对照（写进 CSV 就能证明改进）
        std_raw_t = self._std(temps[-self.smooth_window:])
        std_raw_h = self._std(humis[-self.smooth_window:])

        # 第四版的鲁棒 std
        std_t = self._std(clean_t[-self.smooth_window:])
        std_h = self._std(clean_h[-self.smooth_window:])

        smooth_t = r_t["smooth"] if r_t else temps[-1]
        smooth_h = r_h["smooth"] if r_h else humis[-1]

        slope_t = self._slope(clean_t[-self.slope_window:])
        slope_h = self._slope(clean_h[-self.slope_window:])

        # ---------- 趋势方向 / 置信度：判据换成 std_robust ----------
        if std_t > V.STD_TEMP_MAX:
            temp_trend = "unstable"
        elif slope_t >= V.SLOPE_TEMP_DEAD:
            temp_trend = "rising"
        elif slope_t <= -V.SLOPE_TEMP_DEAD:
            temp_trend = "falling"
        else:
            temp_trend = "stable"

        if std_h > V.STD_HUMI_MAX:
            humi_trend = "unstable"
        elif slope_h >= V.SLOPE_HUMI_DEAD:
            humi_trend = "rising"
        elif slope_h <= -V.SLOPE_HUMI_DEAD:
            humi_trend = "falling"
        else:
            humi_trend = "stable"

        noisy = (std_t > V.STD_TEMP_MAX) or (std_h > V.STD_HUMI_MAX)
        clear = (abs(slope_t) >= V.SLOPE_TEMP_DEAD) or (abs(slope_h) >= V.SLOPE_HUMI_DEAD)
        clean_enough = (std_t < V.STD_TEMP_GOOD) and (std_h < V.STD_HUMI_GOOD)

        if noisy:
            confidence = "low"
        elif clean_enough and clear:
            confidence = "high"
        else:
            confidence = "medium"

        pred_t = smooth_t + slope_t
        pred_h = smooth_h + slope_h

        return {
            "smooth_temp": smooth_t, "smooth_humi": smooth_h,
            "slope_temp": slope_t, "slope_humi": slope_h,
            "std_temp": std_t, "std_humi": std_h,
            "pred_temp": pred_t, "pred_humi": pred_h,
            "confidence": confidence,
            "temp_trend": temp_trend, "humi_trend": humi_trend,
            "insufficient": False,
            **self._v4_fields(r_t, r_h, std_t, std_h, pred_t, pred_h,
                              clean_t[-1], clean_h[-1], std_raw_t, std_raw_h, win_t),
        }

    # ---------- v4 附加字段 ----------
    def _empty_v4_fields(self):
        return self._v4_fields(None, None, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, self.smooth_window)

    @staticmethod
    def _v4_fields(r_t, r_h, std_t, std_h, pred_t, pred_h,
                   clean_t, clean_h, std_raw_t, std_raw_h, win_t):
        def val(r, k, default=0.0):
            return r[k] if r else default
        return {
            "pred_temp_lo": round(pred_t - Z95 * std_t, 3),
            "pred_temp_hi": round(pred_t + Z95 * std_t, 3),
            "pred_humi_lo": round(pred_h - Z95 * std_h, 3),
            "pred_humi_hi": round(pred_h + Z95 * std_h, 3),
            "outlier_temp": int(bool(val(r_t, "is_outlier"))),
            "outlier_humi": int(bool(val(r_h, "is_outlier"))),
            "clean_temp": round(clean_t, 3),
            "clean_humi": round(clean_h, 3),
            "std_raw_temp": round(std_raw_t, 4),
            "std_raw_humi": round(std_raw_h, 4),
            "window_temp": int(win_t),
            "alpha": round(val(r_t, "alpha"), 4),
        }


def demo():
    """不依赖硬件的对比演示：python predict_v4.py"""
    base = [23.4, 23.5, 23.4, 23.5, 23.4, 23.5, 23.4, 23.5, 23.4, 23.5]
    spike = base[:-1] + [60.0]        # 最后一个点被 DHT11 跳飞

    old = V.TrendPredictor()
    new = TrendPredictorV4()

    print("=== 场景一：平稳无异常（新方法不能把好的情况搞坏）===")
    for name, p in (("第三版", old), ("第四版", new)):
        r = p.analyze(base, [55.0] * 10)
        print("  %s  smooth_t=%.3f  std=%.4f  conf=%s"
              % (name, r["smooth_temp"], r["std_temp"], r["confidence"]))
    new.reset()

    print()
    print("=== 场景二：末位一个尖峰（正是今天 CSV 里真实发生过的）===")
    r_old = old.analyze(spike, [55.0] * 10)
    r_new = new.analyze(spike, [55.0] * 10)
    print("  第三版  smooth_t=%.3f  std=%.4f  conf=%s  outlier标记=%s"
          % (r_old["smooth_temp"], r_old["std_temp"], r_old["confidence"], "-"))
    print("  第四版  smooth_t=%.3f  std=%.4f  conf=%s  outlier=%d  区间=[%.2f, %.2f]"
          % (r_new["smooth_temp"], r_new["std_temp"], r_new["confidence"],
             r_new["outlier_temp"], r_new["pred_temp_lo"], r_new["pred_temp_hi"]))
    print("  偏移：第三版 %.2f℃   第四版 %.2f℃"
          % (abs(r_old["smooth_temp"] - 23.45), abs(r_new["smooth_temp"] - 23.45)))


if __name__ == "__main__":
    demo()
