# -*- coding: utf-8 -*-
"""
第四版实验层：把 CSV 变成能写进论文的七项指标 + 一张图

用法：
    python report.py                                  # 读默认 data/experiment_log.csv
    python report.py --csv data/xxx.csv --heater-W 40
    python report.py --csv a.csv b.csv --label A组 B组   # 多组对照

七项指标（第四版文档第 5 节定义）：
    ess              稳态误差     后半段温度与目标中值的平均偏差 ℃
    overshoot_%      超调量       超过目标上限的最大值，按目标带宽归一化
    settling_time_s  调节时间     从开始到首次进入目标带并稳定 60 秒所需时间
    switch_count     开关次数     ACTION 次数，越少继电器寿命越长
    duty_ratio       占空比       加热开启时间 / 总时长
    energy_Wh        能耗估算     duty_ratio × 加热功率 × 时长 / 3600
    out_of_band_%    越界占比     温度落在目标区间之外的时间占比

还有两组专门给第四版"预测层改进"举证的对比：
    std_raw vs std_robust   离群剔除前后同一个窗口的标准差
    confidence 分布           检查是否还像第三版那样被尖峰打成 low
"""

import argparse
import csv
import os
import sys

DEFAULT_CSV = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "..", "data", "experiment_log.csv")

SETTLE_HOLD_SEC = 60.0      # 连续保持在带内这么久才算"稳定"


def _f(v):
    try:
        return float(v)
    except Exception:
        return None


def load_rows(path):
    rows = []
    with open(path, "r", encoding="utf-8-sig", newline="") as f:
        for r in csv.DictReader(f):
            rows.append(r)
    return rows


def analyze(path, heater_w=40.0, t_min=18.0, t_max=25.0):
    rows = load_rows(path)
    if not rows:
        return None

    temps, humis, elapsed, heater = [], [], [], []
    switches = {"heater": 0, "fan": 0, "humidifier": 0}
    modes, reasons = {}, {}
    conf = {}
    std_raw, std_robust, outliers = [], [], 0

    for r in rows:
        rt = r.get("row_type", "")
        t = _f(r.get("temp"))
        h = _f(r.get("humi"))
        e = _f(r.get("elapsed"))
        if rt == "DATA" and t is None:
            continue
        if t is not None and e is not None:
            temps.append(t)
            humis.append(h if h is not None else 0.0)
            elapsed.append(e)
            band = r.get("t_min")
            if band:
                t_min = _f(band) or t_min
            tm = r.get("t_max")
            if tm:
                t_max = _f(tm) or t_max
        if rt == "CONTROL":
            hv = r.get("heater")
            heater.append(1 if hv == "1" else 0)
            m = r.get("mode", "")
            modes[m] = modes.get(m, 0) + 1
            reason = r.get("action_reason", "")
            if reason:
                reasons[reason] = reasons.get(reason, 0) + 1
            for k in switches:
                v = _f(r.get("switch_count_" + k))
                if v is not None:
                    switches[k] = max(switches[k], int(v))
            c = r.get("confidence", "")
            if c:
                conf[c] = conf.get(c, 0) + 1
            sr = _f(r.get("std_raw_temp"))
            sb = _f(r.get("std_temp"))
            if sr is not None:
                std_raw.append(sr)
            if sb is not None:
                std_robust.append(sb)
            if r.get("outlier_temp") == "1":
                outliers += 1

    if not temps:
        return None

    target = (t_min + t_max) / 2.0
    span = max(1e-6, t_max - t_min)
    duration = elapsed[-1] if elapsed else 0.0

    # ---- ess：目标是一个区间而不是一个点 ----
    # 所以"稳态误差"必须定义成"偏离目标带的距离"：在带内就是 0，
    # 低于下限时算 (t_min - T)，高于上限时算 (T - t_max)。
    # 别拿带宽中值当设定值去减，那样一个完美控制在带内运行的系统
    # 也会被算出 2℃ 的"误差"，纯属自欺欺人。
    tail = temps[len(temps) // 2:]

    def band_dev(x):
        if x < t_min:
            return t_min - x
        if x > t_max:
            return x - t_max
        return 0.0

    ess = sum(band_dev(x) for x in tail) / len(tail)

    # ---- 超调量 ----
    over = max(temps) - t_max
    overshoot_pct = 100.0 * max(0.0, over) / span

    # ---- 越界占比 ----
    bad = sum(1 for x in temps if x < t_min or x > t_max)
    out_of_band = 100.0 * bad / len(temps)

    # ---- 调节时间：首次进入目标带并连续保持 SETTLE_HOLD_SEC ----
    settling = None
    for i in range(len(temps)):
        if t_min <= temps[i] <= t_max:
            j = i
            while j < len(temps) and t_min <= temps[j] <= t_max:
                j += 1
            if j >= len(temps) or (elapsed[j - 1] - elapsed[i]) >= SETTLE_HOLD_SEC:
                settling = elapsed[i]
                break

    # ---- 占空比 / 能耗 ----
    if heater:
        duty = sum(heater) / len(heater)
    else:
        duty = sum(1 for x in temps if x < t_min) / max(1, len(temps))
    energy_wh = duty * heater_w * duration / 3600.0

    return {
        "file": os.path.basename(path),
        "samples": len(temps),
        "control_rows": len(heater),
        "duration_s": round(duration, 1),
        "target": (t_min, t_max),
        "ess": round(ess, 3),
        "overshoot_%": round(overshoot_pct, 2),
        "settling_time_s": round(settling, 1) if settling is not None else "未稳定",
        "switch_count": switches,
        "duty_ratio": round(duty, 3),
        "energy_Wh": round(energy_wh, 3),
        "out_of_band_%": round(out_of_band, 2),
        "modes": modes,
        "top_reasons": sorted(reasons.items(), key=lambda kv: -kv[1])[:3],
        "confidence": conf,
        "std_raw_max": round(max(std_raw), 3) if std_raw else None,
        "std_robust_max": round(max(std_robust), 3) if std_robust else None,
        "outlier_points": outliers,
        "temp_range": (round(min(temps), 2), round(max(temps), 2)),
        "humi_range": (round(min(humis), 2), round(max(humis), 2)),
    }


def print_report(res, heater_w=40.0):
    if not res:
        print("没有可用数据")
        return
    print("=" * 60)
    print("文件：%s" % res["file"])
    print("样本：%d 点 / 控制行：%d / 时长：%.0f 秒（%.1f 分钟）"
          % (res["samples"], res["control_rows"], res["duration_s"],
             res["duration_s"] / 60.0))
    print("目标区间：%.1f ~ %.1f ℃   实测范围：%.2f ~ %.2f   湿度 %.1f ~ %.1f"
          % (res["target"][0], res["target"][1],
             res["temp_range"][0], res["temp_range"][1],
             res["humi_range"][0], res["humi_range"][1]))
    print("-" * 60)
    print("  ess              稳态误差      %.3f ℃" % res["ess"])
    print("  overshoot        超调量        %.2f %%" % res["overshoot_%"])
    print("  settling_time    调节时间      %s" % res["settling_time_s"])
    print("  switch_count     开关次数      加热%d 风扇%d 加湿%d"
          % (res["switch_count"]["heater"], res["switch_count"]["fan"],
             res["switch_count"]["humidifier"]))
    print("  duty_ratio       占空比        %.1f %%（按 %dW 加热片估算）"
          % (res["duty_ratio"] * 100, heater_w))
    print("  energy           能耗估算      %.2f Wh" % res["energy_Wh"])
    print("  out_of_band      越界占比      %.2f %%" % res["out_of_band_%"])
    print("-" * 60)
    if res["modes"]:
        print("  控制模式分布：%s"
              % ", ".join("%s×%d" % kv for kv in sorted(
                  res["modes"].items(), key=lambda kv: -kv[1])))
    if res["confidence"]:
        tot = sum(res["confidence"].values())
        print("  置信度分布：%s"
              % ", ".join("%s %.0f%%" % (k, 100.0 * v / tot)
                          for k, v in sorted(res["confidence"].items())))
    if res["std_raw_max"] is not None:
        print("  裸 std 峰值 %.3f  ->  鲁棒 std 峰值 %.3f（离群剔除效果）"
              % (res["std_raw_max"], res["std_robust_max"]))
        print("  被判为离群的点：%d 个" % res["outlier_points"])
    if res["top_reasons"]:
        print("  主要动作：%s"
              % "；".join("%s×%d" % kv for kv in res["top_reasons"]))
    print("=" * 60)


def plot_figures(path, out_dir, res):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib import rcParams
        # 中文字体：Windows 有微软雅黑。图里不要出现 emoji —— matplotlib
        # 不做逐字形回退，缺一个字就是一整排方块。
        rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]
        rcParams["axes.unicode_minus"] = False
    except Exception as e:
        print("（跳过出图：%s）" % e)
        return None
    rows = load_rows(path)
    t, h, temp, lo, hi = [], [], [], [], []
    for r in rows:
        e = _f(r.get("elapsed"))
        v = _f(r.get("temp"))
        if e is None or v is None:
            continue
        t.append(e / 60.0)
        temp.append(v)
        h.append(_f(r.get("humi")) or 0.0)
        lo.append(_f(r.get("pred_temp_lo")))
        hi.append(_f(r.get("pred_temp_hi")))

    os.makedirs(out_dir, exist_ok=True)
    fig, ax = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    ax[0].plot(t, temp, lw=1.2, color="#c0392b", label="实测温度")
    tmin, tmax = res["target"]
    ax[0].axhline(tmin, ls="--", lw=.8, color="#27ae60")
    ax[0].axhline(tmax, ls="--", lw=.8, color="#27ae60", label="目标区间")
    if any(x is not None for x in lo) and any(x is not None for x in hi):
        ax[0].fill_between(t, [x if x is not None else v for x, v in zip(lo, temp)],
                           [x if x is not None else v for x, v in zip(hi, temp)],
                           alpha=.18, color="#2980b9", label="95% 预测区间")
    ax[0].set_ylabel("温度 ℃")
    ax[0].legend(loc="upper right", fontsize=8)
    ax[0].set_title("温度闭环曲线 · %s" % res["file"])

    ax[1].plot(t, h, lw=1.2, color="#2980b9")
    ax[1].set_ylabel("湿度 %RH")
    ax[1].set_xlabel("时间（分钟）")
    plt.tight_layout()
    out = os.path.join(out_dir, "fig_temperature_%s.png"
                       % os.path.splitext(res["file"])[0])
    plt.savefig(out, dpi=150)
    plt.close()
    print("图已生成：%s" % out)
    return out


def main():
    ap = argparse.ArgumentParser(description="AI 智慧农业 · 实验指标分析")
    ap.add_argument("--csv", nargs="+", default=[DEFAULT_CSV])
    ap.add_argument("--label", nargs="*", default=None)
    ap.add_argument("--heater-W", type=float, default=40.0)
    ap.add_argument("--t-min", type=float, default=18.0)
    ap.add_argument("--t-max", type=float, default=25.0)
    ap.add_argument("--out", default=None, help="出图目录，不给就不出图")
    a = ap.parse_args()

    ok = False
    for i, p in enumerate(a.csv):
        if not os.path.exists(p):
            print("跳过（文件不存在）：%s" % p)
            continue
        res = analyze(p, a.heater_W, a.t_min, a.t_max)
        if not res:
            print("跳过（无有效数据）：%s" % p)
            continue
        if a.label and i < len(a.label):
            print("\n【%s】" % a.label[i])
        print_report(res, a.heater_W)
        if a.out:
            plot_figures(p, a.out, res)
        ok = True
    if not ok:
        print("没有任何可分析的 CSV。")


if __name__ == "__main__":
    main()
