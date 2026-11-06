# -*- coding: utf-8 -*-
"""对比第三版原逻辑与第四版鲁棒预处理：关注尖峰进入窗口期间的最坏状态。"""
import sys
sys.path.insert(0, r"D:\Workbuddy project\2026-10-25-21-36-42\AI-Smart-Greenhouse\pc")
from robust_preprocess import RobustPreprocess, _mean, _std

WIN = 5
STD_TEMP_MAX = 0.8
TRUTH = 23.75          # 尖峰未发生时的真实水平


def run(name, seq, truth=TRUTH):
    pp = RobustPreprocess()
    acc = []
    # 尖峰在窗口内停留期间的最坏值
    worst_old = {"std": 0.0, "err": 0.0}
    worst_new = {"std": 0.0, "err": 0.0}
    n_out = 0
    for x in seq:
        acc.append(x)
        w = acc[-WIN:]
        o_std = _std(w)
        o_mean = _mean(w)
        r = pp.add(x)
        n_out += 1 if r["is_outlier"] else 0
        for bag, std_v, val in ((worst_old, o_std, o_mean),
                                (worst_new, r["std_robust"], r["smooth"])):
            if std_v > bag["std"]:
                bag["std"] = std_v
            if abs(val - truth) > abs(bag["err"]):
                bag["err"] = val - truth
    def verdict(std_v):
        return "low" if std_v > STD_TEMP_MAX else "ok"
    print("### %s" % name)
    print("   旧 最坏 std=%.3f(%s)  平滑值最大偏移=%+.2f℃"
          % (worst_old["std"], verdict(worst_old["std"]), worst_old["err"]))
    print("   新 最坏 std=%.3f(%s)  平滑值最大偏移=%+.2f℃  剔除 %d 点"
          % (worst_new["std"], verdict(worst_new["std"]), worst_new["err"], n_out))


print("alpha=%.3f  理论滞后=%.1fs（tau=10s, dt=2s）\n"
      % (RobustPreprocess().alpha, RobustPreprocess().lag_sec))

run("A 平稳无异常（新逻辑不能把好的搞坏）",
    [23.6, 23.7, 23.8, 23.7, 23.8, 23.6, 23.7, 23.9, 23.8, 23.7])

run("B 混入 1 个尖峰 60.0",
    [23.6, 23.7, 23.8, 23.7, 23.8, 60.0, 23.7, 23.8, 23.9, 23.7, 23.8, 23.6])

run("C 连续 2 个尖峰 60.0 / 60.2",
    [23.6, 23.7, 23.8, 23.7, 60.0, 60.2, 23.7, 23.8, 23.9, 23.7, 23.8, 23.6])

print("\n### D 真实阶跃 23.0 -> 25.0（加热刚开，不能被当成噪声抹掉）")
pp = RobustPreprocess()
seq = [23.0] * 5 + [23.3, 23.8, 24.3, 24.7, 25.0, 25.0, 25.1]
for x in seq:
    r = pp.add(x)
print("   最终 EMA=%.3f  误判 %d 点（应为 0）  当前 window=%d"
      % (r["smooth"], pp.n_outlier, r["window"]))
print("   参考：EMA 对阶跃的滞后应接近 tau=10s，即 5 个采样点")
