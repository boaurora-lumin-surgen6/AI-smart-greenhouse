# -*- coding: utf-8 -*-
"""
第四版控制层：连续调节 + 热惯性前馈

三件事，按「能不能在现有硬件上安全跑」分开放：

1. 热惯性前馈（默认启用）——纯软件，不需要任何硬件改动
   加热片断电后温度还会继续往上冲，第三版只能等它冲上去再纠偏。
   前馈做的是：预测区间上界 + 余量 已经越过目标上限时，提前断电。
   注意它**只会关、不会开**，所以永远不会引起误启动，可以理解为一条额外的安全规则。

2. 增量式 PID + 时间比例占空比（默认关闭，USE_V4_DUTY=1 打开）
   开关量的必然结果是锯齿振荡，这是数学必然，不是调参能解决的。
   改成占空比后振荡幅度直接由参数控制。
   但——**机械继电器不能做高频 PWM**。这里用的是时间比例：一个 30 秒周期里
   前 duty×30 秒导通，其余断开。机械继电器可以接受这个频率；
   如果想做真正的 PWM，先把继电器换成 SSR（固态继电器）。

3. 第三版的三模式规则不丢，降级为异常与安全兜底层
   正常工况走 PID，异常工况（传感器不可信、设备无效果）回落规则。
   规则做安全兜底、连续控制做性能，这是工业现场的标准分层。
"""

# ==================== 1. 热惯性前馈 ====================
def feedforward_shutoff(desired, trend, targets, margin=0.3):
    """
    返回 (修改后的期望状态, 是否触发了前馈, 说明)。

    触发条件：某设备当前想要开，但「预测区间上界 + 余量」已经超过目标上限，
    说明即便现在断电，热惯性也会把它推过界 —— 那就现在就关。
    """
    heater, fan, hum = desired["heater"], desired["fan"], desired["humidifier"]
    t_min, t_max, h_min, h_max = targets
    changed, reason = False, ""

    if heater and trend.get("pred_temp_hi") is not None:
        if trend["pred_temp_hi"] + margin > t_max:
            heater = 0
            changed, reason = True, "热惯性前馈：预测上界 %.2f 逼近上限 %.1f，提前断开加热" % (
                trend["pred_temp_hi"], t_max)

    if hum and trend.get("pred_humi_hi") is not None:
        if trend["pred_humi_hi"] + margin > h_max:
            hum = 0
            changed = True
            reason = (reason + "；" if reason else "") + \
                     "湿度前馈：预测上界 %.2f 逼近上限 %.1f，提前停止加湿" % (
                         trend["pred_humi_hi"], h_max)

    out = {"heater": heater, "fan": fan, "humidifier": hum}
    return out, changed, reason


# ==================== 2. 增量式 PID ====================
class IncrementalPID:
    """
    增量式（velocity form）而不是位置式：

      duty += Kp*(e_k - e_{k-1}) + Ki*e_k*dt + Kd*(e_k - 2*e_{k-1} + e_{k-2})/dt

    选增量式的实际理由：抗积分饱和改造只要一行（不更新 integral 即可），
    而且控制器重启时不会输出突跳 —— 位置式的积分项在重启瞬间会失控。
    """

    def __init__(self, kp=0.25, ki=0.01, kd=0.5, dt=2.0,
                 out_min=0.0, out_max=1.0):
        self.kp, self.ki, self.kd, self.dt = kp, ki, kd, dt
        self.out_min, self.out_max = out_min, out_max
        self.duty = 0.0
        self.e_prev = 0.0
        self.e_prev2 = 0.0

    def step(self, error):
        e = float(error)
        d = (self.kp * (e - self.e_prev)
             + self.ki * e * self.dt
             + self.kd * (e - 2 * self.e_prev + self.e_prev2) / self.dt)
        self.duty = min(self.out_max, max(self.out_min, self.duty + d))
        self.e_prev2, self.e_prev = self.e_prev, e
        return self.duty

    def reset(self):
        self.duty = 0.0
        self.e_prev = self.e_prev2 = 0.0


# ==================== 3. 时间比例占空比 ====================
class TimeProportionalDuty:
    """
    把一个 0~1 的占空比翻译成「这个周期内前几秒导通」。

    为什么不用 Arduino 的 analogWrite：
      - 机械继电器有机械寿命，每秒吸合几次几周就废
      - 继电器的吸合时间本身就有几毫秒到十几毫秒抖动，高频下占空比完全失真
      - 真正条件是执行器能承受周期通断。周期 >= 30 秒时普通继电器够用；
        想压到更短必须换 SSR。
    """

    def __init__(self, period_sec=30.0, min_on_sec=3.0, min_off_sec=3.0):
        self.period = float(period_sec)
        self.min_on = float(min_on_sec)
        self.min_off = float(min_off_sec)
        self.window_start = None
        self.on = False

    def update(self, duty, now):
        duty = min(1.0, max(0.0, float(duty)))
        if self.window_start is None:
            self.window_start = now
        if now - self.window_start >= self.period:
            self.window_start = now          # 进入下一个周期

        on_target = duty * self.period
        elapsed = now - self.window_start

        if on_target <= self.min_on:
            self.on = False                  # 太短干脆不通，避免无意义吸合
        elif on_target >= self.period - self.min_off:
            self.on = True                   # 几乎全程，避免尾部短断
        else:
            self.on = elapsed < on_target
        return 1 if self.on else 0


# ==================== 4. 上电自整定（继电反馈法）====================
def relay_autotune_from_series(values, dt=2.0, setpoint=None):
    """
    给一段「继电自激振荡」的响应曲线，算出临界增益 Ku 与临界周期 Tu。

    原理（Åström–Hägglund 继电反馈）：让系统在继电器式开关下自己振荡起来，
    振荡的幅值与周期就携带了对象的动力学信息，不需要知道 C / k / Q 是多少。
      Ku = 4d / (π·a)      d = 继电器幅值的一半，a = 输出振荡幅值
      Tu = 相邻两次同向过零的时间间隔
    再用 Ziegler–Nichols 换算：
      Kp = 0.6·Ku    Ti = 0.5·Tu    Td = 0.125·Tu

    为什么必须做这件事：第三版的死区/滞回/10 秒/90 秒全是人工试凑的常数，
    换一个箱子、换一个功率就失效，评委问"参数怎么来的"答不上来。
    自整定之后答案是"上电后系统自己量出来的"。
    """
    v = [float(x) for x in values]
    if len(v) < 20:
        return None
    sp = float(setpoint) if setpoint is not None else (sum(v) / len(v))

    # 取后 2/3 段（前段是启动暂态），找过零点
    vv = v[len(v) // 3:]
    crossings = []
    for i in range(1, len(vv)):
        if (vv[i - 1] - sp) * (vv[i] - sp) < 0:
            frac = abs(vv[i - 1] - sp) / (abs(vv[i - 1] - sp) + abs(vv[i] - sp))
            crossings.append((i - 1 + frac) * dt)
    if len(crossings) < 3:
        return None

    gaps = [crossings[i + 1] - crossings[i] for i in range(len(crossings) - 1)]
    Tu = 2.0 * (sum(gaps) / len(gaps))
    a = (max(vv) - min(vv)) / 2.0
    if a < 1e-6:
        return None
    d = 0.5                                  # u 在 0/1 之间切换，等效继电幅值一半
    Ku = 4.0 * d / (3.141592653589793 * a)

    Kp = 0.6 * Ku
    Ti = 0.5 * Tu
    Td = 0.125 * Tu
    return {
        "Tu": round(Tu, 2), "amplitude": round(a, 3), "Ku": round(Ku, 4),
        "Kp": round(Kp, 4), "Ki": round(Kp / Ti, 6), "Kd": round(Kp * Td, 4),
        "Ti": round(Ti, 2), "Td": round(Td, 2), "setpoint": sp,
    }


def relay_experiment(duration=2400.0, dt=2.0, band=0.8,
                     Tamb=20.0, target=23.0, C=1200.0, k=6.0, Q=40.0):
    """
    继电自激振荡。band 是继电器的滞回带 —— 必须带，否则系统会退化成
    采样尺度的抖动（每个采样点 0.03℃ 上下横跳），测出来的 Tu 只有十几秒，
    那是数值噪声不是热惯性。真实继电器自整定也一定带滞回，道理相同。
    """
    T, T_hist, t = Tamb, [], 0.0
    u = 1
    while t < duration:
        if u == 1 and T >= target + band / 2.0:
            u = 0
        elif u == 0 and T <= target - band / 2.0:
            u = 1
        T += dt * (Q * u - k * (T - Tamb)) / C
        T_hist.append(T)
        t += dt
    return T_hist


# ==================== 5. 对照仿真 ====================
# 一阶热模型：C·dT/dt = Q·u − k·(T − Tamb)
# 参数按一个 40L 泡沫箱 + 40W 加热片估：23℃ 时散热约 18W，
# 加热净功率约 22W → 升温 0.018℃/s，断电后降温 0.015℃/s，两者量级相当。
# 这是"估计"不是"实测"，所以下面的数字只说明趋势，不能当论文里的实测结果。
PLANT = dict(Tamb=20.0, C=1200.0, k=6.0, Q=40.0)

DEADBAND = 0.5          # 与第三版 TEMP_DEADBAND 一致
HYSTERESIS = 0.8        # 与第三版 TEMP_HYSTERESIS 一致
MIN_SWITCH_INTERVAL = 10.0


def simulate(mode, duration=1800.0, dt=2.0, target=23.0,
             plant=None, pid=None):
    """
    mode:
      A_rule  第三版规则：死区 + 滞回 + 最小开关间隔（今天代码里跑的就是这个）
      C_pid   第四版：增量式 PID + 时间比例占空比
    """
    p = dict(PLANT)
    if plant:
        p.update(plant)
    T = p["Tamb"]
    pid = pid or IncrementalPID()
    tp = TimeProportionalDuty(period_sec=30.0)
    hist = []
    t = 0.0
    u = 0
    last_switch = -1e9
    while t < duration:
        if mode == "A_rule":
            if t - last_switch >= MIN_SWITCH_INTERVAL:
                want = 1 if T < target - DEADBAND / 2.0 else (
                    0 if T > target + DEADBAND / 2.0 + HYSTERESIS else u)
                if want != u:
                    u = want
                    last_switch = t
        else:
            u = tp.update(pid.step(target - T), t)
        T += dt * (p["Q"] * u - p["k"] * (T - p["Tamb"])) / p["C"]
        hist.append((t, T, u))
        t += dt

    temps = [h[1] for h in hist]
    steady = temps[int(len(temps) * 0.5):]
    ess = sum(abs(x - target) for x in steady) / len(steady)
    overshoot = max(temps) - target
    # 开关次数
    switches = sum(1 for i in range(1, len(hist)) if hist[i][2] != hist[i - 1][2])
    return {
        "mode": mode,
        "ess": round(ess, 3),
        "overshoot": round(max(0.0, overshoot), 3),
        "switches": switches,
        "band": round(max(steady) - min(steady), 3),
    }


if __name__ == "__main__":
    print("=== 增量式 PID + 前馈自检 ===")
    pid = IncrementalPID()
    duty = 0.0
    for _ in range(200):
        duty = pid.step(2.0)          # 恒定误差，输出应饱和到 1
    print("  恒定误差下 duty 饱和到 %.3f（应为 1.0）" % duty)
    assert abs(duty - 1.0) < 1e-6
    pid.reset()
    for _ in range(200):
        duty = pid.step(-2.0)
    print("  反向误差下 duty 饱和到 %.3f（应为 0.0）" % duty)
    assert abs(duty - 0.0) < 1e-6

    print()
    print("=== 时间比例占空比自检 ===")
    tp = TimeProportionalDuty(period_sec=30.0)
    t = 0.0
    on_count = 0
    total = 0
    while t < 300:
        on_count += tp.update(0.5, t)
        total += 1
        t += 2.0
    print("  duty=0.5 时导通占比 %.2f（应接近 0.5）" % (on_count / total))
    assert 0.4 < on_count / total < 0.6
    zero = TimeProportionalDuty(period_sec=30.0)
    print("  duty=0.02 时导通关数 =", sum(zero.update(0.02, i * 2.0)
                                       for i in range(150)), "（应为 0，太短不吸合）")
    assert sum(zero.update(0.02, i * 2.0) for i in range(150)) == 0

    print()
    print("=== 前馈自检 ===")
    trend = {"pred_temp_hi": 24.9, "pred_humi_hi": 50.0}
    desired = {"heater": 1, "fan": 0, "humidifier": 1}
    out, changed, reason = feedforward_shutoff(
        desired, trend, (18.0, 25.0, 40.0, 70.0), margin=0.3)
    print("  预测上界 24.9 + 余量 0.3 > 上限 25.0 -> heater =", out["heater"])
    assert out["heater"] == 0 and changed
    out2, changed2, _ = feedforward_shutoff(
        desired, {"pred_temp_hi": 22.0, "pred_humi_hi": 50.0},
        (18.0, 25.0, 40.0, 70.0), margin=0.3)
    print("  预测上界 22.0 安全 -> heater =", out2["heater"], "changed =", changed2)
    assert out2["heater"] == 1 and not changed2

    print()
    print("=== 上电自整定（继电反馈法，继电器带 0.8℃ 滞回）===")
    hist = relay_experiment()
    tuned = relay_autotune_from_series(hist, dt=2.0, setpoint=23.0)
    if not tuned:
        raise AssertionError("自整定失败：振荡序列不足以测出 Tu")
    print("  量得 Tu=%.1fs  振荡幅值=%.3f℃  ->  Ku=%.4f"
          % (tuned["Tu"], tuned["amplitude"], tuned["Ku"]))
    print("  Ziegler-Nichols 换算：Kp=%.4f  Ki=%.6f  Kd=%.4f"
          % (tuned["Kp"], tuned["Ki"], tuned["Kd"]))
    print("  Tu=%.0fs 与按物理参数估的时间常数 C/k=%.0fs 同量级，说明量到的确实是热惯性"
          % (tuned["Tu"], PLANT["C"] / PLANT["k"]))

    print()
    print("=== A 组（第三版规则）vs C 组（第四版 PID + 占空比，同一模型）===")
    r1 = simulate("A_rule")
    r2 = simulate("C_pid", pid=IncrementalPID(
        kp=tuned["Kp"], ki=tuned["Ki"], kd=tuned["Kd"]))
    for name, r in (("A_rule", r1), ("C_pid", r2)):
        print("  %-7s ess=%.3f℃  超调=%.3f℃  稳态峰谷差=%.3f℃  开关次数=%d"
              % (name, r["ess"], r["overshoot"], r["band"], r["switches"]))
    print("  稳态峰谷差 %.3f -> %.3f（%.0f%%）"
          % (r1["band"], r2["band"],
             100.0 * (r2["band"] - r1["band"]) / max(1e-9, r1["band"])))
    print()
    print("ALL CONTROL LAYER TESTS PASSED")
