# -*- coding: utf-8 -*-
"""
第四版通信层：CRC8 + 自动重传 + 心跳状态机

这一层解决第四版文档第 1 节里「不换硬件也能做的三件事」：
  1. 检错：累加和 -> CRC8（表驱动），能检出字节顺序交换、多数双比特翻转
  2. 丢包：超时只写日志 -> 自动重传 + 指数退避 100 / 300 / 900 ms
  3. 断线：被动超时 -> 心跳状态机 ONLINE / DEGRADED / OFFLINE

刻意没做的：ADDR 地址字段、ESP-NOW / MQTT / LoRa 物理层。
那三件事必须换硬件才能真正跑起来，现在硬写进帧格式，只会让
「默认兼容旧协议」这条保证失效——第二版固件就得跟着重烧。
等真的插上无线模块时再来加 ADDR，那时候一次性改干净。
"""

import time

# ==================== 常量 ====================
MAX_RETRY = 3
BACKOFF_MS = (100, 300, 900)        # 第 1 / 2 / 3 次重传的退避毫秒数
ACK_TIMEOUT_SEC = 3.0               # 与第三版保持一致

HB_WARN_SEC = 6.0                   # 超过这么久没收到下位机帧 -> DEGRADED
HB_LOST_SEC = 15.0                  # 超过这么久 -> OFFLINE（与下位机 LINK_TIMEOUT_MS 对齐）

# ==================== CRC8 ====================
# 多项式 0x07（CRC8/ATM：x^8 + x^2 + x + 1），初值 0x00，无反射，无异或输出。
# 选它是因为在短帧（<40 字节）上位翻转检出率与常用 CRC8 相当，而实现最简单，
# 下位机 C 侧只要 16 行就能对齐——两边必须完全一致，否则增强模式会一直 ACK 超时。
_CRC8_TABLE = []


def _build_table():
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
        _CRC8_TABLE.append(crc)


_build_table()


def crc8(body):
    """逐字节查表 CRC8。body 为 str 或 bytes，返回 0..255。"""
    if isinstance(body, str):
        body = body.encode("utf-8", errors="ignore")
    crc = 0x00
    for b in body:
        crc = _CRC8_TABLE[crc ^ b]
    return crc


def checksum_sum(body):
    """第三版的逐字节累加和 % 256，保留用于兼容旧固件。"""
    if isinstance(body, bytes):
        body = body.decode("utf-8", errors="ignore")
    return sum(ord(c) for c in body) % 256


def calc_check(body, use_crc=False):
    """统一入口：关掉就用累加和（旧固件），打开就用 CRC8（需同步烧新固件）。"""
    return crc8(body) if use_crc else checksum_sum(body)


# ==================== 待确认帧 + 重传 ====================
class PendingFrame:
    __slots__ = ("seq", "msg", "first_sent", "last_sent", "retries",
                 "acked", "failed")

    def __init__(self, seq, msg):
        self.seq = seq
        self.msg = msg
        self.first_sent = time.time()
        self.last_sent = time.time()
        self.retries = 0
        self.acked = False
        self.failed = False


class RetransmitManager:
    """
    管理待确认帧并在 ACK 超时后重传。

    为什么可以放心重传 CTRL：这一帧的语义是「把三个设备设成这个值」，
    重复执行同一条命令结果完全相同（幂等）。最多重复一次的代价是 relay
    多咔哒一声，远小于命令丢失导致停在错误状态的风险。
    """

    def __init__(self, ack_timeout=ACK_TIMEOUT_SEC, max_retry=MAX_RETRY,
                 backoff_ms=BACKOFF_MS):
        self.ack_timeout = ack_timeout
        self.max_retry = max_retry
        self.backoff_ms = backoff_ms
        self.pending = {}                     # seq -> PendingFrame
        self.stats = {"sent": 0, "acked": 0, "timeout": 0, "err": 0,
                      "retransmit": 0, "giveup": 0,
                      "latency_sum": 0.0, "rtt_samples": 0}

    # ---------- 登记 ----------
    def register(self, seq, msg):
        if seq is None:
            return None
        self.pending[seq] = PendingFrame(seq, msg)
        self.stats["sent"] += 1
        return self.pending[seq]

    # ---------- 收到 ACK / ERR ----------
    def on_ack(self, seq, ok=True):
        f = self.pending.pop(seq, None)
        if f is None:
            return None                        # 迟到或重复的 ACK，忽略
        if ok:
            f.acked = True
            self.stats["acked"] += 1
            self.stats["latency_sum"] += (time.time() - f.first_sent)
            self.stats["rtt_samples"] += 1
        else:
            self.stats["err"] += 1
        return f

    # ---------- 每轮检查该重传谁 ----------
    def due_frames(self, now=None):
        """返回到点该重传、且还没超过最大重试次数的帧列表。"""
        now = now or time.time()
        out = []
        expired = []
        for seq, f in list(self.pending.items()):
            age = now - f.last_sent
            if age < self.ack_timeout:
                continue
            if f.retries >= self.max_retry:
                expired.append(seq)
                continue
            wait = self.backoff_ms[min(f.retries, len(self.backoff_ms) - 1)] / 1000.0
            if age >= self.ack_timeout + wait:
                out.append(f)
        for seq in expired:
            self.pending.pop(seq, None)
            self.stats["giveup"] += 1
        return out, expired

    def mark_retransmit(self, seq):
        f = self.pending.get(seq)
        if f is None:
            return
        f.retries += 1
        f.last_sent = time.time()
        self.stats["retransmit"] += 1

    # ---------- 统计 ----------
    def summary(self):
        s = self.stats
        sent = s["sent"]
        rtt = (s["latency_sum"] / s["rtt_samples"]) if s["rtt_samples"] else 0.0
        return {
            "sent": sent,
            "acked": s["acked"],
            "timeout": s["timeout"],
            "err": s["err"],
            "retransmit": s["retransmit"],
            "giveup": s["giveup"],
            "pending": len(self.pending),
            "ack_rate": (s["acked"] / sent) if sent else 0.0,
            "avg_rtt": round(rtt, 3),
        }


# ==================== 心跳状态机 ====================
class HeartbeatMonitor:
    """
    LINK_STATE = ONLINE / DEGRADED / OFFLINE，由「最后一次收到下位机帧的时间」驱动。

    上位机侧不需要下位机真的发 HEARTBEAT 帧：DATA 本身每 2 秒一条，
    天然就是心跳。这样做的好处是**不用改下位机固件**就能拿到链路健康度，
    而且和 v3 已有的 link（下位机 15 秒失联自动关执行器）刚好互补：
      - 下位机发现 PC 消失 -> 自己关执行器
      - 上位机发现下位机消失 -> 标记 OFFLINE，日志报警，不再下发无效命令
    """

    def __init__(self, warn_sec=HB_WARN_SEC, lost_sec=HB_LOST_SEC):
        self.warn_sec = warn_sec
        self.lost_sec = lost_sec
        self.last_rx = 0.0
        self.state = "OFFLINE"
        self.prev_state = "OFFLINE"
        self.frames = 0
        self.outages = 0
        self._lost_since = None

    def on_frame(self, now=None):
        now = now or time.time()
        self.last_rx = now
        self.frames += 1
        if self.state != "ONLINE":
            self._transit("ONLINE")
            self._lost_since = None
            return True                     # 刚刚恢复
        return False

    def update(self, now=None):
        """返回本次是否发生状态跳变，便于上层只在变化时打日志。"""
        now = now or time.time()
        if self.last_rx == 0.0:
            return False
        gap = now - self.last_rx
        target = "ONLINE" if gap < self.warn_sec else (
            "DEGRADED" if gap < self.lost_sec else "OFFLINE")
        if target != self.state:
            if target == "OFFLINE" and self._lost_since is None:
                self._lost_since = now
                self.outages += 1
            elif target == "ONLINE":
                self._lost_since = None
            self._transit(target)
            return True
        return False

    def _transit(self, new_state):
        self.prev_state = self.state
        self.state = new_state

    def gap(self, now=None):
        if self.last_rx == 0.0:
            return -1.0
        return (now or time.time()) - self.last_rx


def self_test():
    """不需要串口的自检，任何人都能跑：python link_v4.py"""
    print("=== CRC8 自检 ===")
    cases = [
        ("", 0x00),
        ("123456789", 0xF4),        # CRC8/ATM 标准向量
        ("SEQ=1,TS=100,T=25.6,H=60.0", None),
    ]
    for body, expect in cases:
        v = crc8(body)
        tag = "" if expect is None else (" OK" if v == expect else " MISMATCH")
        print("  crc8(%r) = 0x%02X%s" % (body[:24], v, tag))
    if expect_ok := (crc8("123456789") == 0xF4):
        print("  标准向量通过")
    else:
        raise AssertionError("CRC8 标准向量不匹配，表或多项式写错了")

    # 累加和被 CRC8 抓住的经典失效场景：字节顺序交换
    a = "AB"
    b = "BA"
    print("  字节交换：累加和 %s vs %s -> %s；CRC8 %d vs %d -> %s"
          % (checksum_sum(a), checksum_sum(b),
             "漏检" if checksum_sum(a) == checksum_sum(b) else "可检出",
             crc8(a), crc8(b),
             "漏检" if crc8(a) == crc8(b) else "可检出"))

    print()
    print("=== 重传状态机自检 ===")
    rm = RetransmitManager(ack_timeout=0.05, max_retry=3, backoff_ms=(10, 20, 30))
    rm.register(1, "CTRL:SEQ=1,HEAT=1,FAN=0,HUM=0,CHK=1")
    due, expired = rm.due_frames()
    print("  刚发出时应无需重传：due=%d" % len(due))
    assert len(due) == 0
    time.sleep(0.09)
    due, expired = rm.due_frames()
    print("  超时后应重传一次：due=%d" % len(due))
    assert len(due) == 1
    rm.mark_retransmit(1)
    print("  retransmit 计数 =", rm.stats["retransmit"])
    rm.on_ack(1, ok=True)
    print("  ACK 后 pending =", len(rm.pending), "acked =", rm.stats["acked"])
    assert len(rm.pending) == 0

    rm2 = RetransmitManager(ack_timeout=0.01, max_retry=2, backoff_ms=(1, 1))
    rm2.register(9, "CTRL:SEQ=9,CHK=1")
    for _ in range(6):
        time.sleep(0.02)
        due, expired = rm2.due_frames()
        for f in due:
            rm2.mark_retransmit(f.seq)
    print("  超过最大重试后应放弃：giveup=%d pending=%d"
          % (rm2.stats["giveup"], len(rm2.pending)))
    assert rm2.stats["giveup"] == 1 and len(rm2.pending) == 0

    print()
    print("=== 心跳状态机自检 ===")
    hb = HeartbeatMonitor(warn_sec=0.05, lost_sec=0.15)
    hb.on_frame()
    print("  收到帧 ->", hb.state)
    assert hb.state == "ONLINE"
    time.sleep(0.08)
    hb.update()
    print("  静默 0.08s ->", hb.state)
    assert hb.state == "DEGRADED"
    time.sleep(0.10)
    hb.update()
    print("  静默 0.18s ->", hb.state, "| outages =", hb.outages)
    assert hb.state == "OFFLINE" and hb.outages == 1
    hb.on_frame()
    print("  再来一帧 ->", hb.state)
    assert hb.state == "ONLINE"

    print()
    print("ALL LINK LAYER TESTS PASSED")


if __name__ == "__main__":
    self_test()
