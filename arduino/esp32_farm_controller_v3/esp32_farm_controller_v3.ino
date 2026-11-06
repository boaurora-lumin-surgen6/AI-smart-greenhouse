/*
 * 环境监控系统 · 第三版下位机（端云协同鲁棒边缘智能温室系统）
 * 在第二版 sketch_apr11a.ino 基础上升级，接线与 OLED 内容全部保留。
 *
 * 接线（未改动）：
 *   DHT11  -> GPIO 4
 *   OLED   -> SDA:21, SCL:22
 *   加热片 -> GPIO 18
 *   风扇   -> GPIO 14
 *   加湿器 -> GPIO 25
 *   串口   -> 115200
 *
 * 第三版新增：
 *   1. seq 计数器
 *   2. USE_ENHANCED_PROTOCOL 宏（默认 false，保持与旧上位机兼容）
 *   3. 增强协议上行 DATA:SEQ=..,TS=..,T=..,H=..,HEAT=..,FAN=..,HUM=..,CHK=..
 *   4. 解析增强 CTRL / PRED / TARGET，校验 CHK，成功回 ACK，失败回 ERR
 *   5. 传感器失败上行 DATA:ERROR 或 ERR
 *   6. 解析失败不重启、不卡死
 *   7. 可选失联保护：超时未收到上位机命令则关闭全部执行器
 *
 * CHK 算法：取 "TYPE:" 之后、",CHK=" 之前的全部字符，ASCII 求和 % 256。
 *           必须与上位机 Protocol.checksum() 完全一致。
 */

#include <U8g2lib.h>
#include <DHT.h>

// ===== 协议开关：默认 0（关闭）=====
// 改 1 需同时把上位机的 USE_ENHANCED_PROTOCOL 打开（或勾选 GUI 复选框）
// 用 0 / 1 而不是 true / false，避免不同编译器对 #if 的处理差异
#define USE_ENHANCED_PROTOCOL 0

// ===== 失联保护：超过此毫秒未收到任何有效命令则关闭全部执行器 =====
// 设为 0 可禁用（保持第二版行为）
#define LINK_TIMEOUT_MS 15000

// ===== 引脚定义（未改动）=====
#define DHTPIN 4
#define DHTTYPE DHT11

#define HEATER_PIN 18
#define FAN_PIN 14
#define HUMI_PIN 25

// ===== 继电器触发电平 =====
// 高电平触发模块保持默认；若你的模块是低电平触发（IN 接 GND 会吸合），
// 把下面两行改成：#define RELAY_ON LOW   /  #define RELAY_OFF HIGH
#define RELAY_ON  HIGH
#define RELAY_OFF LOW

// ===== 对象 =====
DHT dht(DHTPIN, DHTTYPE);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 22, 21);

// ===== 执行器状态 =====
bool heater = false;
bool fan = false;
bool humi = false;

// ===== AI 数据 =====
float pred_t = 0.0;
float pred_h = 0.0;
float pred_conf = -1.0;          // 新增：上位机下发的趋势置信度 0~1，<0 表示未收到

float t_min = 0.0, t_max = 0.0;
float h_min = 0.0, h_max = 0.0;

bool ai_active = false;

// ===== 传感器读取失败计数 =====
uint8_t dht_fail_count = 0;
const uint8_t MAX_FAIL_COUNT = 5;

// ===== 协议序号 / 链路状态 =====
uint32_t seq_out = 0;            // 上行报文序号
uint32_t last_cmd_ms = 0;        // 最后一次收到有效命令的时刻
bool link_armed = false;         // 已收到过第一条命令后才启用失联保护

// ===== 工具函数 =====

// ASCII 求和 % 256，与上位机一致
int calcChecksum(const String &body) {
  int sum = 0;
  for (unsigned int i = 0; i < body.length(); i++) {
    sum += (int)body.charAt(i);
  }
  return sum % 256;
}

// 从 "K=V,K=V" 形式的 body 中取字段值，取不到返回空串
String fieldValue(const String &body, const char *key) {
  String k = String(key) + "=";
  int s = body.indexOf(k);
  if (s < 0) return String("");
  s += k.length();
  int e = body.indexOf(',', s);
  if (e < 0) e = body.length();
  return body.substring(s, e);
}

// 拆分增强报文：返回校验是否通过，body 通过引用返回（不含 "TYPE:" 前缀）
bool splitFrame(const String &cmd, String &type, String &body, int &chk) {
  int colon = cmd.indexOf(':');
  int chkIdx = cmd.lastIndexOf(",CHK=");
  if (colon < 0 || chkIdx < 0) return false;
  type = cmd.substring(0, colon);
  body = cmd.substring(colon + 1, chkIdx);
  chk = cmd.substring(chkIdx + 5).toInt();
  return (calcChecksum(body) == chk);
}

void sendAck(uint32_t seq, bool ok, const char *reason = "") {
  if (!USE_ENHANCED_PROTOCOL) return;
  if (ok) {
    Serial.print("ACK:SEQ=");
    Serial.print(seq);
    Serial.println(",OK=1");
  } else {
    Serial.print("ERR:SEQ=");
    Serial.print(seq);
    Serial.print(",REASON=");
    Serial.println(reason);
  }
}

void applyActuators() {
  digitalWrite(HEATER_PIN, heater ? RELAY_ON : RELAY_OFF);
  digitalWrite(FAN_PIN,    fan    ? RELAY_ON : RELAY_OFF);
  digitalWrite(HUMI_PIN,   humi   ? RELAY_ON : RELAY_OFF);
}

// 处理一条命令，返回是否识别为有效命令
bool handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return false;

  // ---------- ACK / ERR（上位机侧才会发，这里只忽略）----------
  if (cmd.startsWith("ACK:") || cmd.startsWith("ERR:")) return false;

  // ---------- DATA:ERROR（旧协议的错误上行使能时可能出现）----------
  if (cmd.startsWith("DATA:")) return false;

  // ---------- CTRL ----------
  if (cmd.startsWith("CTRL:")) {
#if USE_ENHANCED_PROTOCOL
    if (cmd.indexOf("CHK=") >= 0) {
      String type, body; int chk;
      if (!splitFrame(cmd, type, body, chk)) {
        sendAck(fieldValue(body, "SEQ").toInt(), false, "BAD_CHECKSUM");
        return false;
      }
      heater = (fieldValue(body, "HEAT").toInt() == 1);
      fan    = (fieldValue(body, "FAN").toInt() == 1);
      humi   = (fieldValue(body, "HUM").toInt() == 1);
      applyActuators();
      sendAck(fieldValue(body, "SEQ").toInt(), true);
      return true;
    }
#endif
    // 旧协议 CTRL:HEAT=1,FAN=0,HUM=0
    int ht = -1, f = -1, hu = -1;
    if (sscanf(cmd.c_str(), "CTRL:HEAT=%d,FAN=%d,HUM=%d", &ht, &f, &hu) == 3) {
      heater = (ht == 1);
      fan    = (f == 1);
      humi   = (hu == 1);
      applyActuators();
      return true;
    }
    return false;
  }

  // ---------- PRED ----------
  if (cmd.startsWith("PRED:")) {
#if USE_ENHANCED_PROTOCOL
    if (cmd.indexOf("CHK=") >= 0) {
      String type, body; int chk;
      if (!splitFrame(cmd, type, body, chk)) {
        sendAck(fieldValue(body, "SEQ").toInt(), false, "BAD_CHECKSUM");
        return false;
      }
      pred_t = fieldValue(body, "T").toFloat();
      pred_h = fieldValue(body, "H").toFloat();
      String cf = fieldValue(body, "CONF");
      if (cf.length() > 0) pred_conf = cf.toFloat();
      sendAck(fieldValue(body, "SEQ").toInt(), true);
      return true;
    }
#endif
    // 旧协议 PRED:T=23.5,H=55.2
    // 说明：部分 Arduino-ESP32 工具链的 sscanf 不支持 %f，因此改用手工截取
    int p1 = cmd.indexOf("T=");
    if (p1 >= 0) {
      int c1 = cmd.indexOf(',', p1);
      if (c1 < 0) c1 = cmd.length();
      pred_t = cmd.substring(p1 + 2, c1).toFloat();
    }
    int p2 = cmd.indexOf("H=");
    if (p2 >= 0) {
      pred_h = cmd.substring(p2 + 2).toFloat();
    }
    return true;
  }

  // ---------- TARGET ----------
  if (cmd.startsWith("TARGET:")) {
#if USE_ENHANCED_PROTOCOL
    if (cmd.indexOf("CHK=") >= 0) {
      String type, body; int chk;
      if (!splitFrame(cmd, type, body, chk)) {
        sendAck(fieldValue(body, "SEQ").toInt(), false, "BAD_CHECKSUM");
        return false;
      }
      t_min = fieldValue(body, "TMIN").toFloat();
      t_max = fieldValue(body, "TMAX").toFloat();
      h_min = fieldValue(body, "HMIN").toFloat();
      h_max = fieldValue(body, "HMAX").toFloat();
      sendAck(fieldValue(body, "SEQ").toInt(), true);
      return true;
    }
#endif
    // 旧协议 TARGET:T=18.0-25.0,H=40.0-70.0
    // 同样不用 %f，手工截取，避免工具链差异
    int tStart = cmd.indexOf("T=");
    if (tStart >= 0) {
      int dash = cmd.indexOf('-', tStart);
      int comma = cmd.indexOf(',', tStart);
      if (dash > 0) {
        t_min = cmd.substring(tStart + 2, dash).toFloat();
        if (comma > dash) t_max = cmd.substring(dash + 1, comma).toFloat();
      }
    }
    int hStart = cmd.indexOf("H=");
    if (hStart >= 0) {
      int dash2 = cmd.indexOf('-', hStart);
      if (dash2 > 0) {
        h_min = cmd.substring(hStart + 2, dash2).toFloat();
        h_max = cmd.substring(dash2 + 1).toFloat();
      }
    }
    return true;
  }

  // ---------- AI_ON ----------
  if (cmd == "AI_ON") {
    ai_active = true;
    return true;
  }

  return false;   // 未知命令，忽略且不重启
}

void updateOLED(float t, float h) {
  u8g2.clearBuffer();

  // 第一行：当前温度 + 趋势参考温度
  u8g2.setCursor(0, 10);
  u8g2.print("T:");
  u8g2.print(t, 1);
  u8g2.print("(");
  u8g2.print(pred_t, 1);
  u8g2.print(")");

  // 第二行：当前湿度 + 趋势参考湿度
  u8g2.setCursor(0, 22);
  u8g2.print("H:");
  u8g2.print(h, 1);
  u8g2.print("(");
  u8g2.print(pred_h, 1);
  u8g2.print(")");

  // 第三行：目标温度区间 / 目标湿度区间
  u8g2.setCursor(0, 36);
  u8g2.print("Tg:");
  u8g2.print(t_min, 1);
  u8g2.print("~");
  u8g2.print(t_max, 1);

  u8g2.setCursor(64, 36);
  u8g2.print("Hg:");
  u8g2.print(h_min, 1);
  u8g2.print("~");
  u8g2.print(h_max, 1);

  // 第四行：加热 / 风扇
  u8g2.setCursor(0, 50);
  u8g2.print("Ht:");
  u8g2.print(heater ? "ON " : "OFF");
  u8g2.setCursor(64, 50);
  u8g2.print("Fn:");
  u8g2.print(fan ? "ON " : "OFF");

  // 第五行：加湿 / AI（若收到置信度则一并显示百分比）
  u8g2.setCursor(0, 62);
  u8g2.print("Hu:");
  u8g2.print(humi ? "ON " : "OFF");
  u8g2.setCursor(64, 62);
  u8g2.print("AI:");
  u8g2.print(ai_active ? "ON" : "OFF");
  if (pred_conf >= 0.0f) {
    u8g2.print(" ");
    u8g2.print((int)(pred_conf * 100));
  }

  u8g2.sendBuffer();
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);          // 避免 readStringUntil 白等 1 秒导致周期漂移
  dht.begin();
  u8g2.begin();
  u8g2.setFont(u8g2_font_ncenB08_tr);

  pinMode(HEATER_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(HUMI_PIN, OUTPUT);

  // 初始状态：全部关闭
  heater = fan = humi = false;
  applyActuators();

  u8g2.clearBuffer();
  u8g2.setCursor(0, 10);
  u8g2.print("System Ready");
  u8g2.setCursor(0, 25);
  u8g2.print("Waiting for AI...");
  u8g2.setCursor(0, 40);
  u8g2.print(USE_ENHANCED_PROTOCOL ? "Proto: ENHANCED" : "Proto: LEGACY");
  u8g2.sendBuffer();

  delay(1000);
}

void loop() {
  // ===== 1. 读取传感器（带重试）=====
  float t = dht.readTemperature();
  float h = dht.readHumidity();

  if (isnan(t) || isnan(h)) {
    dht_fail_count++;
    if (dht_fail_count >= MAX_FAIL_COUNT) {
#if USE_ENHANCED_PROTOCOL
      Serial.print("ERR:SEQ=");
      Serial.print(seq_out++);
      Serial.println(",REASON=SENSOR_ERROR");
#else
      Serial.println("DATA:ERROR");
#endif
      dht_fail_count = 0;
    }
    // 失败也要处理上位机命令，避免屏幕冻住、命令堆积
    while (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      if (handleCommand(cmd)) last_cmd_ms = millis();
    }
    delay(500);
    return;
  }

  dht_fail_count = 0;

  // ===== 2. 上报数据给上位机 =====
#if USE_ENHANCED_PROTOCOL
  {
    seq_out++;
    char body[96];
    snprintf(body, sizeof(body),
             "SEQ=%lu,TS=%lu,T=%.2f,H=%.2f,HEAT=%d,FAN=%d,HUM=%d",
             (unsigned long)seq_out, (unsigned long)(millis() / 1000),
             t, h, heater ? 1 : 0, fan ? 1 : 0, humi ? 1 : 0);
    Serial.print("DATA:");
    Serial.print(body);
    Serial.print(",CHK=");
    Serial.println(calcChecksum(String(body)));
  }
#else
  Serial.print("DATA:T=");
  Serial.print(t);
  Serial.print(",H=");
  Serial.println(h);
#endif

  // ===== 3. 接收并处理上位机指令 =====
  while (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    if (handleCommand(cmd)) {
      last_cmd_ms = millis();
      link_armed = true;
    }
  }

  // ===== 3.5 失联保护 =====
#if LINK_TIMEOUT_MS > 0
  if (link_armed && (millis() - last_cmd_ms > LINK_TIMEOUT_MS)) {
    if (heater || fan || humi) {
      heater = fan = humi = false;
      applyActuators();
      Serial.println("WARN:LINK_LOST,ALL_OFF");
      ai_active = false;
    }
  }
#endif

  // ===== 4. 更新 OLED 显示 =====
  updateOLED(t, h);

  // ===== 5. 等待 2 秒后再进行下一次循环 =====
  delay(2000);
}
