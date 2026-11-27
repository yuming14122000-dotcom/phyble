/*
  PhyBLE_Group — ESP32 多感測器 + phyphox (BLE)，分組用

  學生流程：iPad phyphox 連上 PHY-Gxx → 最後一頁「換感測器」選感測器 → ESP32 重開
            → 回 phyphox 重新加入 PHY-Gxx → 做實驗 → 用 phyphox 匯出資料

  模式（存在 NVS：namespace "phyble"，key "mode"）
    1 溫度        DS18B20（可接 1~2 支）  DQ → GPIO4（裸探頭需 4.7kΩ 上拉到 3.3V；模組板已內建）
    2 光閘計時    XD-51 槽型光耦           DO → GPIO27
    3 轉速        霍爾感測器模組           DO → GPIO26（磁鐵貼在轉盤上）
    4 電壓電流    INA219                   SDA → GPIO21，SCL → GPIO22（I2C 位址 0x40）
    5 馬達特性    L9110S + 霍爾 + INA219   A-IA → GPIO25，A-IB → GPIO33（馬達接 A 路）
  所有感測器 VCC 接 3.3V、GND 共地；L9110S 的 VCC 接馬達電源（2.5~12V），GND 與 ESP32 共地。

  組別（key "group"）→ 藍牙名稱 PHY-G01 … PHY-G99（未設定為 PHY-G00）

  序列埠 115200：
    G 3 / GROUP=3     設定組別（存 NVS 後重開）
    M 2 / MODE=2      設定模式（存 NVS 後重開）
    SET 3 2           同時設定組別 3、模式 2
    ?                 顯示目前設定
    LED               重新閃出組別

  BOOT 鍵：長按 1 秒切到下一個感測器（藍燈閃出新編號後重開）
  板載 LED（GPIO2）：開機閃組別（長閃 = 十位、短閃 = 個位；未設定 = 快閃 10 下）
                     之後每秒短亮 = 等待藍牙連線；恆亮 = phyphox 已連上
  韌體版本：FW_VERSION（序列埠輸入 ? 會顯示）
  函式庫：phyphox BLE 1.3.1、OneWire、DallasTemperature；開發板 esp32 2.0.17
*/

#include <phyphoxBle.h>
#include <Preferences.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ---------- 韌體版本（燒錄站「查看板子資訊」用來比對） ----------
#define FW_VERSION "multi-2026.11.27-2"

// ---------- 腳位 ----------
#define LED_PIN    2
#define DS18_PIN   4
#define GATE_PIN   27
#define HALL_PIN   26
#define SDA_PIN    21
#define SCL_PIN    22
#define MOTOR_A    25
#define MOTOR_B    33

// ---------- 模式 ----------
enum { M_TEMP = 1, M_GATE, M_RPM, M_POWER, M_MOTOR, M_COUNT = M_MOTOR };
const char *const MODE_NAMES[] = {"", "溫度", "光閘計時", "轉速", "電壓電流", "馬達特性"};
const char *const MODE_LABELS[] = {"1 溫度 (DS18B20)", "2 光閘計時 (XD-51)", "3 轉速 (霍爾)",
                                   "4 電壓電流 (INA219)", "5 馬達特性 (L9110S)"};
const float MODE_VALUES[] = {1, 2, 3, 4, 5};

static const char *NVS_NS = "phyble";
Preferences prefs;
uint8_t groupNo = 0, modeNo = M_TEMP, pulsesPerRev = 1;
char bleName[16], expTitle[48];
String serialLine;

// ---------- NVS ----------
uint8_t nvsGetU8(const char *key, uint8_t def) {
  prefs.begin(NVS_NS, true);
  uint8_t v = prefs.getUChar(key, def);
  prefs.end();
  return v;
}
void nvsPutU8(const char *key, uint8_t v) {
  prefs.begin(NVS_NS, false);
  prefs.putUChar(key, v);
  prefs.end();
}
void restartSoon(const char *why) {
  Serial.printf("%s，重新開機中…\n", why);
  Serial.flush();
  delay(400);
  ESP.restart();
}

// ---------- LED（非阻塞） ----------
uint16_t ledSeq[64];
uint8_t ledSeqLen = 0, ledSeqPos = 0;
uint32_t ledStepStart = 0;
bool ledSeqActive = false, lastConnected = false;

void ledPush(uint16_t onMs, uint16_t offMs) {
  if (ledSeqLen + 2 <= sizeof(ledSeq) / sizeof(ledSeq[0])) { ledSeq[ledSeqLen++] = onMs; ledSeq[ledSeqLen++] = offMs; }
}
void ledShowGroup() {
  ledSeqLen = 0;
  if (groupNo == 0) {
    for (int i = 0; i < 10; i++) ledPush(60, 60);
  } else {
    uint8_t tens = groupNo / 10, ones = groupNo % 10;
    for (int i = 0; i < tens; i++) ledPush(700, 400);
    if (tens && ones) ledSeq[ledSeqLen - 1] = 900;
    for (int i = 0; i < ones; i++) ledPush(200, 350);
  }
  if (ledSeqLen) ledSeq[ledSeqLen - 1] = 1500;
  ledSeqPos = 0; ledStepStart = millis(); ledSeqActive = true;
  digitalWrite(LED_PIN, HIGH);
}
bool bleConnected() { return PhyphoxBLE::currentConnections > 0; }
void ledUpdate() {
  uint32_t now = millis();
  if (ledSeqActive) {
    if (now - ledStepStart >= ledSeq[ledSeqPos]) {
      ledStepStart = now;
      if (++ledSeqPos >= ledSeqLen) ledSeqActive = false;
      else digitalWrite(LED_PIN, (ledSeqPos % 2 == 0) ? HIGH : LOW);
    }
    return;
  }
  bool c = bleConnected();
  if (c != lastConnected) { lastConnected = c; Serial.println(c ? "BLE 已連線" : "BLE 等待連線中"); }
  digitalWrite(LED_PIN, c ? HIGH : ((now % 1000) < 100 ? HIGH : LOW));
}

// 用藍燈閃出數字（阻塞，只在切換模式重開前使用）
void blinkCount(int n) {
  digitalWrite(LED_PIN, LOW); delay(400);
  for (int i = 0; i < n; i++) { digitalWrite(LED_PIN, HIGH); delay(250); digitalWrite(LED_PIN, LOW); delay(300); }
  delay(300);
}

// ---------- BOOT 鍵（GPIO0）：長按 1 秒切到下一個感測器 ----------
void pollBootButton() {
  static uint32_t downAt = 0;
  bool down = digitalRead(0) == LOW;
  if (down && !downAt) downAt = millis() | 1;
  if (!down) { downAt = 0; return; }
  if (millis() - downAt >= 1000) {
    int m = modeNo % M_COUNT + 1;
    if (modeNo == M_MOTOR) motorSet(0);
    nvsPutU8("mode", m);
    Serial.printf("BOOT 鍵切換模式 → %d %s\n", m, MODE_NAMES[m]);
    blinkCount(m);
    while (digitalRead(0) == LOW) delay(10);   // 等放開，避免重開後又進下載模式
    restartSoon("已儲存模式");
  }
}

// ---------- 由 phyphox 傳來的設定（輸入頻道 1 = 模式，2 = 依模式而定） ----------
volatile bool cfgPending = false;
volatile float cfgIn1 = 0, cfgIn2 = 0;
void onPhyphoxConfig() {
  float a = 0, b = 0;
  PhyphoxBLE::read(a, b);
  cfgIn1 = a; cfgIn2 = b; cfgPending = true;
}

// ---------- 脈衝擷取（光閘、霍爾） ----------
volatile uint32_t edgeT[64];
volatile uint8_t edgeLv[64];
volatile uint16_t edgeHead = 0;
uint16_t edgeTail = 0;
uint8_t pulsePin = 255, idleLevel = 1;

void IRAM_ATTR onEdge() {
  uint16_t h = edgeHead;
  edgeT[h & 63] = micros();
  edgeLv[h & 63] = (GPIO.in >> pulsePin) & 1;
  edgeHead = h + 1;
}
void startPulseCapture(uint8_t pin) {
  pulsePin = pin;
  pinMode(pin, INPUT_PULLUP);
  delay(50);
  // 開機時的狀態視為「沒有遮光 / 沒有磁鐵」
  int highs = 0;
  for (int i = 0; i < 50; i++) { highs += digitalRead(pin); delay(2); }
  idleLevel = highs > 25 ? 1 : 0;
  Serial.printf("脈衝輸入 GPIO%u，平常電位 = %s（開機時請勿遮住光閘 / 磁鐵不要停在感測器前）\n",
                pin, idleLevel ? "HIGH" : "LOW");
  attachInterrupt(digitalPinToInterrupt(pin), onEdge, CHANGE);
}

// 光閘
uint32_t gateStart = 0, gateLastStart = 0;
bool gateBlocked = false;
float gateLastDur = 0, gateLastPeriod = 0;
uint32_t gateCount = 0;

// 轉速
uint32_t rpmLastPulse = 0, rpmLastSend = 0;
float rpmValue = 0, rpmPeriod = 0;
bool rpmHavePulse = false;

void processEdges(uint32_t nowUs) {
  while (edgeTail != edgeHead) {
    uint32_t t = edgeT[edgeTail & 63];
    bool active = edgeLv[edgeTail & 63] != idleLevel;
    edgeTail++;
    if (modeNo == M_GATE) {
      if (active && !gateBlocked) {
        gateBlocked = true;
        gateLastPeriod = gateLastStart ? (t - gateLastStart) / 1e6f : 0;
        gateLastStart = gateStart = t;
      } else if (!active && gateBlocked) {
        gateBlocked = false;
        uint32_t d = t - gateStart;
        if (d < 50) continue;                       // 濾除雜訊
        gateLastDur = d / 1e6f;
        gateCount++;
        float tt = gateStart / 1e6f, cnt = gateCount;
        float dur = gateLastDur, per = gateLastPeriod;
        PhyphoxBLE::write(tt, dur, per, cnt);
      }
    } else if (active) {                            // 轉速：每次進入作用狀態算一個脈衝
      if (rpmHavePulse) {
        uint32_t dt = t - rpmLastPulse;
        if (dt < 300) continue;                     // 濾除彈跳
        rpmPeriod = dt / 1e6f * pulsesPerRev;       // 轉一圈的時間
        rpmValue = 60.0f / rpmPeriod;
      }
      rpmHavePulse = true;
      rpmLastPulse = t;
    }
  }
  if ((modeNo == M_RPM || modeNo == M_MOTOR) && rpmHavePulse && nowUs - rpmLastPulse > 3000000UL) {
    rpmValue = 0; rpmPeriod = 0;                    // 超過 3 秒沒脈衝視為停止
  }
}

// ---------- INA219（自寫驅動，分流電阻 0.1Ω） ----------
const uint8_t INA_ADDR = 0x40;
bool inaOk = false;
bool inaWrite(uint8_t reg, uint16_t v) {
  Wire.beginTransmission(INA_ADDR);
  Wire.write(reg); Wire.write(v >> 8); Wire.write(v & 0xFF);
  return Wire.endTransmission() == 0;
}
bool inaRead(uint8_t reg, int16_t &v) {
  Wire.beginTransmission(INA_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)INA_ADDR, 2) != 2) return false;
  v = (int16_t)((Wire.read() << 8) | Wire.read());
  return true;
}
bool inaBegin() {
  Wire.begin(SDA_PIN, SCL_PIN);
  inaOk = inaWrite(0x00, 0x399F);  // 32V、±320mV、12-bit、連續量測
  Serial.println(inaOk ? "INA219 已偵測到" : "找不到 INA219（檢查 SDA=21、SCL=22、VCC、GND）");
  return inaOk;
}
// 回傳 負載電壓 (V)、電流 (mA)
bool inaMeasure(float &volt, float &mA) {
  int16_t sh, bus;
  if (!inaRead(0x01, sh) || !inaRead(0x02, bus)) { inaOk = false; return false; }
  float vShunt = sh * 10e-6f;
  float vBus = ((uint16_t)bus >> 3) * 0.004f;
  mA = vShunt / 0.1f * 1000.0f;
  volt = vBus + vShunt;
  return true;
}

// ---------- DS18B20 ----------
OneWire oneWire(DS18_PIN);
DallasTemperature ds18(&oneWire);
uint8_t dsCount = 0;
uint32_t dsRequestAt = 0, dsRetryAt = 0;
bool dsWaiting = false;

void dsBegin() {
  ds18.begin();
  dsCount = ds18.getDeviceCount();
  if (dsCount > 2) dsCount = 2;
  ds18.setResolution(12);
  ds18.setWaitForConversion(false);
  Serial.printf("DS18B20 數量：%u%s\n", dsCount, dsCount ? "" : "（檢查 DQ=GPIO4、4.7kΩ 上拉、VCC、GND）");
}

// ---------- 馬達（L9110S） ----------
float motorPct = 0;
void motorSet(float pct) {
  if (pct > 100) pct = 100;
  if (pct < -100) pct = -100;
  motorPct = pct;
  uint32_t duty = (uint32_t)(fabsf(pct) * 255.0f / 100.0f + 0.5f);
  ledcWrite(0, pct > 0 ? duty : 0);
  ledcWrite(1, pct < 0 ? duty : 0);
}

// ---------- phyphox 實驗畫面 ----------
// 注意：phyphox 函式庫只存物件指標，所以畫面元件都用 static 保持存在
void addModeSelectorView(PhyphoxBleExperiment &exp) {
  static PhyphoxBleExperiment::View v;
  v.setLabel("換感測器");
  // 用「數字輸入」而非下拉選單：下拉選單要 phyphox 1.2.0 以上才支援，數字輸入所有版本都能用
  static PhyphoxBleExperiment::InfoField list;
  list.setInfo("輸入感測器編號：1 溫度｜2 光閘計時｜3 轉速｜4 電壓電流｜5 馬達特性");
  static PhyphoxBleExperiment::InfoField info;
  info.setInfo("輸入編號後按上方 ▶ 開始，ESP32 會儲存並自動重開（約 3 秒，藍燈會先閃出編號）。接著回到 phyphox 首頁，重新加入藍牙實驗，選同一個 PHY-G 名稱。也可以長按板子上的 BOOT 鍵 1 秒切到下一個感測器。");
  static PhyphoxBleExperiment::Edit dd;
  dd.setLabel("感測器編號");
  dd.setDefault(modeNo);
  dd.setSigned(false);
  dd.setDecimal(false);
  dd.setXMLAttribute("min=\"1\" max=\"5\"");
  dd.setChannel(1);
  static PhyphoxBleExperiment::InfoField pins;
  pins.setInfo("接線：DS18B20 DQ→4｜光閘 DO→27｜霍爾 DO→26｜INA219 SDA→21 SCL→22｜L9110S A-IA→25 A-IB→33｜VCC 3.3V、GND 共地");
  v.addElement(list);
  v.addElement(dd);
  v.addElement(info);
  v.addElement(pins);
  exp.addView(v);
}

void addExport(PhyphoxBleExperiment &exp, const char *setName, int n, const char *const labels[]) {
  static PhyphoxBleExperiment::ExportSet es;
  es.setLabel(setName);
  static PhyphoxBleExperiment::ExportData ed[5];
  for (int i = 0; i < n && i < 5; i++) {
    ed[i].setLabel(labels[i]);
    ed[i].setDatachannel(i + 1);
    es.addElement(ed[i]);
  }
  exp.addExportSet(es);
}

void makeGraph(PhyphoxBleExperiment::Graph &g, const char *label, int cx, int cy,
               const char *lx, const char *ux, const char *ly, const char *uy) {
  g.setLabel(label);
  g.setLabelX(lx); g.setUnitX(ux);
  g.setLabelY(ly); g.setUnitY(uy);
  g.setChannel(cx, cy);
}
void makeValue(PhyphoxBleExperiment::Value &v, const char *label, int ch, const char *unit, int prec) {
  v.setLabel(label); v.setChannel(ch); v.setUnit(unit); v.setPrecision(prec);
}

void buildExperiment() {
  PhyphoxBleExperiment exp;
  exp.setTitle(expTitle);
  exp.setCategory("ESP32 物理實驗");

  if (modeNo == M_TEMP) {
    exp.setDescription("DS18B20 溫度。頻道1 = 時間 t (s)，頻道2 = 溫度1 (°C)，頻道3 = 溫度2 (°C，第二支探頭)。每秒一筆。");
    static PhyphoxBleExperiment::View v; v.setLabel("溫度");
    static PhyphoxBleExperiment::Graph g; makeGraph(g, "溫度 vs 時間", 1, 2, "時間 t", "s", "溫度 T", "°C");
    static PhyphoxBleExperiment::Graph::Subgraph sg2;
    sg2.setChannel(1, 3); sg2.setColor("39a0ff");
    g.addSubgraph(sg2);
    static PhyphoxBleExperiment::Value v1, v2;
    makeValue(v1, "溫度 1", 2, "°C", 2);
    makeValue(v2, "溫度 2", 3, "°C", 2);
    v.addElement(g); v.addElement(v1); v.addElement(v2);
    exp.addView(v);
    const char *const L[] = {"t (s)", "T1 (°C)", "T2 (°C)"};
    addExport(exp, "溫度", 3, L);
  } else if (modeNo == M_GATE) {
    exp.setDescription("XD-51 光閘。每次遮光送一筆：頻道1 = 開始遮光時刻 (s)，2 = 遮光時間 Δt (s)，3 = 與上次遮光的間隔 (s)，4 = 次數。速度 = 擋片寬度 / Δt。");
    static PhyphoxBleExperiment::View v; v.setLabel("計時");
    static PhyphoxBleExperiment::Value a, b, c;
    makeValue(a, "遮光時間 Δt", 2, "s", 4);
    makeValue(b, "間隔（週期）", 3, "s", 4);
    makeValue(c, "遮光次數", 4, "", 0);
    static PhyphoxBleExperiment::Graph g; makeGraph(g, "遮光時間 vs 時刻", 1, 2, "時刻 t", "s", "遮光時間 Δt", "s");
    g.setStyle(STYLE_DOTS);
    v.addElement(a); v.addElement(b); v.addElement(c); v.addElement(g);
    exp.addView(v);
    static PhyphoxBleExperiment::View v2; v2.setLabel("週期");
    static PhyphoxBleExperiment::Graph g2; makeGraph(g2, "間隔 vs 時刻", 1, 3, "時刻 t", "s", "間隔 T", "s");
    g2.setStyle(STYLE_DOTS);
    v2.addElement(g2);
    exp.addView(v2);
    const char *const L[] = {"t (s)", "遮光時間 (s)", "間隔 (s)", "次數"};
    addExport(exp, "光閘", 4, L);
  } else if (modeNo == M_RPM) {
    exp.setDescription("霍爾感測器量轉速。頻道1 = 時間 (s)，2 = 轉速 (rpm)，3 = 轉一圈時間 (s)。轉盤上磁鐵數量請在下方輸入。");
    static PhyphoxBleExperiment::View v; v.setLabel("轉速");
    static PhyphoxBleExperiment::Graph g; makeGraph(g, "轉速 vs 時間", 1, 2, "時間 t", "s", "轉速", "rpm");
    static PhyphoxBleExperiment::Value a, b;
    makeValue(a, "轉速", 2, "rpm", 1);
    makeValue(b, "週期", 3, "s", 4);
    static PhyphoxBleExperiment::Edit s;
    s.setLabel("每圈磁鐵數（按 ▶ 生效）"); s.setDefault(pulsesPerRev); s.setSigned(false); s.setDecimal(false);
    s.setXMLAttribute("min=\"1\" max=\"12\""); s.setChannel(2);
    v.addElement(g); v.addElement(a); v.addElement(b); v.addElement(s);
    exp.addView(v);
    const char *const L[] = {"t (s)", "轉速 (rpm)", "週期 (s)"};
    addExport(exp, "轉速", 3, L);
  } else if (modeNo == M_POWER) {
    exp.setDescription("INA219 電壓電流。頻道1 = 時間 (s)，2 = 電壓 (V)，3 = 電流 (mA)，4 = 功率 (mW)。每 0.1 秒一筆。");
    static PhyphoxBleExperiment::View v; v.setLabel("電壓電流");
    static PhyphoxBleExperiment::Value a, b, c;
    makeValue(a, "電壓 V", 2, "V", 3);
    makeValue(b, "電流 I", 3, "mA", 1);
    makeValue(c, "功率 P", 4, "mW", 1);
    static PhyphoxBleExperiment::Graph g1; makeGraph(g1, "電流 vs 時間", 1, 3, "時間 t", "s", "電流 I", "mA");
    v.addElement(a); v.addElement(b); v.addElement(c); v.addElement(g1);
    exp.addView(v);
    static PhyphoxBleExperiment::View v2; v2.setLabel("V-I 圖");
    static PhyphoxBleExperiment::Graph g2; makeGraph(g2, "電壓 vs 電流", 3, 2, "電流 I", "mA", "電壓 V", "V");
    g2.setStyle(STYLE_DOTS);
    static PhyphoxBleExperiment::Graph g3; makeGraph(g3, "電壓 vs 時間", 1, 2, "時間 t", "s", "電壓 V", "V");
    v2.addElement(g2); v2.addElement(g3);
    exp.addView(v2);
    const char *const L[] = {"t (s)", "V (V)", "I (mA)", "P (mW)"};
    addExport(exp, "電壓電流", 4, L);
  } else {  // M_MOTOR
    exp.setDescription("L9110S 馬達。在「馬達輸出」輸入 -100~100（%，負值反轉），量測進行中改數字即時生效。頻道1 = 時間，2 = 輸出 %，3 = 轉速 rpm（霍爾 GPIO26），4 = 電壓 V，5 = 電流 mA（INA219）。");
    static PhyphoxBleExperiment::View v; v.setLabel("馬達");
    static PhyphoxBleExperiment::Edit s;
    s.setLabel("馬達輸出（-100～100）"); s.setDefault(0); s.setSigned(true); s.setDecimal(false); s.setUnit("%");
    s.setXMLAttribute("min=\"-100\" max=\"100\""); s.setChannel(2);
    static PhyphoxBleExperiment::Value a, b, c;
    makeValue(a, "轉速", 3, "rpm", 0);
    makeValue(b, "電壓", 4, "V", 2);
    makeValue(c, "電流", 5, "mA", 0);
    static PhyphoxBleExperiment::Graph g; makeGraph(g, "轉速 vs 時間", 1, 3, "時間 t", "s", "轉速", "rpm");
    v.addElement(s); v.addElement(a); v.addElement(b); v.addElement(c); v.addElement(g);
    exp.addView(v);
    static PhyphoxBleExperiment::View v2; v2.setLabel("特性曲線");
    static PhyphoxBleExperiment::Graph g2; makeGraph(g2, "轉速 vs 輸出", 2, 3, "輸出", "%", "轉速", "rpm");
    g2.setStyle(STYLE_DOTS);
    static PhyphoxBleExperiment::Graph g3; makeGraph(g3, "電流 vs 轉速", 3, 5, "轉速", "rpm", "電流", "mA");
    g3.setStyle(STYLE_DOTS);
    v2.addElement(g2); v2.addElement(g3);
    exp.addView(v2);
    const char *const L[] = {"t (s)", "輸出 (%)", "轉速 (rpm)", "V (V)", "I (mA)"};
    addExport(exp, "馬達", 5, L);
  }

  addModeSelectorView(exp);
  PhyphoxBLE::addExperiment(exp);
}

// ---------- 序列埠 ----------
void printInfo() {
  // 給燒錄網頁解析用，格式請勿更改
  Serial.printf("PHYBLE GROUP=%02u MODE=%u NAME=%s FW=%s\n", groupNo, modeNo, bleName, FW_VERSION);
  Serial.printf("模式 %u：%s\n", modeNo, MODE_NAMES[modeNo]);
  if (groupNo == 0) Serial.println("尚未設定組別。輸入 G 1 ~ G 99 設定（例如 G 3）。");
}

int parseNum(String s) {
  s.replace("=", " "); s.replace(":", " "); s.trim();
  if (s.length() == 0 || s.length() > 2) return -1;
  for (unsigned i = 0; i < s.length(); i++) if (!isDigit(s.charAt(i))) return -1;
  return s.toInt();
}

void handleCommand(String cmd) {
  cmd.trim();
  if (!cmd.length()) return;
  String up = cmd; up.toUpperCase();
  if (up == "?" || up == "INFO") { printInfo(); return; }
  if (up == "LED") { Serial.printf("LED 閃出組別 %02u\n", groupNo); ledShowGroup(); return; }
  if (up.startsWith("SET")) {
    String rest = up.substring(3); rest.trim(); rest.replace(",", " ");
    int sp = rest.indexOf(' ');
    int g = sp > 0 ? parseNum(rest.substring(0, sp)) : -1;
    int m = sp > 0 ? parseNum(rest.substring(sp + 1)) : -1;
    if (g >= 1 && g <= 99 && m >= 1 && m <= M_COUNT) {
      nvsPutU8("group", g); nvsPutU8("mode", m);
      Serial.printf("OK GROUP=%02d MODE=%d\n", g, m);
      restartSoon("已儲存");
    }
    Serial.println("ERR 格式：SET <組別 1-99> <模式 1-5>，例如 SET 3 2");
    return;
  }
  int start = -1; bool isMode = false;
  if (up.startsWith("GROUP")) start = 5;
  else if (up.startsWith("MODE")) { start = 4; isMode = true; }
  else if (up.startsWith("G")) start = 1;
  else if (up.startsWith("M")) { start = 1; isMode = true; }
  if (start >= 0) {
    int n = parseNum(up.substring(start));
    if (!isMode && n >= 1 && n <= 99) {
      nvsPutU8("group", n);
      Serial.printf("OK GROUP=%02d NAME=PHY-G%02d\n", n, n);
      restartSoon("已儲存組別");
    } else if (isMode && n >= 1 && n <= M_COUNT) {
      nvsPutU8("mode", n);
      Serial.printf("OK MODE=%d %s\n", n, MODE_NAMES[n]);
      restartSoon("已儲存模式");
    }
    Serial.println(isMode ? "ERR 模式需為 1~5" : "ERR 組別需為 1~99");
    return;
  }
  Serial.println("ERR 可用指令：G <1-99>、M <1-5>、SET <組別> <模式>、?、LED");
}

void pollSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') { handleCommand(serialLine); serialLine = ""; }
    else if (serialLine.length() < 32) serialLine += c;
  }
}

// ---------- 主程式 ----------
void setup() {
  Serial.begin(115200);
  delay(200);
  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW);
  pinMode(0, INPUT_PULLUP);
  // 馬達腳先拉低，避免開機時馬達亂轉
  pinMode(MOTOR_A, OUTPUT); digitalWrite(MOTOR_A, LOW);
  pinMode(MOTOR_B, OUTPUT); digitalWrite(MOTOR_B, LOW);

  groupNo = nvsGetU8("group", 0);
  if (groupNo > 99) groupNo = 0;
  modeNo = nvsGetU8("mode", M_TEMP);
  if (modeNo < 1 || modeNo > M_COUNT) modeNo = M_TEMP;
  pulsesPerRev = nvsGetU8("ppr", 1);
  if (pulsesPerRev < 1 || pulsesPerRev > 12) pulsesPerRev = 1;

  snprintf(bleName, sizeof(bleName), "PHY-G%02u", groupNo);
  snprintf(expTitle, sizeof(expTitle), "%s %s", MODE_NAMES[modeNo], bleName);

  PhyphoxBLE::start(bleName);
  PhyphoxBLE::configHandler = &onPhyphoxConfig;
  buildExperiment();

  switch (modeNo) {
    case M_TEMP:  dsBegin(); break;
    case M_GATE:  startPulseCapture(GATE_PIN); break;
    case M_RPM:   startPulseCapture(HALL_PIN); break;
    case M_POWER: inaBegin(); break;
    case M_MOTOR:
      ledcSetup(0, 5000, 8); ledcAttachPin(MOTOR_A, 0);
      ledcSetup(1, 5000, 8); ledcAttachPin(MOTOR_B, 1);
      motorSet(0);
      startPulseCapture(HALL_PIN);
      inaBegin();
      break;
  }
  printInfo();
  ledShowGroup();
}

void handlePhoneInput() {
  if (!cfgPending) return;
  cfgPending = false;
  float in1 = cfgIn1, in2 = cfgIn2;
  static float last1 = -999, last2 = -999;
  if (in1 != last1 || (in2 != last2 && !(isnan(in2) && isnan(last2)))) {
    Serial.printf("phyphox 傳來：輸入1=%.2f 輸入2=%.2f\n", in1, in2);
    last1 = in1; last2 = in2;
  }
  int m = (int)lroundf(in1);
  if (m >= 1 && m <= M_COUNT && fabsf(in1 - m) < 0.01f && m != modeNo) {
    if (modeNo == M_MOTOR) motorSet(0);
    nvsPutU8("mode", m);
    Serial.printf("iPad 切換模式 → %d %s\n", m, MODE_NAMES[m]);
    blinkCount(m);
    restartSoon("已儲存模式");
  }
  if (isnan(in2)) return;
  if (modeNo == M_MOTOR && in2 >= -100.5f && in2 <= 100.5f) {
    if (fabsf(in2 - motorPct) < 0.5f) return;
    motorSet(in2);
    Serial.printf("馬達輸出 %.0f%%\n", motorPct);
  } else if (modeNo == M_RPM) {
    int p = (int)lroundf(in2);
    if (p >= 1 && p <= 12 && p != pulsesPerRev) {
      pulsesPerRev = p; nvsPutU8("ppr", p);
      Serial.printf("每圈磁鐵數 = %d\n", p);
    }
  }
}

void loop() {
  static uint32_t lastSample = 0;
  pollSerial();
  ledUpdate();
  handlePhoneInput();
  pollBootButton();

  uint32_t nowMs = millis();
  float t = nowMs / 1000.0f;

  switch (modeNo) {
    case M_TEMP:
      if (dsCount == 0) {
        if (nowMs >= dsRetryAt) { dsRetryAt = nowMs + 3000; dsBegin(); }
      } else if (!dsWaiting) {
        ds18.requestTemperatures();
        dsRequestAt = nowMs; dsWaiting = true;
      } else if (nowMs - dsRequestAt >= 800) {
        dsWaiting = false;
        float t1 = ds18.getTempCByIndex(0);
        float t2 = dsCount > 1 ? ds18.getTempCByIndex(1) : NAN;
        if (t1 == DEVICE_DISCONNECTED_C) { dsCount = 0; Serial.println("DS18B20 讀取失敗，重新偵測"); break; }
        if (t2 == DEVICE_DISCONNECTED_C) t2 = NAN;
        float ts = dsRequestAt / 1000.0f;
        PhyphoxBLE::write(ts, t1, t2);
      }
      break;

    case M_GATE:
      processEdges(micros());
      break;

    case M_RPM:
      processEdges(micros());
      if (nowMs - rpmLastSend >= 200) {
        rpmLastSend = nowMs;
        float r = rpmValue, p = rpmPeriod;
        PhyphoxBLE::write(t, r, p);
      }
      break;

    case M_POWER:
      if (nowMs - lastSample >= 100) {
        lastSample = nowMs;
        float v, i;
        if (inaOk && inaMeasure(v, i)) {
          float p = v * i;
          PhyphoxBLE::write(t, v, i, p);
        } else if (!inaOk && nowMs % 3000 < 100) {
          inaBegin();
        }
      }
      break;

    case M_MOTOR:
      processEdges(micros());
      if (nowMs - lastSample >= 100) {
        lastSample = nowMs;
        float v = NAN, i = NAN;
        if (!(inaOk && inaMeasure(v, i))) { v = NAN; i = NAN; }
        float pct = motorPct, r = rpmValue;
        PhyphoxBLE::write(t, pct, r, v, i);
      }
      break;
  }
  PhyphoxBLE::poll();
}
