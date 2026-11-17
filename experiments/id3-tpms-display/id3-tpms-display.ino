/*
 * id3-tpms-display —— ID.3 蓝牙胎压小屏（正式固件 v1）
 *
 * 硬件：ESP32-2424S012（ESP32-C3 + GC9A01 240×240 圆屏 + CST816D 触摸）
 *   LCD : SCLK=6 MOSI=7 DC=2 CS=10 RST=-1  BL=3(高有效, LEDC 通道必须 0~5)
 *   触摸: SDA=4 SCL=5 INT=0 RST=1（本版未使用）
 *
 * 功能：被动扫描 BLE 广播 → 解析胎压 → 圆屏显示四轮 + 低压变色 + 数据时效
 * 依赖：LovyanGFX（core 自带 Bluedroid，无其他库）
 *
 * 串口（115200，换行）：h 帮助 | a 显示全部设备 | m dump | d 实时hex
 *                        c <bar> BR 校准 | u 切 BR 组合 | z 清空轮位绑定
 */

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

/* ---------------- 显示 ---------------- */
#define PIN_LCD_SCLK 6
#define PIN_LCD_MOSI 7
#define PIN_LCD_DC   2
#define PIN_LCD_CS   10
#define PIN_LCD_BL   3

class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_GC9A01 _panel;
  lgfx::Bus_SPI      _bus;
  lgfx::Light_PWM    _light;
 public:
  LGFX() {
    { auto c = _bus.config();
      c.spi_host = SPI2_HOST; c.spi_mode = 0;
      c.freq_write = 40000000; c.freq_read = 16000000;
      c.pin_sclk = PIN_LCD_SCLK; c.pin_mosi = PIN_LCD_MOSI;
      c.pin_miso = -1; c.pin_dc = PIN_LCD_DC;
      _bus.config(c); _panel.setBus(&_bus); }
    { auto c = _panel.config();
      c.pin_cs = PIN_LCD_CS; c.pin_rst = -1;
      c.panel_width = 240; c.panel_height = 240;
      c.readable = false; c.invert = true; c.rgb_order = false;
      _panel.config(c); }
    { auto c = _light.config();
      c.pin_bl = PIN_LCD_BL; c.freq = 12000; c.pwm_channel = 0;   // C3 只有 0~5
      _light.config(c); _panel.setLight(&_light); }
    setPanel(&_panel);
  }
};
static LGFX lcd;

static const uint16_t C_BG      = TFT_BLACK;
static const uint16_t C_RING    = 0xB01FE0;
static const uint16_t C_RING_IN = 0x50107;
static const uint16_t C_VAL     = TFT_WHITE;
static const uint16_t C_DIM     = 0x7BEF;
static const uint16_t C_UNIT    = 0x8C71;
static const uint16_t C_OK      = 0x07E0;
static const uint16_t C_WARN    = 0xFD20;
static const uint16_t C_BAD     = 0xF800;
static const uint16_t C_CAR     = 0xCE79;

/* ---------------- 传感器数据 ---------------- */
#define MAX_SENSORS 8
#define WHEELS      4
#define LOW_BAR     2.0f      // 低于此值报警（ID.3 建议冷态 2.3~2.5）
#define STALE_MS    600000UL  // 10 分钟无更新视为失效

enum SensorType : uint8_t { T_UNKNOWN = 0, T_BR = 1, T_ZEEPIN = 2 };

struct Slot {
  bool     used;
  char     mac[20];
  uint8_t  wheel;         // 0..3，0xFF 未绑定
  uint8_t  type;
  float    bar;
  float    tempC;
  int      batt;
  float    battV;
  uint32_t lastSeenMs;
  uint32_t count;
  uint16_t rawP;
  uint8_t  raw[24];
  uint8_t  rawLen;
};

struct AdvInfo {
  const uint8_t* mfr;
  uint8_t  mfrLen;
  char     name[16];
  bool     uuid27a5;
};

static Slot    g_slot[MAX_SENSORS];
static int8_t  g_brCombo = -1;
static float   g_calScale = 0.0f;
static bool    g_showAll = false;
static bool    g_rawDump = false;
static bool    g_dirty = true;
static uint32_t g_lastDraw = 0;
static uint32_t g_scanCount = 0;

/* ---------------- 工具 ---------------- */
static bool plausibleBar(float bar) { return bar > 0.5f && bar < 6.0f; }

static uint32_t rd32le(const uint8_t* d, uint8_t o) {
  return (uint32_t)d[o] | ((uint32_t)d[o + 1] << 8) | ((uint32_t)d[o + 2] << 16) | ((uint32_t)d[o + 3] << 24);
}

static void hexDump(const uint8_t* p, uint8_t n, char* out, size_t outSize) {
  size_t k = 0;
  if (outSize) out[0] = 0;
  for (uint8_t i = 0; i < n && k + 4 < outSize; i++) k += snprintf(out + k, outSize - k, "%02X ", p[i]);
}

static void parseAdv(const uint8_t* p, size_t n, AdvInfo& out) {
  out.mfr = nullptr; out.mfrLen = 0; out.name[0] = 0; out.uuid27a5 = false;
  size_t i = 0;
  while (i + 1 < n) {
    uint8_t len = p[i];
    if (len == 0 || i + 1 + len > n) break;
    uint8_t type = p[i + 1];
    const uint8_t* d = p + i + 2;
    uint8_t dlen = len - 1;
    if (type == 0xFF && dlen > 0) { out.mfr = d; out.mfrLen = dlen; }
    else if ((type == 0x08 || type == 0x09) && dlen > 0) {
      uint8_t c = dlen < (uint8_t)(sizeof(out.name) - 1) ? dlen : (uint8_t)(sizeof(out.name) - 1);
      for (uint8_t k = 0; k < c; k++) out.name[k] = (d[k] >= 0x20 && d[k] <= 0x7E) ? (char)d[k] : '.';
      out.name[c] = 0;
    } else if (type == 0x03 && dlen >= 2) {
      for (uint8_t k = 0; k + 1 < dlen; k += 2) if (d[k] == 0xA5 && d[k + 1] == 0x27) out.uuid27a5 = true;
    }
    i += (size_t)len + 1;
  }
}

static const char* brComboName(int c) {
  switch (c) {
    case 0: return "小端+0.1psi";
    case 1: return "大端+0.1psi";
    case 2: return "小端+0.1kPa";
    case 3: return "大端+0.1kPa";
    default: return "未判定";
  }
}

static float brBar(uint16_t rawLE, int combo) {
  uint16_t v = (combo & 1) ? (uint16_t)((rawLE >> 8) | (rawLE << 8)) : rawLE;
  float unit = v * 0.1f;
  return (combo & 2) ? (unit / 100.0f) : (unit * 0.06894757f);
}

static bool zeepinPlausible(const uint8_t* d, uint8_t len) {
  if (len < 18) return false;
  float t = rd32le(d, 12) / 100.0f;
  if (t < -40.0f || t > 120.0f) return false;
  if (d[16] > 100) return false;
  if (d[17] > 1) return false;
  return plausibleBar(rd32le(d, 8) / 100000.0f) || plausibleBar(rd32le(d, 8) / 10000.0f);
}

static bool brPlausible(const uint8_t* d, uint8_t len) {
  if (len < 5 || len > 9) return false;
  float v = d[1] * 0.1f;
  if (v < 2.0f || v > 3.5f) return false;
  float t = (float)(int8_t)d[2];
  if (t < -40.0f || t > 85.0f) return false;
  uint16_t raw = (uint16_t)d[3] | ((uint16_t)d[4] << 8);
  for (int c = 0; c < 4; c++) if (plausibleBar(brBar(raw, c))) return true;
  return false;
}

static void decodeBR(const uint8_t* d, Slot& s) {
  s.battV = d[1] * 0.1f;
  s.tempC = (float)(int8_t)d[2];
  s.rawP  = (uint16_t)d[3] | ((uint16_t)d[4] << 8);
  if (g_brCombo < 0) {
    for (int c = 0; c < 4; c++) {
      if (plausibleBar(brBar(s.rawP, c))) { g_brCombo = (int8_t)c; Serial.printf("[自动判定] BR = %s\r\n", brComboName(g_brCombo)); break; }
    }
  }
  if (g_calScale > 0.0f)     s.bar = s.rawP * g_calScale;
  else if (g_brCombo >= 0)   s.bar = brBar(s.rawP, g_brCombo);
  else                       s.bar = brBar(s.rawP, 0);
}

static void decodeZeepin(const uint8_t* d, Slot& s) {
  uint32_t praw = rd32le(d, 8), traw = rd32le(d, 12);
  float a = praw / 100000.0f, b = praw / 10000.0f;
  s.bar = plausibleBar(a) ? a : (plausibleBar(b) ? b : a);
  s.tempC = traw / 100.0f;
  s.batt = d[16];
  uint8_t w = (d[2] >= 0x80 && d[2] <= 0x8F) ? (uint8_t)(d[2] - 0x80) : 0xFF;
  if (s.wheel == 0xFF && w < WHEELS) s.wheel = w;
}

/* ---------------- 轮位管理 ---------------- */
static int findSlot(const char* mac) {
  for (int i = 0; i < MAX_SENSORS; i++) if (g_slot[i].used && strcmp(g_slot[i].mac, mac) == 0) return i;
  return -1;
}

static uint8_t nextFreeWheel() {
  for (uint8_t w = 0; w < WHEELS; w++) {
    bool taken = false;
    for (int i = 0; i < MAX_SENSORS; i++) if (g_slot[i].used && g_slot[i].wheel == w) { taken = true; break; }
    if (!taken) return w;
  }
  return 0xFF;
}

static int addSlot(const char* mac) {
  for (int i = 0; i < MAX_SENSORS; i++) {
    if (g_slot[i].used) continue;
    memset(&g_slot[i], 0, sizeof(Slot));
    g_slot[i].used = true;
    strncpy(g_slot[i].mac, mac, sizeof(g_slot[i].mac) - 1);
    g_slot[i].wheel = nextFreeWheel();
    g_slot[i].bar = NAN;
    g_slot[i].tempC = NAN;
    g_slot[i].batt = -1;
    g_slot[i].battV = NAN;
    return i;
  }
  return -1;
}

/* ---------------- 界面 ---------------- */
static void drawStatic() {
  lcd.fillScreen(C_BG);
  lcd.drawArc(120, 120, 117, 119, 0, 359, C_RING);      // 外环外沿 r=119（实测边界）
  lcd.drawArc(120, 120, 113, 113, 0, 359, C_RING_IN);

  lcd.setTextDatum(textdatum_t::middle_center);
  lcd.setTextSize(1);
  lcd.setTextColor(C_UNIT, C_BG);
  for (int i = 0; i < WHEELS; i++) { }
  lcd.drawString("bar", 76, 104);  lcd.drawString("bar", 164, 104);
  lcd.drawString("bar", 76, 176);  lcd.drawString("bar", 164, 176);

  // 俯视车模
  lcd.fillRoundRect(106, 100, 28, 44, 7, C_CAR);
  lcd.fillRoundRect(100, 106, 6, 11, 2, C_CAR);
  lcd.fillRoundRect(134, 106, 6, 11, 2, C_CAR);
  lcd.fillRoundRect(100, 128, 6, 11, 2, C_CAR);
  lcd.fillRoundRect(134, 128, 6, 11, 2, C_CAR);
  lcd.fillRoundRect(110, 106, 20, 8, 3, 0x2104);

  lcd.setTextColor(C_UNIT, C_BG);
  lcd.drawString("ID.3", 120, 190);
}

static const int WX[WHEELS] = {76, 164, 76, 164};
static const int WY[WHEELS] = {84, 84, 156, 156};

static void drawWheel(uint8_t w) {
  if (w >= WHEELS) return;
  // 找这个轮位的传感器
  Slot* s = nullptr;
  for (int i = 0; i < MAX_SENSORS; i++) if (g_slot[i].used && g_slot[i].wheel == w) { s = &g_slot[i]; break; }

  int cx = WX[w], cy = WY[w];
  lcd.fillRect(cx - 30, cy - 14, 60, 28, C_BG);      // 清空数值区

  char buf[10];
  uint16_t col = C_DIM;
  if (!s || isnan(s->bar)) {
    snprintf(buf, sizeof(buf), "--");
    col = C_DIM;
  } else {
    bool stale = (millis() - s->lastSeenMs) > STALE_MS;
    snprintf(buf, sizeof(buf), "%.1f", s->bar);
    if (stale)               col = C_DIM;
    else if (s->bar < LOW_BAR) col = C_BAD;
    else if (s->bar < 2.2f)  col = C_WARN;
    else                     col = C_VAL;
  }
  lcd.setTextDatum(textdatum_t::middle_center);
  lcd.setTextSize(3);
  lcd.setTextColor(col, C_BG);
  lcd.drawString(buf, cx, cy);
}

static void drawStatus() {
  // 顶部状态
  int n = 0, low = 0, warn = 0, stale = 0;
  uint32_t newest = 0;
  for (int i = 0; i < MAX_SENSORS; i++) {
    if (!g_slot[i].used || isnan(g_slot[i].bar)) continue;
    n++;
    if ((millis() - g_slot[i].lastSeenMs) > STALE_MS) { stale++; continue; }
    if (g_slot[i].bar < LOW_BAR) low++;
    else if (g_slot[i].bar < 2.2f) warn++;
    if (g_slot[i].lastSeenMs > newest) newest = g_slot[i].lastSeenMs;
  }

  lcd.fillRect(40, 30, 160, 20, C_BG);
  lcd.setTextDatum(textdatum_t::top_center);
  lcd.setTextSize(2);
  const char* txt; uint16_t col;
  if (n == 0)            { txt = "SCAN..."; col = C_DIM; }
  else if (low > 0)      { txt = "LOW PRESSURE"; col = C_BAD; }
  else if (warn > 0)     { txt = "CHECK"; col = C_WARN; }
  else if (stale == n)   { txt = "NO DATA"; col = C_DIM; }
  else                   { txt = "ALL OK"; col = C_OK; }
  lcd.setTextColor(col, C_BG);
  lcd.drawString(txt, 120, 34);

  // 底部时效
  lcd.fillRect(60, 196, 120, 14, C_BG);
  lcd.setTextDatum(textdatum_t::middle_center);
  lcd.setTextSize(1);
  lcd.setTextColor(C_UNIT, C_BG);
  char age[16];
  if (n == 0) snprintf(age, sizeof(age), "waiting");
  else {
    uint32_t sec = (millis() - newest) / 1000;
    if (sec < 60)       snprintf(age, sizeof(age), "%lus ago", (unsigned long)sec);
    else if (sec < 3600) snprintf(age, sizeof(age), "%lum ago", (unsigned long)(sec / 60));
    else                 snprintf(age, sizeof(age), "%lh ago", (unsigned long)(sec / 3600));
  }
  lcd.drawString(age, 108, 202);

  // 电量图标（用第一个有数据的传感器电量/电压示意）
  lcd.drawRect(148, 198, 16, 8, n ? C_OK : C_DIM);
  lcd.fillRect(164, 200, 2, 4, n ? C_OK : C_DIM);
  if (n) lcd.fillRect(149, 199, 14, 6, C_OK);
}

static void redrawAll() {
  drawStatic();
  for (uint8_t w = 0; w < WHEELS; w++) drawWheel(w);
  drawStatus();
  lcd.fillRect(100, 112, 40, 22, C_BG);      // 车顶区避免和文字重叠时留白
  lcd.fillRoundRect(106, 100, 28, 44, 7, C_CAR);
  lcd.fillRoundRect(100, 106, 6, 11, 2, C_CAR);
  lcd.fillRoundRect(134, 106, 6, 11, 2, C_CAR);
  lcd.fillRoundRect(100, 128, 6, 11, 2, C_CAR);
  lcd.fillRoundRect(134, 128, 6, 11, 2, C_CAR);
  lcd.fillRoundRect(110, 106, 20, 8, 3, 0x2104);
}

/* ---------------- BLE ---------------- */
static void handleAdv(const char* mac, int8_t rssi, const uint8_t* payload, size_t plen,
                      const std::string& mfrFallback, const char* nameFallback) {
  AdvInfo info;
  parseAdv(payload, plen, info);

  const uint8_t* mfr = info.mfr;
  uint8_t mfrLen = info.mfrLen;
  if (!mfr && mfrFallback.size() > 0 && mfrFallback.size() <= 24) {
    mfr = (const uint8_t*)mfrFallback.data();
    mfrLen = (uint8_t)mfrFallback.size();
  }
  if (info.name[0] == 0 && nameFallback && nameFallback[0]) {
    strncpy(info.name, nameFallback, sizeof(info.name) - 1);
  }
  g_scanCount++;

  bool okZP = mfr && zeepinPlausible(mfr, mfrLen);
  bool okBR = mfr && brPlausible(mfr, mfrLen);
  if (!okZP && !okBR) {
    if (g_showAll && mfr) Serial.printf("[skip] %-17s len=%u %s\r\n", mac, mfrLen, info.name);
    return;
  }

  int idx = findSlot(mac);
  if (idx < 0) {
    idx = addSlot(mac);
    if (idx < 0) return;
    Serial.printf("[新传感器] %s 轮位=%d\r\n", mac, g_slot[idx].wheel);
  }
  Slot& s = g_slot[idx];
  s.lastSeenMs = millis();
  s.count++;
  s.rawLen = mfrLen < sizeof(s.raw) ? mfrLen : (uint8_t)sizeof(s.raw);
  memcpy(s.raw, mfr, s.rawLen);

  if (okZP && !(okBR && (info.uuid27a5 || strcmp(info.name, "BR") == 0))) {
    s.type = T_ZEEPIN; decodeZeepin(mfr, s);
  } else {
    s.type = T_BR;     decodeBR(mfr, s);
  }

  char hex[80]; hexDump(s.raw, s.rawLen, hex, sizeof(hex));
  Serial.printf("[%s] wheel=%d %s %.2fbar %.0fC raw=%s\r\n",
                mac, s.wheel, s.type == T_BR ? "BR" : "ZEEPIN", s.bar, s.tempC, hex);
  g_dirty = true;
}

static const uint8_t* advPayload(BLEAdvertisedDevice& dev, size_t& len) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  std::string s = dev.getPayload(); len = s.size(); return (const uint8_t*)s.data();
#else
  len = dev.getPayloadLength(); return dev.getPayload();
#endif
}

class CB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) {
    size_t plen = 0;
    const uint8_t* pl = advPayload(dev, plen);
    std::string mfr = dev.getManufacturerData();
    const char* nm = dev.haveName() ? dev.getName().c_str() : "";
    handleAdv(dev.getAddress().toString().c_str(), (int8_t)dev.getRSSI(), pl, plen, mfr, nm);
  }
};

/* ---------------- 串口 ---------------- */
static char    g_line[48];
static uint8_t g_len = 0;

static void dumpSlots() {
  Serial.printf("\r\n---- 已绑定传感器 ----\r\n");
  int n = 0;
  for (int i = 0; i < MAX_SENSORS; i++) {
    if (!g_slot[i].used) continue;
    char hex[80]; hexDump(g_slot[i].raw, g_slot[i].rawLen, hex, sizeof(hex));
    Serial.printf("wheel=%d %-17s type=%s %.2fbar %.0fC n=%lu %s\r\n",
                  g_slot[i].wheel, g_slot[i].mac, g_slot[i].type == T_BR ? "BR    " : "ZEEPIN",
                  g_slot[i].bar, g_slot[i].tempC, (unsigned long)g_slot[i].count, hex);
    n++;
  }
  if (!n) Serial.printf("(空) 传感器还没广播\r\n");
  Serial.printf("扫描次数=%lu  BR=%s  cal=%s\r\n--------- --------------\r\n",
                (unsigned long)g_scanCount, brComboName(g_brCombo), g_calScale > 0 ? "on" : "off");
}

static void pollSerial() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch != '\n') { if (g_len < sizeof(g_line) - 1) g_line[g_len++] = ch; continue; }
    g_line[g_len] = 0; g_len = 0;
    String cmd(g_line); cmd.trim();
    if (cmd.length() == 0) { dumpSlots(); continue; }
    char c0 = cmd[0];
    if (c0 == 'h') Serial.printf("h 帮助 | a 显示跳过项 | m dump | d 实时hex | c <bar> 校准 | u 切BR组合 | z 清空\r\n");
    else if (c0 == 'a') { g_showAll = !g_showAll; Serial.printf("显示跳过项：%s\r\n", g_showAll ? "开" : "关"); }
    else if (c0 == 'd') { g_rawDump = !g_rawDump; Serial.printf("实时hex：%s\r\n", g_rawDump ? "开" : "关"); }
    else if (c0 == 'm') dumpSlots();
    else if (c0 == 'z') { memset(g_slot, 0, sizeof(g_slot)); g_dirty = true; Serial.printf("已清空\r\n"); }
    else if (c0 == 'u') { g_brCombo = (int8_t)((g_brCombo + 1) % 4); Serial.printf("BR 组合：%s\r\n", brComboName(g_brCombo)); g_dirty = true; }
    else if (c0 == 'c') {
      float bar = cmd.substring(1).toFloat();
      for (int i = 0; i < MAX_SENSORS; i++) {
        if (g_slot[i].used && g_slot[i].type == T_BR && g_slot[i].rawP) {
          g_calScale = bar / (float)g_slot[i].rawP;
          Serial.printf("校准：raw=%u → %.2fbar, scale=%.6f\r\n", g_slot[i].rawP, bar, g_calScale);
          g_dirty = true;
          break;
        }
      }
    } else Serial.printf("未知命令\r\n");
  }
}

/* ---------------- 主流程 ---------------- */
void setup() {
  Serial.begin(115200);
  delay(300);
  memset(g_slot, 0, sizeof(g_slot));
  Serial.printf("\r\n=== ID.3 TPMS display v1 ===\r\n");

  lcd.init();
  lcd.setBrightness(230);
  redrawAll();
  Serial.printf("屏 %dx%d 就绪；heap=%u\r\n", (int)lcd.width(), (int)lcd.height(), (unsigned)ESP.getFreeHeap());

  BLEDevice::init("id3-tpms");
  BLEScan* scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new CB(), true);
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(80);
  Serial.printf("BLE 扫描已启动；heap=%u\r\n", (unsigned)ESP.getFreeHeap());
  Serial.printf("命令：h 帮助 | m dump | c <bar> 校准\r\n");
}

void loop() {
  pollSerial();

  BLEScan* scan = BLEDevice::getScan();
  scan->start(2);
  scan->clearResults();

  uint32_t now = millis();
  static uint32_t lastStatus = 0;
  if (g_dirty || now - g_lastDraw > 1000) {
    g_dirty = false;
    g_lastDraw = now;
    for (uint8_t w = 0; w < WHEELS; w++) drawWheel(w);
    drawStatus();
  }
  delay(10);
}
