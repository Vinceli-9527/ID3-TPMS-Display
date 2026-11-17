/*
 * id3-tpms-ui v2 —— 局部刷新版（消除全屏闪烁）
 *
 * 关键改动：不再每 500ms 清屏重画。屏幕分三层：
 *   drawStatic()  只画一次：外环、车模、ID.3
 *   drawCell(i)   仅清除+重画该轮位那一小块（数据变化时）
 *   drawStatus()  仅清除+重画状态文字和状态灯那一小块（闪烁时）
 *   drawAge()     仅重画时效文字
 *
 * 协议（LYTPMS 1.0.8.821 反编译）：
 *   adv 前 7 字节 02 01 06 03 03 B0 FB
 *   V = b[4]*0.01+1.22 ; P = b[5]*3.144 kPa (b[10]==1 → (b[5]+256)*3.144)
 *   T = b[6]-55 ; st = b[7] ; ID = b[18]b[17]b[16]
 */

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#define PIN_LCD_SCLK 6
#define PIN_LCD_MOSI 7
#define PIN_LCD_DC   2
#define PIN_LCD_CS   10
#define PIN_LCD_BL   3
#define PIN_BOOT     9
#define LOW_BAR      2.0f
#define WARN_BAR     2.2f
#define STALE_MS     600000UL
#define FW_VERSION   "0.1.0"

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
      c.pin_bl = PIN_LCD_BL; c.freq = 12000; c.pwm_channel = 0;
      _light.config(c); _panel.setLight(&_light); }
    setPanel(&_panel);
  }
};
static LGFX lcd;

struct W { bool has; char id[8]; float kPa, tempC, volt; uint8_t state; uint32_t at; uint16_t cnt; };
static W g_w[4];
static uint8_t g_n = 0;
static uint32_t g_frames = 0;
static bool g_blink = false;
static bool g_dirty = true;
static char g_lastStatus[10] = "";

static const uint16_t C_BG = TFT_BLACK;
static const uint16_t C_RING = 0x9400D3;    // 紫色外环
static const uint16_t C_RINGIN = 0x4B0082;  // 深紫（indigo）内圈
static const uint16_t C_VAL = TFT_WHITE;
static const uint16_t C_DIM = 0x5AEB;
static const uint16_t C_UNIT = 0x8C71;
static const uint16_t C_OK = 0x07E0;
static const uint16_t C_WARN = 0xFD20;
static const uint16_t C_BAD = 0xF800;
static const uint16_t C_CAR = 0xCE79;

static const int WX[4] = {70, 170, 70, 170};
static const int WY[4] = {78, 78, 154, 154};

/* ---------- 静态层：只在开机/清空时画 ---------- */
static void drawStatic() {
  lcd.fillScreen(C_BG);
  lcd.drawArc(120, 120, 117, 119, 0, 359, C_RING);
  lcd.drawArc(120, 120, 113, 113, 0, 359, C_RINGIN);

  // 车模
  lcd.fillRoundRect(110, 102, 20, 34, 6, C_CAR);
  lcd.fillRoundRect(106, 107, 4, 9, 2, C_CAR);
  lcd.fillRoundRect(130, 107, 4, 9, 2, C_CAR);
  lcd.fillRoundRect(106, 124, 4, 9, 2, C_CAR);
  lcd.fillRoundRect(130, 124, 4, 9, 2, C_CAR);
  lcd.fillRoundRect(113, 106, 14, 6, 2, 0x2104);

  // 车标
  lcd.setTextDatum(textdatum_t::middle_center);
  lcd.setTextSize(1);
  lcd.setTextColor(C_UNIT, C_BG);
  lcd.drawString("ID.3", 120, 192);

  lcd.setTextColor(0x39E7, C_BG);
  lcd.drawString("v" FW_VERSION, 120, 218);
}

/* ---------- 轮位单元格：只擦自己那块 ---------- */
static void drawCell(int i) {
  int cx = WX[i], cy = WY[i];
  lcd.fillRect(cx - 34, cy - 34, 68, 62, C_BG);     // 只清这一块，不碰外环

  char v[10] = "--", t[8] = "--", id[6] = "----";
  uint16_t col = C_DIM;
  if (i < g_n && g_w[i].has) {
    float bar = g_w[i].kPa / 100.0f;
    bool stale = (millis() - g_w[i].at) > STALE_MS;
    snprintf(v, sizeof(v), "%.1f", bar);
    snprintf(t, sizeof(t), "%.0fC", g_w[i].tempC);
    snprintf(id, sizeof(id), "%s", g_w[i].id + 3);
    if (stale)               col = C_DIM;
    else if (bar < LOW_BAR)  col = C_BAD;
    else if (bar < WARN_BAR) col = C_WARN;
    else                     col = C_VAL;
  }

  lcd.setTextDatum(textdatum_t::middle_center);
  lcd.setTextSize(1);
  lcd.setTextColor(C_UNIT, C_BG);
  lcd.drawString(id, cx, cy - 26);

  lcd.setTextSize(4);
  lcd.setTextColor(col, C_BG);
  lcd.drawString(v, cx, cy);

  lcd.setTextSize(1);
  lcd.setTextColor(C_UNIT, C_BG);
  lcd.drawString("bar", cx + 24, cy + 12);
  lcd.setTextColor(0x5AEB, C_BG);
  lcd.drawString(t, cx, cy + 26);
}

/* ---------- 状态层：文字 + 状态灯（这是唯一会闪的地方） ---------- */
static void drawStatus() {
  int low = 0, warn = 0, live = 0;
  for (int i = 0; i < g_n; i++) {
    if (!g_w[i].has) continue;
    if ((millis() - g_w[i].at) > STALE_MS) continue;
    live++;
    float bar = g_w[i].kPa / 100.0f;
    if (bar < LOW_BAR) low++;
    else if (bar < WARN_BAR) warn++;
  }
  const char* txt; uint16_t col;
  if (g_n == 0)       { txt = "SCAN";    col = C_DIM; }
  else if (low)       { txt = "LOW";     col = C_BAD; }
  else if (warn)      { txt = "CHECK";   col = C_WARN; }
  else if (live == 0) { txt = "NO DATA"; col = C_DIM; }
  else                { txt = "ALL OK";  col = C_OK; }

  // 只清状态区（y 24..46，x 64..178，避开外环）
  lcd.fillRect(64, 24, 114, 23, C_BG);

  lcd.setTextDatum(textdatum_t::middle_center);
  lcd.setTextSize(2);
  lcd.setTextColor(col, C_BG);
  lcd.drawString(txt, 108, 35);

  bool working = (g_n > 0 && live > 0 && low == 0 && warn == 0);
  uint16_t dotCol;
  if (working)      dotCol = C_OK;
  else if (g_blink) dotCol = C_BAD;
  else              dotCol = 0x3000;
  lcd.fillCircle(160, 35, 6, dotCol);
  lcd.drawCircle(160, 35, 8, working ? 0x0320 : 0x2000);
  snprintf(g_lastStatus, sizeof(g_lastStatus), "%s", txt);
}

/* ---------- 时效 ---------- */
static void drawAge() {
  uint32_t newest = 0;
  for (int i = 0; i < g_n; i++) if (g_w[i].at > newest) newest = g_w[i].at;
  char age[14];
  if (g_n == 0) snprintf(age, sizeof(age), "waiting");
  else {
    uint32_t s2 = (millis() - newest) / 1000;
    if (s2 < 120) snprintf(age, sizeof(age), "%lus ago", (unsigned long)s2);
    else          snprintf(age, sizeof(age), "%lum ago", (unsigned long)(s2 / 60));
  }
  lcd.fillRect(84, 196, 72, 12, C_BG);
  lcd.setTextDatum(textdatum_t::middle_center);
  lcd.setTextSize(1);
  lcd.setTextColor(C_UNIT, C_BG);
  lcd.drawString(age, 120, 202);
}

/* ---------- 协议 ---------- */
static void handleAdv(const uint8_t* p, size_t n) {
  if (n < 26) return;
  if (!(p[0] == 0x02 && p[1] == 0x01 && p[2] == 0x06 &&
        p[3] == 0x03 && p[4] == 0x03 && p[5] == 0xB0 && p[6] == 0xFB)) return;

  const uint8_t* d = p + 7;
  float volt = (float)d[4] * 0.01f + 1.22f;
  float tempC = (float)d[6] - 55.0f;
  float kPa = (float)d[5] * 3.144f;
  if (d[10] == 1) kPa = ((float)d[5] + 256.0f) * 3.144f;
  uint8_t state = d[7];
  char id[8];
  snprintf(id, sizeof(id), "%02X%02X%02X", d[18], d[17], d[16]);

  g_frames++;
  int slot = -1;
  for (int i = 0; i < g_n; i++) if (strcmp(g_w[i].id, id) == 0) { slot = i; break; }
  if (slot < 0) {
    if (g_n < 4) { slot = g_n++; memset(&g_w[slot], 0, sizeof(W)); }
    else slot = 3;
  }
  W& w = g_w[slot];
  w.has = true;
  snprintf(w.id, sizeof(w.id), "%s", id);
  w.kPa = kPa; w.tempC = tempC; w.volt = volt; w.state = state;
  w.at = millis(); w.cnt++;
  g_dirty = true;
  Serial.printf("[%s] %.2fbar %.0fC %.2fV st=%u n=%u\r\n", id, kPa / 100.0f, tempC, volt, state, w.cnt);
}

class CB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) {
    size_t plen = 0; const uint8_t* pl = nullptr;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    std::string s = dev.getPayload(); plen = s.size(); pl = (const uint8_t*)s.data();
#else
    plen = dev.getPayloadLength(); pl = dev.getPayload();
#endif
    handleAdv(pl, plen);
  }
};

void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(PIN_BOOT, INPUT_PULLUP);
  memset(g_w, 0, sizeof(g_w));
  lcd.init();
  lcd.setBrightness(235);
  drawStatic();
  for (int i = 0; i < 4; i++) drawCell(i);
  drawStatus();
  drawAge();
  Serial.printf("\r\n=== ID.3 TPMS v%s ===\r\n", FW_VERSION);
  BLEDevice::init("id3-tpms");
  BLEScan* sc = BLEDevice::getScan();
  sc->setAdvertisedDeviceCallbacks(new CB(), true);
  sc->setActiveScan(true);
  sc->setInterval(100);
  sc->setWindow(90);
}

void loop() {
  BLEDevice::getScan()->start(1);
  BLEDevice::getScan()->clearResults();

  static bool lastBtn = false;
  bool btn = (digitalRead(PIN_BOOT) == LOW);
  if (btn && !lastBtn) {
    memset(g_w, 0, sizeof(g_w)); g_n = 0; g_frames = 0;
    drawStatic(); for (int i = 0; i < 4; i++) drawCell(i); drawStatus(); drawAge();
    Serial.printf("[清空]\r\n");
  }
  lastBtn = btn;

  // 数据变化 → 只重画变化的单元格
  if (g_dirty) { g_dirty = false; for (int i = 0; i < 4; i++) drawCell(i); }

  // 500ms：只刷新状态文字和状态灯（唯一的动态区域）
  static uint32_t t1 = 0;
  if (millis() - t1 > 500) { t1 = millis(); g_blink = !g_blink; drawStatus(); }

  // 1s：只刷新时效
  static uint32_t t2 = 0;
  if (millis() - t2 > 1000) { t2 = millis(); drawAge(); }

  delay(5);
}
