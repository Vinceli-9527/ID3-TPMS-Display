/*
 * id3-tpms-find —— 不依赖任何协议假设的定位 + 连接诊断
 *
 * 放弃 FBB0 过滤：列出所有设备，按信号强度排序（板子贴着传感器时它会是第一名）
 * 屏幕同时显示最近 6 条 ESP 日志（清空后只抓连接期间产生的），用来定位 connect 失败原因
 *
 * 操作：把板子贴到传感器上 → 屏幕第一行 RSSI 会明显变强（>-55）→
 *       自动发起连接；也可按 BOOT 手动对第一名发起连接。
 */

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEClient.h>
#include <BLEAdvertisedDevice.h>
#include <BLERemoteService.h>
#include <BLERemoteCharacteristic.h>
#include <esp_log.h>
#include <stdarg.h>

#define PIN_LCD_SCLK 6
#define PIN_LCD_MOSI 7
#define PIN_LCD_DC   2
#define PIN_LCD_CS   10
#define PIN_LCD_BL   3
#define PIN_BOOT     9

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

#define MAXD 16
struct D { char mac[18]; int8_t rssi; uint32_t cnt; uint32_t seen; uint8_t at; bool used; };
static D g_d[MAXD];
static uint8_t g_nd = 0;

#define LOGLINES 6
static char g_log[LOGLINES][39];
static volatile uint8_t g_logPos = 0;
static bool g_logEnabled = true;
static char g_lineBuf[160];
static uint8_t g_lineLen = 0;

static char g_status[20] = "SCAN";
static char g_lastHex[80] = {0};
static bool g_connected = false;
static BLEClient* g_client = nullptr;
static uint32_t g_cooldown = 0;
static uint8_t  g_tries = 0;
static uint8_t  g_resA = 0, g_resB = 0;
#define MAXG 10
static char    g_gUuid[MAXG][12];
static uint8_t g_gProp[MAXG];
static uint8_t g_gN = 0;
static uint8_t g_svcN = 0;

static int logHook(const char* fmt, va_list ap) {
  char tmp[192];
  int r = vsnprintf(tmp, sizeof(tmp), fmt, ap);
  if (!g_logEnabled) return r;
  for (int i = 0; tmp[i]; i++) {
    char c = tmp[i];
    if (c == '\n' || c == '\r') {
      if (g_lineLen > 4) {
        g_lineBuf[g_lineLen] = 0;
        strncpy(g_log[g_logPos], g_lineBuf, sizeof(g_log[0]) - 1);
        g_log[g_logPos][sizeof(g_log[0]) - 1] = 0;
        g_logPos = (uint8_t)((g_logPos + 1) % LOGLINES);
      }
      g_lineLen = 0;
    } else if (g_lineLen < sizeof(g_lineBuf) - 1) {
      g_lineBuf[g_lineLen++] = c;
    }
  }
  return r;
}

static void clearLog() { for (int i = 0; i < LOGLINES; i++) g_log[i][0] = 0; g_logPos = 0; g_lineLen = 0; }

static int topIdx(int rank) {                 // rank 0 = 最强
  int used[MAXD] = {0};
  for (int r = 0; r <= rank; r++) {
    int best = -1; int bestR = -200;
    for (int i = 0; i < MAXD; i++) {
      if (!g_d[i].used || used[i]) continue;
      if (g_d[i].rssi > bestR) { bestR = g_d[i].rssi; best = i; }
    }
    if (best < 0) return -1;
    used[best] = 1;
    if (r == rank) return best;
  }
  return -1;
}

// 圆内可用宽度：给定行中心 y，算出这行最多能放多少字符（字体 size1 每字符 6px）
static int fitChars(int yCenter, int margin) {
  const int R = 114;                 // 安全半径（比外环 119 再留 5px）
  int dy = yCenter + 3 - 120;        // 以字形中心估算
  int r2 = R * R - dy * dy;
  if (r2 <= 0) return 0;
  int half = (int)sqrt((double)r2);
  int w = half * 2 - margin * 2;
  return w > 0 ? w / 6 : 0;
}

// 居中 + 按该行圆内宽度裁剪
static void drawFitted(const char* s, int yCenter, uint16_t fg, uint8_t size = 1) {
  int perChar = 6 * size;
  const int R = 114;
  int dy = yCenter + 3 - 120;
  int r2 = R * R - dy * dy;
  if (r2 <= 0) return;
  int half = (int)sqrt((double)r2);
  int maxChars = (half * 2 - 8) / perChar;
  if (maxChars <= 0) return;
  char buf[64];
  size_t n = strlen(s);
  if ((int)n > maxChars) n = maxChars;
  if (n >= sizeof(buf)) n = sizeof(buf) - 1;
  memcpy(buf, s, n);
  buf[n] = 0;
  lcd.setTextDatum(textdatum_t::middle_center);
  lcd.setTextSize(size);
  lcd.setTextColor(fg, TFT_BLACK);
  lcd.drawString(buf, 120, yCenter);
}

static void drawScreen() {
  lcd.fillScreen(TFT_BLACK);
  lcd.drawArc(120, 120, 119, 119, 0, 359, g_connected ? 0x07E0 : 0xB01FE0);

  char h[24];
  if (g_connected) snprintf(h, sizeof(h), "OK svc%u ch%u", g_svcN, g_gN);
  else             snprintf(h, sizeof(h), "%s %u", g_status, g_nd);
  drawFitted(h, 24, 0x07FF, 2);

  if (g_connected) {
    for (int i = 0; i < g_gN && i < 6; i++) {
      char line[40];
      snprintf(line, sizeof(line), "%d %s %c%c%c", i + 1, g_gUuid[i],
               (g_gProp[i] & 0x02) ? 'R' : '-',
               (g_gProp[i] & 0x08) ? 'W' : '-',
               (g_gProp[i] & 0x10) ? 'N' : '-');
      drawFitted(line, 46 + i * 15, (g_gProp[i] & 0x10) ? 0x07E0 : 0xFFFF);
    }
    if (g_gN == 0) drawFitted("no service found", 70, 0xF800);
    if (g_lastHex[0]) { drawFitted(g_lastHex, 152, 0xFFE0); drawFitted(g_lastHex + 26, 166, 0xFFE0); }
    else drawFitted("waiting data...", 152, 0xFD20);
    drawFitted("BOOT = disconnect", 212, 0xFD20);
    return;
  }

  // 最强 4 个设备
  for (int r = 0; r < 4; r++) {
    int i = topIdx(r);
    if (i < 0) break;
    char line[40];
    snprintf(line, sizeof(line), "%d %s %d c%lu", r + 1, g_d[i].mac, g_d[i].rssi, (unsigned long)g_d[i].cnt);
    drawFitted(line, 46 + r * 15, r == 0 ? 0x07E0 : 0xFFFF);
  }

  char l2[40];
  snprintf(l2, sizeof(l2), "try %u A%d B%d", g_tries, g_resA, g_resB);
  drawFitted(l2, 108, 0xFFE0);

  // 日志（5 行，越靠下越窄，自动裁剪）
  lcd.drawFastHLine(40, 118, 160, 0x4208);
  for (int k = 0; k < 5; k++) {
    uint8_t idx = (uint8_t)((g_logPos + k) % LOGLINES);
    if (!g_log[idx][0]) continue;
    drawFitted(g_log[idx], 130 + k * 13, 0xFD20);
  }

  if (g_lastHex[0]) drawFitted(g_lastHex, 198, 0x07E0);
  drawFitted("BOOT = connect #1", 212, 0xFD20);
}

static void pushHex(const uint8_t* d, size_t n) {
  size_t k = 0; g_lastHex[0] = 0;
  for (size_t i = 0; i < n && i < 26 && k + 4 < sizeof(g_lastHex); i++)
    k += snprintf(g_lastHex + k, sizeof(g_lastHex) - k, "%02X", d[i]);
  Serial.printf("[RX] %s\r\n", g_lastHex);
}
static void notifyCB(BLERemoteCharacteristic* ch, uint8_t* d, size_t n, bool) { pushHex(d, n); drawScreen(); }

static void doConnect(int idx) {
  if (idx < 0) return;
  snprintf(g_status, sizeof(g_status), "CONN");
  g_tries = 0; g_resA = 0; g_resB = 0;
  drawScreen();

  BLEDevice::getScan()->stop();
  BLEDevice::getScan()->clearResults();
  delay(800);

  if (!g_client) g_client = BLEDevice::createClient();
  if (g_client->isConnected()) g_client->disconnect();

  clearLog();                       // 日志只保留连接期间产生的
  snprintf(g_status, sizeof(g_status), "TRY");
  drawScreen();

  for (g_tries = 1; g_tries <= 8; g_tries++) {
    drawScreen();
    bool live = (millis() - g_d[idx].seen) < 4000;
    Serial.printf("[try %u] %s live=%d\r\n", g_tries, g_d[idx].mac, (int)live);
    if (!live) { delay(300); continue; }

    // 先按广播报告的地址类型
    bool ok = g_client->connect(BLEAddress(g_d[idx].mac), (esp_ble_addr_type_t)g_d[idx].at);
    g_resA = ok ? 1 : 0;
    if (!ok) {
      esp_ble_addr_type_t alt = (g_d[idx].at == BLE_ADDR_TYPE_RANDOM) ? BLE_ADDR_TYPE_PUBLIC : BLE_ADDR_TYPE_RANDOM;
      ok = g_client->connect(BLEAddress(g_d[idx].mac), alt);
      g_resB = ok ? 1 : 0;
    }
    if (ok) break;
    delay(300);
  }

  if (!g_client->isConnected()) {
    snprintf(g_status, sizeof(g_status), "FAIL");
    drawScreen();
    Serial.printf("[失败]\r\n");
    delay(2500);
    snprintf(g_status, sizeof(g_status), "SCAN");
    g_cooldown = millis() + 8000;
    drawScreen();
    return;
  }

  g_connected = true;
  snprintf(g_status, sizeof(g_status), "OK");
  drawScreen();
  Serial.printf("[已连接]\r\n");

  g_svcN = 0; g_gN = 0;
  std::map<std::string, BLERemoteService*>* svcs = g_client->getServices();
  if (svcs) {
    for (auto& kv : *svcs) {
      BLERemoteService* svc = kv.second;
      g_svcN++;
      Serial.printf("服务 %s\r\n", svc->getUUID().toString().c_str());
      auto* chrs = svc->getCharacteristics();
      if (!chrs) continue;
      for (auto& c2 : *chrs) {
        BLERemoteCharacteristic* ch = c2.second;
        if (g_gN < MAXG) {
          String u = ch->getUUID().toString().c_str();
          if (u.length() >= 8 && u.startsWith("0000")) snprintf(g_gUuid[g_gN], sizeof(g_gUuid[0]), "%s", u.substring(4, 8).c_str());
          else snprintf(g_gUuid[g_gN], sizeof(g_gUuid[0]), "%s", u.substring(0, 8).c_str());
          g_gProp[g_gN] = (ch->canRead() ? 0x02 : 0) | (ch->canWrite() ? 0x08 : 0) | (ch->canNotify() ? 0x10 : 0);
          g_gN++;
        }
        Serial.printf("  特征 %s R=%d W=%d N=%d\r\n", ch->getUUID().toString().c_str(), ch->canRead(), ch->canWrite(), ch->canNotify());
        if (ch->canNotify() || ch->canIndicate()) ch->registerForNotify(notifyCB);
        if (ch->canRead()) { std::string v = ch->readValue(); if (v.size()) pushHex((const uint8_t*)v.data(), v.size()); }
      }
    }
  }
  drawScreen();
}

class AdvCB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) {
    if (g_connected) return;
    char mac[18];
    snprintf(mac, sizeof(mac), "%s", dev.getAddress().toString().c_str());
    for (char* q = mac; *q; q++) *q = tolower(*q);
    for (int i = 0; i < MAXD; i++) {
      if (g_d[i].used && strcmp(g_d[i].mac, mac) == 0) {
        g_d[i].rssi = (int8_t)dev.getRSSI();
        g_d[i].cnt++;
        g_d[i].seen = millis();
        return;
      }
    }
    for (int i = 0; i < MAXD; i++) {
      if (!g_d[i].used) {
        memset(&g_d[i], 0, sizeof(D));
        g_d[i].used = true;
        strncpy(g_d[i].mac, mac, sizeof(g_d[i].mac) - 1);
        g_d[i].rssi = (int8_t)dev.getRSSI();
        g_d[i].cnt = 1;
        g_d[i].seen = millis();
        g_d[i].at = (uint8_t)dev.getAddressType();
        g_nd++;
        return;
      }
    }
  }
};

void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(PIN_BOOT, INPUT_PULLUP);
  lcd.init();
  lcd.setBrightness(255);
  memset(g_d, 0, sizeof(g_d));
  esp_log_set_vprintf(&logHook);
  clearLog();
  drawScreen();

  BLEDevice::init("tpms-find");
  BLEScan* scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new AdvCB(), true);
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(90);
  Serial.printf("\r\n=== find（无协议假设）===\r\n");
}

void loop() {
  static bool lastBtn = false;
  bool btn = (digitalRead(PIN_BOOT) == LOW);
  if (btn && !lastBtn) {
    if (g_connected) { g_client->disconnect(); g_connected = false; snprintf(g_status, sizeof(g_status), "SCAN"); }
    else doConnect(topIdx(0));
  }
  lastBtn = btn;

  if (!g_connected) {
    BLEDevice::getScan()->start(1);
    BLEDevice::getScan()->clearResults();

    // 贴到传感器上时 RSSI 会突然变强 → 自动尝试
    int t0 = topIdx(0);
    if (t0 >= 0 && g_d[t0].rssi > -55 && millis() > g_cooldown && g_status[0] != 'T') {
      doConnect(t0);
    }

    static uint32_t last = 0;
    if (millis() - last > 600) { last = millis(); drawScreen(); }
  }
  delay(10);
}