/*
 * id3-tpms-diag —— 连接诊断版（所有信息都在屏幕上，不需要串口）
 *
 * 关键设计：
 *   1) 用 esp_log_set_vprintf 钩住 ESP 日志，把最后一行错误显示到屏幕
 *      —— Bluedroid 连接失败的真实原因（如 esp_ble_gattc_open failed / 0x85）就在里面
 *   2) "连接风暴"：看到一个广播后，400ms 间隔连试 12 次（传感器广播有间隔，
 *      单次 connect 很容易正好撞在它不响应的窗口里）
 *   3) 屏幕上显示：设备 MAC 尾段 / RSSI / 距今多久见过 / 第几次尝试 / 两种地址类型结果
 *
 * BOOT(GPIO9)：对信号最强的已登记设备发起连接风暴；再按一次停止
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
#include <Preferences.h>
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

#define MAXS 4
static char   g_mac[MAXS][18];
static uint8_t g_at[MAXS];
static int8_t g_rssi[MAXS];
static uint32_t g_seenMs[MAXS];
static uint32_t g_cnt[MAXS];
static uint8_t g_n = 0;

static char   g_logTail[3][40];      // 最近 3 行 ESP 日志（截断）
static volatile uint8_t g_logLine = 0;
static char   g_status[20] = "SCAN";
static uint8_t g_tryN = 0;           // 第几次尝试
static uint8_t g_resA = 0, g_resB = 0;
static bool   g_storm = false;
static uint8_t g_targetIdx = 0;
static char   g_lastHex[90] = {0};
static BLEClient* g_client = nullptr;
static bool   g_connected = false;
static bool   g_autoReq = false;          // 自动连接请求（不需要按键）
static uint32_t g_cooldownUntil = 0;      // 失败后的冷却期
static Preferences g_pref;

/* ---------- ESP 日志钩子 ---------- */
static char g_lineBuf[160];
static uint8_t g_lineLen = 0;
static int logHook(const char* fmt, va_list ap) {
  char tmp[192];
  int r = vsnprintf(tmp, sizeof(tmp), fmt, ap);
  for (int i = 0; tmp[i]; i++) {
    char c = tmp[i];
    if (c == '\n' || c == '\r') {
      if (g_lineLen > 3) {
        g_lineBuf[g_lineLen] = 0;
        // 只保留含 E/W 或多字母内容的行
        strncpy(g_logTail[g_logLine], g_lineBuf, sizeof(g_logTail[0]) - 1);
        g_logTail[g_logLine][sizeof(g_logTail[0]) - 1] = 0;
        g_logLine = (uint8_t)((g_logLine + 1) % 3);
      }
      g_lineLen = 0;
    } else if (g_lineLen < sizeof(g_lineBuf) - 1) {
      g_lineBuf[g_lineLen++] = c;
    }
  }
  return r;
}

/* ---------- 屏幕 ---------- */
static void drawScreen() {
  lcd.fillScreen(TFT_BLACK);
  lcd.drawArc(120, 120, 117, 119, 0, 359, g_connected ? 0x07E0 : (g_storm ? 0xFFE0 : 0xB01FE0));

  lcd.setTextDatum(textdatum_t::top_center);
  lcd.setTextSize(2);
  lcd.setTextColor(0x07FF, TFT_BLACK);
  char head[26];
  snprintf(head, sizeof(head), "%s %u/%d", g_status, g_n, MAXS);
  lcd.drawString(head, 120, 14);

  lcd.setTextSize(1);
  lcd.setTextDatum(textdatum_t::top_left);
  char l1[42];
  if (g_n == 0) {
    lcd.setTextDatum(textdatum_t::top_center);
    lcd.setTextColor(0x8C71, TFT_BLACK);
    lcd.drawString("screw sensor on tire", 120, 100);
  } else {
    uint8_t i = g_targetIdx < g_n ? g_targetIdx : 0;
    uint32_t age = (millis() - g_seenMs[i]) / 100;
    snprintf(l1, sizeof(l1), "..%s %d t%lu.%lus", g_mac[i] + 9, g_rssi[i], (unsigned long)(age / 10), (unsigned long)(age % 10));
    lcd.setTextColor(0xFFFF, TFT_BLACK);
    lcd.drawString(l1, 10, 38);
    snprintf(l1, sizeof(l1), "try %u/12  A%d B%d", g_tryN, g_resA, g_resB);
    lcd.setTextColor(0xFFE0, TFT_BLACK);
    lcd.drawString(l1, 10, 52);
  }

  // 最近日志（诊断核心）
  lcd.setTextColor(0xFD20, TFT_BLACK);
  for (int k = 0; k < 3; k++) {
    uint8_t idx = (uint8_t)((g_logLine + k) % 3);
    if (!g_logTail[idx][0]) continue;
    char buf[40];
    strncpy(buf, g_logTail[idx], 38); buf[38] = 0;
    lcd.drawString(buf, 8, 74 + k * 13);
  }

  if (g_lastHex[0]) {
    lcd.setTextColor(0x07E0, TFT_BLACK);
    char b1[40], b2[40];
    strncpy(b1, g_lastHex, 36); b1[36] = 0;
    strncpy(b2, g_lastHex + 36, 36); b2[36] = 0;
    lcd.drawString(b1, 8, 128);
    if (b2[0]) lcd.drawString(b2, 8, 142);
  }

  lcd.setTextDatum(textdatum_t::top_center);
  lcd.setTextColor(0xFD20, TFT_BLACK);
  lcd.drawString(g_storm ? "BOOT = stop" : "BOOT = connect", 120, 200);
}

/* ---------- 数据 ---------- */
static void pushHex(const uint8_t* d, size_t n) {
  size_t k = 0; g_lastHex[0] = 0;
  for (size_t i = 0; i < n && i < 28 && k + 4 < sizeof(g_lastHex); i++)
    k += snprintf(g_lastHex + k, sizeof(g_lastHex) - k, "%02X", d[i]);
  Serial.printf("[RX] %s\r\n", g_lastHex);
}
static void notifyCB(BLERemoteCharacteristic* ch, uint8_t* d, size_t n, bool) { pushHex(d, n); drawScreen(); }

/* ---------- 连接风暴 ---------- */
static void stormConnect(uint8_t idx) {
  if (idx >= g_n) return;
  g_storm = true; g_tryN = 0; g_resA = 0; g_resB = 0;
  snprintf(g_status, sizeof(g_status), "TRY");
  drawScreen();

  BLEScan* scan = BLEDevice::getScan();
  scan->stop(); scan->clearResults();
  delay(400);

  if (!g_client) g_client = BLEDevice::createClient();

  for (g_tryN = 1; g_tryN <= 12; g_tryN++) {
    drawScreen();
    // 每轮先确认它还在广播（不超过 5 秒）
    bool live = (millis() - g_seenMs[idx]) < 5000;
    esp_ble_addr_type_t at = (esp_ble_addr_type_t)g_at[idx];
    Serial.printf("[try %u] %s live=%d at=%d\r\n", g_tryN, g_mac[idx], (int)live, (int)at);
    bool ok = false;
    if (live) {
      ok = g_client->connect(BLEAddress(g_mac[idx]), at);
      g_resA = ok ? 1 : 0;
      if (!ok) {
        esp_ble_addr_type_t at2 = (at == BLE_ADDR_TYPE_RANDOM) ? BLE_ADDR_TYPE_PUBLIC : BLE_ADDR_TYPE_RANDOM;
        ok = g_client->connect(BLEAddress(g_mac[idx]), at2);
        g_resB = ok ? 1 : 0;
      }
      if (ok) break;
    }
    delay(400);
    // 期间继续吃一下广播，刷新 seen
    BLEDevice::getScan()->start(1);
    BLEDevice::getScan()->clearResults();
    BLEDevice::getScan()->stop();
  }

  if (!g_client->isConnected()) {
    snprintf(g_status, sizeof(g_status), "FAIL");
    g_storm = false;
    Serial.printf("[失败] 12 次都没连上\r\n");
    drawScreen();
    delay(1500);
    snprintf(g_status, sizeof(g_status), "SCAN");
    drawScreen();
    return;
  }

  g_connected = true;
  snprintf(g_status, sizeof(g_status), "OK");
  drawScreen();
  Serial.printf("[已连接] 枚举服务\r\n");

  std::map<std::string, BLERemoteService*>* svcs = g_client->getServices();
  if (svcs) {
    for (auto& kv : *svcs) {
      BLERemoteService* svc = kv.second;
      Serial.printf("服务 %s\r\n", svc->getUUID().toString().c_str());
      std::map<std::string, BLERemoteCharacteristic*>* chrs = svc->getCharacteristics();
      if (!chrs) continue;
      for (auto& c2 : *chrs) {
        BLERemoteCharacteristic* ch = c2.second;
        Serial.printf("  特征 %s R=%d W=%d N=%d\r\n", ch->getUUID().toString().c_str(), ch->canRead(), ch->canWrite(), ch->canNotify());
        if (ch->canNotify() || ch->canIndicate()) ch->registerForNotify(notifyCB);
        if (ch->canRead()) { std::string v = ch->readValue(); if (v.size()) { pushHex((const uint8_t*)v.data(), v.size()); } }
      }
    }
  }
  drawScreen();
}

/* ---------- 广播 ---------- */
static bool advHasFBB0(const uint8_t* p, size_t n) {
  size_t i = 0;
  while (i + 1 < n) {
    uint8_t len = p[i];
    if (len == 0 || i + 1 + len > n) break;
    uint8_t t = p[i + 1];
    const uint8_t* d = p + i + 2;
    uint8_t dl = len - 1;
    if ((t == 0x02 || t == 0x03) && dl >= 2) { for (uint8_t k = 0; k + 1 < dl; k += 2) if (d[k] == 0xB0 && d[k + 1] == 0xFB) return true; }
    else if ((t == 0x06 || t == 0x07) && dl >= 16) { for (uint8_t k = 0; k + 1 + 16 <= dl; k += 16) if (d[k] == 0xB0 && d[k + 1] == 0xFB) return true; }
    else if (t == 0x16 && dl >= 2) { if (d[0] == 0xB0 && d[1] == 0xFB) return true; }
    i += (size_t)len + 1;
  }
  return false;
}

class AdvCB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) {
    if (g_connected || g_storm) return;
    size_t plen = 0; const uint8_t* pl = nullptr;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    std::string s = dev.getPayload(); plen = s.size(); pl = (const uint8_t*)s.data();
#else
    plen = dev.getPayloadLength(); pl = dev.getPayload();
#endif
    if (!advHasFBB0(pl, plen)) return;
    char mac[18];
    snprintf(mac, sizeof(mac), "%s", dev.getAddress().toString().c_str());
    for (char* q = mac; *q; q++) *q = tolower(*q);
    for (int i = 0; i < g_n; i++) {
      if (strcmp(g_mac[i], mac) == 0) {
        bool wasQuiet = (millis() - g_seenMs[i]) > 10000;
        g_rssi[i] = (int8_t)dev.getRSSI(); g_seenMs[i] = millis(); g_cnt[i]++;
        if (wasQuiet && !g_connected && !g_storm) g_autoReq = true;   // 重新活跃 → 再自动连一次
        return;
      }
    }
    if (g_n >= MAXS) return;
    strncpy(g_mac[g_n], mac, sizeof(g_mac[0]) - 1);
    g_at[g_n] = (uint8_t)dev.getAddressType();
    g_rssi[g_n] = (int8_t)dev.getRSSI();
    g_seenMs[g_n] = millis();
    g_cnt[g_n] = 1;
    Serial.printf("[登记 %u] %s at=%d rssi=%d\r\n", g_n + 1, mac, (int)g_at[g_n], (int)g_rssi[g_n]);
    g_n++;
    g_pref.putString("mac", mac);
    g_autoReq = true;                    // 一出现就自动连，不用按键
    drawScreen();
  }
};

void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(PIN_BOOT, INPUT_PULLUP);
  lcd.init();
  lcd.setBrightness(255);
  g_pref.begin("tpms", false);
  esp_log_set_vprintf(&logHook);      // 抓 ESP 日志显示到屏幕
  memset(g_mac, 0, sizeof(g_mac));
  drawScreen();

  BLEDevice::init("tpms-diag");
  BLEScan* scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new AdvCB(), true);
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(90);
  snprintf(g_status, sizeof(g_status), "SCAN");
  drawScreen();
}

void loop() {
  static bool lastBtn = false;
  bool btn = (digitalRead(PIN_BOOT) == LOW);
  if (btn && !lastBtn) {
    if (g_storm || g_connected) { g_storm = false; if (g_client) { g_client->disconnect(); } g_connected = false; snprintf(g_status, sizeof(g_status), "SCAN"); drawScreen(); }
    else if (g_n > 0) {
      int best = 0;
      for (int i = 1; i < g_n; i++) if (g_rssi[i] > g_rssi[best]) best = i;
      g_targetIdx = (uint8_t)best;
      stormConnect((uint8_t)best);
    }
  }
  lastBtn = btn;

  if (g_autoReq && !g_storm && !g_connected && millis() > g_cooldownUntil && g_n > 0) {
    g_autoReq = false;
    int best = 0;
    for (int i = 1; i < g_n; i++) if (g_rssi[i] > g_rssi[best]) best = i;
    g_targetIdx = (uint8_t)best;
    stormConnect((uint8_t)best);
    g_cooldownUntil = millis() + 12000;    // 失败后 12 秒再来下一轮
  }

  if (!g_storm && !g_connected) {
    BLEDevice::getScan()->start(1);
    BLEDevice::getScan()->clearResults();
    static uint32_t t = 0;
    if (millis() - t > 700) { t = millis(); drawScreen(); }
  }
  delay(10);
}
