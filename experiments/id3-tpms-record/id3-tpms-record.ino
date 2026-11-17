/*
 * id3-tpms-record v3
 *
 * v2 的两个致命问题：
 *   ① 一发现就连接 → 连接期间扫描停止，第二颗传感器登记不上，屏幕还被连接视图顶掉
 *   ② connect(mac) 默认按 public 地址连；而这类传感器的 MAC 首字节高位常是 11
 *      （static random address）→ 地址类型不对，永远连不上
 *
 * v3 策略：
 *   RECORD 模式（默认）：只扫描登记，绝不连接 → 四颗可以逐个登记
 *   PROBE  模式：按 BOOT（GPIO9）或空闲 120 秒后自动进入，逐个连接已登记传感器
 *   连接时按 MAC 首个字节自动选择 public / random 地址类型
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

#define MAX_SENS 6
#define FRAMES   3
#define HEXLINE  36

static char     g_mac[MAX_SENS][18];
static uint8_t  g_addrType[MAX_SENS];
static int8_t   g_rssi[MAX_SENS];
static uint32_t g_seen[MAX_SENS];
static uint32_t g_lastSeen[MAX_SENS];   // 用于 PROBE 前判断"它现在是否在广播"
static BLEAdvertisedDevice g_devCopy[MAX_SENS];  // 保留设备对象：connect(&dev) 才能带对地址类型
static uint8_t  g_n = 0;

static bool     g_probeMode = false;
static char     g_status[20] = "BOOT";
static char     g_frames[FRAMES][HEXLINE * 2 + 4];
static uint8_t  g_frameIdx = 0;
static uint16_t g_rxCount = 0;
static uint8_t  g_svcN = 0, g_chrN = 0;
static bool     g_connected = false;
static char     g_peer[18] = {0};
static uint32_t g_connAt = 0;
static int      g_tryIdx = -1;
static uint32_t g_lastChangeMs = 0;
static bool     g_connectReq = false;
static char     g_pendingMac[18] = {0};
static uint8_t  g_pendingType = 0;
static Preferences g_pref;
static BLEClient* g_client = nullptr;

/* ---------- NVS ---------- */
static void saveList() {
  String s;
  for (uint8_t i = 0; i < g_n; i++) { if (i) s += ","; s += g_mac[i]; }
  g_pref.putString("macs", s);
}
static void loadList() {
  String s = g_pref.getString("macs", "");
  g_n = 0; int start = 0;
  for (int i = 0; i <= (int)s.length() && g_n < MAX_SENS; i++) {
    if (i == (int)s.length() || s[i] == ',') {
      String one = s.substring(start, i); one.trim(); one.toLowerCase();
      if (one.length() >= 17) {
        strncpy(g_mac[g_n], one.c_str(), sizeof(g_mac[0]) - 1);
        int b0 = (int)strtol(one.c_str(), nullptr, 16);
        g_addrType[g_n] = ((b0 & 0xC0) == 0xC0 || (b0 & 0xC0) == 0x40 || (b0 & 0xC0) == 0x00) ? BLE_ADDR_TYPE_RANDOM : BLE_ADDR_TYPE_PUBLIC;
        g_rssi[g_n] = -127; g_n++;
      }
      start = i + 1;
    }
  }
}
static int findMac(const char* mac) {
  for (int i = 0; i < g_n; i++) if (strcasecmp(g_mac[i], mac) == 0) return i;
  return -1;
}

/* ---------- 屏幕 ---------- */
static void drawScreen() {
  lcd.fillScreen(TFT_BLACK);
  lcd.drawArc(120, 120, 117, 119, 0, 359, g_connected ? 0x07E0 : (g_probeMode ? 0xFFE0 : 0xB01FE0));

  lcd.setTextDatum(textdatum_t::top_center);
  lcd.setTextSize(2);
  lcd.setTextColor(0x07FF, TFT_BLACK);
  char head[24];
  if (g_connected) snprintf(head, sizeof(head), "%s", g_status);
  else             snprintf(head, sizeof(head), "%s %u/4", g_probeMode ? "PROBE" : "SCAN", g_n);
  lcd.drawString(head, 120, 18);

  lcd.setTextSize(1);
  lcd.setTextDatum(textdatum_t::top_left);

  if (!g_connected) {
    int y = 44;
    for (int i = 0; i < g_n && i < 4; i++) {
      char line[34];
      if (g_rssi[i] > -127) snprintf(line, sizeof(line), "%d %s %d", i + 1, g_mac[i], g_rssi[i]);
      else                  snprintf(line, sizeof(line), "%d %s --", i + 1, g_mac[i]);
      lcd.setTextColor(g_rssi[i] > -127 ? 0xFFFF : 0x5AEB, TFT_BLACK);
      lcd.drawString(line, 12, y);
      y += 13;
    }
    if (g_n == 0) {
      lcd.setTextDatum(textdatum_t::top_center);
      lcd.setTextColor(0x8C71, TFT_BLACK);
      lcd.drawString("screw sensor on tire", 120, 90);
    }
    lcd.setTextDatum(textdatum_t::top_center);
    lcd.setTextColor(0xFD20, TFT_BLACK);
    if (g_probeMode) {
      char h[26]; snprintf(h, sizeof(h), "PROBE #%d  boot=cancel", g_tryIdx + 1);
      lcd.drawString(h, 120, 190);
    } else {
      lcd.drawString("boot = probe", 120, 190);
    }
  } else {
    lcd.setTextColor(0x8C71, TFT_BLACK);
    lcd.drawString(g_peer, 14, 42);
    char line[34];
    snprintf(line, sizeof(line), "svc %u chr %u rx %u", g_svcN, g_chrN, g_rxCount);
    lcd.setTextColor(0xFFE0, TFT_BLACK);
    lcd.drawString(line, 14, 56);
    lcd.drawFastHLine(24, 72, 192, 0x4208);
    for (int k = 0; k < FRAMES; k++) {
      int idx = (g_frameIdx + k) % FRAMES;
      if (!g_frames[idx][0]) continue;
      char b1[HEXLINE + 1], b2[HEXLINE + 1];
      for (int i = 0; i < HEXLINE; i++) { b1[i] = g_frames[idx][i]; b2[i] = g_frames[idx][HEXLINE + i]; }
      b1[HEXLINE] = 0; b2[HEXLINE] = 0;
      lcd.setTextColor(0xFFFF, TFT_BLACK);
      lcd.drawString(b1, 10, 80 + k * 26);
      if (b2[0]) lcd.drawString(b2, 10, 93 + k * 26);
    }
    if (g_rxCount == 0) {
      lcd.setTextDatum(textdatum_t::top_center);
      lcd.setTextColor(0xFD20, TFT_BLACK);
      lcd.drawString("waiting data...", 120, 150);
    }
  }
}

/* ---------- 数据 ---------- */
static void pushFrame(const uint8_t* data, size_t len) {
  size_t n = len > 36 ? 36 : len;
  char hex[HEXLINE * 2 + 4]; size_t k = 0; hex[0] = 0;
  for (size_t i = 0; i < n; i++) k += snprintf(hex + k, sizeof(hex) - k, "%02X", data[i]);
  snprintf(g_frames[g_frameIdx], sizeof(g_frames[g_frameIdx]), "%s", hex);
  g_frameIdx = (g_frameIdx + 1) % FRAMES;
  g_rxCount++;
  Serial.printf("[RX] len=%u %s\r\n", (unsigned)len, hex);
  drawScreen();
}
static void notifyCB(BLERemoteCharacteristic* ch, uint8_t* data, size_t len, bool isNotify) { pushFrame(data, len); }

static void disconnectNow() {
  if (g_client) { g_client->disconnect(); delete g_client; g_client = nullptr; }
  g_connected = false;
  snprintf(g_status, sizeof(g_status), "SCAN");
  drawScreen();
}

/* ---------- 连接（关键：地址类型）---------- */
static void doConnect(uint8_t idx) {
  if (idx >= g_n) return;
  snprintf(g_status, sizeof(g_status), "CONNECT #%u", idx + 1);
  snprintf(g_peer, sizeof(g_peer), "%s", g_mac[idx]);
  drawScreen();

  BLEScan* scan = BLEDevice::getScan();
  scan->stop();
  scan->clearResults();
  delay(600);                       // 停扫要留足时间，否则 connect 会立刻失败

  if (!g_client) g_client = BLEDevice::createClient();

  bool ok = false;
  // 方式 A：用广播设备对象连接（地址类型取自广播报告，最可靠）
  if (g_lastSeen[idx] != 0 && millis() - g_lastSeen[idx] < 10000) {
    Serial.printf("[连接A #%u] %s via device object\r\n", idx + 1, g_mac[idx]);
    ok = g_client->connect(&g_devCopy[idx]);
    Serial.printf("   结果=%d\r\n", (int)ok);
  } else {
    Serial.printf("[跳过A #%u] 最近 10 秒没见到它在广播\r\n", idx + 1);
  }

  // 方式 B：按地址 + 两种地址类型各试一次
  if (!ok) {
    esp_ble_addr_type_t at = (esp_ble_addr_type_t)g_addrType[idx];
    Serial.printf("[连接B #%u] %s addrType=%d\r\n", idx + 1, g_mac[idx], (int)at);
    ok = g_client->connect(BLEAddress(g_mac[idx]), at);
    Serial.printf("   结果=%d\r\n", (int)ok);
    if (!ok) {
      esp_ble_addr_type_t at2 = (at == BLE_ADDR_TYPE_RANDOM) ? BLE_ADDR_TYPE_PUBLIC : BLE_ADDR_TYPE_RANDOM;
      Serial.printf("[连接B2 #%u] addrType=%d\r\n", idx + 1, (int)at2);
      ok = g_client->connect(BLEAddress(g_mac[idx]), at2);
      Serial.printf("   结果=%d\r\n", (int)ok);
    }
  }

  if (!ok) {
    snprintf(g_status, sizeof(g_status), "FAIL #%u", idx + 1);
    drawScreen();
    g_client->disconnect();
    g_lastChangeMs = millis();
    delay(500);
    return;
  }

  g_connected = true; g_connAt = millis(); g_rxCount = 0;
  snprintf(g_status, sizeof(g_status), "OK #%u", idx + 1);
  drawScreen();
  Serial.printf("[已连接] %s\r\n", g_mac[idx]);

  g_svcN = 0; g_chrN = 0;
  std::map<std::string, BLERemoteService*>* svcs = g_client->getServices();
  if (svcs) {
    for (auto& kv : *svcs) {
      BLERemoteService* svc = kv.second; g_svcN++;
      Serial.printf("服务 %s\r\n", svc->getUUID().toString().c_str());
      std::map<std::string, BLERemoteCharacteristic*>* chrs = svc->getCharacteristics();
      if (!chrs) continue;
      for (auto& c2 : *chrs) {
        BLERemoteCharacteristic* ch = c2.second; g_chrN++;
        Serial.printf("  特征 %s R=%d W=%d N=%d I=%d\r\n", ch->getUUID().toString().c_str(),
                      ch->canRead(), ch->canWrite(), ch->canNotify(), ch->canIndicate());
        if (ch->canNotify() || ch->canIndicate()) ch->registerForNotify(notifyCB);
        if (ch->canRead()) {
          std::string v = ch->readValue();
          if (v.size()) pushFrame((const uint8_t*)v.data(), v.size());
        }
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
    uint8_t type = p[i + 1];
    const uint8_t* d = p + i + 2;
    uint8_t dlen = len - 1;
    if ((type == 0x02 || type == 0x03) && dlen >= 2) {
      for (uint8_t k = 0; k + 1 < dlen; k += 2) if (d[k] == 0xB0 && d[k + 1] == 0xFB) return true;
    } else if ((type == 0x06 || type == 0x07) && dlen >= 16) {
      for (uint8_t k = 0; k + 1 + 16 <= dlen; k += 16) if (d[k] == 0xB0 && d[k + 1] == 0xFB) return true;
    } else if (type == 0x16 && dlen >= 2) {
      if (d[0] == 0xB0 && d[1] == 0xFB) return true;
    }
    i += (size_t)len + 1;
  }
  return false;
}

class AdvCB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) {
    if (g_connected) return;                 // 连接期间不处理
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

    int i = findMac(mac);
    if (i < 0) {
      if (g_n >= MAX_SENS) return;
      strncpy(g_mac[g_n], mac, sizeof(g_mac[0]) - 1);
      g_addrType[g_n] = (uint8_t)dev.getAddressType();   // core 2.0.5：用 BLEAdvertisedDevice 的接口
      g_rssi[g_n] = (int8_t)dev.getRSSI();
      g_seen[g_n] = 1;
      g_lastSeen[g_n] = millis();
      g_devCopy[g_n] = dev;
      g_n++;
      saveList();
      char hex[100]; size_t hk = 0; hex[0] = 0;
      for (size_t b = 0; b < plen && b < 30 && hk + 4 < sizeof(hex); b++) hk += snprintf(hex + hk, sizeof(hex) - hk, "%02X", pl[b]);
      Serial.printf("[登记 %u] %s addrType=%d rssi=%d adv=%s\r\n", g_n, mac, (int)g_addrType[g_n-1], (int)dev.getRSSI(), hex);
      g_lastChangeMs = millis();
      drawScreen();
    } else {
      g_rssi[i] = (int8_t)dev.getRSSI();
      g_seen[i]++;
      g_lastSeen[i] = millis();
      g_devCopy[i] = dev;
    }
  }
};

/* ---------- 串口 ---------- */
static char    g_line[48];
static uint8_t g_len = 0;

static void pollSerial() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch != '\n') { if (g_len < sizeof(g_line) - 1) g_line[g_len++] = ch; continue; }
    g_line[g_len] = 0; g_len = 0;
    String cmd(g_line); cmd.trim();
    if (cmd.length() == 0) continue;
    if (cmd[0] == 'l') {
      Serial.printf("登记 %u：\r\n", g_n);
      for (uint8_t i = 0; i < g_n; i++)
        Serial.printf("  %u %s type=%d rssi=%d seen=%lu\r\n", i + 1, g_mac[i], g_addrType[i], g_rssi[i], (unsigned long)g_seen[i]);
      Serial.printf("probeMode=%d connected=%d\r\n", (int)g_probeMode, (int)g_connected);
    } else if (cmd[0] == 'C') { g_n = 0; saveList(); drawScreen(); Serial.printf("已清空\r\n"); }
    else if (cmd[0] == 'p') { g_probeMode = true; g_tryIdx = -1; drawScreen(); Serial.printf("进入 PROBE\r\n"); }
    else if (cmd[0] == 'r') { g_probeMode = false; disconnectNow(); Serial.printf("回到 RECORD\r\n"); }
    else if (cmd[0] == 'd') disconnectNow();
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(PIN_BOOT, INPUT_PULLUP);
  lcd.init();
  lcd.setBrightness(255);
  g_pref.begin("tpms", false);
  loadList();
  drawScreen();
  Serial.printf("\r\n=== record v3 ===\r\n已登记 %u\r\n", g_n);

  BLEDevice::init("tpms-record");
  BLEScan* scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new AdvCB(), true);
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(90);
  g_lastChangeMs = millis();
  snprintf(g_status, sizeof(g_status), "SCAN");
  drawScreen();
}

void loop() {
  pollSerial();

  // BOOT(IO9)：切换 RECORD / PROBE
  static bool lastBtn = false;
  bool btn = (digitalRead(PIN_BOOT) == LOW);
  if (btn && !lastBtn) {
    if (g_probeMode) { g_probeMode = false; disconnectNow(); }
    else { g_probeMode = true; g_tryIdx = -1; drawScreen(); }
    g_lastChangeMs = millis();
  }
  lastBtn = btn;

  if (g_connectReq) { g_connectReq = false; doConnect(g_tryIdx); }

  if (!g_connected) {
    if (!g_probeMode) {
      // RECORD：只扫描，不连接
      BLEScan* scan = BLEDevice::getScan();
      scan->start(2);
      scan->clearResults();
      // 空闲 120 秒且已登记 → 自动进入 PROBE
      if (g_n > 0 && millis() - g_lastChangeMs > 120000) { g_probeMode = true; g_tryIdx = -1; drawScreen(); }
    } else {
      // PROBE：依次连接每个已登记传感器
      BLEScan* scan = BLEDevice::getScan();
      scan->start(1);
      scan->clearResults();
      if (g_n > 0) {
        // 只挑"最近 3 秒还在广播"的，否则 connect 会白等 30 秒超时
        int cand = -1;
        for (uint8_t k = 0; k < g_n; k++) {
          uint8_t i2 = (uint8_t)((g_tryIdx + 1 + k) % g_n);
          if (millis() - g_lastSeen[i2] < 3000) { cand = i2; break; }
        }
        if (cand >= 0) { g_tryIdx = (int8_t)cand; g_connectReq = true; }
      }
    }
    static uint32_t lastDraw = 0;
    if (millis() - lastDraw > 1000) { lastDraw = millis(); drawScreen(); }
  } else {
    if (g_rxCount > 0) {
      if (millis() - g_connAt > 240000) { disconnectNow(); g_lastChangeMs = millis(); }
    } else if (millis() - g_connAt > 45000) {
      disconnectNow(); g_lastChangeMs = millis();
    }
    delay(50);
  }
}