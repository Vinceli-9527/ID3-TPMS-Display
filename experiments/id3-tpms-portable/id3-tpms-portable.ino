/*
 * id3-tpms-portable v2 —— 便携连接式胎压探针（脱离电脑，看屏幕拍照）
 *
 * v1 的 bug：connect() 写在 BLE 扫描回调里，阻塞扫描任务 → 状态永远停在 CONNECT。
 * v2 修法：回调里只记 MAC + 置标志，真正的 connect 放到 loop 里做，并先 stop 扫描。
 *
 * 屏幕：
 *   状态（SCAN / FOUND / CONNECT / OK / FAIL）
 *   对端 MAC
 *   服务/特征值统计
 *   最近收到的 4 帧原始 hex（每行 18 字节）
 *
 * 串口：t <mac> 设目标并保存 | T 清除 | l 列表 | d 断开重连
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

#define FRAMES 4
#define HEXLINE 36          // 每行 18 字节

static char   g_target[20] = {0};
static char   g_status[16] = "BOOT";
static char   g_peer[20]   = {0};
static char   g_frames[FRAMES][HEXLINE * 2 + 4];
static uint8_t g_frameIdx = 0;
static uint16_t g_rxCount = 0;
static uint8_t g_svcN = 0, g_chrN = 0;
static volatile bool g_connectReq = false;
static char   g_pendingMac[20] = {0};
static bool   g_connected = false;
static uint32_t g_connAt = 0;
static Preferences g_pref;
static BLEClient* g_client = nullptr;

/* ---------- 绘制 ---------- */
static void drawScreen() {
  lcd.fillScreen(TFT_BLACK);
  lcd.drawArc(120, 120, 117, 119, 0, 359, 0x07E0);

  lcd.setTextDatum(textdatum_t::top_center);
  lcd.setTextSize(2);
  lcd.setTextColor(0x07FF, TFT_BLACK);
  lcd.drawString(g_status, 120, 24);

  lcd.setTextSize(1);
  lcd.setTextDatum(textdatum_t::top_left);
  char line[40];
  snprintf(line, sizeof(line), "%s", g_peer[0] ? g_peer : (g_target[0] ? g_target : "no target"));
  lcd.setTextColor(0x8C71, TFT_BLACK);
  lcd.drawString(line, 20, 48);

  snprintf(line, sizeof(line), "svc %u  chr %u  rx %u", g_svcN, g_chrN, g_rxCount);
  lcd.setTextColor(0xFFE0, TFT_BLACK);
  lcd.drawString(line, 20, 62);

  lcd.drawFastHLine(24, 78, 192, 0x4208);

  // 最近 4 帧
  for (int k = 0; k < FRAMES; k++) {
    int idx = (g_frameIdx + k) % FRAMES;
    if (!g_frames[idx][0]) continue;
    char buf[HEXLINE + 1];
    for (int i = 0; i < HEXLINE; i++) buf[i] = g_frames[idx][i];
    buf[HEXLINE] = 0;
    lcd.setTextColor(0xFFFF, TFT_BLACK);
    lcd.drawString(buf, 16, 86 + k * 14);
    char buf2[HEXLINE + 1];
    for (int i = 0; i < HEXLINE; i++) buf2[i] = g_frames[idx][HEXLINE + i];
    buf2[HEXLINE] = 0;
    if (buf2[0]) lcd.drawString(buf2, 16, 100 + k * 14);
  }
  if (g_rxCount == 0) {
    lcd.setTextColor(0xFD20, TFT_BLACK);
    lcd.setTextDatum(textdatum_t::top_center);
    lcd.drawString("waiting for data...", 120, 150);
  }
}

/* ---------- 数据 ---------- */
static void pushFrame(const uint8_t* data, size_t len) {
  size_t n = len > 36 ? 36 : len;
  char hex[HEXLINE * 2 + 4]; size_t k = 0;
  hex[0] = 0;
  for (size_t i = 0; i < n; i++) k += snprintf(hex + k, sizeof(hex) - k, "%02X", data[i]);
  hex[k] = 0;
  snprintf(g_frames[g_frameIdx], sizeof(g_frames[g_frameIdx]), "%s", hex);
  g_frameIdx = (g_frameIdx + 1) % FRAMES;
  g_rxCount++;
  Serial.printf("[RX] len=%u %s\r\n", (unsigned)len, hex);
  drawScreen();
}

static void notifyCB(BLERemoteCharacteristic* ch, uint8_t* data, size_t len, bool isNotify) {
  pushFrame(data, len);
}

/* ---------- 连接（只在 loop 里调用）---------- */
static void doConnect(const char* mac) {
  snprintf(g_status, sizeof(g_status), "CONNECT");
  snprintf(g_peer, sizeof(g_peer), "%s", mac);
  drawScreen();

  BLEScan* scan = BLEDevice::getScan();
  scan->stop();
  scan->clearResults();
  delay(300);

  g_client = BLEDevice::createClient();
  Serial.printf("[连接] %s\r\n", mac);
  if (!g_client->connect(BLEAddress(mac))) {
    snprintf(g_status, sizeof(g_status), "FAIL");
    drawScreen();
    Serial.printf("[失败] 连接超时\r\n");
    delete g_client; g_client = nullptr;
    delay(800);
    return;
  }
  g_connected = true;
  g_connAt = millis();
  snprintf(g_status, sizeof(g_status), "OK");
  drawScreen();
  Serial.printf("[已连接]\r\n");

  g_svcN = 0; g_chrN = 0;
  std::map<std::string, BLERemoteService*>* svcs = g_client->getServices();
  if (svcs) {
    for (auto& kv : *svcs) {
      BLERemoteService* svc = kv.second;
      g_svcN++;
      Serial.printf("服务 %s\r\n", svc->getUUID().toString().c_str());
      std::map<std::string, BLERemoteCharacteristic*>* chrs = svc->getCharacteristics();
      if (!chrs) continue;
      for (auto& c2 : *chrs) {
        BLERemoteCharacteristic* ch = c2.second;
        g_chrN++;
        Serial.printf("  特征 %s R=%d W=%d N=%d I=%d\r\n", ch->getUUID().toString().c_str(),
                      ch->canRead(), ch->canWrite(), ch->canNotify(), ch->canIndicate());
        if (ch->canNotify() || ch->canIndicate()) {
          ch->registerForNotify(notifyCB);
          Serial.printf("     已订阅\r\n");
        }
        if (ch->canRead()) {
          std::string v = ch->readValue();
          if (v.size()) pushFrame((const uint8_t*)v.data(), v.size());
        }
      }
    }
  }
  drawScreen();
}

/* ---------- 扫描回调：只记 MAC ---------- */
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
      for (uint8_t k = 0; k + 1 < dlen; k += 16) if (d[k] == 0xB0 && d[k + 1] == 0xFB) return true;
    } else if (type == 0x16 && dlen >= 2) {
      if (d[0] == 0xB0 && d[1] == 0xFB) return true;
    }
    i += (size_t)len + 1;
  }
  return false;
}

class AdvCB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) {
    if (g_connectReq || g_connected) return;
    size_t plen = 0; const uint8_t* pl = nullptr;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    std::string s = dev.getPayload(); plen = s.size(); pl = (const uint8_t*)s.data();
#else
    plen = dev.getPayloadLength(); pl = dev.getPayload();
#endif
    char mac[20];
    snprintf(mac, sizeof(mac), "%s", dev.getAddress().toString().c_str());
    bool match = false;
    if (g_target[0] && strcasecmp(mac, g_target) == 0) match = true;
    else if (advHasFBB0(pl, plen)) match = true;
    if (!match) return;

    strncpy(g_pendingMac, mac, sizeof(g_pendingMac) - 1);
    g_connectReq = true;
    Serial.printf("[发现] %s rssi=%d fbb0=%d\r\n", mac, (int)dev.getRSSI(), (int)advHasFBB0(pl, plen));
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
    if (cmd[0] == 't') {
      String mac = cmd.substring(1); mac.trim(); mac.toLowerCase();
      if (mac.length()) { strncpy(g_target, mac.c_str(), sizeof(g_target) - 1); g_pref.putString("target", g_target); Serial.printf("目标=%s\r\n", g_target); }
    } else if (cmd[0] == 'T') { g_target[0] = 0; g_pref.putString("target", ""); Serial.printf("已清除\r\n"); }
    else if (cmd[0] == 'd') {
      if (g_client) { g_client->disconnect(); delete g_client; g_client = nullptr; }
      g_connected = false; g_connectReq = false;
      snprintf(g_status, sizeof(g_status), "SCAN"); drawScreen();
    } else if (cmd[0] == 'l') {
      Serial.printf("target=%s connected=%d svc=%u chr=%u rx=%u\r\n", g_target, g_connected, g_svcN, g_chrN, g_rxCount);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  lcd.init();
  lcd.setBrightness(255);
  snprintf(g_status, sizeof(g_status), "BOOT");
  drawScreen();

  g_pref.begin("tpms", false);
  String t = g_pref.getString("target", "");
  if (t.length()) strncpy(g_target, t.c_str(), sizeof(g_target) - 1);

  BLEDevice::init("tpms-portable");
  BLEScan* scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new AdvCB(), true);
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(90);

  snprintf(g_status, sizeof(g_status), "SCAN");
  drawScreen();
  Serial.printf("\r\n=== portable v2 ===\r\ntarget=%s\r\n", g_target[0] ? g_target : "(none, 会用 FBB0 自动识别)");
}

void loop() {
  pollSerial();

  if (g_connectReq) {                    // 连接放在 loop 里做，绝不阻塞扫描回调
    g_connectReq = false;
    doConnect(g_pendingMac);
  }

  if (!g_connected) {
    BLEScan* scan = BLEDevice::getScan();
    scan->start(2);
    scan->clearResults();
    static uint32_t lastDraw = 0;
    if (millis() - lastDraw > 1000) { lastDraw = millis(); drawScreen(); }
  } else {
    // 连上后保持；120 秒没收到任何帧就重连
    if (g_rxCount == 0 && millis() - g_connAt > 120000) {
      Serial.printf("[超时] 无数据，重连\r\n");
      if (g_client) { g_client->disconnect(); delete g_client; g_client = nullptr; }
      g_connected = false;
      snprintf(g_status, sizeof(g_status), "SCAN");
      drawScreen();
    }
    delay(50);
  }
}
