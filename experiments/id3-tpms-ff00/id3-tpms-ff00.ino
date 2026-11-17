/*
 * id3-tpms-ff00 v2 —— 先连后验（不再依赖广播里的服务列表）
 *
 * 上一版错误：要求广播里声明 FF00 服务。多数 FF00 类模块不在广播里写服务表，
 *             结果被过滤成 0 个设备。本版改为：
 *             按 RSSI 挑最靠近的设备 → 连上去 → 检查 GATT 里有没有 ff00
 *             → 有就订阅 ff01 并试探写 ff02；没有就断开换下一个。
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

#define MAXD 8
struct D { bool used; char mac[18]; int8_t rssi; uint32_t seen; uint8_t at; uint32_t skipUntil; };
static D g_d[MAXD];
static uint8_t g_nd = 0;

static bool  g_conn = false;
static char  g_status[20] = "SCAN";
static char  g_detail[26] = {0};
static char  g_hex[56] = {0};
static uint16_t g_frames = 0;
static uint32_t g_connAt = 0;
static uint32_t g_cooldown = 0;
static uint8_t  g_writeTry = 0;
static int      g_curIdx = -1;
static BLEClient* g_cli = nullptr;
static BLERemoteCharacteristic* g_ch1 = nullptr;
static BLERemoteCharacteristic* g_ch2 = nullptr;
static char g_lastRes[26] = "none";

static void fitted(const char* s, int y, uint16_t fg, uint8_t size = 1) {
  const int R = 114;
  int dy = y + 3 - 120;
  int r2 = R * R - dy * dy;
  if (r2 <= 0) return;
  int half = (int)sqrt((double)r2);
  int maxChars = (half * 2 - 10) / (6 * size);
  if (maxChars <= 0) return;
  char buf[64];
  size_t n = strlen(s);
  if ((int)n > maxChars) n = (size_t)maxChars;
  if (n >= sizeof(buf)) n = sizeof(buf) - 1;
  memcpy(buf, s, n); buf[n] = 0;
  lcd.setTextDatum(textdatum_t::middle_center);
  lcd.setTextSize(size);
  lcd.setTextColor(fg, TFT_BLACK);
  lcd.drawString(buf, 120, y);
}

static void draw() {
  lcd.fillScreen(TFT_BLACK);
  lcd.drawArc(120, 120, 119, 119, 0, 359, g_conn ? 0x07E0 : 0xB01FE0);
  fitted(g_status, 22, 0x07FF, 2);
  if (g_conn) {
    fitted(g_detail, 48, 0xFFFF);
    char l[26]; snprintf(l, sizeof(l), "frames %u  wr %u", g_frames, g_writeTry);
    fitted(l, 66, 0xFFE0);
    if (g_hex[0]) { fitted(g_hex, 118, 0x07E0); fitted(g_hex + 24, 132, 0x07E0); }
    else fitted("waiting ff01...", 118, 0xFD20);
  } else {
    for (int r = 0; r < 3; r++) {
      int best = -1, bestR = -300;
      for (int i = 0; i < MAXD; i++) {
        if (!g_d[i].used) continue;
        bool dup = false;
        for (int k = 0; k < r; k++) {
          int b2 = -1, b2R = -300;
          for (int j = 0; j < MAXD; j++) if (g_d[j].used && g_d[j].rssi > b2R && g_d[j].rssi < 200) { /* noop */ }
          (void)b2; (void)b2R; (void)dup;
        }
        if (g_d[i].rssi > bestR) { bestR = g_d[i].rssi; best = i; }
      }
      if (best < 0) break;
      char l[30];
      snprintf(l, sizeof(l), "%s %d", g_d[best].mac, g_d[best].rssi);
      fitted(l, 50 + r * 15, r == 0 ? 0x07E0 : 0xFFFF);
      g_d[best].rssi = 200;   // 临时排除，绘制后还原
    }
    for (int i = 0; i < MAXD; i++) if (g_d[i].rssi == 200) g_d[i].rssi = -100;
    char s1[30];
    snprintf(s1, sizeof(s1), "devs %u", g_nd);
    fitted(s1, 106, 0xFFE0);
    fitted(g_lastRes, 124, 0x8C71);
    fitted("BOOT = retry all", 172, 0xFD20);
  }
}

static void onData(BLERemoteCharacteristic* c, uint8_t* d, size_t n, bool) {
  size_t k = 0; g_hex[0] = 0;
  for (size_t i = 0; i < n && i < 22 && k + 4 < sizeof(g_hex); i++)
    k += snprintf(g_hex + k, sizeof(g_hex) - k, "%02X", d[i]);
  g_frames++;
  Serial.printf("[RX %u] %s\r\n", (unsigned)n, g_hex);
  draw();
}

static bool uuidIs(const char* u, const char* hex4) {
  return strncmp(u, "0000", 4) == 0 && strncmp(u + 4, hex4, 4) == 0;
}

static void pokeFF02() {
  if (!g_ch2 || g_writeTry >= 3) return;
  static const uint8_t cmds[3][2] = { {0x01, 0x00}, {0xA5, 0x00}, {0xAA, 0x55} };
  g_ch2->writeValue((uint8_t*)cmds[g_writeTry], 2, true);
  Serial.printf("[TX] %02X %02X\r\n", cmds[g_writeTry][0], cmds[g_writeTry][1]);
  g_writeTry++;
  draw();
}

static void tryConnect(int idx) {
  if (idx < 0 || idx >= MAXD) return;
  g_curIdx = idx;
  snprintf(g_status, sizeof(g_status), "TRY");
  snprintf(g_detail, sizeof(g_detail), "%.17s", g_d[idx].mac);
  draw();

  BLEDevice::getScan()->stop();
  BLEDevice::getScan()->clearResults();
  delay(500);

  if (!g_cli) g_cli = BLEDevice::createClient();
  if (g_cli->isConnected()) g_cli->disconnect();

  bool ok = g_cli->connect(BLEAddress(g_d[idx].mac), (esp_ble_addr_type_t)g_d[idx].at);
  if (!ok) {
    esp_ble_addr_type_t alt = (g_d[idx].at == BLE_ADDR_TYPE_RANDOM) ? BLE_ADDR_TYPE_PUBLIC : BLE_ADDR_TYPE_RANDOM;
    ok = g_cli->connect(BLEAddress(g_d[idx].mac), alt);
  }
  Serial.printf("[连接] %s ok=%d\r\n", g_d[idx].mac, (int)ok);
  if (!ok) {
    snprintf(g_lastRes, sizeof(g_lastRes), "conn fail");
    g_d[idx].skipUntil = millis() + 20000;
    g_cooldown = millis() + 2500;
    snprintf(g_status, sizeof(g_status), "SCAN");
    draw();
    return;
  }

  g_ch1 = nullptr; g_ch2 = nullptr;
  int svcN = 0, chrN = 0;
  bool hasFF00 = false;
  auto* svcs = g_cli->getServices();
  if (svcs) {
    for (auto& kv : *svcs) {
      BLERemoteService* s = kv.second;
      String su = s->getUUID().toString().c_str();
      svcN++;
      if (uuidIs(su.c_str(), "ff00")) hasFF00 = true;
      auto* chs = s->getCharacteristics();
      if (!chs) continue;
      for (auto& c2 : *chs) {
        BLERemoteCharacteristic* ch = c2.second;
        String cu = ch->getUUID().toString().c_str();
        chrN++;
        if (uuidIs(cu.c_str(), "ff01")) { g_ch1 = ch; hasFF00 = true; }
        if (uuidIs(cu.c_str(), "ff02")) { g_ch2 = ch; hasFF00 = true; }
        Serial.printf("  特征 %s R=%d W=%d N=%d\r\n", cu.c_str(), ch->canRead(), ch->canWrite(), ch->canNotify());
      }
    }
  }
  Serial.printf("[枚举] svc=%d chr=%d ff00=%d ff01=%d ff02=%d\r\n", svcN, chrN, (int)hasFF00, g_ch1 != nullptr, g_ch2 != nullptr);

  if (!hasFF00) {
    snprintf(g_detail, sizeof(g_detail), "no ff00 (%d/%d)", svcN, chrN);
    snprintf(g_lastRes, sizeof(g_lastRes), "no ff00 s%d c%d", svcN, chrN);
    draw();
    g_cli->disconnect();
    g_d[idx].skipUntil = millis() + 60000;   // 这个不是目标，别反复试
    g_cooldown = millis() + 1200;
    snprintf(g_status, sizeof(g_status), "SCAN");
    draw();
    return;
  }

  g_conn = true;
  g_connAt = millis(); g_frames = 0; g_hex[0] = 0; g_writeTry = 0;
  snprintf(g_status, sizeof(g_status), "OK");
  snprintf(g_detail, sizeof(g_detail), "svc%d chr%d %s%s", svcN, chrN, g_ch1 ? "ff01" : "", g_ch2 ? "+ff02" : "");

  if (g_ch1 && (g_ch1->canNotify() || g_ch1->canIndicate())) { g_ch1->registerForNotify(onData); Serial.printf("  已订阅 ff01\r\n"); }
  if (g_ch1 && g_ch1->canRead()) {
    std::string v = g_ch1->readValue();
    Serial.printf("  ff01 读 %u 字节\r\n", (unsigned)v.size());
    if (v.size()) onData(g_ch1, (uint8_t*)v.data(), v.size(), false);
  }
  draw();
}

class CB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) {
    if (g_conn) return;
    char mac[18];
    snprintf(mac, sizeof(mac), "%s", dev.getAddress().toString().c_str());
    for (char* q = mac; *q; q++) *q = tolower(*q);
    for (int i = 0; i < MAXD; i++) {
      if (g_d[i].used && strcmp(g_d[i].mac, mac) == 0) {
        g_d[i].rssi = (int8_t)dev.getRSSI();
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
  memset(g_d, 0, sizeof(g_d));
  lcd.init();
  lcd.setBrightness(255);
  draw();
  Serial.printf("\r\n=== FF00 先连后验 v2 ===\r\n");
  BLEDevice::init("id3-ff00");
  BLEScan* sc = BLEDevice::getScan();
  sc->setAdvertisedDeviceCallbacks(new CB(), true);
  sc->setActiveScan(true);
  sc->setInterval(100);
  sc->setWindow(90);
}

void loop() {
  static bool lastBtn = false;
  bool btn = (digitalRead(PIN_BOOT) == LOW);
  if (btn && !lastBtn) {
    if (g_conn) { g_cli->disconnect(); g_conn = false; snprintf(g_status, sizeof(g_status), "SCAN"); }
    else {
      for (int i = 0; i < MAXD; i++) g_d[i].skipUntil = 0;
      g_cooldown = 0;
    }
    draw();
  }
  lastBtn = btn;

  if (!g_conn) {
    BLEDevice::getScan()->start(1);
    BLEDevice::getScan()->clearResults();

    if (millis() > g_cooldown) {
      // 挑"最近 4 秒还在广播 + RSSI 最强 + 未被跳过"的设备
      int best = -1, bestScore = -1000;
      for (int i = 0; i < MAXD; i++) {
        if (!g_d[i].used) continue;
        if (millis() - g_d[i].seen > 5000) continue;     // 5 秒内还在广播
        if (g_d[i].skipUntil > millis()) continue;       // 失败过的先跳过
        int score = (g_d[i].skipUntil == 0 ? 200 : 0) + g_d[i].rssi;  // 没试过的优先
        if (score > bestScore) { bestScore = score; best = i; }
      }
      if (best >= 0) tryConnect(best);                   // 不再设 RSSI 门槛
    }
    static uint32_t t = 0;
    if (millis() - t > 600) { t = millis(); snprintf(g_detail, sizeof(g_detail), "devs %u", g_nd); draw(); }
  } else {
    if (g_frames == 0 && millis() - g_connAt > 2500 && g_writeTry < 3) {
      static uint32_t lastPoke = 0;
      if (millis() - lastPoke > 1800) { lastPoke = millis(); pokeFF02(); }
    }
    static uint32_t t = 0;
    if (millis() - t > 800) { t = millis(); draw(); }
    delay(30);
  }
}
