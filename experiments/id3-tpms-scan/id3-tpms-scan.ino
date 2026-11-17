/*
 * id3-tpms-scan v2 —— 干净重写版
 *
 * 布局：所有文字按"该行在圆内的可用宽度"裁剪并居中，不会出圈
 * 心跳：右上角一个小方块每 500ms 翻转，用来确认固件在跑
 *
 * 流程：扫描 → 按 RSSI 排序显示前 4 个 → 最强且强于 -55 时自动连接
 *       连上后显示 GATT 表（服务数/特征数/UUID/属性）→ 收到通知显示 hex
 *
 * 串口 115200 同步打印全部细节。
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

#define MAXD 16
struct D { bool used; char mac[18]; int8_t rssi; uint32_t cnt; uint32_t seen; uint8_t at; };
static D g_d[MAXD];
static uint8_t g_nd = 0;

#define MAXG 8
static char    g_gu[MAXG][10];
static uint8_t g_gp[MAXG];
static uint8_t g_gn = 0;
static uint8_t g_svc = 0;

static bool     g_conn = false;
static char     g_hex[56] = {0};
static uint32_t g_cooldown = 0;
static uint8_t  g_tries = 0;
static BLEClient* g_cli = nullptr;
static bool     g_beat = false;
static char     g_note[24] = "boot";

/* ---------- 圆内自适应绘制 ---------- */
static void fitted(const char* s, int y, uint16_t fg, uint8_t size = 1) {
  const int R = 114;
  int dy = y + 3 - 120;
  int r2 = R * R - dy * dy;
  if (r2 <= 0) return;
  int half = (int)sqrt((double)r2);
  int perChar = 6 * size;
  int maxChars = (half * 2 - 10) / perChar;
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

static int topIdx(int rank) {
  bool used[MAXD] = {false};
  for (int r = 0; r <= rank; r++) {
    int best = -1, bestR = -300;
    for (int i = 0; i < MAXD; i++) {
      if (!g_d[i].used || used[i]) continue;
      if (g_d[i].rssi > bestR) { bestR = g_d[i].rssi; best = i; }
    }
    if (best < 0) return -1;
    used[best] = true;
    if (r == rank) return best;
  }
  return -1;
}

static void draw() {
  lcd.fillScreen(TFT_BLACK);
  lcd.drawArc(120, 120, 119, 119, 0, 359, g_conn ? 0x07E0 : 0xB01FE0);
  lcd.fillRect(210, 14, 6, 6, g_beat ? 0x07E0 : 0x0320);   // 心跳点

  char h[26];
  if (g_conn)      snprintf(h, sizeof(h), "OK s%u c%u", g_svc, g_gn);
  else if (g_tries) snprintf(h, sizeof(h), "TRY %u", g_tries);
  else              snprintf(h, sizeof(h), "SCAN %u", g_nd);
  fitted(h, 24, 0x07FF, 2);

  if (g_conn) {
    for (int i = 0; i < g_gn && i < 5; i++) {
      char l[36];
      snprintf(l, sizeof(l), "%s %c%c%c", g_gu[i],
               (g_gp[i] & 2) ? 'R' : '-', (g_gp[i] & 8) ? 'W' : '-', (g_gp[i] & 16) ? 'N' : '-');
      fitted(l, 48 + i * 15, (g_gp[i] & 16) ? 0x07E0 : 0xFFFF);
    }
    if (g_gn == 0) fitted(g_note, 60, 0xF800);
    if (g_hex[0]) { fitted(g_hex, 150, 0xFFE0); fitted(g_hex + 24, 164, 0xFFE0); }
    else fitted("wait data", 156, 0xFD20);
  } else {
    for (int r = 0; r < 4; r++) {
      int i = topIdx(r);
      if (i < 0) break;
      char l[36];
      snprintf(l, sizeof(l), "%d %s %d", r + 1, g_d[i].mac, g_d[i].rssi);
      fitted(l, 46 + r * 15, r == 0 ? 0x07E0 : 0xFFFF);
    }
    fitted(g_nd ? "" : "put board on sensor", 110, 0x8C71);
    fitted(g_note, 178, 0xFD20);
  }
  fitted(g_conn ? "BOOT = disc" : "BOOT = connect", 212, 0xFD20);
}

static void pushHex(const uint8_t* d, size_t n) {
  size_t k = 0; g_hex[0] = 0;
  for (size_t i = 0; i < n && i < 20 && k + 4 < sizeof(g_hex); i++)
    k += snprintf(g_hex + k, sizeof(g_hex) - k, "%02X", d[i]);
  Serial.printf("[RX] %s\r\n", g_hex);
  draw();
}
static void ncb(BLERemoteCharacteristic* c, uint8_t* d, size_t n, bool) { pushHex(d, n); }

static void connectTo(int idx) {
  if (idx < 0) return;
  g_tries = 1;
  snprintf(g_note, sizeof(g_note), "connecting");
  draw();

  BLEDevice::getScan()->stop();
  BLEDevice::getScan()->clearResults();
  delay(600);

  if (!g_cli) g_cli = BLEDevice::createClient();
  if (g_cli->isConnected()) g_cli->disconnect();

  Serial.printf("[连接] %s at=%u\r\n", g_d[idx].mac, g_d[idx].at);
  bool ok = g_cli->connect(BLEAddress(g_d[idx].mac), (esp_ble_addr_type_t)g_d[idx].at);
  Serial.printf("  结果=%d\r\n", (int)ok);
  if (!ok) {
    esp_ble_addr_type_t alt = (g_d[idx].at == BLE_ADDR_TYPE_RANDOM) ? BLE_ADDR_TYPE_PUBLIC : BLE_ADDR_TYPE_RANDOM;
    ok = g_cli->connect(BLEAddress(g_d[idx].mac), alt);
    Serial.printf("  备用地址类型结果=%d\r\n", (int)ok);
  }
  g_tries = 0;

  if (!ok) {
    snprintf(g_note, sizeof(g_note), "connect failed");
    draw();
    g_cooldown = millis() + 6000;
    delay(1200);
    return;
  }

  g_conn = true;
  g_svc = 0; g_gn = 0; g_hex[0] = 0;
  snprintf(g_note, sizeof(g_note), "connected");
  draw();
  Serial.printf("[已连接] 枚举服务\r\n");

  std::map<std::string, BLERemoteService*>* svcs = g_cli->getServices();
  if (!svcs || svcs->empty()) {
    snprintf(g_note, sizeof(g_note), "no service found");
    Serial.printf("  !! getServices 为空\r\n");
    draw();
    return;
  }
  for (auto& kv : *svcs) {
    BLERemoteService* s = kv.second;
    g_svc++;
    Serial.printf("服务 %s\r\n", s->getUUID().toString().c_str());
    auto* chs = s->getCharacteristics();
    if (!chs) { Serial.printf("  (无特征值)\r\n"); continue; }
    for (auto& c2 : *chs) {
      BLERemoteCharacteristic* ch = c2.second;
      Serial.printf("  特征 %s R=%d W=%d N=%d I=%d\r\n", ch->getUUID().toString().c_str(),
                    ch->canRead(), ch->canWrite(), ch->canNotify(), ch->canIndicate());
      if (g_gn < MAXG) {
        String u = ch->getUUID().toString().c_str();
        if (u.length() >= 8 && u.startsWith("0000")) snprintf(g_gu[g_gn], sizeof(g_gu[0]), "%s", u.substring(4, 8).c_str());
        else snprintf(g_gu[g_gn], sizeof(g_gu[0]), "%s", u.substring(0, 8).c_str());
        g_gp[g_gn] = (ch->canRead() ? 2 : 0) | (ch->canWrite() ? 8 : 0) | (ch->canNotify() ? 16 : 0);
        g_gn++;
      }
      if (ch->canNotify() || ch->canIndicate()) { ch->registerForNotify(ncb); Serial.printf("    已订阅\r\n"); }
      if (ch->canRead()) {
        std::string v = ch->readValue();
        Serial.printf("    读出 %u 字节\r\n", (unsigned)v.size());
        if (v.size()) pushHex((const uint8_t*)v.data(), v.size());
      }
    }
  }
  if (g_gn == 0) snprintf(g_note, sizeof(g_note), "no char found");
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
  memset(g_d, 0, sizeof(g_d));
  lcd.init();
  lcd.setBrightness(255);
  lcd.fillScreen(TFT_BLACK);
  fitted("booting", 120, 0xFFFF, 2);
  Serial.printf("\r\n=== scan v2 ===\r\n");

  BLEDevice::init("id3-scan");
  BLEScan* sc = BLEDevice::getScan();
  sc->setAdvertisedDeviceCallbacks(new CB(), true);
  sc->setActiveScan(true);
  sc->setInterval(100);
  sc->setWindow(90);
  snprintf(g_note, sizeof(g_note), "scanning");
  draw();
}

void loop() {
  static uint32_t beat = 0;
  if (millis() - beat > 500) { beat = millis(); g_beat = !g_beat; }

  static bool lastBtn = false;
  bool btn = (digitalRead(PIN_BOOT) == LOW);
  if (btn && !lastBtn) {
    if (g_conn) { g_cli->disconnect(); g_conn = false; snprintf(g_note, sizeof(g_note), "scanning"); g_cooldown = millis() + 3000; }
    else connectTo(topIdx(0));
    draw();
  }
  lastBtn = btn;

  if (!g_conn) {
    BLEDevice::getScan()->start(1);
    BLEDevice::getScan()->clearResults();

    int t0 = topIdx(0);
    if (t0 >= 0 && g_d[t0].rssi > -55 && millis() > g_cooldown && g_tries == 0) {
      connectTo(t0);
    }
    static uint32_t last = 0;
    if (millis() - last > 500) { last = millis(); draw(); }
  } else {
    static uint32_t last = 0;
    if (millis() - last > 800) { last = millis(); draw(); }
    delay(40);
  }
}
