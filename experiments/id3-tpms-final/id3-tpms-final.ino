/*
 * id3-tpms-final —— 按实测通信格式实现
 *
 *   数据帧：55 AA 0A ?? idx pLo t+50 alarm vHi|pHi2 crc
 *           pressure_kPa = (((b[8]&3)<<8) | b[5]) * 2.5
 *           temp_C      = b[6] - 50
 *           voltage_V   = (b[8]>>4)*0.1 + 1.9
 *   查询命令：5A A5 06 06 <id> <xor>
 *   配对命令：5A A5 06 03 <id> <xor>
 *   校验：前 n-1 字节异或，放在最后一字节
 *
 * 流程：扫描 → 连最强设备 → 找 ff01(通知)/ff02(写) → 订阅 ff01
 *       → 依次对 id=0..3 发查询命令 → 解析 55AA0A 帧 → 屏幕显示四轮
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
struct D { bool used; char mac[18]; int8_t rssi; uint32_t seen; uint8_t at; uint32_t skip; };
static D g_d[MAXD];
static uint8_t g_nd = 0;

struct Wheel { bool has; uint8_t idx; float kPa; float tempC; float volt; uint8_t alarm; uint32_t at; };
static Wheel g_w[4];

static bool g_conn = false;
static char g_status[16] = "SCAN";
static char g_hex[56] = {0};
static uint16_t g_frames = 0;
static uint8_t g_qIdx = 0;
static uint32_t g_nextQuery = 0;
static uint32_t g_cooldown = 0;
static BLEClient* g_cli = nullptr;
static BLERemoteCharacteristic* g_ch1 = nullptr;
static BLERemoteCharacteristic* g_ch2 = nullptr;

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
  fitted(g_status, 20, 0x07FF, 2);

  int shown = 0;
  for (int i = 0; i < 4; i++) {
    if (!g_w[i].has) continue;
    char l[34];
    float bar = g_w[i].kPa / 100.0f;
    snprintf(l, sizeof(l), "%u %.2fbar %.0fC %.1fV", g_w[i].idx, bar, g_w[i].tempC, g_w[i].volt);
    uint16_t col = (bar < 2.0f) ? 0xF800 : (bar < 2.2f ? 0xFD20 : 0xFFFF);
    fitted(l, 46 + shown * 15, col);
    shown++;
  }
  if (shown == 0) fitted(g_conn ? "querying..." : "put board on sensor", 60, 0x8C71);

  char s2[26]; snprintf(s2, sizeof(s2), "frames %u q%u", g_frames, g_qIdx);
  fitted(s2, 108, 0xFFE0);
  if (g_hex[0]) { fitted(g_hex, 150, 0x07E0); fitted(g_hex + 22, 164, 0x07E0); }
  fitted(g_conn ? "BOOT = disc" : "BOOT = connect", 206, 0xFD20);
}

/* ---- 命令构造 ---- */
static void xorFinish(uint8_t* b, uint8_t n) {
  uint8_t x = 0;
  for (uint8_t i = 0; i + 1 < n; i++) x ^= b[i];
  b[n - 1] = x;
}

static void sendQuery(uint8_t id) {
  if (!g_ch2) return;
  uint8_t cmd[6] = {0x5A, 0xA5, 0x06, 0x06, id, 0x00};
  xorFinish(cmd, sizeof(cmd));
  g_ch2->writeValue(cmd, sizeof(cmd), true);
  Serial.printf("[TX] query id=%u -> %02X %02X %02X %02X %02X %02X\r\n",
                id, cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5]);
}

static void sendPair(uint8_t id) {
  if (!g_ch2) return;
  uint8_t cmd[6] = {0x5A, 0xA5, 0x06, 0x03, id, 0x00};
  xorFinish(cmd, sizeof(cmd));
  g_ch2->writeValue(cmd, sizeof(cmd), true);
  Serial.printf("[TX] pair id=%u\r\n", id);
}

/* ---- 解析 55AA0A 帧 ---- */
static void parseFrame(const uint8_t* b, size_t n) {
  if (n < 10) return;
  if (!(b[0] == 0x55 && b[1] == 0xAA && b[2] == 0x0A)) return;
  uint8_t idx    = b[4];
  uint16_t praw  = (((uint16_t)(b[8] & 0x03)) << 8) | b[5];
  float kPa      = praw * 2.5f;
  float tempC    = (float)b[6] - 50.0f;
  float volt     = (float)(b[8] >> 4) * 0.1f + 1.9f;
  uint8_t alarm  = b[7];

  int slot = idx < 4 ? idx : 3;
  g_w[slot].has = true; g_w[slot].idx = idx;
  g_w[slot].kPa = kPa; g_w[slot].tempC = tempC; g_w[slot].volt = volt;
  g_w[slot].alarm = alarm; g_w[slot].at = millis();

  Serial.printf("[帧] idx=%u %.2fbar %.0fC %.1fV alarm=%u\r\n", idx, kPa / 100.0f, tempC, volt, alarm);
}

static void onData(BLERemoteCharacteristic* c, uint8_t* d, size_t n, bool) {
  size_t k = 0; g_hex[0] = 0;
  for (size_t i = 0; i < n && i < 20 && k + 4 < sizeof(g_hex); i++)
    k += snprintf(g_hex + k, sizeof(g_hex) - k, "%02X", d[i]);
  g_frames++;
  Serial.printf("[RX %u] %s\r\n", (unsigned)n, g_hex);
  parseFrame(d, n);
  draw();
}

static bool uuidIs(const char* u, const char* h4) {
  return strncmp(u, "0000", 4) == 0 && strncmp(u + 4, h4, 4) == 0;
}

static void tryConnect(int idx) {
  if (idx < 0 || idx >= MAXD) return;
  snprintf(g_status, sizeof(g_status), "TRY");
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
  if (!ok) { g_d[idx].skip = millis() + 15000; g_cooldown = millis() + 2000; snprintf(g_status, sizeof(g_status), "SCAN"); draw(); return; }

  g_ch1 = nullptr; g_ch2 = nullptr;
  auto* svcs = g_cli->getServices();
  if (svcs) for (auto& kv : *svcs) {
    auto* chs = kv.second->getCharacteristics();
    if (!chs) continue;
    for (auto& c2 : *chs) {
      String cu = c2.second->getUUID().toString().c_str();
      if (uuidIs(cu.c_str(), "ff01")) g_ch1 = c2.second;
      if (uuidIs(cu.c_str(), "ff02")) g_ch2 = c2.second;
    }
  }
  if (!g_ch1 && !g_ch2) {
    Serial.printf("[跳过] 无 ff01/ff02\r\n");
    g_cli->disconnect();
    g_d[idx].skip = millis() + 60000;
    g_cooldown = millis() + 800;
    snprintf(g_status, sizeof(g_status), "SCAN"); draw();
    return;
  }

  g_conn = true; g_frames = 0; g_hex[0] = 0; g_qIdx = 0;
  memset(g_w, 0, sizeof(g_w));
  snprintf(g_status, sizeof(g_status), "OK");
  if (g_ch1 && (g_ch1->canNotify() || g_ch1->canIndicate())) {
    g_ch1->registerForNotify(onData);
    Serial.printf("[订阅] ff01\r\n");
  }
  sendPair(0);                 // 先试一次配对，很多模块要它才认
  g_nextQuery = millis() + 300;
  draw();
}

class CB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) {
    if (g_conn) return;
    char mac[18];
    snprintf(mac, sizeof(mac), "%s", dev.getAddress().toString().c_str());
    for (char* q = mac; *q; q++) *q = tolower(*q);
    for (int i = 0; i < MAXD; i++) {
      if (g_d[i].used && strcmp(g_d[i].mac, mac) == 0) { g_d[i].rssi = (int8_t)dev.getRSSI(); g_d[i].seen = millis(); return; }
    }
    for (int i = 0; i < MAXD; i++) if (!g_d[i].used) {
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
};

void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(PIN_BOOT, INPUT_PULLUP);
  memset(g_d, 0, sizeof(g_d));
  memset(g_w, 0, sizeof(g_w));
  lcd.init();
  lcd.setBrightness(255);
  draw();
  Serial.printf("\r\n=== final ===\r\n");
  BLEDevice::init("id3-tpms");
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
    else { for (int i = 0; i < MAXD; i++) g_d[i].skip = 0; g_cooldown = 0; }
    draw();
  }
  lastBtn = btn;

  if (!g_conn) {
    BLEDevice::getScan()->start(1);
    BLEDevice::getScan()->clearResults();
    if (millis() > g_cooldown) {
      int best = -1, score = -1000;
      for (int i = 0; i < MAXD; i++) {
        if (!g_d[i].used) continue;
        if (millis() - g_d[i].seen > 5000) continue;
        if (g_d[i].skip > millis()) continue;
        int s = (g_d[i].skip == 0 ? 200 : 0) + g_d[i].rssi;
        if (s > score) { score = s; best = i; }
      }
      if (best >= 0) tryConnect(best);
    }
    static uint32_t t = 0;
    if (millis() - t > 600) { t = millis(); draw(); }
  } else {
    // 轮流查询 id 0..3
    if (millis() > g_nextQuery) {
      sendQuery(g_qIdx);
      g_qIdx = (g_qIdx + 1) & 0x03;
      g_nextQuery = millis() + 700;
    }
    static uint32_t t = 0;
    if (millis() - t > 700) { t = millis(); draw(); }
    delay(20);
  }
}
