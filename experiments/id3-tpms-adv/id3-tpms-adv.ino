/*
 * id3-tpms-adv —— 纯广播解析版
 *
 * 通信格式：本项目实测整理
 *     scanRecord.getBytes() 前 7 字节 = 02 01 06 03 03 B0 FB
 *     voltage     = b[4] * 0.01 + 1.22
 *     pressure    = b[5] * 3.144 kPa ; 若 b[10]==1 → (b[5]+256)*3.144
 *     temperature = b[6] - 55
 *     state       = b[7]
 *     ID          = b[18] b[17] b[16]
 *
 * 因此：完全不需要连接，扫描广播即可。
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

struct W {
  bool has;
  char id[8];
  char mac[18];
  float kPa, tempC, volt;
  uint8_t state;
  int8_t rssi;
  uint32_t at;
  uint32_t cnt;
};
static W g_w[4];
static uint8_t g_nw = 0;
static uint32_t g_frames = 0;
static char g_lastHex[52] = {0};
static char g_lastId[8] = {0};

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
  lcd.drawArc(120, 120, 119, 119, 0, 359, g_nw ? 0x07E0 : 0xB01FE0);
  char h[20];
  snprintf(h, sizeof(h), "%u/%u  f%lu", g_nw, 4, (unsigned long)g_frames);
  fitted(h, 20, 0x07FF, 2);

  for (int i = 0; i < g_nw && i < 4; i++) {
    char l[36];
    float bar = g_w[i].kPa / 100.0f;
    snprintf(l, sizeof(l), "%s %.2fbar %.0fC", g_w[i].id, bar, g_w[i].tempC);
    uint16_t col = (bar < 2.0f) ? 0xF800 : (bar < 2.2f ? 0xFD20 : 0xFFFF);
    fitted(l, 46 + i * 15, col);
  }
  if (g_nw == 0) fitted("waiting 55AA0A...", 60, 0x8C71);

  if (g_lastHex[0]) { fitted(g_lastHex, 112, 0xFFE0); fitted(g_lastHex + 26, 126, 0xFFE0); }
  fitted("adv only, no connect", 176, 0x8C71);
  fitted("BOOT = clear", 206, 0xFD20);
}

static void handleAdv(const uint8_t* p, size_t n, const char* mac, int8_t rssi) {
  // 实测：前 7 字节必须是 02 01 06 03 03 B0 FB
  if (n < 26) return;
  if (!(p[0] == 0x02 && p[1] == 0x01 && p[2] == 0x06 &&
        p[3] == 0x03 && p[4] == 0x03 && p[5] == 0xB0 && p[6] == 0xFB)) return;

  const uint8_t* d = p + 7;      // 19 字节载荷
  float volt  = (float)d[4] * 0.01f + 1.22f;
  float tempC = (float)d[6] - 55.0f;
  float kPa   = (float)d[5] * 3.144f;
  if (d[10] == 1) kPa = ((float)d[5] + 256.0f) * 3.144f;
  uint8_t state = d[7];
  char id[8];
  snprintf(id, sizeof(id), "%02X%02X%02X", d[18], d[17], d[16]);

  g_frames++;
  size_t k = 0;
  for (size_t i = 0; i < 19 && k + 3 < sizeof(g_lastHex); i++)
    k += snprintf(g_lastHex + k, sizeof(g_lastHex) - k, "%02X", d[i]);
  snprintf(g_lastId, sizeof(g_lastId), "%s", id);
  Serial.printf("[ADV] id=%s %.2fbar %.0fC %.2fV st=%u rssi=%d payload=%s\r\n",
                id, kPa / 100.0f, tempC, volt, state, (int)rssi, g_lastHex);

  int slot = -1;
  for (int i = 0; i < g_nw; i++) if (strcmp(g_w[i].id, id) == 0) { slot = i; break; }
  if (slot < 0) {
    if (g_nw < 4) { slot = g_nw++; memset(&g_w[slot], 0, sizeof(W)); }
    else slot = 3;
  }
  W& w = g_w[slot];
  w.has = true;
  snprintf(w.id, sizeof(w.id), "%s", id);
  snprintf(w.mac, sizeof(w.mac), "%s", mac);
  w.kPa = kPa; w.tempC = tempC; w.volt = volt; w.state = state;
  w.rssi = rssi; w.at = millis(); w.cnt++;
  draw();
}

class CB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) {
    size_t plen = 0; const uint8_t* pl = nullptr;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    std::string s = dev.getPayload(); plen = s.size(); pl = (const uint8_t*)s.data();
#else
    plen = dev.getPayloadLength(); pl = dev.getPayload();
#endif
    handleAdv(pl, plen, dev.getAddress().toString().c_str(), (int8_t)dev.getRSSI());
  }
};

void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(PIN_BOOT, INPUT_PULLUP);
  memset(g_w, 0, sizeof(g_w));
  lcd.init();
  lcd.setBrightness(255);
  draw();
  Serial.printf("\r\n=== adv parser ===\r\n");
  BLEDevice::init("id3-tpms-adv");
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
  if (btn && !lastBtn) { memset(g_w, 0, sizeof(g_w)); g_nw = 0; g_frames = 0; g_lastHex[0] = 0; draw(); }
  lastBtn = btn;

  static uint32_t t = 0;
  if (millis() - t > 700) { t = millis(); draw(); }
  delay(5);
}
