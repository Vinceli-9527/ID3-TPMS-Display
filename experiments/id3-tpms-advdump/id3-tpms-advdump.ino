/*
 * id3-tpms-advdump —— 把 FBB0 设备的完整广播原始字节画在屏幕上
 *
 * 依据：APK 里类名 com.lytpms.ble.TireAdvertisementParser + Crc16
 *       → 数据在 BLE 广播包内，不需要连接（连接失败不影响取数）
 *
 * 屏幕：顶部 ADV n（找到几个）
 *       当前展示设备的 MAC 尾段 + RSSI
 *       完整 adv 十六进制（每行 20 字节，最多 4 行 = 80 字节）
 *       BOOT 键切换上一个/下一个设备
 * 串口：每收到一帧都打印 [ADV] 完整 hex
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

#define MAXD 4
#define MAXLEN 80

struct ADev {
  bool     used;
  char     mac[18];
  int8_t   rssi;
  uint8_t  len;
  uint8_t  data[MAXLEN];
  uint32_t n;
};
static ADev g_d[MAXD];
static uint8_t g_nd = 0;
static int8_t  g_show = 0;      // 当前展示第几个
static uint32_t g_lastDraw = 0;

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
    } else if (type == 0xFF && dlen > 2) {
      // 有些厂家把数据放厂商数据里：全包再查一次 FBB0 字样
      for (uint8_t k = 0; k + 1 < dlen; k++) if (d[k] == 0xB0 && d[k + 1] == 0xFB) return true;
    }
    i += (size_t)len + 1;
  }
  return false;
}

static void drawScreen() {
  lcd.fillScreen(TFT_BLACK);
  lcd.drawArc(120, 120, 117, 119, 0, 359, g_nd ? 0x07E0 : 0xB01FE0);

  lcd.setTextDatum(textdatum_t::top_center);
  lcd.setTextSize(2);
  lcd.setTextColor(0x07FF, TFT_BLACK);
  char h[20];
  snprintf(h, sizeof(h), "ADV %u/%d", g_nd, MAXD);
  lcd.drawString(h, 120, 18);

  if (g_nd == 0) {
    lcd.setTextSize(1);
    lcd.setTextColor(0x8C71, TFT_BLACK);
    lcd.drawString("screw sensor on tire", 120, 110);
    return;
  }
  if (g_show >= g_nd) g_show = 0;
  ADev& d = g_d[g_show];

  lcd.setTextSize(1);
  lcd.setTextDatum(textdatum_t::top_left);
  char line[40];
  snprintf(line, sizeof(line), "..%s %d n%lu", d.mac + 9, d.rssi, (unsigned long)d.n);
  lcd.setTextColor(0xFFE0, TFT_BLACK);
  lcd.drawString(line, 12, 40);

  lcd.setTextColor(0xFFFF, TFT_BLACK);
  for (int r = 0; r < 4; r++) {
    char buf[41];
    int k = 0;
    for (int i = r * 20; i < d.len && i < (r + 1) * 20 && k < 40; i++) {
      k += snprintf(buf + k, sizeof(buf) - k, "%02X ", d.data[i]);
    }
    buf[k] = 0;
    if (!k) break;
    lcd.drawString(buf, 8, 58 + r * 22);
  }

  lcd.setTextDatum(textdatum_t::top_center);
  lcd.setTextColor(0xFD20, TFT_BLACK);
  lcd.drawString("BOOT = next", 120, 196);
}

class AdvCB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) {
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

    int idx = -1;
    for (int i = 0; i < g_nd; i++) if (strcmp(g_d[i].mac, mac) == 0) { idx = i; break; }
    if (idx < 0) {
      if (g_nd >= MAXD) return;
      idx = g_nd++;
      memset(&g_d[idx], 0, sizeof(ADev));
      g_d[idx].used = true;
      strncpy(g_d[idx].mac, mac, sizeof(g_d[idx].mac) - 1);
    }
    ADev& d = g_d[idx];
    d.rssi = (int8_t)dev.getRSSI();
    d.len = plen < MAXLEN ? (uint8_t)plen : MAXLEN;
    memcpy(d.data, pl, d.len);
    d.n++;

    char hex[3 * MAXLEN + 4]; size_t k = 0; hex[0] = 0;
    for (uint8_t i = 0; i < d.len && k + 4 < sizeof(hex); i++) k += snprintf(hex + k, sizeof(hex) - k, "%02X ", d.data[i]);
    Serial.printf("[ADV] %s rssi=%d len=%u %s\r\n", mac, d.rssi, d.len, hex);
  }
};

void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(PIN_BOOT, INPUT_PULLUP);
  memset(g_d, 0, sizeof(g_d));
  lcd.init();
  lcd.setBrightness(255);
  drawScreen();
  Serial.printf("\r\n=== advdump ===\r\n");

  BLEDevice::init("tpms-advdump");
  BLEScan* scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new AdvCB(), true);
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(90);
}

void loop() {
  BLEScan* scan = BLEDevice::getScan();
  scan->start(1);
  scan->clearResults();

  static bool lastBtn = false;
  bool btn = (digitalRead(PIN_BOOT) == LOW);
  if (btn && !lastBtn && g_nd > 0) { g_show = (g_show + 1) % g_nd; drawScreen(); }
  lastBtn = btn;

  if (millis() - g_lastDraw > 900) { g_lastDraw = millis(); drawScreen(); }
  delay(10);
}
