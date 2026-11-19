/*
 * touch-probe —— CST816D 触摸验证
 * I2C: SDA=4 SCL=5 addr=0x15
 * 读 0x01 起 6 字节：gesture, fingerNum, xH, xL, yH, yL
 * 关闭自动休眠：写 0xFE = 0x01
 */

#include <Wire.h>
#include <Arduino.h>
static TwoWire tp(0);   // 必须在 LovyanGFX 之前，否则 TwoWire 不可见

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

#define PIN_LCD_SCLK 6
#define PIN_LCD_MOSI 7
#define PIN_LCD_DC   2
#define PIN_LCD_CS   10
#define PIN_LCD_BL   3
#define TP_SDA 4
#define TP_SCL 5
#define TP_RST 1
#define TP_ADDR 0x15

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

static int  g_addr = -1;
static int  g_fingers = 0;
static int  g_x = 0, g_y = 0;
static uint16_t g_taps = 0;
static bool g_touched = false;
static uint32_t g_lastTouch = 0;

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

static int i2cScan() {
  int found = -1;
  for (uint8_t a = 1; a < 127; a++) {
    tp.beginTransmission(a);
    if (tp.endTransmission() == 0) {
      Serial.printf("[I2C] 发现设备 0x%02X\r\n", a);
      if (a == TP_ADDR) found = a;
    }
  }
  return found;
}

static bool tpInit() {
  pinMode(TP_RST, OUTPUT);
  digitalWrite(TP_RST, LOW); delay(10);
  digitalWrite(TP_RST, HIGH); delay(60);
  tp.beginTransmission(TP_ADDR);
  tp.write(0xFE); tp.write(0x01);        // 关闭自动休眠
  if (tp.endTransmission() != 0) return false;
  delay(10);
  return true;
}

static bool tpRead(int& fingers, int& x, int& y) {
  tp.beginTransmission(TP_ADDR);
  tp.write(0x01);
  if (tp.endTransmission(false) != 0) return false;
  if (tp.requestFrom((uint8_t)TP_ADDR, (uint8_t)6) != 6) return false;
  uint8_t b[6];
  for (int i = 0; i < 6; i++) b[i] = tp.read();
  fingers = b[1] & 0x0F;
  x = ((b[2] & 0x0F) << 8) | b[3];
  y = ((b[4] & 0x0F) << 8) | b[5];
  return fingers > 0;
}

static void draw() {
  lcd.fillScreen(TFT_BLACK);
  lcd.drawArc(120, 120, 119, 119, 0, 359, 0x9400D3);
  fitted(g_addr > 0 ? "TOUCH OK" : "NO TOUCH IC", 26, g_addr > 0 ? 0x07E0 : 0xF800, 2);

  char l[32];
  snprintf(l, sizeof(l), "addr 0x%02X", g_addr > 0 ? g_addr : 0);
  fitted(l, 50, 0x8C71);
  snprintf(l, sizeof(l), "x %d  y %d", g_x, g_y);
  fitted(l, 176, 0xFFFF, 2);
  snprintf(l, sizeof(l), "taps %u  f%d", g_taps, g_fingers);
  fitted(l, 204, 0xFFE0);

  if (g_touched) {
    lcd.fillCircle(g_x, g_y, 10, 0x07E0);
    lcd.drawCircle(g_x, g_y, 14, 0x0320);
  } else if (g_lastTouch && millis() - g_lastTouch < 800) {
    lcd.drawCircle(g_x, g_y, 12, 0x5AEB);
  }
  fitted("tap anywhere", 150, 0x8C71);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  lcd.init();
  lcd.setBrightness(240);
  lcd.fillScreen(TFT_BLACK);
  fitted("scanning I2C...", 120, 0xFFFF, 2);

  tp.begin(TP_SDA, TP_SCL);
  tp.setClock(400000);
  g_addr = i2cScan();
  Serial.printf("[结果] 触摸芯片 %s (0x%02X)\r\n", g_addr > 0 ? "找到" : "未找到", g_addr > 0 ? g_addr : 0);
  if (g_addr > 0) {
    Serial.printf("[初始化] %s\r\n", tpInit() ? "ok（已关闭自动休眠）" : "写寄存器失败");
  }
  draw();
}

void loop() {
  static uint32_t redraw = 0;
  if (g_addr > 0) {
    int f = 0, x = 0, y = 0;
    if (tpRead(f, x, y)) {
      if (!g_touched) g_taps++;
      g_touched = true;
      g_fingers = f; g_x = x; g_y = y;
      g_lastTouch = millis();
      Serial.printf("[触摸] x=%d y=%d f=%d taps=%u\r\n", x, y, f, g_taps);
      draw();
    } else if (g_touched) {
      g_touched = false;
      g_fingers = 0;
      draw();
    }
  }
  if (millis() - redraw > 1500) { redraw = millis(); if (!g_touched) draw(); }
  delay(20);
}
