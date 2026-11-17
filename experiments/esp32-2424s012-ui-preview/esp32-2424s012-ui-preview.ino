/*
 * esp32-2424s012-ui-preview —— 胎压界面版式预览（占位数据，外环 r=119）
 *
 * 外环：r=117~119，3px 粗，紫罗兰色（对应参考图里那圈）
 * 布局：四角四轮数值 + 中心俯视车模 + 顶部状态 + 底部电量/时效
 * 串口：ring <r> 单独调外环半径；ui 回到预览
 */

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <Arduino.h>

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

static const uint16_t C_RING    = 0xB01FE0;   // 外环紫罗兰
static const uint16_t C_RING_IN = 0x50107;    // 内圈暗紫
static const uint16_t C_BG      = TFT_BLACK;
static const uint16_t C_VAL     = TFT_WHITE;
static const uint16_t C_UNIT    = 0x8C71;
static const uint16_t C_OK      = 0x07E0;
static const uint16_t C_CAR     = 0xCE79;

static void drawRing() {
  lcd.drawArc(120, 120, 117, 119, 0, 359, C_RING);      // 外环贴边（外沿 119）
  lcd.drawArc(120, 120, 113, 113, 0, 359, C_RING_IN);   // 内侧细线，增加层次
}

static void wheelValue(int cx, int cy, const char* v, bool warn) {
  lcd.setTextDatum(textdatum_t::middle_center);
  lcd.setTextSize(3);
  lcd.setTextColor(warn ? 0xFD20 : C_VAL, C_BG);
  lcd.drawString(v, cx, cy);
  lcd.setTextSize(1);
  lcd.setTextColor(C_UNIT, C_BG);
  lcd.drawString("bar", cx, cy + 20);
}

static void drawCar() {
  // 俯视车模：车身 + 四轮
  lcd.fillRoundRect(106, 98, 28, 46, 7, C_CAR);
  lcd.fillRoundRect(100, 104, 6, 12, 2, C_CAR);
  lcd.fillRoundRect(134, 104, 6, 12, 2, C_CAR);
  lcd.fillRoundRect(100, 128, 6, 12, 2, C_CAR);
  lcd.fillRoundRect(134, 128, 6, 12, 2, C_CAR);
  // 前挡风
  lcd.fillRoundRect(110, 104, 20, 9, 3, 0x2104);
}

static void drawBattery(int x, int y, int pct, bool ok) {
  uint16_t col = ok ? C_OK : 0xFD20;
  lcd.drawRect(x, y, 16, 8, col);
  lcd.fillRect(x + 16, y + 2, 2, 4, col);
  int w = (14 * pct) / 100;
  lcd.fillRect(x + 1, y + 1, w, 6, col);
}

static void drawUi() {
  lcd.fillScreen(C_BG);
  drawRing();

  // 顶部状态
  lcd.setTextDatum(textdatum_t::top_center);
  lcd.setTextSize(2);
  lcd.setTextColor(C_OK, C_BG);
  lcd.drawString("ALL OK", 120, 40);

  // 四轮数值（四角）
  wheelValue(76, 84, "2.6", false);
  wheelValue(164, 84, "2.6", false);
  wheelValue(76, 156, "2.6", false);
  wheelValue(164, 156, "2.6", false);

  drawCar();

  // 底部：车标 + 电量 + 时效
  lcd.setTextDatum(textdatum_t::middle_center);
  lcd.setTextSize(1);
  lcd.setTextColor(C_UNIT, C_BG);
  lcd.drawString("ID.3", 120, 190);
  drawBattery(148, 198, 86, true);
  lcd.setTextDatum(textdatum_t::middle_left);
  lcd.drawString("1min", 92, 202);

  Serial.printf("UI 预览已绘制（外环外沿 r=119）\r\n");
}

static char    g_line[32];
static uint8_t g_len = 0;

static void pollSerial() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch != '\n') { if (g_len < sizeof(g_line) - 1) g_line[g_len++] = ch; continue; }
    g_line[g_len] = 0; g_len = 0;
    String cmd(g_line); cmd.trim();
    if (cmd.length() == 0) continue;
    if (cmd == "ui") { drawUi(); }
    else if (cmd.startsWith("ring ")) {
      int r = cmd.substring(5).toInt();
      if (r >= 100 && r <= 122) {
        lcd.fillScreen(C_BG);
        lcd.drawArc(120, 120, r - 2, r, 0, 359, C_RING);
        lcd.setTextDatum(textdatum_t::middle_center);
        lcd.setTextSize(3); lcd.setTextColor(TFT_WHITE, C_BG);
        lcd.drawString(String(r).c_str(), 120, 120);
        Serial.printf("外环外沿 r=%d\r\n", r);
      }
    } else Serial.printf("命令：ui | ring <r>\r\n");
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  lcd.init();
  lcd.setBrightness(255);
  drawUi();
  Serial.printf("\r\n=== ui preview (ring outer r=119) ===\r\n");
}

void loop() {
  pollSerial();
  delay(20);
}
