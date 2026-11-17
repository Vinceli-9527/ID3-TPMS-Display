/*
 * esp32-2424s012-quad —— 4 个候选外环半径并排对比
 * 四个象限各画一个不同半径的外环（半径随象限递增），中心标出象限对应的半径。
 * 目的：一眼挑出"贴边但不被裁切"的外环半径。
 *
 * 串口：ring <r>  画完整外环（最终确认用）；quad 回到对比界面
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

// 每个象限的候选半径（顺时针：右上→右下→左下→左上）
static const int   Q_R[4]  = {116, 117, 118, 119};
static const uint16_t Q_C[4] = {0x07FF, 0x07E0, 0xFFE0, 0xF81F};   // 青 绿 黄 品红

static void drawQuad() {
  lcd.fillScreen(TFT_BLACK);

  for (int q = 0; q < 4; q++) {
    float a0 = q * 90.0f - 45.0f;      // 每象限中心对齐坐标轴，便于比较
    float a1 = a0 + 88.0f;
    lcd.drawArc(120, 120, Q_R[q] - 1, Q_R[q] + 1, a0, a1, Q_C[q]);   // 2px 粗
    // 端点小方块，方便看清是否被裁
    for (float a = a0; a <= a1; a += 44.0f) {
      float rd = a * DEG_TO_RAD;
      int x = 120 + (int)(cosf(rd) * Q_R[q]);
      int y = 120 + (int)(sinf(rd) * Q_R[q]);
      lcd.fillRect(x - 1, y - 1, 3, 3, Q_C[q]);
    }
    // 象限标签
    float am = (a0 + a1) / 2.0f * DEG_TO_RAD;
    int lx = 120 + (int)(cosf(am) * (Q_R[q] - 22));
    int ly = 120 + (int)(sinf(am) * (Q_R[q] - 22));
    lcd.setTextDatum(textdatum_t::middle_center);
    lcd.setTextSize(2);
    lcd.setTextColor(Q_C[q], TFT_BLACK);
    lcd.drawString(String(Q_R[q]).c_str(), lx, ly);
  }

  lcd.setTextDatum(textdatum_t::middle_center);
  lcd.setTextSize(1);
  lcd.setTextColor(0x8410, TFT_BLACK);
  lcd.drawString("pick", 120, 120);
  lcd.drawString("edge", 120, 132);
}

static void drawRing(int r) {
  lcd.fillScreen(TFT_BLACK);
  lcd.drawArc(120, 120, r - 1, r + 1, 0, 359, 0xF81F);
  lcd.setTextDatum(textdatum_t::middle_center);
  lcd.setTextSize(3);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.drawString(String(r).c_str(), 120, 120);
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
    if (cmd == "quad") { drawQuad(); Serial.printf("对比界面\r\n"); }
    else if (cmd.startsWith("ring ")) {
      int r = cmd.substring(5).toInt();
      if (r >= 100 && r <= 125) { drawRing(r); Serial.printf("外环 r=%d\r\n", r); }
      else Serial.printf("范围 100~125\r\n");
    } else Serial.printf("命令：quad | ring <r>\r\n");
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  lcd.init();
  lcd.setBrightness(255);
  drawQuad();
  Serial.printf("\r\n=== quad compare ===\r\n右上=116 右下=117 左下=118 左上=119\r\n");
}

void loop() {
  pollSerial();
  delay(20);
}
