/*
 * esp32-2424s012-edge —— 圆屏可视边界标尺
 *
 * 一次画出多圈同心圆 + 中心图例，用来判断"屏幕边缘到底在哪"。
 * 背光：GPIO3，高电平有效；LEDC 通道必须 0~5（填 7 会静默失败）。
 *
 * 串口命令（115200，换行）：
 *   r <n>   只画半径为 n 的单圈，并在圈上标数字（细调用）
 *   all     回到多圈标尺界面
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
      c.pin_bl = PIN_LCD_BL; c.freq = 12000; c.pwm_channel = 0;   // C3 只有 0~5
      _light.config(c); _panel.setLight(&_light); }
    setPanel(&_panel);
  }
};

static LGFX lcd;

struct Ring { int r; uint16_t color; const char* name; };

static Ring g_rings[] = {
  {120, 0xF800, "120 最外"},
  {119, 0xFFE0, "119"},
  {118, 0x07E0, "118"},
  {116, 0x07FF, "116"},
  {114, 0xF81F, "114"},
  {110, 0xFD20, "110"},
  {104, 0xFFFF, "104 最内"},
};
static const int RING_N = sizeof(g_rings) / sizeof(g_rings[0]);

static void drawLegend() {
  int y = 62;
  lcd.setTextSize(1);
  for (int i = 0; i < RING_N; i++) {
    lcd.fillRect(84, y - 3, 8, 8, g_rings[i].color);
    lcd.setTextDatum(textdatum_t::middle_left);
    lcd.setTextColor(g_rings[i].color, TFT_BLACK);
    lcd.drawString(g_rings[i].name, 98, y + 1);
    y += 15;
  }
  lcd.setTextDatum(textdatum_t::top_center);
  lcd.setTextColor(0x7BEF, TFT_BLACK);
  lcd.drawString("最外可见 = ?", 120, 176);
}

static void drawAll() {
  lcd.fillScreen(TFT_BLACK);
  for (int i = 0; i < RING_N; i++) {
    lcd.drawCircle(120, 120, g_rings[i].r, g_rings[i].color);
    // 四个方向的短标记，便于看清每圈
    lcd.fillRect(120 - 1, 120 - g_rings[i].r - 1, 3, 3, g_rings[i].color);
    lcd.fillRect(120 - 1, 120 + g_rings[i].r - 1, 3, 3, g_rings[i].color);
    lcd.fillRect(120 - g_rings[i].r - 1, 120 - 1, 3, 3, g_rings[i].color);
    lcd.fillRect(120 + g_rings[i].r - 1, 120 - 1, 3, 3, g_rings[i].color);
  }
  drawLegend();
}

static void drawSingle(int r) {
  lcd.fillScreen(TFT_BLACK);
  lcd.drawCircle(120, 120, r, TFT_GREEN);
  lcd.fillRect(120 - 1, 120 - r - 1, 3, 3, TFT_GREEN);
  lcd.fillRect(120 - 1, 120 + r - 1, 3, 3, TFT_GREEN);
  lcd.fillRect(120 - r - 1, 120 - 1, 3, 3, TFT_GREEN);
  lcd.fillRect(120 + r - 1, 120 - 1, 3, 3, TFT_GREEN);
  lcd.setTextDatum(textdatum_t::middle_center);
  lcd.setTextSize(3);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.drawString(String(r).c_str(), 120, 120);
  lcd.setTextSize(1);
  lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  lcd.drawString("radius", 120, 148);
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
    if (cmd == "all") { drawAll(); Serial.printf("多圈标尺已重画\r\n"); }
    else if (cmd.startsWith("r ")) {
      int r = cmd.substring(2).toInt();
      if (r >= 10 && r <= 130) { drawSingle(r); Serial.printf("单圈 r=%d\r\n", r); }
      else Serial.printf("半径范围 10~130\r\n");
    } else Serial.printf("命令：r <n> | all\r\n");
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  lcd.init();
  lcd.setBrightness(255);
  drawAll();
  Serial.printf("\r\n=== edge ruler ===\r\ninit ok %dx%d\r\n", (int)lcd.width(), (int)lcd.height());
  Serial.printf("命令：r <n> 单圈 | all 多圈\r\n");
}

void loop() {
  pollSerial();
  delay(20);
}
