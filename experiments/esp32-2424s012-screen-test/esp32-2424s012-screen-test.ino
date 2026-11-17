/*
 * esp32-2424s012-screen-test —— 只干一件事：把这块板的圆屏点亮，证明屏幕是好的。
 *
 * 板型：ESP32-2424S012（ESP32-C3 + GC9A01 240x240 圆屏 + CST816D 触摸）
 * 引脚（来自社区已验证的板级配置）：
 *   LCD  SCLK=6  MOSI=7  DC=2  CS=10  BL=3   RST 板上直连复位(-1)
 *   触摸 SDA=4  SCL=5  INT=0  RST=1  (addr 0x15)  —— 本测试不用触摸
 * 依赖：LovyanGFX
 *
 * 现象说明：串口会依次打印 red/green/blue，屏幕应跟着闪这三种颜色，
 *           最后停在深色圆盘界面并显示 SCREEN OK。
 * 如果串口有日志但屏幕全黑 → 背光极性或引脚问题，把日志发我。
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
    {
      auto c = _bus.config();
      c.spi_host   = SPI2_HOST;
      c.spi_mode   = 0;
      c.freq_write = 40000000;
      c.freq_read  = 16000000;
      c.pin_sclk   = PIN_LCD_SCLK;
      c.pin_mosi   = PIN_LCD_MOSI;
      c.pin_miso   = -1;
      c.pin_dc     = PIN_LCD_DC;
      _bus.config(c);
      _panel.setBus(&_bus);
    }
    {
      auto c = _panel.config();
      c.pin_cs       = PIN_LCD_CS;
      c.pin_rst      = -1;      // 板上直连复位
      c.panel_width  = 240;
      c.panel_height = 240;
      c.offset_x     = 0;
      c.offset_y     = 0;
      c.readable     = false;
      c.invert       = true;    // GC9A01 通常需要反色
      c.rgb_order    = false;
      _panel.config(c);
    }
    {
      auto c = _light.config();
      c.pin_bl      = PIN_LCD_BL;
      c.freq        = 12000;
      c.pwm_channel = 0;      // 关键：ESP32-C3 的 LEDC 只有 0~5 通道，写 7 会静默失败 → 背光不亮
      _light.config(c);
      _panel.setLight(&_light);
    }
    setPanel(&_panel);
  }
};

static LGFX lcd;

static void solid(uint16_t color, const char* name) {
  Serial.printf("[点屏] %s\r\n", name);
  lcd.fillScreen(color);
  delay(600);
}

static void finalScreen() {
  lcd.fillScreen(TFT_BLACK);

  // 外环（验证圆屏边缘和圆心对齐）
  lcd.drawCircle(120, 120, 116, lcd.color565(160, 40, 200));
  lcd.drawCircle(120, 120, 114, lcd.color565(90, 20, 120));

  // 顶部方向标记：文字正着 = 方向正确
  lcd.setTextDatum(textdatum_t::top_center);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.setTextSize(2);
  lcd.drawString("SCREEN OK", 120, 44);

  lcd.setTextSize(1);
  lcd.setTextColor(lcd.color565(0, 220, 180), TFT_BLACK);
  lcd.drawString("ESP32-2424S012", 120, 78);
  lcd.setTextColor(lcd.color565(180, 180, 180), TFT_BLACK);
  lcd.drawString("GC9A01 240x240", 120, 96);

  // 中间一个大数字，验证字库与缩放
  lcd.setTextSize(5);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.setTextDatum(textdatum_t::middle_center);
  lcd.drawString("2.6", 120, 138);

  lcd.setTextSize(1);
  lcd.setTextColor(lcd.color565(200, 200, 60), TFT_BLACK);
  lcd.setTextDatum(textdatum_t::middle_center);
  lcd.drawString("bar", 120, 172);

  // 底部刻度弧，验证圆的几何
  for (int a = 150; a <= 390; a += 10) {
    float r = a * DEG_TO_RAD;
    float rr = (a % 30 == 0) ? 100.0f : 106.0f;
    int x1 = 120 + (int)(cosf(r) * rr);
    int y1 = 120 + (int)(sinf(r) * rr);
    int x2 = 120 + (int)(cosf(r) * 112.0f);
    int y2 = 120 + (int)(sinf(r) * 112.0f);
    lcd.drawLine(x1, y1, x2, y2, lcd.color565(60, 60, 90));
  }
}

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.printf("\r\n=== screen test: ESP32-2424S012 ===\r\n");
  Serial.printf("SDK=%s  free heap=%u\r\n", ESP.getSdkVersion(), (unsigned)ESP.getFreeHeap());

  lcd.init();
  lcd.setBrightness(220);
  Serial.printf("init done, width=%d height=%d\r\n", (int)lcd.width(), (int)lcd.height());

  solid(TFT_RED,   "1/3 red");
  solid(TFT_GREEN, "2/3 green");
  solid(TFT_BLUE,  "3/3 blue");

  finalScreen();
  Serial.printf("静态界面已绘制完成。屏幕应该：深色底 + 紫色外环 + 顶部 SCREEN OK + 中间 2.6 bar\r\n");
  Serial.printf("若全黑：请把这段日志发我，可能是背光极性/引脚差异。\r\n");
  Serial.printf("free heap after init=%u\r\n", (unsigned)ESP.getFreeHeap());
}

void loop() {
  static uint32_t last = 0;
  if (millis() - last > 3000) {
    last = millis();
    Serial.printf("[alive] heap=%u\r\n", (unsigned)ESP.getFreeHeap());
  }
  delay(50);
}
