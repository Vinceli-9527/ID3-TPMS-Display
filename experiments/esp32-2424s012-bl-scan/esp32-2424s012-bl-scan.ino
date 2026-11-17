/*
 * esp32-2424s012-bl-scan —— 背光排查固件
 *
 * 现象：面板初始化成功(240x240)但屏幕完全无光 → 背光没通。
 * 做法：把候选 GPIO 挨个当背光用，第 N 个候选引脚就闪 N 下：
 *        第 1 组（高电平有效）：引脚 1..9  → 分别闪 1..9 下
 *        间隔 3 秒全黑
 *        第 2 组（低电平有效）：同样的引脚 → 分别闪 1..9 下
 *      你只要记住"在哪一组、闪了几下"，就能反推出引脚和极性。
 *
 * 注意：C3 的 GPIO18/19/20/21 是 USB，绝不能驱动，已排除。
 *
 * 串口命令（115200，换行结尾）：
 *   h        帮助
 *   scan     开始闪烁排查（开机不自动跑，避免和别的日志混在一起）
 *   bl <pin> 直接把某个引脚当背光拉满
 *   draw     重画测试图案
 *   inv      切换背光极性后重设
 */

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <Arduino.h>

/* ---- 候选背光引脚（已排除 USB 18/19/20/21 与 SPI 6/7）---- */
static const uint8_t CAND[] = {3, 2, 4, 5, 10, 0, 1, 8, 9};
static const uint8_t CAND_N = sizeof(CAND) / sizeof(CAND[0]);

static int8_t  g_blPin  = 3;      // 社区资料写的背光脚
static bool    g_blInv  = false;  // 是否低电平有效

#define PIN_LCD_SCLK 6
#define PIN_LCD_MOSI 7
#define PIN_LCD_DC   2
#define PIN_LCD_CS   10

class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_GC9A01 _panel;
  lgfx::Bus_SPI      _bus;
  lgfx::Light_PWM    _light;
 public:
  LGFX() {
    {
      auto c = _bus.config();
      c.spi_host = SPI2_HOST; c.spi_mode = 0;
      c.freq_write = 40000000; c.freq_read = 16000000;
      c.pin_sclk = PIN_LCD_SCLK; c.pin_mosi = PIN_LCD_MOSI;
      c.pin_miso = -1; c.pin_dc = PIN_LCD_DC;
      _bus.config(c); _panel.setBus(&_bus);
    }
    {
      auto c = _panel.config();
      c.pin_cs = PIN_LCD_CS; c.pin_rst = -1;
      c.panel_width = 240; c.panel_height = 240;
      c.readable = false; c.invert = true; c.rgb_order = false;
      _panel.config(c);
    }
    {
      auto c = _light.config();
      c.pin_bl = g_blPin; c.freq = 12000; c.pwm_channel = 7; c.invert = g_blInv;
      _light.config(c); _panel.setLight(&_light);
    }
    setPanel(&_panel);
  }
};

static LGFX lcd;

static void drawPattern() {
  lcd.fillScreen(TFT_BLACK);
  lcd.drawCircle(120, 120, 116, lcd.color565(160, 40, 200));
  lcd.setTextDatum(textdatum_t::top_center);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.setTextSize(2);
  lcd.drawString("BL SCAN", 120, 60);
  lcd.setTextSize(1);
  lcd.setTextColor(lcd.color565(0, 220, 180), TFT_BLACK);
  lcd.drawString("240x240 GC9A01", 120, 96);
  lcd.setTextSize(4);
  lcd.setTextColor(lcd.color565(255, 210, 0), TFT_BLACK);
  lcd.setTextDatum(textdatum_t::middle_center);
  lcd.drawString("2.6", 120, 140);
  lcd.setTextSize(1);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.drawString("bar", 120, 172);
}

static void pinAllLow() {
  for (uint8_t i = 0; i < CAND_N; i++) {
    pinMode(CAND[i], OUTPUT);
    digitalWrite(CAND[i], LOW);
  }
}

static void blinkPin(uint8_t pin, uint8_t times, bool activeHigh) {
  pinAllLow();
  pinMode(pin, OUTPUT);
  for (uint8_t k = 0; k < times; k++) {
    digitalWrite(pin, activeHigh ? HIGH : LOW);
    if (!activeHigh) digitalWrite(pin, LOW);   // 已在 LOW，保持
    delay(180);
    digitalWrite(pin, activeHigh ? LOW : HIGH);
    delay(180);
  }
  pinAllLow();
}

static void runScan() {
  Serial.printf("\r\n===== 第 1 组：高电平有效 =====\r\n");
  for (uint8_t i = 0; i < CAND_N; i++) {
    Serial.printf("[1] pin=%-2u 闪 %u 下\r\n", CAND[i], i + 1);
    blinkPin(CAND[i], i + 1, true);
    delay(600);
  }
  Serial.printf("---- 间隔 3 秒（全黑）----\r\n");
  delay(3000);

  Serial.printf("===== 第 2 组：低电平有效 =====\r\n");
  for (uint8_t i = 0; i < CAND_N; i++) {
    Serial.printf("[2] pin=%-2u 闪 %u 下\r\n", CAND[i], i + 1);
    pinAllLow();
    pinMode(CAND[i], OUTPUT);
    digitalWrite(CAND[i], HIGH);           // 先拉高=灭
    for (uint8_t k = 0; k < i + 1; k++) {
      digitalWrite(CAND[i], LOW);          // 低=亮
      delay(180);
      digitalWrite(CAND[i], HIGH);
      delay(180);
    }
    delay(600);
  }
  pinAllLow();
  Serial.printf("扫描结束。记住你看到的：第几组、闪几下。\r\n");
}

static char    g_line[48];
static uint8_t g_len = 0;

static void pollSerial() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch != '\n') { if (g_len < sizeof(g_line) - 1) g_line[g_len++] = ch; continue; }
    g_line[g_len] = 0; g_len = 0;
    String cmd(g_line); cmd.trim();
    if (cmd.length() == 0) continue;
    if (cmd == "h") {
      Serial.printf("命令：all1/all0 全部候选脚拉高/拉低 | set 3,2,4 1 指定脚 | scan 闪烁排查 | bl <pin> | draw | inv\r\n");
    } else if (cmd == "scan") {
      runScan();
    } else if (cmd.startsWith("bl ")) {
      int p = cmd.substring(3).toInt();
      g_blPin = (int8_t)p;
      pinMode(p, OUTPUT);
      digitalWrite(p, g_blInv ? LOW : HIGH);
      Serial.printf("背光脚设为 GPIO%d（%s），已拉满\r\n", p, g_blInv ? "低有效" : "高有效");
      drawPattern();
    } else if (cmd == "all1" || cmd == "all0") {
      bool lvl = (cmd == "all1");
      for (uint8_t i = 0; i < CAND_N; i++) { pinMode(CAND[i], OUTPUT); digitalWrite(CAND[i], lvl ? HIGH : LOW); }
      Serial.printf("全部候选脚 = %s（%u 个）\r\n", lvl ? "HIGH" : "LOW", CAND_N);
    } else if (cmd.startsWith("set ")) {
      String rest = cmd.substring(4);
      int lastSp = rest.lastIndexOf(' ');
      String pins = (lastSp > 0) ? rest.substring(0, lastSp) : rest;
      int lvl = (lastSp > 0) ? rest.substring(lastSp + 1).toInt() : 1;
      pinAllLow();
      int start = 0;
      for (int i = 0; i <= (int)pins.length(); i++) {
        if (i == (int)pins.length() || pins[i] == ',') {
          int pn = pins.substring(start, i).toInt();
          pinMode(pn, OUTPUT); digitalWrite(pn, lvl ? HIGH : LOW);
          start = i + 1;
        }
      }
      Serial.printf("设置 [%s] = %d\r\n", pins.c_str(), lvl);
    } else if (cmd == "draw") {
      drawPattern();
      Serial.printf("已重画\r\n");
    } else if (cmd == "inv") {
      g_blInv = !g_blInv;
      pinMode(g_blPin, OUTPUT);
      digitalWrite(g_blPin, g_blInv ? LOW : HIGH);
      Serial.printf("背光极性切换为：%s\r\n", g_blInv ? "低电平有效" : "高电平有效");
    } else {
      Serial.printf("未知命令 %s\r\n", cmd.c_str());
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.printf("\r\n=== BL scan: ESP32-2424S012 ===\r\n");
  Serial.printf("SDK=%s heap=%u\r\n", ESP.getSdkVersion(), (unsigned)ESP.getFreeHeap());

  lcd.init();
  lcd.setBrightness(255);
  Serial.printf("panel init ok %dx%d, BL=GPIO%d\r\n", (int)lcd.width(), (int)lcd.height(), (int)g_blPin);
  drawPattern();
  Serial.printf("图案已画。屏幕没光就发 scan 命令，然后数闪几下。\r\n");
}

void loop() {
  pollSerial();
  delay(20);
}
