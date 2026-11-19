/*
 * id3-tpms-ui v0.5.0 —— 触摸 + 轮位绑定 + 分页键盘
 *
 * 页面：MAIN（四轮） / PICK（点选绑定） / KEYPAD（手输，4 页 × 4 大键，可左右滑动换页）
 * 数据：被动监听广播解析（通信格式见 README 第六节）
 *   adv[0..6]=02 01 06 03 03 B0 FB
 *   V=b[4]*0.01+1.22  P=b[5]*3.144kPa(b[10]==1→+256)  T=b[6]-55  ID=b[18]b[17]b[16]
 *   MAC = ID + 11:11:11
 */

#include <Wire.h>
#include <Arduino.h>
static TwoWire tp(0);

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <Preferences.h>
#include "car_img.h"

#define PIN_LCD_SCLK 6
#define PIN_LCD_MOSI 7
#define PIN_LCD_DC   2
#define PIN_LCD_CS   10
#define PIN_LCD_BL   3
#define TP_ADDR 0x15
#define TP_RST  1

#define FW_VERSION   "0.5.0"
#define LOW_BAR      2.0f
#define WARN_BAR     2.2f
#define STALE_MS     600000UL
#define CAND_MS      30000UL
#define MAX_CAND     5

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

struct W    { bool has; char id[8]; float kPa, tempC, volt; uint8_t state; int8_t rssi; uint32_t at; };
struct Cand { char id[8]; int8_t rssi; uint32_t at; };

static W       g_w[4];
static char    g_bind[4][8];
static bool    g_anyBind = false;
static Cand    g_cand[MAX_CAND];
static uint8_t g_nc = 0;
static uint8_t g_nextFree = 0;
static uint32_t g_frames = 0;
static bool    g_blink = false, g_dirty = true;

enum Mode { M_MAIN, M_PICK, M_KEYPAD };
static Mode   g_mode = M_MAIN;
static int8_t g_sel = -1;
static char   g_input[8] = {0};
static uint8_t g_inLen = 0;
static uint8_t g_page = 0;
static bool   g_touchOk = false;
static uint32_t g_lastTap = 0;
static int    g_lx = -1, g_ly = -1, g_lf = 0;
static int    g_sx = 0;
static bool   g_swiped = false, g_pressOnKey = false;
static Preferences g_pref;

static const char* HEXKEYS = "0123456789ABCDEF";

static const uint16_t C_BG = TFT_BLACK;
static const uint16_t C_RING = 0xF81F;
static const uint16_t C_RINGIN = 0x4B0082;
static const uint16_t C_VAL = TFT_WHITE;
static const uint16_t C_DIM = 0x5AEB;
static const uint16_t C_UNIT = 0x8C71;
static const uint16_t C_OK = 0x07E0;
static const uint16_t C_WARN = 0xFD20;
static const uint16_t C_BAD = 0xF800;
static const uint16_t C_CAR = 0xCE79;
static const uint16_t C_TITLE = 0x07FF;
static const uint16_t C_DASH  = 0x07E0;   // 引导线：绿色
static const uint16_t C_BTN = 0x10A2;

static const int CX[4] = {70, 170, 70, 170};
static const int CY[4] = {70, 70, 162, 162};
static const char* POSN[4] = {"LF", "RF", "LR", "RR"};

/* ---------- 触摸 ---------- */
static bool tpInit() {
  pinMode(TP_RST, OUTPUT);
  digitalWrite(TP_RST, LOW); delay(10);
  digitalWrite(TP_RST, HIGH); delay(60);
  tp.beginTransmission(TP_ADDR); tp.write(0xFE); tp.write(0x01);
  return tp.endTransmission() == 0;
}
static bool tpRead(int& f, int& x, int& y) {
  tp.beginTransmission(TP_ADDR); tp.write(0x01);
  if (tp.endTransmission(false) != 0) return false;
  if (tp.requestFrom((uint8_t)TP_ADDR, (uint8_t)6) != 6) return false;
  uint8_t b[6];
  for (int i = 0; i < 6; i++) b[i] = tp.read();
  f = b[1] & 0x0F;
  x = ((b[2] & 0x0F) << 8) | b[3];
  y = ((b[4] & 0x0F) << 8) | b[5];
  return f > 0;
}

/* ---------- 绑定存取 ---------- */
static void loadBind() {
  g_anyBind = false;
  for (int i = 0; i < 4; i++) {
    String v = g_pref.getString((String("b") + i).c_str(), "");
    v.trim(); v.toUpperCase();
    snprintf(g_bind[i], sizeof(g_bind[i]), "%s", v.c_str());
    if (strlen(g_bind[i]) == 6) g_anyBind = true;
  }
}
static void saveBind(int i) { g_pref.putString((String("b") + i).c_str(), g_bind[i]); }
static void recomputeBind() {
  g_anyBind = false;
  for (int k = 0; k < 4; k++) if (strlen(g_bind[k]) == 6) g_anyBind = true;
}
static void clearBind(int i) {
  g_bind[i][0] = 0; saveBind(i);
  recomputeBind();
  g_w[i].has = false;
}

/* ---------- 通用绘制（照效果图重排）---------- */
static void btn(int x, int y, int w, int h, const char* label, uint16_t fill, uint16_t fg) {
  lcd.fillRoundRect(x, y, w, h, 6, fill);
  lcd.drawRoundRect(x, y, w, h, 6, C_RINGIN);
  lcd.setTextDatum(textdatum_t::middle_center);
  lcd.setTextSize(1);
  lcd.setTextColor(fg, fill);
  lcd.drawString(label, x + w / 2, y + h / 2);
}

static void drawTextOutlined(const char* s, int x, int y, uint8_t size, uint16_t outline) {
  lcd.setTextDatum(textdatum_t::middle_center);
  lcd.setTextSize(size);
  lcd.setTextColor(outline, C_BG);
  for (int dx = -1; dx <= 1; dx++)
    for (int dy = -1; dy <= 1; dy++)
      if (dx || dy) lcd.drawString(s, x + dx, y + dy);
  lcd.setTextColor(0xFFFF, C_BG);
  lcd.drawString(s, x, y);
}

static void drawStatic() {
  lcd.fillScreen(C_BG);
  lcd.drawArc(120, 120, 116, 119, 0, 359, C_RING);    // 4px 厚外环
  lcd.drawArc(120, 120, 110, 110, 0, 359, C_RINGIN);  // 内侧细线

  // 俯视车模（PNG 素材，已预乘黑底，车头朝上）
  lcd.pushImage((240 - CAR_IMG_W) / 2, (240 - CAR_IMG_H) / 2, CAR_IMG_W, CAR_IMG_H, car_img);

  drawTextOutlined("ID.3", 120, 208, 3, C_RING);
}

static void drawUnitTemp(int cx, int cy, bool has, float tempC) {
  int y = cy + 17;
  int x0 = cx - 22;
  lcd.setTextDatum(textdatum_t::top_left);
  lcd.setTextSize(1);
  lcd.setTextColor(C_UNIT);
  lcd.drawString("bar", x0, y);

  if (!has) {
    lcd.setTextColor(0x430C);
    lcd.drawString("--", x0 + 21, y);
    return;
  }
  uint16_t tcol = (tempC > 60.0f) ? C_BAD : 0x8C71;
  char t[6];
  snprintf(t, sizeof(t), "%d", (int)lroundf(tempC));
  lcd.setTextColor(tcol);
  lcd.drawString(t, x0 + 21, y);
  int w = (int)strlen(t) * 6;
  lcd.drawCircle(x0 + 21 + w + 2, y + 2, 1, tcol);
  lcd.drawString("C", x0 + 21 + w + 4, y);
}

static void drawCell(int i) {
  int cx = CX[i], cy = CY[i];
  lcd.fillRect(cx - 33, cy - 24, 66, 54, C_BG);

  // 斜向虚线段（车 → 轮位）
  int sx = (i % 2 == 0) ? -1 : 1;
  int sy = (i < 2) ? -1 : 1;
  for (int k = 0; k < 3; k++) {          // 3 段即可，终点半径 46，避开数字区（数字框内缘约 r=48）
    int r0 = 30 + k * 6, r1 = r0 + 4;
    int x0 = 120 + (int)(sx * r0 * 0.707f), y0 = 120 + (int)(sy * r0 * 0.707f);
    int x1 = 120 + (int)(sx * r1 * 0.707f), y1 = 120 + (int)(sy * r1 * 0.707f);
    lcd.drawLine(x0, y0, x1, y1, C_DASH);
  }

  char v[10] = "--";
  uint16_t col = C_DIM;
  if (g_w[i].has) {
    float bar = g_w[i].kPa / 100.0f;
    bool stale = (millis() - g_w[i].at) > STALE_MS;
    snprintf(v, sizeof(v), "%.1f", bar);
    if (stale)               col = C_DIM;
    else if (bar < LOW_BAR)  col = C_BAD;
    else if (bar < WARN_BAR) col = C_WARN;
    else                     col = C_VAL;
  }
  lcd.setTextDatum(textdatum_t::middle_center);
  lcd.setTextSize(4);
  lcd.setTextColor(col);                 // 透明底：不覆盖身后的引导线
  lcd.drawString(v, cx, cy);
  drawUnitTemp(cx, cy, g_w[i].has, g_w[i].tempC);
}

static void drawStatus() {
  int low = 0, warn = 0, live = 0;
  for (int i = 0; i < 4; i++) {
    if (!g_w[i].has) continue;
    if ((millis() - g_w[i].at) > STALE_MS) continue;
    live++;
    float bar = g_w[i].kPa / 100.0f;
    if (bar < LOW_BAR) low++;
    else if (bar < WARN_BAR) warn++;
  }
  lcd.fillRect(92, 8, 58, 32, C_BG);

  // 四个轮位状态点
  for (int i = 0; i < 4; i++) {
    uint16_t c = 0x2104;
    bool fresh = g_w[i].has && (millis() - g_w[i].at) <= STALE_MS;
    if (fresh) {
      float bar = g_w[i].kPa / 100.0f;
      c = (bar < LOW_BAR) ? C_BAD : (bar < WARN_BAR ? C_WARN : C_OK);
    } else if (g_blink) {
      c = 0x0861;
    }
    lcd.fillCircle(102 + i * 12, 16, 3, c);
  }

  const char* txt; uint16_t col;
  if (live == 0)      { txt = "SCAN";   col = C_DIM; }
  else if (low)       { txt = "LOW";    col = C_BAD; }
  else if (warn)      { txt = "CHECK";  col = C_WARN; }
  else                { txt = "ALL OK"; col = 0xCE79; }
  lcd.setTextDatum(textdatum_t::middle_center);
  lcd.setTextSize(1);
  lcd.setTextColor(col, C_BG);
  lcd.drawString(txt, 120, 30);
}

static void drawAge() {
  uint32_t newest = 0;
  for (int i = 0; i < 4; i++) if (g_w[i].at > newest) newest = g_w[i].at;
  char age[12];
  if (newest == 0) snprintf(age, sizeof(age), "--");
  else {
    uint32_t s2 = (millis() - newest) / 1000;
    if (s2 < 120) snprintf(age, sizeof(age), "%lus", (unsigned long)s2);
    else          snprintf(age, sizeof(age), "%lum", (unsigned long)(s2 / 60));
  }
  lcd.fillRect(104, 218, 32, 10, C_BG);
  lcd.setTextDatum(textdatum_t::middle_center);
  lcd.setTextSize(1);
  lcd.setTextColor(0x39E7, C_BG);
  lcd.drawString(age, 120, 223);
}

static void drawMain() {
  drawStatic();
  for (int i = 0; i < 4; i++) drawCell(i);
  drawStatus();
  drawAge();
}


/* ---------- PICK 页 ---------- */
static void drawPick() {
  lcd.fillScreen(C_BG);
  lcd.drawArc(120, 120, 119, 119, 0, 359, C_RING);
  char t[24];
  snprintf(t, sizeof(t), "bind %s", (g_sel >= 0) ? POSN[g_sel] : "?");
  lcd.setTextDatum(textdatum_t::middle_center);
  lcd.setTextSize(2);
  lcd.setTextColor(C_TITLE, C_BG);
  lcd.drawString(t, 120, 22);

  int shown = 0;
  for (int i = 0; i < g_nc && shown < 3; i++) {
    if (millis() - g_cand[i].at > CAND_MS) continue;
    int y = 50 + shown * 26;
    btn(38, y, 164, 22, g_cand[i].id, C_BTN, C_VAL);
    shown++;
  }
  if (shown == 0) {
    lcd.setTextSize(1);
    lcd.setTextColor(C_DIM, C_BG);
    lcd.drawString("no sensor seen yet", 120, 60);
  }
  btn(38, 138, 78, 22, "manual", 0x18C3, C_VAL);
  btn(124, 138, 78, 22, "cancel", 0x4000, C_VAL);
  btn(38, 166, 78, 22, "clear",  0x3000, C_VAL);
  btn(124, 166, 78, 22, "rescan", 0x18C3, C_VAL);

  char dbg[26];
  snprintf(dbg, sizeof(dbg), "last %d,%d f%d", g_lx, g_ly, g_lf);
  lcd.setTextSize(1);
  lcd.setTextColor(0x4208, C_BG);
  lcd.drawString(dbg, 120, 204);
}

/* ---------- KEYPAD 页（4 页 × 4 大键）---------- */
static int8_t keyHit(int x, int y) {
  for (int k = 0; k < 4; k++) {
    int r = k / 2, c = k % 2;
    int bx = 36 + c * 92, by = 58 + r * 48;
    if (x >= bx && x < bx + 76 && y >= by && y < by + 44) return (int8_t)k;
  }
  return -1;
}

static void drawKeypad() {
  lcd.fillScreen(C_BG);
  lcd.drawArc(120, 120, 119, 119, 0, 359, C_RING);

  char t[26];
  snprintf(t, sizeof(t), "%s hex %u/4", (g_sel >= 0) ? POSN[g_sel] : "?", g_page + 1);
  lcd.setTextDatum(textdatum_t::middle_center);
  lcd.setTextSize(1);
  lcd.setTextColor(C_TITLE, C_BG);
  lcd.drawString(t, 120, 20);

  char buf[10];
  for (int i = 0; i < 6; i++) buf[i] = (i < g_inLen) ? g_input[i] : '_';
  buf[6] = 0;
  lcd.setTextSize(3);
  lcd.setTextColor(C_VAL, C_BG);
  lcd.drawString(buf, 120, 42);

  for (int k = 0; k < 4; k++) {
    int r = k / 2, c = k % 2;
    int x = 36 + c * 92, y = 58 + r * 48;
    lcd.fillRoundRect(x, y, 76, 44, 10, C_BTN);
    lcd.drawRoundRect(x, y, 76, 44, 10, C_RINGIN);
    char kl[2] = { HEXKEYS[g_page * 4 + k], 0 };
    lcd.setTextSize(4);
    lcd.setTextColor(C_VAL, C_BTN);
    lcd.drawString(kl, x + 38, y + 22);
  }

  btn(34, 156, 80, 26, "DEL", 0x4000, C_VAL);
  uint16_t okCol = (g_inLen == 6) ? 0x0320 : 0x2104;
  lcd.fillRoundRect(126, 156, 80, 26, 8, okCol);
  lcd.drawRoundRect(126, 156, 80, 26, 8, C_RINGIN);
  lcd.setTextDatum(textdatum_t::middle_center);
  lcd.setTextSize(2);
  lcd.setTextColor(C_VAL, okCol);
  lcd.drawString("OK", 166, 169);

  btn(58, 186, 34, 20, "<", 0x18C3, C_VAL);
  btn(148, 186, 34, 20, ">", 0x18C3, C_VAL);
  char pg[10];
  snprintf(pg, sizeof(pg), "%u/4", g_page + 1);
  lcd.setTextSize(1);
  lcd.setTextColor(C_UNIT, C_BG);
  lcd.drawString(pg, 120, 196);
}


/* ---------- 触摸命中 ---------- */
static bool inRect(int x, int y, int rx, int ry, int rw, int rh) {
  return x >= rx && x < rx + rw && y >= ry && y < ry + rh;
}

static void doTap(int x, int y) {
  if (g_mode == M_MAIN) {
    for (int i = 0; i < 4; i++) {
      int dx = x - CX[i], dy = y - CY[i];
      if (dx * dx + dy * dy < 46 * 46) {
        g_sel = (int8_t)i;
        g_mode = M_PICK;
        drawPick();
        return;
      }
    }
    return;
  }

  if (g_mode == M_PICK) {
    int shown = 0;
    for (int i = 0; i < g_nc; i++) {
      if (millis() - g_cand[i].at > CAND_MS) continue;
      if (shown >= 3) break;
      int yy = 50 + shown * 26;
      if (inRect(x, y, 38, yy, 164, 22)) {
        snprintf(g_bind[g_sel], sizeof(g_bind[0]), "%s", g_cand[i].id);
        saveBind(g_sel);
        recomputeBind();
        for (int k = 0; k < 4; k++)
          if (k != g_sel && strcmp(g_bind[k], g_bind[g_sel]) == 0) { g_bind[k][0] = 0; saveBind(k); }
        g_w[g_sel].has = false;
        g_mode = M_MAIN; drawMain();
        return;
      }
      shown++;
    }
    if (inRect(x, y, 38, 138, 78, 22)) {
      g_mode = M_KEYPAD; g_inLen = 0; memset(g_input, 0, sizeof(g_input)); drawKeypad(); return;
    }
    if (inRect(x, y, 124, 138, 78, 22)) { g_mode = M_MAIN; drawMain(); return; }
    if (inRect(x, y, 38, 166, 78, 22)) { clearBind(g_sel); g_mode = M_MAIN; drawMain(); return; }
    if (inRect(x, y, 124, 166, 78, 22)) {
      g_mode = M_MAIN;
      g_nc = 0; g_nextFree = 0;
      for (int i = 0; i < 4; i++) { g_bind[i][0] = 0; saveBind(i); g_w[i].has = false; }
      recomputeBind();
      drawMain();
      return;
    }
    return;
  }

  if (g_mode == M_KEYPAD) {
    int8_t hk = keyHit(x, y);
    if (hk >= 0) {
      if (g_inLen < 6) { g_input[g_inLen++] = HEXKEYS[g_page * 4 + hk]; g_input[g_inLen] = 0; }
      drawKeypad();
      return;
    }
    if (inRect(x, y, 34, 156, 80, 26)) {
      if (g_inLen) { g_inLen--; g_input[g_inLen] = 0; }
      drawKeypad();
      return;
    }
    if (inRect(x, y, 126, 156, 80, 26)) {
      if (g_inLen == 6) {
        snprintf(g_bind[g_sel], sizeof(g_bind[0]), "%s", g_input);
        saveBind(g_sel);
        recomputeBind();
        for (int k = 0; k < 4; k++)
          if (k != g_sel && strcmp(g_bind[k], g_bind[g_sel]) == 0) { g_bind[k][0] = 0; saveBind(k); }
        g_w[g_sel].has = false;
        g_mode = M_MAIN; drawMain();
      }
      return;
    }
    if (inRect(x, y, 58, 186, 34, 20)) { g_page = (uint8_t)((g_page + 3) & 3); drawKeypad(); return; }
    if (inRect(x, y, 148, 186, 34, 20)) { g_page = (uint8_t)((g_page + 1) & 3); drawKeypad(); return; }
    return;
  }
}

/* ---------- 广播 ---------- */
static void candAdd(const char* id, int8_t rssi) {
  for (int i = 0; i < g_nc; i++) if (strcmp(g_cand[i].id, id) == 0) {
    g_cand[i].at = millis(); g_cand[i].rssi = rssi; return;
  }
  int slot = (g_nc < MAX_CAND) ? g_nc++ : 0;
  if (g_nc >= MAX_CAND) {
    for (int i = 1; i < MAX_CAND; i++) if (g_cand[i].at < g_cand[slot].at) slot = i;
  }
  memset(&g_cand[slot], 0, sizeof(Cand));
  snprintf(g_cand[slot].id, sizeof(g_cand[slot].id), "%s", id);
  g_cand[slot].at = millis(); g_cand[slot].rssi = rssi;
}

static void handleAdv(const uint8_t* p, size_t n, int8_t rssi) {
  if (n < 26) return;
  if (!(p[0] == 0x02 && p[1] == 0x01 && p[2] == 0x06 &&
        p[3] == 0x03 && p[4] == 0x03 && p[5] == 0xB0 && p[6] == 0xFB)) return;

  const uint8_t* d = p + 7;
  float volt = (float)d[4] * 0.01f + 1.22f;
  float tempC = (float)d[6] - 55.0f;
  float kPa = (float)d[5] * 3.144f;
  if (d[10] == 1) kPa = ((float)d[5] + 256.0f) * 3.144f;
  uint8_t state = d[7];
  char id[8];
  snprintf(id, sizeof(id), "%02X%02X%02X", d[18], d[17], d[16]);
  g_frames++;
  candAdd(id, rssi);

  int slot = -1;
  if (g_anyBind) {
    for (int i = 0; i < 4; i++) if (strlen(g_bind[i]) == 6 && strcmp(g_bind[i], id) == 0) { slot = i; break; }
  } else {
    for (int i = 0; i < 4; i++) if (g_w[i].has && strcmp(g_w[i].id, id) == 0) { slot = i; break; }
    if (slot < 0 && g_nextFree < 4) {
      slot = g_nextFree++;
      memset(&g_w[slot], 0, sizeof(W));
    }
  }
  if (slot < 0) return;

  W& w = g_w[slot];
  w.has = true;
  snprintf(w.id, sizeof(w.id), "%s", id);
  w.kPa = kPa; w.tempC = tempC; w.volt = volt; w.state = state; w.rssi = rssi;
  w.at = millis();
  g_dirty = true;
}

class CB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) {
    size_t plen = 0; const uint8_t* pl = nullptr;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    std::string s = dev.getPayload(); plen = s.size(); pl = (const uint8_t*)s.data();
#else
    plen = dev.getPayloadLength(); pl = dev.getPayload();
#endif
    handleAdv(pl, plen, (int8_t)dev.getRSSI());
  }
};

/* ---------- 生命周期 ---------- */
void setup() {
  Serial.begin(115200);
  delay(300);
  memset(g_w, 0, sizeof(g_w));
  memset(g_bind, 0, sizeof(g_bind));
  g_pref.begin("tpms3", false);
  loadBind();

  tp.begin(4, 5); tp.setClock(400000);
  g_touchOk = tpInit();
  lcd.init();
  lcd.setBrightness(235);
  drawMain();
  Serial.printf("\r\n=== ID.3 TPMS v%s ===\r\n", FW_VERSION);
  Serial.printf("[touch] %s  [bind] %s %s %s %s\r\n", g_touchOk ? "ok" : "fail",
                g_bind[0], g_bind[1], g_bind[2], g_bind[3]);

  BLEDevice::init("id3-tpms");
  BLEScan* sc = BLEDevice::getScan();
  sc->setAdvertisedDeviceCallbacks(new CB(), true);
  sc->setActiveScan(true);
  sc->setInterval(100);
  sc->setWindow(90);
}

void loop() {
  BLEDevice::getScan()->start(1);
  BLEDevice::getScan()->clearResults();

  if (g_touchOk) {
    int f = 0, x = 0, y = 0;
    bool now = tpRead(f, x, y);
    if (now && millis() - g_lastTap > 350) {
      g_lastTap = millis();
      g_lx = x; g_ly = y; g_lf = f;
      g_sx = x; g_swiped = false;
      g_pressOnKey = (g_mode == M_KEYPAD && keyHit(x, y) >= 0);
      Serial.printf("[tap] %d,%d mode=%d\r\n", x, y, (int)g_mode);
      doTap(x, y);
    } else if (now && g_mode == M_KEYPAD && !g_swiped) {
      int dxm = x - g_sx;
      if (dxm > 45 || dxm < -45) {
        g_swiped = true;
        if (g_pressOnKey && g_inLen) { g_inLen--; g_input[g_inLen] = 0; }
        g_page = (dxm > 0) ? (uint8_t)((g_page + 1) & 3) : (uint8_t)((g_page + 3) & 3);
        Serial.printf("[swipe] page %u\r\n", g_page + 1);
        drawKeypad();
      }
    }
  }

  if (g_mode == M_MAIN) {
    if (g_dirty) { g_dirty = false; for (int i = 0; i < 4; i++) drawCell(i); }
    static uint32_t t1 = 0;
    if (millis() - t1 > 500) { t1 = millis(); g_blink = !g_blink; drawStatus(); }
    static uint32_t t2 = 0;
    if (millis() - t2 > 1000) { t2 = millis(); drawAge(); }
  } else if (g_mode == M_PICK) {
    static uint32_t t3 = 0;
    if (millis() - t3 > 700) { t3 = millis(); drawPick(); }
  }

  delay(15);
}

