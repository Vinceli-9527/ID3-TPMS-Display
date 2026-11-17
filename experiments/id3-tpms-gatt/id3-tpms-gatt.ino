/*
 * id3-tpms-gatt —— 连接式(FBB0)胎压传感器协议探针
 *
 * 背景：LYTPMS App 用的服务 UUID = 0000fbb0-0000-1000-8000-00805f9b34fb，
 *       这类传感器是"连上去读/订阅"，不靠广播带数据。
 *
 * 特点：
 *   1. 列出所有 BLE 设备（包括只广播 UUID、没有厂商数据的——之前的固件会漏掉它们）
 *   2. 标出 adv 里带 FBB0 服务的设备，基本就是胎压传感器
 *   3. g <n> 连接 → 自动枚举服务/特征值 → 读所有可读的 → 订阅所有可通知的
 *
 * 串口命令（115200，换行）：
 *   l            列出设备
 *   g <n>        连接第 n 个设备并自动探测
 *   x            断开
 *   r <si> <ci>  读某个特征值（服务序号/特征序号，探测时会打印）
 *   s <si> <ci>  订阅某个特征值
 *   w <si> <ci> <hex>  写入（例：w 0 1 0100）
 *   o            重新开始扫描
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEClient.h>
#include <BLEAdvertisedDevice.h>
#include <BLERemoteService.h>
#include <BLERemoteCharacteristic.h>

#define MAX_DEV 24
#define SCAN_SLICE 2

struct Dev {
  bool     used;
  char     mac[20];
  char     name[16];
  int8_t   rssi;
  uint32_t count;
  uint32_t lastSeen;
  bool     fbb0;       // adv 里带 FBB0 服务
  uint8_t  advType;
};

static Dev g_dev[MAX_DEV];
static bool g_showAll = true;

static BLEClient*            g_client = nullptr;
static BLERemoteService*     g_svc[8];
static BLERemoteCharacteristic* g_chr[8][8];
static uint8_t g_svcCount = 0;
static uint8_t g_chrCount[8] = {0};
static char    g_connMac[20] = {0};

static void hexStr(const uint8_t* p, size_t n, char* out, size_t cap) {
  size_t k = 0; if (cap) out[0] = 0;
  for (size_t i = 0; i < n && k + 4 < cap; i++) k += snprintf(out + k, cap - k, "%02X ", p[i]);
}

static int findDev(const char* mac) {
  for (int i = 0; i < MAX_DEV; i++) if (g_dev[i].used && strcmp(g_dev[i].mac, mac) == 0) return i;
  for (int i = 0; i < MAX_DEV; i++) {
    if (!g_dev[i].used) {
      memset(&g_dev[i], 0, sizeof(Dev));
      g_dev[i].used = true;
      strncpy(g_dev[i].mac, mac, sizeof(g_dev[i].mac) - 1);
      return i;
    }
  }
  return -1;
}

/* adv 解析：找 16 位/128 位服务 UUID 里的 FBB0，以及名字 */
static void scanAdvInfo(const uint8_t* p, size_t n, bool& fbb0, char* nameOut, size_t nameCap) {
  fbb0 = false; if (nameCap) nameOut[0] = 0;
  size_t i = 0;
  while (i + 1 < n) {
    uint8_t len = p[i];
    if (len == 0 || i + 1 + len > n) break;
    uint8_t type = p[i + 1];
    const uint8_t* d = p + i + 2;
    uint8_t dlen = len - 1;
    if ((type == 0x02 || type == 0x03) && dlen >= 2) {
      for (uint8_t k = 0; k + 1 < dlen; k += 2) if (d[k] == 0xB0 && d[k + 1] == 0xFB) fbb0 = true;
    } else if ((type == 0x06 || type == 0x07) && dlen >= 2) {
      // 128 位 UUID 小端存放，FBB0 打头两字节是 B0 FB
      for (uint8_t k = 0; k + 1 < dlen; k += 16) if (d[k] == 0xB0 && d[k + 1] == 0xFB) fbb0 = true;
    } else if ((type == 0x08 || type == 0x09) && dlen > 0 && nameCap) {
      uint8_t c = dlen < (uint8_t)(nameCap - 1) ? dlen : (uint8_t)(nameCap - 1);
      for (uint8_t k = 0; k < c; k++) nameOut[k] = (d[k] >= 0x20 && d[k] <= 0x7E) ? (char)d[k] : '.';
      nameOut[c] = 0;
    }
    i += (size_t)len + 1;
  }
}

static void handleAdv(BLEAdvertisedDevice dev) {
  size_t plen = 0; const uint8_t* pl = nullptr;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  std::string s = dev.getPayload(); plen = s.size(); pl = (const uint8_t*)s.data();
#else
  plen = dev.getPayloadLength(); pl = dev.getPayload();
#endif
  bool fbb0 = false; char nm[16] = {0};
  scanAdvInfo(pl, plen, fbb0, nm, sizeof(nm));
  if (nm[0] == 0 && dev.haveName()) { strncpy(nm, dev.getName().c_str(), sizeof(nm) - 1); }

  int i = findDev(dev.getAddress().toString().c_str());
  if (i < 0) return;
  Dev& d = g_dev[i];
  if (nm[0]) strncpy(d.name, nm, sizeof(d.name) - 1);
  d.rssi = (int8_t)dev.getRSSI();
  d.count++;
  d.lastSeen = millis();
  bool isNew = (d.count == 1);
  d.fbb0 = d.fbb0 || fbb0;
  if (isNew) {
    char hex[96]; hexStr(pl, plen > 28 ? 28 : plen, hex, sizeof(hex));
    Serial.printf("[new] %s name=%-14s rssi=%d%s adv=%s\r\n",
                  d.mac, d.name[0] ? d.name : "-", d.rssi, fbb0 ? " FBB0" : "", hex);
  }
  if (fbb0) {
    static uint32_t lastLog = 0;
    if (millis() - lastLog > 2000) {
      lastLog = millis();
      char hex[96]; hexStr(pl, plen > 28 ? 28 : plen, hex, sizeof(hex));
      Serial.printf("[FBB0!] %s name=%s rssi=%d adv=%s\r\n", d.mac, d.name, d.rssi, hex);
    }
  }
}

class AdvCB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) { handleAdv(dev); }
};

static void listDevs() {
  Serial.printf("\r\n idx  MAC                name             rssi  n     FBB0\r\n");
  for (int i = 0; i < MAX_DEV; i++) {
    if (!g_dev[i].used) continue;
    Serial.printf(" [%2d] %-17s %-16s %4d %-5lu %s\r\n",
                  i, g_dev[i].mac, g_dev[i].name[0] ? g_dev[i].name : "-",
                  g_dev[i].rssi, (unsigned long)g_dev[i].count, g_dev[i].fbb0 ? "YES <== 疑似胎压" : "");
  }
  Serial.printf("共 %d 个设备\r\n", MAX_DEV);
}

static void notifyCB(BLERemoteCharacteristic* ch, uint8_t* data, size_t len, bool isNotify) {
  char hex[96]; hexStr(data, len > 28 ? 28 : len, hex, sizeof(hex));
  Serial.printf("[通知] %s %s len=%u %s\r\n", ch->getUUID().toString().c_str(), isNotify ? "NTF" : "IND", (unsigned)len, hex);
}

static void disconnect() {
  if (g_client) {
    Serial.printf("断开 %s\r\n", g_connMac);
    g_client->disconnect();
    delete g_client;
    g_client = nullptr;
  }
  g_connMac[0] = 0; g_svcCount = 0;
}

static void connectAndProbe(int idx) {
  if (idx < 0 || idx >= MAX_DEV || !g_dev[idx].used) { Serial.printf("设备序号无效\r\n"); return; }
  disconnect();
  strncpy(g_connMac, g_dev[idx].mac, sizeof(g_connMac) - 1);
  Serial.printf("连接 %s ...\r\n", g_connMac);

  g_client = BLEDevice::createClient();
  if (!g_client->connect(BLEAddress(g_connMac))) {
    Serial.printf("连接失败（超时）。如果传感器已断电，请先给它压力唤醒。\r\n");
    delete g_client; g_client = nullptr; g_connMac[0] = 0;
    return;
  }
  Serial.printf("已连接，开始枚举服务...\r\n");

  std::map<std::string, BLERemoteService*>* svcs = g_client->getServices();
  g_svcCount = 0;
  if (svcs) {
    for (auto& kv : *svcs) {
      if (g_svcCount >= 8) break;
      BLERemoteService* svc = kv.second;
      g_svc[g_svcCount] = svc;
      Serial.printf("服务[%u] %s\r\n", g_svcCount, svc->getUUID().toString().c_str());
      g_chrCount[g_svcCount] = 0;
      std::map<std::string, BLERemoteCharacteristic*>* chrs = svc->getCharacteristics();
      if (chrs) {
        for (auto& c2 : *chrs) {
          if (g_chrCount[g_svcCount] >= 8) break;
          BLERemoteCharacteristic* ch = c2.second;
          uint8_t ci = g_chrCount[g_svcCount];
          g_chr[g_svcCount][ci] = ch;
          bool cR = ch->canRead(), cW = ch->canWrite(), cN = ch->canNotify(), cI = ch->canIndicate();
          Serial.printf("   特征[%u][%u] %s  props=%s%s%s%s\r\n",
                        g_svcCount, ci, ch->getUUID().toString().c_str(),
                        cR ? "R" : "-", cW ? "W" : "-", cN ? "N" : "-", cI ? "I" : "-");
          if (cR) {
            std::string v = ch->readValue();
            char hex[96]; hexStr((const uint8_t*)v.data(), v.size() > 28 ? 28 : v.size(), hex, sizeof(hex));
            Serial.printf("        读到 len=%u %s\r\n", (unsigned)v.size(), hex);
          }
          if (cN || cI) {
            ch->registerForNotify(notifyCB);
            Serial.printf("        已订阅通知\r\n");
          }
          g_chrCount[g_svcCount]++;
        }
      }
      g_svcCount++;
    }
  }
  Serial.printf("枚举完成：服务 %u 个。接下来等通知，或用 r/s/w 命令操作。\r\n", g_svcCount);
}

static char    g_line[64];
static uint8_t g_len = 0;

static void pollSerial() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch != '\n') { if (g_len < sizeof(g_line) - 1) g_line[g_len++] = ch; continue; }
    g_line[g_len] = 0; g_len = 0;
    String cmd(g_line); cmd.trim();
    if (cmd.length() == 0) { listDevs(); continue; }
    char c0 = cmd[0];
    if (c0 == 'l') listDevs();
    else if (c0 == 'x') disconnect();
    else if (c0 == 'g') { int n = cmd.substring(1).toInt(); connectAndProbe(n); }
    else if (c0 == 'r') {
      int a = 0, b = 0;
      sscanf(cmd.c_str(), "r %d %d", &a, &b);
      if (a < g_svcCount && b < g_chrCount[a] && g_chr[a][b]) {
        std::string v = g_chr[a][b]->readValue();
        char hex[96]; hexStr((const uint8_t*)v.data(), v.size() > 28 ? 28 : v.size(), hex, sizeof(hex));
        Serial.printf("读[%d][%d] len=%u %s\r\n", a, b, (unsigned)v.size(), hex);
      } else Serial.printf("索引无效\r\n");
    } else if (c0 == 's') {
      int a = 0, b = 0;
      sscanf(cmd.c_str(), "s %d %d", &a, &b);
      if (a < g_svcCount && b < g_chrCount[a] && g_chr[a][b]) {
        g_chr[a][b]->registerForNotify(notifyCB);
        Serial.printf("已订阅[%d][%d]\r\n", a, b);
      } else Serial.printf("索引无效\r\n");
    } else if (c0 == 'w') {
      int a = 0, b = 0; char hexs[48] = {0};
      sscanf(cmd.c_str(), "w %d %d %47s", &a, &b, hexs);
      if (a < g_svcCount && b < g_chrCount[a] && g_chr[a][b]) {
        uint8_t out[16]; size_t n = 0;
        for (size_t i = 0; hexs[i] && hexs[i + 1] && n < sizeof(out); i += 2) {
          char t[3] = { hexs[i], hexs[i + 1], 0 };
          out[n++] = (uint8_t)strtol(t, nullptr, 16);
        }
        g_chr[a][b]->writeValue(out, n, true);
        Serial.printf("已写[%d][%d] %u 字节\r\n", a, b, (unsigned)n);
      } else Serial.printf("索引无效\r\n");
    } else if (c0 == 'o') { memset(g_dev, 0, sizeof(g_dev)); Serial.printf("设备表已清空\r\n"); }
    else Serial.printf("命令：l 列表 | g <n> 连接 | x 断开 | r/s/w | o 清空\r\n");
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  memset(g_dev, 0, sizeof(g_dev));
  Serial.printf("\r\n=== id3-tpms-gatt 连接式探针 ===\r\n");
  BLEDevice::init("tpms-gatt");
  BLEScan* scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new AdvCB(), true);
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(90);
  Serial.printf("命令：l 列表 | g <n> 连接并枚举 | x 断开\r\n\r\n");
}

void loop() {
  pollSerial();
  if (!g_connMac[0]) {
    BLEScan* scan = BLEDevice::getScan();
    scan->start(SCAN_SLICE);
    scan->clearResults();
  }
  delay(5);
}
