// ISDT FD-200 BLE remote for ESP32
// Web UI: scan for the discharger, connect, set cells / cutoff / current, start & stop.
//
// Protocol (decoded from a capture of the ISDT app talking to an FD-200):
//   Frame: [len][0xAA][dir][plen][cmd][data...][chk]
//     len  = bytes after the len byte (= plen + 4)
//     dir  = 0x12 phone->device, 0x21 device->phone
//     plen = 1 + number of data bytes
//     chk  = (dir + plen + cmd + sum(data)) & 0xFF
//   0x18 -> 0x19  auth/bind, reply status 00 = OK
//   0xE0 -> 0xE1  device info (contains "FD200")
//   0xE4 -> 0xE5  pack voltage + 8 cell slots (mV)
//   0xE6 -> 0xE7  run state (00 idle / 02 discharging) + echo of settings
//   0xE8 -> 0xE9  misc status
//   0xEA -> 0xEB  start (sub-command 02) / stop (sub-command 03)
//   0x48 -> 0x49  sent by app right after start (purpose unknown)
//   0xD2 -> 0xD3  set auto-discharge: data [01 on / 00 off][9 x 00], reply status 00 = OK
//   0xD4 -> 0xD5  read auto-discharge: reply data FF = on, 00 = off

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEClient.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <vector>
#include <algorithm>

// ===================== USER SETTINGS =====================
#define AP_SSID "FD200-Discharger"
#define AP_PASS "12345678"   // 8+ characters. Change this before sharing.
#define HOSTNAME "fd200"     // on home WiFi the UI is also at http://fd200.local

// Cutoff voltage safety limits (per cell, millivolts). Enforced on the ESP32, not just the page.
#define CUTOFF_MIN_MV   3000 // below this a start/preset is refused outright
#define CUTOFF_WARN_MV  3400 // below this the user must confirm (LiPo storage is ~3.80-3.85 V)
#define CUTOFF_MAX_MV   4200
// =========================================================
//
// WiFi behaviour:
//   - No home WiFi saved (or turned off): runs its own hotspot (AP_SSID), UI at http://192.168.4.1
//   - Home WiFi saved: joins it at boot. If it can't within 20 s, the hotspot starts.
//   - Home WiFi drops for 30 s: the hotspot starts. It retries home WiFi every 5 minutes
//     (only while nobody is connected to the hotspot, since retrying can briefly disturb it).
//   - Once on home WiFi, the hotspot switches itself off after 60 s with nobody on it.

// Auth/bind frame captured from the ISDT app. This may be specific to the
// phone/app install it was captured from; other users may need their own.
static const uint8_t AUTH_FRAME[22] = {
  0x15, 0xAA, 0x12, 0x11, 0x18, 0xF2, 0xBA, 0x35, 0x71, 0x83, 0xE5,
  0x40, 0x3F, 0xA1, 0x43, 0xF8, 0x37, 0x87, 0xCD, 0x7E, 0xBB, 0x14
};

static BLEUUID serviceUUID("0000fff0-0000-1000-8000-00805f9b34fb");
static BLEUUID charUUID("0000fff7-0000-1000-8000-00805f9b34fb");

WebServer server(80);
Preferences prefs;

// Declared here, above every function, because the Arduino IDE inserts
// auto-generated prototypes before the first function in the file.
enum Job { JOB_NONE, JOB_START, JOB_STOP, JOB_SCAN, JOB_AUTO };
enum WState { W_OFF, W_CONNECTING, W_CONNECTED, W_LOST, W_WAIT };

// ---------- Home WiFi (only touched from loop(), no locking needed) ----------
String  wifiSsid = "";
String  wifiPass = "";
bool    wifiEnabled = false;
WState  wState = W_OFF;
bool    apOn = false;
bool    wifiApplyPending = false;
uint32_t wStateSince = 0;
uint32_t apIdleSince = 0;
String  wifiMsg = "";

// ---------- Presets (only touched from loop()) ----------
struct Preset { String name; uint8_t cells; uint16_t cutoffMv; uint16_t currentMa; };
std::vector<Preset> presets;
#define MAX_PRESETS 30

// ---------- Selected device (shared between web handlers and BLE task) ----------
SemaphoreHandle_t cfgMutex;
String cfgAddr = "";
String cfgName = "";
uint8_t cfgType = 0;          // BLE address type (0 = public)
String lastError = "";
volatile bool reconnectRequested = false;
volatile bool dropRequested = false;

void setError(const String& e) {
  xSemaphoreTake(cfgMutex, portMAX_DELAY);
  lastError = e;
  xSemaphoreGive(cfgMutex);
  if (e.length()) Serial.println("ERROR: " + e);
}

// ---------- BLE connection state ----------
BLEClient* pClient = nullptr;
BLERemoteCharacteristic* pChar = nullptr;
volatile bool bleConnected = false;

volatile bool     seen[256];
volatile uint8_t  respStatus[256];
volatile uint16_t packMv = 0;
volatile uint8_t  runState = 0xFF;
volatile uint32_t elapsedMs = 0;
volatile uint8_t  setCells = 0;
volatile uint16_t setCutoffMv = 0;
volatile uint16_t setCurrentMa = 0;
volatile uint16_t actualMa = 0;       // E5 [14..15]  current actually flowing (ramps up over ~30 s)
volatile uint32_t capMah = 0;         // E7 [8..11]   discharged this run, mAh
volatile uint32_t energyMwh = 0;      // E7 [12..15]  discharged this run, mWh
volatile uint16_t cycles = 0;         // E7 [29..30]  number of completed discharges (lifetime)
volatile uint32_t lifeMwh = 0;        // E7 [35..38]  lifetime energy discharged, mWh
volatile int16_t  tempC = -1;         // E9 [6]       discharger temperature, deg C
volatile int8_t   autoMode = -1;      // device's auto-discharge setting: -1 unknown, 0 off, 1 on

// Reassembly buffer: long replies (E5, E7) can arrive split across several notifications
uint8_t rxBuf[96];
size_t  rxLen = 0, rxNeed = 0;

// Link diagnostics shown on the web page
volatile uint32_t rxOk = 0, rxBad = 0, rxCut = 0;
volatile uint8_t  lastCutCmd = 0, lastCutLen = 0, lastCutNeed = 0;
volatile uint16_t linkMtu = 0;

// ---------- Jobs handed from web handlers to the BLE task ----------
portMUX_TYPE jobMux = portMUX_INITIALIZER_UNLOCKED;
volatile Job  job = JOB_NONE;
volatile bool jobPicked = false;
volatile bool jobDone = false;
bool   jobOk = false;
String jobMsg = "";
uint8_t  jobCells = 6;
uint16_t jobCutoffMv = 3700;
uint16_t jobCurrentMa = 10000;
bool     jobAutoOn = false;

// ---------- Scan results ----------
struct FoundDev { String addr; String name; int rssi; uint8_t type; bool match; };
std::vector<FoundDev> found;

// ==================================================================
// Frame helpers
// ==================================================================
size_t buildFrame(uint8_t cmd, const uint8_t* data, uint8_t dataLen, uint8_t* out) {
  uint8_t plen = dataLen + 1;
  out[0] = plen + 4;
  out[1] = 0xAA;
  out[2] = 0x12;
  out[3] = plen;
  out[4] = cmd;
  uint8_t sum = 0x12 + plen + cmd;
  for (uint8_t i = 0; i < dataLen; i++) {
    out[5 + i] = data[i];
    sum += data[i];
  }
  out[5 + dataLen] = sum;
  return 6 + dataLen;
}

void printHex(const char* prefix, const uint8_t* d, size_t n) {
  Serial.print(prefix);
  for (size_t i = 0; i < n; i++) {
    if (d[i] < 0x10) Serial.print('0');
    Serial.print(d[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

void sendFrame(const uint8_t* f, size_t n) {
  if (!bleConnected || !pChar) return;
  printHex(">> ", f, n);
  pChar->writeValue((uint8_t*)f, n, true);
}

bool sendAndWait(const uint8_t* f, size_t n, uint8_t expect, uint32_t timeoutMs, int tries) {
  for (int t = 0; t < tries; t++) {
    seen[expect] = false;
    sendFrame(f, n);
    uint32_t t0 = millis();
    while (millis() - t0 < timeoutMs) {
      if (seen[expect]) return true;
      delay(10);
    }
  }
  return false;
}

// ==================================================================
// Notifications
// ==================================================================
// complete = whole frame received (checksum is verified).
// complete = false: the FD-200 cut the reply short (seen with small BLE packet sizes);
// only the live-data fields that fall inside the bytes we did get are used.
void handleFrame(const uint8_t* d, size_t n, bool complete) {
  if (n < 6) return;
  uint8_t cmd = d[4];
  if (complete) {
    uint8_t sum = 0;
    for (size_t i = 2; i < n - 1; i++) sum += d[i];
    if (sum != d[n - 1]) { rxBad++; Serial.println("   (bad checksum, ignored)"); return; }
    rxOk++;
    respStatus[cmd] = d[5];
    seen[cmd] = true;
  } else {
    rxCut++;
    lastCutCmd = cmd; lastCutLen = n; lastCutNeed = d[3] + 5;
    Serial.printf("   (reply 0x%02X cut short: got %u of %u bytes, using what arrived)\n", cmd, (unsigned)n, (unsigned)(d[3] + 5));
    if (cmd != 0xE5 && cmd != 0xE7) return;
  }

  #define HAVE(last) ((size_t)(last) < n)
  if (cmd == 0xE5) {
    if (HAVE(13)) packMv   = d[12] | (d[13] << 8);   // the FD-200 only sees the pack (XT60), no per-cell taps
    if (HAVE(15)) actualMa = d[14] | (d[15] << 8);
  }
  if (cmd == 0xE7) {
    if (HAVE(6))  runState  = d[6];
    if (HAVE(11)) capMah    = d[8]  | (d[9]  << 8) | ((uint32_t)d[10] << 16) | ((uint32_t)d[11] << 24);
    if (HAVE(15)) energyMwh = d[12] | (d[13] << 8) | ((uint32_t)d[14] << 16) | ((uint32_t)d[15] << 24);
    if (HAVE(19)) elapsedMs = d[16] | (d[17] << 8) | ((uint32_t)d[18] << 16) | ((uint32_t)d[19] << 24);
    if (HAVE(21)) setCells  = d[21];
    if (HAVE(24)) setCutoffMv  = d[23] | (d[24] << 8);
    if (HAVE(26)) setCurrentMa = d[25] | (d[26] << 8);
    if (HAVE(30)) cycles  = d[29] | (d[30] << 8);
    if (HAVE(38)) lifeMwh = d[35] | (d[36] << 8) | ((uint32_t)d[37] << 16) | ((uint32_t)d[38] << 24);
  }
  if (cmd == 0xE9 && complete && n >= 8) {
    tempC = d[6];
  }
  #undef HAVE
  if (cmd == 0xD5 && complete) {
    autoMode = d[5] ? 1 : 0;           // FF when on, 00 when off (seen in capture)
  }
}

static void notifyCallback(BLERemoteCharacteristic*, uint8_t* d, size_t n, bool) {
  printHex("<< ", d, n);
  if (n < 2) return;
  // Every notification is [count][count bytes]. Long replies are split into several
  // notifications, each with its own count byte; the frame is the joined payloads.
  size_t cl = std::min((size_t)d[0], n - 1);
  const uint8_t* p = d + 1;

  if (cl >= 3 && p[0] == 0xAA && p[1] == 0x21) {
    // A new frame starts. If the previous one never finished, use what we got of it.
    if (rxNeed && rxLen >= 6) handleFrame(rxBuf, rxLen, false);
    // Rebuild it as [len][AA][21][plen][cmd][data...][chk], len = plen + 4
    rxNeed = p[2] + 5;
    if (rxNeed > sizeof(rxBuf)) { rxNeed = 0; return; }
    rxBuf[0] = p[2] + 4;
    rxLen = 1;
  } else if (rxNeed == 0) {
    return;                            // stray piece or foreign data
  }
  size_t take = std::min(cl, rxNeed - rxLen);
  memcpy(rxBuf + rxLen, p, take);
  rxLen += take;
  if (rxLen < rxNeed) return;          // wait for the rest
  rxNeed = 0;
  handleFrame(rxBuf, rxLen, true);
}

class FDClientCB : public BLEClientCallbacks {
  void onConnect(BLEClient*) override {}
  void onDisconnect(BLEClient*) override {
    bleConnected = false;
    runState = 0xFF;
    autoMode = -1;
    tempC = -1;
    rxNeed = 0;
    Serial.println("BLE disconnected");
  }
};
static FDClientCB clientCB;

class ScanCB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) override {
    String addr = String(dev.getAddress().toString().c_str());
    addr.toLowerCase();
    String name = dev.haveName() ? String(dev.getName().c_str()) : String("");
    bool svc = dev.haveServiceUUID() && dev.isAdvertisingService(serviceUUID);
    bool nameMatch = name.indexOf("FD200") >= 0 || name.indexOf("ISDT") >= 0;

    for (auto& f : found) {
      if (f.addr == addr) {
        f.rssi = dev.getRSSI();
        if (name.length()) f.name = name;
        f.match = f.match || svc || nameMatch;
        return;
      }
    }
    found.push_back({addr, name, dev.getRSSI(), (uint8_t)dev.getAddressType(), svc || nameMatch});
  }
};
static ScanCB scanCB;

// ==================================================================
// BLE operations (only ever called from the BLE task)
// ==================================================================
bool connectFD200(const String& addr, uint8_t type) {
  if (pClient) { delete pClient; pClient = nullptr; pChar = nullptr; }
  pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(&clientCB);
  pClient->setMTU(256);

  Serial.printf("\nConnecting to %s...\n", addr.c_str());
  BLEAddress bleAddr(addr.c_str());
  if (!pClient->connect(bleAddr, (esp_ble_addr_type_t)type)) {
    setError("Could not connect. Is it powered on, and is the phone app closed?");
    return false;
  }
  delay(300);

  BLERemoteService* svc = pClient->getService(serviceUUID);
  if (svc) pChar = svc->getCharacteristic(charUUID);
  if (!pChar || !pChar->canNotify()) {
    setError("Connected, but this doesn't look like an FD-200 (no fff0/fff7 service)");
    pClient->disconnect();
    bleConnected = false;
    return false;
  }
  pChar->registerForNotify(notifyCallback);
  delay(200);

  // Ask for bigger packets now that the link is up (the request made while connecting
  // isn't always honoured). Long replies are 43 bytes; the default MTU only fits 20.
  pClient->setMTU(247);
  delay(300);
  linkMtu = pClient->getMTU();
  rxNeed = 0; rxOk = rxBad = rxCut = 0;
  bleConnected = true;

  if (!sendAndWait(AUTH_FRAME, sizeof(AUTH_FRAME), 0x19, 1500, 3)) {
    setError("Connected, but the FD-200 did not answer the auth frame");
    pClient->disconnect();
    bleConnected = false;
    return false;
  }
  if (respStatus[0x19] != 0x00) {
    setError("FD-200 rejected auth (status 0x" + String(respStatus[0x19], HEX) + ")");
  } else {
    setError("");
  }

  uint8_t f[8];
  size_t n = buildFrame(0xE0, nullptr, 0, f);
  sendAndWait(f, n, 0xE1, 1500, 3);

  n = buildFrame(0xD4, nullptr, 0, f);   // read the auto-discharge setting
  sendAndWait(f, n, 0xD5, 1500, 2);

  Serial.printf(">>> Connected and authenticated (MTU %u) <<<\n", (unsigned)linkMtu);
  return true;
}

bool startDischarge(uint8_t cells, uint16_t cutoffMv, uint16_t currentMa, String& msg) {
  if (!bleConnected) { msg = "Not connected to the FD-200"; return false; }
  // [00][02 start][01][01][current mA lo][hi][00][00][cells][cutoff mV lo][hi]
  uint8_t d[11] = {
    0x00, 0x02, 0x01, 0x01,
    (uint8_t)(currentMa & 0xFF), (uint8_t)(currentMa >> 8),
    0x00, 0x00,
    cells,
    (uint8_t)(cutoffMv & 0xFF), (uint8_t)(cutoffMv >> 8)
  };
  uint8_t f[20];
  size_t n = buildFrame(0xEA, d, sizeof(d), f);
  if (!sendAndWait(f, n, 0xEB, 1500, 2)) { msg = "No acknowledgement from the FD-200"; return false; }
  uint8_t st = respStatus[0xEB];

  uint8_t z = 0x00;
  n = buildFrame(0x48, &z, 1, f);   // mirrors the app
  sendFrame(f, n);

  if (st != 0x00) { msg = "FD-200 refused start (status 0x" + String(st, HEX) + ")"; return false; }
  msg = "Started: " + String(cells) + "S to " + String(cutoffMv / 1000.0, 2) +
        " V/cell at " + String(currentMa / 1000) + " A";
  return true;
}

bool stopDischarge(String& msg) {
  if (!bleConnected) { msg = "Not connected to the FD-200"; return false; }
  uint8_t d[11] = {0x00, 0x03, 0x00, 0x01, 0, 0, 0, 0, 0, 0, 0};
  uint8_t f[20];
  size_t n = buildFrame(0xEA, d, sizeof(d), f);
  if (!sendAndWait(f, n, 0xEB, 1500, 2)) { msg = "No acknowledgement from the FD-200"; return false; }
  msg = (respStatus[0xEB] == 0x00) ? "Stopped" : "Stop status 0x" + String(respStatus[0xEB], HEX);
  return respStatus[0xEB] == 0x00;
}

// Auto-discharge: D2 [on/off][9 x 00] -> D3 status. Read back with D4 -> D5 (FF on / 00 off).
bool setAutoMode(bool on, String& msg) {
  if (!bleConnected) { msg = "Not connected to the FD-200"; return false; }
  uint8_t d[10] = {0};
  d[0] = on ? 0x01 : 0x00;
  uint8_t f[20];
  size_t n = buildFrame(0xD2, d, sizeof(d), f);
  if (!sendAndWait(f, n, 0xD3, 1500, 2)) { msg = "No acknowledgement from the FD-200"; return false; }
  if (respStatus[0xD3] != 0x00) { msg = "FD-200 refused (status 0x" + String(respStatus[0xD3], HEX) + ")"; return false; }

  n = buildFrame(0xD4, nullptr, 0, f);
  sendAndWait(f, n, 0xD5, 1500, 2);
  if (autoMode != (on ? 1 : 0)) { msg = "Sent, but the FD-200 still reports auto-discharge " + String(autoMode == 1 ? "on" : "off"); return false; }
  msg = on ? "Auto-discharge ON" : "Auto-discharge OFF";
  return true;
}

void doScan() {
  found.clear();
  BLEScan* s = BLEDevice::getScan();
  s->setAdvertisedDeviceCallbacks(&scanCB, true);   // duplicates on, so late scan-response names are caught
  s->setActiveScan(true);
  s->setInterval(100);
  s->setWindow(99);
  s->start(5, false);
  s->clearResults();
  std::sort(found.begin(), found.end(), [](const FoundDev& a, const FoundDev& b) {
    if (a.match != b.match) return a.match;
    return a.rssi > b.rssi;
  });
}

void runJobNow(Job j) {
  switch (j) {
    case JOB_SCAN:  doScan(); jobOk = true; jobMsg = ""; break;
    case JOB_START: jobOk = startDischarge(jobCells, jobCutoffMv, jobCurrentMa, jobMsg); break;
    case JOB_STOP:  jobOk = stopDischarge(jobMsg); break;
    case JOB_AUTO:  jobOk = setAutoMode(jobAutoOn, jobMsg); break;
    default: break;
  }
}

void bleTask(void*) {
  uint32_t lastTry = 0, lastPoll = 0;
  uint8_t pollIdx = 0;
  int fails = 0;

  for (;;) {
    // Drop the current connection (device changed or forgotten)
    if (dropRequested) {
      dropRequested = false;
      if (pClient && bleConnected) {
        pClient->disconnect();
        for (int i = 0; i < 100 && bleConnected; i++) delay(20);
      }
      bleConnected = false;
    }

    // Jobs from the web UI
    Job j = JOB_NONE;
    portENTER_CRITICAL(&jobMux);
    if (job != JOB_NONE && !jobPicked) { j = job; jobPicked = true; }
    portEXIT_CRITICAL(&jobMux);
    if (j != JOB_NONE) {
      runJobNow(j);
      portENTER_CRITICAL(&jobMux);
      job = JOB_NONE; jobPicked = false; jobDone = true;
      portEXIT_CRITICAL(&jobMux);
      continue;
    }

    if (!bleConnected) {
      xSemaphoreTake(cfgMutex, portMAX_DELAY);
      String a = cfgAddr;
      uint8_t t = cfgType;
      xSemaphoreGive(cfgMutex);

      uint32_t interval = (fails < 3) ? 10000 : 60000;
      if (a.length() && (reconnectRequested || millis() - lastTry > interval)) {
        if (reconnectRequested) fails = 0;
        reconnectRequested = false;
        lastTry = millis();
        if (connectFD200(a, t)) fails = 0; else fails++;
      }
      delay(50);
      continue;
    }

    // Poll like the app does
    if (millis() - lastPoll > 1000) {
      lastPoll = millis();
      static const uint8_t pollCmds[3] = {0xE8, 0xE6, 0xE4};
      uint8_t z = 0x00, f[8];
      size_t n = buildFrame(pollCmds[pollIdx++ % 3], &z, 1, f);
      sendFrame(f, n);
    }
    delay(20);
  }
}

// ==================================================================
// Home WiFi / hotspot (all called from loop())
// ==================================================================
void startAP() {
  if (apOn) return;
  WiFi.softAP(AP_SSID, AP_PASS);   // adds the AP interface alongside STA if STA is on
  apOn = true;
  apIdleSince = millis();
  Serial.printf("Hotspot on: join %s, open http://%s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
}

void stopAP() {
  if (!apOn) return;
  WiFi.softAPdisconnect(true);
  apOn = false;
  Serial.println("Hotspot off (nobody on it, home WiFi is up)");
}

void setWState(WState s) { wState = s; wStateSince = millis(); }

void beginSTA() {
  WiFi.enableSTA(true);
  WiFi.disconnect();
  delay(50);
  Serial.printf("Joining WiFi \"%s\"...\n", wifiSsid.c_str());
  WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
  wifiMsg = "";
  setWState(W_CONNECTING);
}

void wifiOff() {
  startAP();                  // make sure there's always a way in
  WiFi.disconnect(true);      // drop home WiFi and turn the STA side off
  setWState(W_OFF);
}

String wifiReason(int st) {
  if (st == WL_NO_SSID_AVAIL) return "network not found";
  if (st == WL_CONNECT_FAILED) return "wrong password?";
  return "no answer, check the password and signal";
}

void wifiLoop() {
  uint32_t now = millis();

  if (wifiApplyPending) {
    wifiApplyPending = false;
    if (wifiEnabled && wifiSsid.length()) { startAP(); beginSTA(); }   // keep the hotspot up while trying
    else wifiOff();
    return;
  }

  bool up = (WiFi.status() == WL_CONNECTED);
  switch (wState) {
    case W_OFF:
      break;

    case W_CONNECTING:
      if (up) {
        setWState(W_CONNECTED);
        apIdleSince = now;
        wifiMsg = "";
        Serial.printf("WiFi connected: http://%s  (or http://%s.local)\n",
                      WiFi.localIP().toString().c_str(), HOSTNAME);
      } else if (now - wStateSince > 20000) {
        wifiMsg = "Couldn't join \"" + wifiSsid + "\" (" + wifiReason(WiFi.status()) +
                  "). Hotspot is on; retrying every 5 min.";
        Serial.println(wifiMsg);
        WiFi.disconnect();    // stop hunting so the hotspot channel stays steady
        startAP();
        setWState(W_WAIT);
      }
      break;

    case W_CONNECTED:
      if (!up) { setWState(W_LOST); Serial.println("WiFi lost, waiting for it to come back..."); break; }
      if (apOn) {
        if (WiFi.softAPgetStationNum() > 0) apIdleSince = now;
        else if (now - apIdleSince > 60000) stopAP();
      }
      break;

    case W_LOST:              // the WiFi library keeps reconnecting on its own here
      if (up) { setWState(W_CONNECTED); apIdleSince = now; Serial.println("WiFi back"); }
      else if (now - wStateSince > 30000) {
        wifiMsg = "Lost \"" + wifiSsid + "\". Hotspot is on; retrying every 5 min.";
        Serial.println(wifiMsg);
        WiFi.disconnect();
        startAP();
        setWState(W_WAIT);
      }
      break;

    case W_WAIT:
      if (now - wStateSince > 300000) {
        if (WiFi.softAPgetStationNum() == 0) beginSTA();
        else wStateSince = now;   // someone's using the hotspot, check again later
      }
      break;
  }
}

// ==================================================================
// Web handlers (run in loop(); never touch BLE directly)
// ==================================================================
bool submitJob(Job j, uint32_t waitMs, String& msg) {
  portENTER_CRITICAL(&jobMux);
  bool busy = (job != JOB_NONE);
  if (!busy) { jobDone = false; job = j; }
  portEXIT_CRITICAL(&jobMux);
  if (busy) { msg = "Busy, try again"; return false; }

  uint32_t t0 = millis();
  bool extended = false;
  while (!jobDone) {
    if (millis() - t0 > waitMs) {
      portENTER_CRITICAL(&jobMux);
      bool picked = jobPicked;
      if (!picked) job = JOB_NONE;    // cancel before it runs late
      portEXIT_CRITICAL(&jobMux);
      if (!picked) { msg = "Bluetooth busy (probably mid-connect), try again"; return false; }
      if (extended) { msg = "Timed out"; return false; }
      extended = true;
      t0 = millis();
      waitMs = 15000;
    }
    delay(20);
  }
  msg = jobMsg;
  return jobOk;
}

String jsonEsc(const String& s) {
  String o;
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if ((uint8_t)c >= 0x20) o += c;
  }
  return o;
}

void sendResult(bool ok, const String& msg) {
  server.send(ok ? 200 : 500, "application/json",
              String("{\"success\":") + (ok ? "true" : "false") + ",\"msg\":\"" + jsonEsc(msg) + "\"}");
}

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>FD-200 Remote</title>
<style>
body{font-family:-apple-system,BlinkMacSystemFont,sans-serif;background:#121418;color:#eee;margin:0;padding:16px}
.card{background:#1e222b;max-width:440px;margin:auto;border-radius:12px;padding:20px}
h2{text-align:center;color:#00e5ff;margin:0 0 12px}
h3{font-size:15px;color:#a0aec0;margin:20px 0 6px;text-transform:uppercase;letter-spacing:.05em}
label{display:block;margin-top:12px;font-size:14px;color:#a0aec0}
select,input{width:100%;padding:10px;margin-top:4px;border-radius:6px;border:1px solid #333d4b;background:#252b36;color:#fff;box-sizing:border-box;font-size:16px}
.chk{display:flex;align-items:center;gap:8px}.chk input{width:auto;margin:0}
button{width:100%;padding:14px;margin-top:12px;border:none;border-radius:8px;font-size:17px;font-weight:bold;color:#fff;cursor:pointer}
button:disabled{opacity:.35;cursor:default}
.go{background:#00c853}.stop{background:#dc2626}.scan{background:#3b82f6}.ghost{background:transparent;border:1px solid #4a5568;color:#a0aec0;font-size:14px;padding:10px}
.dev{background:#2d3748;font-size:14px;font-weight:normal;text-align:left;padding:12px;margin-top:8px}
.dev small{color:#a0aec0}
.box{margin-top:12px;padding:10px;border-radius:6px;background:#252b36;font-size:14px;white-space:pre-line}
.err{color:#f87171}
.row{display:flex;gap:8px}
</style></head><body><div class="card">
<h2>FD-200 Remote</h2>

<h3>Discharger</h3>
<div class="box" id="dev">Loading...</div>
<button class="scan" id="scanBtn" onclick="scan()">Scan for FD-200</button>
<label class="chk"><input type="checkbox" id="showAll" onchange="render()">Show all Bluetooth devices</label>
<div id="list"></div>
<button class="ghost" onclick="forget()">Forget this discharger</button>

<h3>Discharge</h3>
<label>Preset</label>
<select id="preset" onchange="usePreset()"><option value="">Manual (no preset)</option></select>
<div class="row">
<button class="ghost" onclick="savePreset()">Save as preset</button>
<button class="ghost" id="delBtn" onclick="delPreset()" disabled>Delete preset</button>
</div>
<label>Cells</label>
<select id="cells">
<option>1</option><option>2</option><option>3</option><option>4</option>
<option>5</option><option selected>6</option><option>7</option><option>8</option>
</select>
<label>Cutoff per cell (V)</label>
<input id="volt" type="number" step="0.01" min="3.00" max="4.20" value="3.70">
<label>Discharge current</label>
<select id="amps">
<option value="5000">5 A</option><option value="10000" selected>10 A</option>
<option value="15000">15 A</option><option value="20000">20 A</option><option value="25000">25 A</option>
</select>
<button class="go" id="goBtn" onclick="start()" disabled>Start Discharge</button>
<button class="stop" id="stopBtn" onclick="act('/stop')" disabled>Stop Discharge</button>
<div class="box" id="msg">Ready</div>
<label class="chk" style="margin-top:16px"><input type="checkbox" id="autoChk" onchange="setAuto()" disabled>Auto-discharge on the FD-200</label>
<div style="font-size:12px;color:#718096;margin-top:4px" id="autoNote">When on, the FD-200 starts discharging by itself a few seconds after a battery is connected or its settings change.</div>

<h3>Live</h3>
<div class="box" id="live">Not connected</div>

<h3>Home WiFi</h3>
<div class="box" id="wstat">Loading...</div>
<label class="chk"><input type="checkbox" id="wOn" onchange="wEnable()">Use home WiFi</label>
<button class="scan" id="wScanBtn" onclick="wScan()">Scan WiFi networks</button>
<div id="wlist"></div>
<label>Network name</label>
<input id="ssid" maxlength="32" autocomplete="off" autocapitalize="none">
<label>Password</label>
<input id="pass" type="password" maxlength="63" autocomplete="off">
<label class="chk"><input type="checkbox" onchange="$('pass').type=this.checked?'text':'password'">Show password</label>
<button class="go" onclick="wSave()">Save &amp; connect</button>
<div class="box" id="wmsg" style="display:none"></div>
<button class="ghost" onclick="wForget()">Clear WiFi settings</button>
</div>
<script>
let devices=[];
const $=id=>document.getElementById(id);
function say(t){$('msg').textContent=t;}
function act(u){say('Sending...');fetch(u).then(r=>r.json()).then(d=>say(d.msg)).catch(()=>say('Network error'));}
// Cutoff limits come from the ESP32 (/status) so the #defines are the single source of truth
function lim(){const s=lastStatus||{};return {min:s.cutMinMv||3000,warn:s.cutWarnMv||3400,max:s.cutMaxMv||4200};}
// Returns true if the cutoff is OK to use (asking the user when it's low), false otherwise
function checkCutoff(action){
  const L=lim(),v=parseFloat($('volt').value),mv=Math.round(v*1000),c=$('cells').value;
  if(isNaN(v)||mv<L.min||mv>L.max){say('Cutoff must be between '+(L.min/1000).toFixed(2)+' and '+(L.max/1000).toFixed(2)+' V per cell');return false;}
  if(mv<L.warn)return confirm('LOW CUTOFF WARNING\n\n'+v.toFixed(2)+' V/cell ('+(v*c).toFixed(2)+' V for '+c+'S) is below '+(L.warn/1000).toFixed(2)+' V/cell.\n\nDischarging this low can permanently damage LiPo / Li-ion packs. Storage voltage is usually 3.80-3.85 V/cell for LiPo.\n\n'+action+' anyway?');
  return true;
}
function start(){
  if(!checkCutoff('Start'))return;
  const low=Math.round(parseFloat($('volt').value)*1000)<lim().warn;
  act('/start?cells='+$('cells').value+'&volt='+$('volt').value+'&amps='+$('amps').value+(low?'&confirm=1':''));
}

// ----- presets -----
let presets=[];
function selP(){const n=$('preset').value;return presets.find(p=>p.name==n);}
function updLabel(){
  const p=selP();
  $('goBtn').textContent='Start: '+(p?p.name+'  \u2013  ':'')+$('cells').value+'S at '+($('amps').value/1000)+' A';
  $('delBtn').disabled=!p;
}
function remember(n){try{localStorage.setItem('fdPreset',n);}catch(e){}}
function loadPresets(keep){
  return fetch('/presets').then(r=>r.json()).then(d=>{
    presets=d.presets;
    const s=$('preset');s.length=1;
    presets.forEach(p=>{const o=document.createElement('option');o.value=p.name;
      o.textContent=p.name+'  ('+p.cells+'S, '+(p.amps/1000)+' A, '+(p.cutoffMv/1000).toFixed(2)+' V)';s.appendChild(o);});
    s.value=presets.some(p=>p.name==keep)?keep:'';
    updLabel();
  });
}
function usePreset(){
  const p=selP();
  if(p){$('cells').value=p.cells;$('volt').value=(p.cutoffMv/1000).toFixed(2);$('amps').value=p.amps;say('Preset "'+p.name+'" loaded');}
  remember($('preset').value);updLabel();
}
function fieldChanged(){
  const p=selP();
  if(p&&($('cells').value!=p.cells||Math.round($('volt').value*1000)!=p.cutoffMv||$('amps').value!=p.amps)){
    $('preset').value='';remember('');say('Settings changed, no longer using preset "'+p.name+'"');
  }
  updLabel();
}
function savePreset(){
  if(!checkCutoff('Save this preset'))return;
  const cur=selP();
  let n=prompt('Name this preset (e.g. the drone or pack name).\nSaves '+$('cells').value+'S, '+($('amps').value/1000)+' A, '+$('volt').value+' V/cell.',cur?cur.name:'');
  if(n===null)return;n=n.trim();if(!n)return;
  if(presets.some(p=>p.name==n)&&(!cur||cur.name!=n)&&!confirm('Replace the existing "'+n+'" preset?'))return;
  fetch('/presetsave',{method:'POST',body:new URLSearchParams({name:n,cells:$('cells').value,volt:$('volt').value,amps:$('amps').value})})
  .then(r=>r.json()).then(d=>{say(d.msg);if(d.success){remember(n);return loadPresets(n);}}).catch(()=>say('Network error'));
}
function delPreset(){
  const p=selP();if(!p||!confirm('Delete preset "'+p.name+'"?'))return;
  fetch('/presetdel',{method:'POST',body:new URLSearchParams({name:p.name})})
  .then(r=>r.json()).then(d=>{say(d.msg);remember('');return loadPresets('');}).catch(()=>say('Network error'));
}
['cells','volt','amps'].forEach(id=>$(id).addEventListener('change',fieldChanged));
let lastP='';try{lastP=localStorage.getItem('fdPreset')||'';}catch(e){}
loadPresets(lastP).then(()=>{if(selP())usePreset();}).catch(()=>updLabel());
function scan(){
  $('scanBtn').disabled=true;$('scanBtn').textContent='Scanning (5 s)...';
  fetch('/scan').then(r=>r.json()).then(d=>{
    if(!d.success){$('list').textContent=d.msg;return;}
    devices=d.devices;render();
  }).catch(()=>$('list').textContent='Network error')
  .finally(()=>{$('scanBtn').disabled=false;$('scanBtn').textContent='Scan for FD-200';});
}
function render(){
  const all=$('showAll').checked,l=$('list');l.innerHTML='';
  devices.filter(x=>all||x.match).forEach(x=>{
    const b=document.createElement('button');b.className='dev';
    b.innerHTML='<b></b><br><small></small>';
    b.querySelector('b').textContent=(x.name||'(no name)')+(x.match?'  \u2713':'');
    b.querySelector('small').textContent=x.addr+'  \u00B7  '+x.rssi+' dBm';
    b.onclick=()=>pick(x);l.appendChild(b);
  });
  if(!l.children.length)l.textContent=devices.length?'No FD-200 found. Tick "show all" to see everything.':'Nothing found. Make sure the FD-200 is powered and the phone app is closed (a connected FD-200 stops advertising).';
}
function pick(x){
  fetch('/select?addr='+encodeURIComponent(x.addr)+'&type='+x.type+'&name='+encodeURIComponent(x.name||''))
  .then(r=>r.json()).then(d=>{say(d.msg);$('list').innerHTML='';});
}
function forget(){fetch('/forget').then(r=>r.json()).then(d=>say(d.msg));}
function fmtTime(ms){const t=Math.floor(ms/1000),h=Math.floor(t/3600),m=Math.floor(t/60)%60,s=t%60;
  return (h?h+':'+String(m).padStart(2,'0'):m)+':'+String(s).padStart(2,'0');}
let lastStatus=null,autoBusy=false;
function setAuto(){
  const c=$('autoChk'),on=c.checked,s=lastStatus;
  if(on){
    const cur=s&&s.setCells?' ('+s.setCells+'S at '+(s.setCurrentMa/1000)+' A to '+(s.setCutoffMv/1000).toFixed(2)+' V/cell)':'';
    if(!confirm('Turn on auto-discharge?\n\nIf a battery is connected, the FD-200 may START DISCHARGING within a few seconds using whatever is set on the device'+cur+'.')){c.checked=false;return;}
  }
  autoBusy=true;c.disabled=true;say('Sending...');
  fetch('/auto?on='+(on?1:0)).then(r=>r.json()).then(d=>{
    say(d.msg+(!on&&s&&s.state.startsWith('discharging')?'. Note: a discharge already running keeps going, tap Stop to end it.':''));
  }).catch(()=>say('Network error')).finally(()=>{autoBusy=false;poll();});
}
function poll(){fetch('/status').then(r=>r.json()).then(s=>{
  let t;
  if(!s.addr)t='No discharger selected. Tap Scan.';
  else t=(s.name||'FD-200')+'\n'+s.addr+'\n'+(s.connected?'\u25CF Connected':'\u25CB Not connected, retrying...');
  $('dev').textContent=t;
  if(s.error){const e=document.createElement('div');e.className='err';e.textContent=s.error;$('dev').appendChild(e);}
  showWifi(s.w);
  $('goBtn').disabled=$('stopBtn').disabled=!s.connected;
  lastStatus=s;
  if(!autoBusy){$('autoChk').checked=s.auto==1;$('autoChk').disabled=!s.connected||s.auto<0;}
  if(!s.connected){$('live').textContent='Not connected';return;}
  const run=s.state.startsWith('discharging');
  $('live').textContent='State: '+s.state+
   '\nPack: '+(s.packMv/1000).toFixed(2)+' V'+
   (run?'\nCurrent: '+(s.actualMa/1000).toFixed(2)+' A   Power: '+Math.round(s.packMv*s.actualMa/1e6)+' W':'')+
   '\nRun time: '+fmtTime(s.elapsedMs)+
   '\nDischarged: '+s.capMah+' mAh / '+(s.energyMwh/1000).toFixed(2)+' Wh'+
   (s.tempC>=0?'\nTemperature: '+s.tempC+' \u00B0C':'')+
   '\nDevice setting: '+s.setCells+'S to '+(s.setCutoffMv/1000).toFixed(2)+' V/cell at '+(s.setCurrentMa/1000).toFixed(1)+' A'+
   (s.cycles?'\nLifetime: '+s.cycles+' discharges, '+(s.lifeMwh/1000).toFixed(1)+' Wh':'');
  const dg=document.createElement('div');dg.style.cssText='font-size:11px;color:#718096;margin-top:6px';
  dg.textContent='Link: '+s.diag;$('live').appendChild(dg);
}).catch(()=>{});}
let wFilled=false,wLast=null,nets=[];
function wsay(t){$('wmsg').style.display='block';$('wmsg').textContent=t;}
function showWifi(w){
  wLast=w;
  if(!wFilled){$('ssid').value=w.ssid;wFilled=true;}
  $('wOn').checked=w.enabled;
  $('pass').placeholder=w.ssid?'Leave blank to keep the saved password':'';
  const ap='Hotspot "'+w.apSsid+'": '+(w.ap?'on (http://192.168.4.1)':'off');
  let t;
  switch(w.mode){
    case 'connected':t='\u25CF Connected to "'+w.ssid+'" ('+w.rssi+' dBm)\nhttp://'+w.ip+'  or  http://'+w.host+'.local';break;
    case 'connecting':t='Joining "'+w.ssid+'"...';break;
    case 'lost':t='\u25CB Lost "'+w.ssid+'", waiting for it to come back...';break;
    case 'waiting':t='\u25CB Not on home WiFi';break;
    default:t=w.ssid?'Home WiFi turned off (saved: "'+w.ssid+'")':'No home WiFi saved';
  }
  $('wstat').textContent=t+'\n'+ap;
  if(w.msg){const e=document.createElement('div');e.className='err';e.textContent=w.msg;$('wstat').appendChild(e);}
}
function wScan(){
  $('wScanBtn').disabled=true;$('wScanBtn').textContent='Scanning...';
  fetch('/wifiscan').then(r=>r.json()).then(d=>{
    const l=$('wlist');l.innerHTML='';
    if(!d.success){l.textContent=d.msg;return;}
    if(!d.nets.length){l.textContent='No networks found (the ESP32 only sees 2.4 GHz WiFi).';return;}
    d.nets.forEach(n=>{
      const b=document.createElement('button');b.className='dev';
      b.innerHTML='<b></b><br><small></small>';
      b.querySelector('b').textContent=n.ssid;
      b.querySelector('small').textContent=n.rssi+' dBm  \u00B7  '+(n.secure?'password':'open');
      b.onclick=()=>{$('ssid').value=n.ssid;$('pass').value='';$('wlist').innerHTML='';$('pass').focus();};
      l.appendChild(b);
    });
  }).catch(()=>$('wlist').textContent='Network error')
  .finally(()=>{$('wScanBtn').disabled=false;$('wScanBtn').textContent='Scan WiFi networks';});
}
function onHomeWifi(){return wLast&&wLast.mode=='connected'&&location.hostname!='192.168.4.1';}
function wPost(u,body){return fetch(u,{method:'POST',body:new URLSearchParams(body)}).then(r=>r.json());}
function wSave(){
  const s=$('ssid').value.trim(),p=$('pass').value;
  if(!s){wsay('Enter or pick a network name');return;}
  if(p.length&&p.length<8){wsay('WiFi passwords are at least 8 characters');return;}
  wPost('/wifisave',{ssid:s,pass:p}).then(d=>{wsay(d.msg);$('pass').value='';}).catch(()=>wsay('Network error'));
}
function wEnable(){
  const on=$('wOn').checked;
  if(!on&&onHomeWifi()&&!confirm('This page is using home WiFi and will drop. Reconnect by joining the "'+wLast.apSsid+'" WiFi and opening http://192.168.4.1. Continue?')){$('wOn').checked=true;return;}
  wPost('/wifienable',{on:on?1:0}).then(d=>{wsay(d.msg);if(!d.success)$('wOn').checked=!on;}).catch(()=>wsay('Network error'));
}
function wForget(){
  if(onHomeWifi()&&!confirm('This page is using home WiFi and will drop. Reconnect by joining the "'+wLast.apSsid+'" WiFi and opening http://192.168.4.1. Continue?'))return;
  wPost('/wififorget',{}).then(d=>{wsay(d.msg);$('ssid').value='';$('pass').value='';}).catch(()=>wsay('Network error'));
}
setInterval(poll,2000);poll();
</script></body></html>
)rawliteral";

void handleRoot() { server.send_P(200, "text/html; charset=utf-8", INDEX_HTML); }

// ---------- Presets: stored as one string, one preset per line: name \t cells \t cutoffMv \t currentMa ----------
void loadPresets() {
  presets.clear();
  prefs.begin("presets", true);
  String raw = prefs.getString("list", "");
  prefs.end();
  int pos = 0;
  while (pos < (int)raw.length()) {
    int nl = raw.indexOf('\n', pos);
    if (nl < 0) nl = raw.length();
    String line = raw.substring(pos, nl);
    pos = nl + 1;
    int a = line.indexOf('\t'), b = line.indexOf('\t', a + 1), c = line.indexOf('\t', b + 1);
    if (a <= 0 || b < 0 || c < 0) continue;
    Preset p;
    p.name      = line.substring(0, a);
    p.cells     = (uint8_t)line.substring(a + 1, b).toInt();
    p.cutoffMv  = (uint16_t)line.substring(b + 1, c).toInt();
    p.currentMa = (uint16_t)line.substring(c + 1).toInt();
    presets.push_back(p);
  }
  Serial.printf("%u preset(s) loaded\n", (unsigned)presets.size());
}

void savePresets() {
  std::sort(presets.begin(), presets.end(), [](const Preset& x, const Preset& y) {
    String a = x.name, b = y.name;
    a.toLowerCase(); b.toLowerCase();
    return a < b;
  });
  String raw;
  for (auto& p : presets)
    raw += p.name + "\t" + String(p.cells) + "\t" + String(p.cutoffMv) + "\t" + String(p.currentMa) + "\n";
  prefs.begin("presets", false);
  prefs.putString("list", raw);
  prefs.end();
}

bool validSettings(int cells, float volt, long amps) {
  bool ampsOk = (amps == 5000 || amps == 10000 || amps == 15000 || amps == 20000 || amps == 25000);
  long mv = lroundf(volt * 1000.0f);
  return cells >= 1 && cells <= 8 && mv >= CUTOFF_MIN_MV && mv <= CUTOFF_MAX_MV && ampsOk;
}

String cleanName(String n) {
  n.replace("\t", " ");
  n.replace("\n", " ");
  n.replace("\r", " ");
  n.trim();
  if (n.length() > 32) n = n.substring(0, 32);
  return n;
}

void handlePresets() {
  String j = "{\"presets\":[";
  for (size_t i = 0; i < presets.size(); i++) {
    if (i) j += ",";
    j += "{\"name\":\"" + jsonEsc(presets[i].name) + "\",\"cells\":" + String(presets[i].cells) +
         ",\"cutoffMv\":" + String(presets[i].cutoffMv) + ",\"amps\":" + String(presets[i].currentMa) + "}";
  }
  j += "]}";
  server.send(200, "application/json", j);
}

void handlePresetSave() {
  String n = cleanName(server.arg("name"));
  int cells = server.arg("cells").toInt();
  float volt = server.arg("volt").toFloat();
  long amps = server.arg("amps").toInt();
  if (!n.length()) { sendResult(false, "Preset needs a name"); return; }
  if (!validSettings(cells, volt, amps)) { sendResult(false, "Invalid settings, preset not saved"); return; }

  Preset p;
  p.name = n;
  p.cells = cells;
  p.cutoffMv = (uint16_t)lroundf(volt * 1000.0f);
  p.currentMa = (uint16_t)amps;

  bool replaced = false;
  for (auto& x : presets) if (x.name == n) { x = p; replaced = true; break; }
  if (!replaced) {
    if (presets.size() >= MAX_PRESETS) { sendResult(false, "Preset list is full, delete one first"); return; }
    presets.push_back(p);
  }
  savePresets();
  sendResult(true, String(replaced ? "Updated" : "Saved") + " preset \"" + n + "\"");
}

void handlePresetDel() {
  String n = server.arg("name");
  for (size_t i = 0; i < presets.size(); i++) {
    if (presets[i].name == n) {
      presets.erase(presets.begin() + i);
      savePresets();
      sendResult(true, "Deleted preset \"" + n + "\"");
      return;
    }
  }
  sendResult(false, "Preset not found");
}

void handleStart() {
  int cells = server.arg("cells").toInt();
  float volt = server.arg("volt").toFloat();
  long amps = server.arg("amps").toInt();
  long mv = lroundf(volt * 1000.0f);
  if (mv < CUTOFF_MIN_MV || mv > CUTOFF_MAX_MV) {
    sendResult(false, "Cutoff must be between " + String(CUTOFF_MIN_MV / 1000.0, 2) + " and " +
                      String(CUTOFF_MAX_MV / 1000.0, 2) + " V per cell");
    return;
  }
  if (mv < CUTOFF_WARN_MV && server.arg("confirm") != "1") {
    sendResult(false, "Cutoff below " + String(CUTOFF_WARN_MV / 1000.0, 2) + " V/cell needs confirmation");
    return;
  }
  if (!validSettings(cells, volt, amps)) {
    sendResult(false, "Invalid parameters");
    return;
  }
  jobCells = cells;
  jobCutoffMv = (uint16_t)mv;
  jobCurrentMa = (uint16_t)amps;
  String msg;
  bool ok = submitJob(JOB_START, 5000, msg);
  sendResult(ok, msg);
}

void handleStop() {
  String msg;
  bool ok = submitJob(JOB_STOP, 5000, msg);
  sendResult(ok, msg);
}

void handleAuto() {
  jobAutoOn = server.arg("on") == "1";
  String msg;
  bool ok = submitJob(JOB_AUTO, 5000, msg);
  sendResult(ok, msg);
}

void handleScan() {
  String msg;
  if (!submitJob(JOB_SCAN, 45000, msg)) { sendResult(false, msg); return; }
  String j = "{\"success\":true,\"devices\":[";
  for (size_t i = 0; i < found.size(); i++) {
    if (i) j += ",";
    j += "{\"addr\":\"" + found[i].addr + "\",\"name\":\"" + jsonEsc(found[i].name) +
         "\",\"rssi\":" + String(found[i].rssi) + ",\"type\":" + String(found[i].type) +
         ",\"match\":" + (found[i].match ? "true" : "false") + "}";
  }
  j += "]}";
  server.send(200, "application/json", j);
}

void handleSelect() {
  String a = server.arg("addr");
  a.toLowerCase();
  if (a.length() != 17) { sendResult(false, "Bad address"); return; }
  uint8_t t = (uint8_t)server.arg("type").toInt();
  String n = server.arg("name");

  xSemaphoreTake(cfgMutex, portMAX_DELAY);
  cfgAddr = a; cfgType = t; cfgName = n; lastError = "";
  xSemaphoreGive(cfgMutex);

  prefs.begin("fd200", false);
  prefs.putString("addr", a);
  prefs.putUChar("type", t);
  prefs.putString("name", n);
  prefs.end();

  dropRequested = true;
  reconnectRequested = true;
  sendResult(true, "Saved. Connecting to " + (n.length() ? n : a) + "...");
}

void handleForget() {
  xSemaphoreTake(cfgMutex, portMAX_DELAY);
  cfgAddr = ""; cfgName = ""; cfgType = 0; lastError = "";
  xSemaphoreGive(cfgMutex);
  prefs.begin("fd200", false);
  prefs.clear();
  prefs.end();
  dropRequested = true;
  sendResult(true, "Discharger forgotten");
}

String linkDiag() {
  String s = "MTU " + String((unsigned)linkMtu) + ", replies ok " + String((unsigned long)rxOk) +
             " / bad " + String((unsigned long)rxBad) + " / cut short " + String((unsigned long)rxCut);
  if (rxCut) {
    s += " (last 0x" + String((unsigned)lastCutCmd, HEX) + ": " + String((unsigned)lastCutLen) +
         " of " + String((unsigned)lastCutNeed) + " bytes)";
  }
  return s;
}

void handleStatus() {
  xSemaphoreTake(cfgMutex, portMAX_DELAY);
  String a = cfgAddr, n = cfgName, e = lastError;
  xSemaphoreGive(cfgMutex);

  String st;
  if (runState == 0x00) st = "idle";
  else if (runState == 0x02) st = "discharging (ramping up)";
  else if (runState == 0x03) st = "discharging";
  else if (runState == 0xFF) st = "unknown";
  else st = "code 0x" + String(runState, HEX);

  String j = "{\"connected\":" + String(bleConnected ? "true" : "false") +
             ",\"addr\":\"" + a + "\",\"name\":\"" + jsonEsc(n) + "\",\"error\":\"" + jsonEsc(e) +
             "\",\"state\":\"" + st + "\",\"packMv\":" + String(packMv);
  j += ",\"setCells\":" + String(setCells) + ",\"setCutoffMv\":" + String(setCutoffMv) +
       ",\"setCurrentMa\":" + String(setCurrentMa) + ",\"elapsedMs\":" + String(elapsedMs) +
       ",\"actualMa\":" + String(actualMa) + ",\"capMah\":" + String(capMah) +
       ",\"energyMwh\":" + String(energyMwh) + ",\"cycles\":" + String(cycles) +
       ",\"lifeMwh\":" + String(lifeMwh) + ",\"tempC\":" + String(tempC) +
       ",\"diag\":\"" + linkDiag() + "\"" +
       ",\"auto\":" + String((int)autoMode) +
       ",\"cutMinMv\":" + String(CUTOFF_MIN_MV) + ",\"cutWarnMv\":" + String(CUTOFF_WARN_MV) +
       ",\"cutMaxMv\":" + String(CUTOFF_MAX_MV);

  static const char* modeNames[] = {"off", "connecting", "connected", "lost", "waiting"};
  bool up = (wState == W_CONNECTED);
  j += ",\"w\":{\"mode\":\"" + String(modeNames[wState]) + "\",\"enabled\":" + (wifiEnabled ? "true" : "false") +
       ",\"ssid\":\"" + jsonEsc(wifiSsid) + "\",\"ip\":\"" + (up ? WiFi.localIP().toString() : String("")) +
       "\",\"rssi\":" + String(up ? WiFi.RSSI() : 0) + ",\"host\":\"" HOSTNAME "\",\"ap\":" + (apOn ? "true" : "false") +
       ",\"apSsid\":\"" + jsonEsc(AP_SSID) + "\",\"msg\":\"" + jsonEsc(wifiMsg) + "\"}}";
  server.send(200, "application/json", j);
}

void handleWifiScan() {
  if (wState == W_CONNECTING) { sendResult(false, "Busy joining WiFi, try again in a few seconds"); return; }
  bool hadSta = WiFi.getMode() & WIFI_STA;
  if (!hadSta) WiFi.enableSTA(true);   // scanning needs the STA side
  int n = WiFi.scanNetworks();
  if (n < 0) {
    if (!hadSta) WiFi.enableSTA(false);
    sendResult(false, "Scan failed, try again");
    return;
  }

  struct Net { String ssid; int rssi; bool secure; };
  std::vector<Net> nets;
  for (int i = 0; i < n; i++) {
    String s = WiFi.SSID(i);
    if (!s.length()) continue;                     // hidden network
    int r = WiFi.RSSI(i);
    bool sec = WiFi.encryptionType(i) != WIFI_AUTH_OPEN;
    bool dup = false;
    for (auto& x : nets) if (x.ssid == s) { if (r > x.rssi) x.rssi = r; dup = true; break; }
    if (!dup) nets.push_back({s, r, sec});
  }
  WiFi.scanDelete();
  if (!hadSta) WiFi.enableSTA(false);
  std::sort(nets.begin(), nets.end(), [](const Net& a, const Net& b) { return a.rssi > b.rssi; });

  String j = "{\"success\":true,\"nets\":[";
  for (size_t i = 0; i < nets.size(); i++) {
    if (i) j += ",";
    j += "{\"ssid\":\"" + jsonEsc(nets[i].ssid) + "\",\"rssi\":" + String(nets[i].rssi) +
         ",\"secure\":" + (nets[i].secure ? "true" : "false") + "}";
  }
  j += "]}";
  server.send(200, "application/json", j);
}

void handleWifiSave() {
  String s = server.arg("ssid");
  String p = server.arg("pass");
  s.trim();
  if (s.length() < 1 || s.length() > 32) { sendResult(false, "Network name must be 1-32 characters"); return; }
  if (p.length() == 0 && s == wifiSsid) p = wifiPass;   // blank = keep the saved password
  if (p.length() > 0 && (p.length() < 8 || p.length() > 63)) { sendResult(false, "Password must be 8-63 characters"); return; }

  wifiSsid = s; wifiPass = p; wifiEnabled = true;
  prefs.begin("wifi", false);
  prefs.putString("ssid", s);
  prefs.putString("pass", p);
  prefs.putBool("on", true);
  prefs.end();

  wifiApplyPending = true;
  sendResult(true, "Saved. Joining \"" + s + "\" now; the address will show above once connected "
                   "(about 10-20 s). If you're on the hotspot it may blip while the ESP32 switches channel.");
}

void handleWifiEnable() {
  bool on = server.arg("on") == "1";
  if (on && !wifiSsid.length()) { sendResult(false, "No WiFi saved yet. Pick a network and save it first."); return; }
  wifiEnabled = on;
  prefs.begin("wifi", false);
  prefs.putBool("on", on);
  prefs.end();
  wifiApplyPending = true;
  String msg;
  if (on) msg = "Home WiFi on, joining \"" + wifiSsid + "\"...";
  else    msg = String("Home WiFi off. Hotspot \"") + AP_SSID + "\" is on at http://192.168.4.1";
  sendResult(true, msg);
}

void handleWifiForget() {
  wifiSsid = ""; wifiPass = ""; wifiEnabled = false; wifiMsg = "";
  prefs.begin("wifi", false);
  prefs.clear();
  prefs.end();
  wifiApplyPending = true;
  sendResult(true, String("WiFi settings cleared. Hotspot \"") + AP_SSID + "\" is on at http://192.168.4.1");
}

// ==================================================================
// Setup / loop
// ==================================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  cfgMutex = xSemaphoreCreateMutex();

  prefs.begin("fd200", true);
  cfgAddr = prefs.getString("addr", "");
  cfgType = prefs.getUChar("type", 0);
  cfgName = prefs.getString("name", "");
  prefs.end();
  if (cfgAddr.length()) {
    Serial.println("Saved discharger: " + cfgAddr);
    reconnectRequested = true;
  } else {
    Serial.println("No discharger saved, use Scan in the web UI");
  }

  BLEDevice::init("ESP32_FD200");

  prefs.begin("wifi", true);
  wifiSsid = prefs.getString("ssid", "");
  wifiPass = prefs.getString("pass", "");
  wifiEnabled = prefs.getBool("on", false);
  prefs.end();

  WiFi.persistent(false);       // we keep credentials ourselves, don't let the WiFi library store its own
  WiFi.setAutoReconnect(true);
  WiFi.setHostname(HOSTNAME);
  if (wifiEnabled && wifiSsid.length()) {
    WiFi.mode(WIFI_STA);        // hotspot starts only if this fails (see wifiLoop)
    beginSTA();
  } else {
    WiFi.mode(WIFI_AP);
    startAP();
    setWState(W_OFF);
    Serial.println("No home WiFi saved/enabled, hotspot only");
  }
  if (MDNS.begin(HOSTNAME)) MDNS.addService("http", "tcp", 80);

  loadPresets();

  server.on("/", HTTP_GET, handleRoot);
  server.on("/presets", HTTP_GET, handlePresets);
  server.on("/presetsave", HTTP_POST, handlePresetSave);
  server.on("/presetdel", HTTP_POST, handlePresetDel);
  server.on("/wifiscan", HTTP_GET, handleWifiScan);
  server.on("/wifisave", HTTP_POST, handleWifiSave);
  server.on("/wifienable", HTTP_POST, handleWifiEnable);
  server.on("/wififorget", HTTP_POST, handleWifiForget);
  server.on("/start", HTTP_GET, handleStart);
  server.on("/stop", HTTP_GET, handleStop);
  server.on("/auto", HTTP_GET, handleAuto);
  server.on("/scan", HTTP_GET, handleScan);
  server.on("/select", HTTP_GET, handleSelect);
  server.on("/forget", HTTP_GET, handleForget);
  server.on("/status", HTTP_GET, handleStatus);
  server.begin();

  xTaskCreatePinnedToCore(bleTask, "ble", 8192, nullptr, 1, nullptr, 1);
}

void loop() {
  server.handleClient();
  wifiLoop();
  delay(2);
}
