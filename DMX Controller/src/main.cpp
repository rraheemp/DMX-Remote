/*
 * DMX512 test transmitter - ESP32-C3 + MAX485
 *
 * Wiring (MAX485 module):
 *   VCC   -> 5V            GND -> GND (+ XLR pin 1)
 *   DI    -> GPIO4 (DMX_TX_PIN)
 *   DE+RE -> tied together, to 3.3V or to GPIO6 (DMX_EN_PIN, driven HIGH)
 *   RO    -> not connected
 *   A     -> XLR pin 3 (Data+)      B -> XLR pin 2 (Data-)
 *
 * Fixtures: CHnd = 11CH, Addr = A001 (both, mirrored) or A001 / A012 (independent).
 *
 * Behaviour on boot:
 *   1. HOLD  : both heads at end stop, white, fixed pattern, no strobe, ~50% dimmer
 *   2. SWEEP : after HOLD_MS, pan coarse sweeps 0 -> 255 -> 0 forever
 *   Any serial command switches to MANUAL mode (sweep stops).
 *
 * Commands work from USB serial (115200, newline-terminated) or over BLE:
 *   BLE: device "DMX-Controller", Nordic UART Service (NUS). Use any BLE UART app
 *   (nRF Connect, Bluefruit Connect, Serial Bluetooth Terminal). Write commands to
 *   the RX characteristic; replies come back as notifications on TX.
 *
 * Commands:
 *   s <slot> <value>   set one slot, e.g. "s 8 255"
 *   f <slot> v1 v2 ... set consecutive slots from <slot> (silent; used by the web remote)
 *   z                  all slots to 0
 *   d                  dump slots 1-22
 *   sweep              resume the pan sweep
 *
 * Eye-protection zone (per head, stored in flash):
 *   The zone is the band of tilt between a FAR edge (beam at eye level at the back of
 *   the crowd) and a NEAR edge (eye level of the front row), interpolated across pan.
 *   Mark each edge at several pan positions to follow the crowd around the sides.
 *   zm <A|B> <F|N>     mark head's current commanded position as a far/near edge point
 *   zc <A|B>           clear that head's zone
 *   ze <0|1>           zone enforcement off/on (off = setup mode, beam visible everywhere)
 *   zs <margin> <settleMs> <panRate> <tiltRate>   margin in tilt units, rates in units/s
 *   zp <A|B> p,t p,t ...   add a no-go polygon (3-12 vertices, raw 16-bit pan,tilt)
 *   zx <A|B> [i]       delete polygon i, or all polygons of that head
 *   za <0|1>           allow the fixtures' built-in Auto/Sound programs while a zone is
 *                      active (they ignore the zone!). Not saved: off again after reboot.
 *   zg                 report zones/config (lines starting with "Z")
 *   reset <A|B|all>    fixture reset (ch11 held at 255 for 6 s); beam kept dark while it re-homes
 *   help               list commands
 *   A head is dark if it is inside the far/near band OR inside any of its polygons.
 *
 * The zone is applied at the output stage, so it protects every source (sliders,
 * presets, sweep, future shows). A head is blanked (dimmer 0) while its commanded
 * position, its estimated physical position, or the path between them crosses the
 * zone, and for settleMs after it has cleared. Built-in programs (ch10 >= 60) are
 * blocked while the zone is active because they ignore our pan/tilt (unless allowed with "za 1").
 */
#include <Arduino.h>
#include <stdarg.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#if ESP_ARDUINO_VERSION_MAJOR < 3
#include <BLE2902.h>
#endif

// ---------------- Pins / config ----------------
static constexpr int DMX_TX_PIN = 4;
static constexpr int DMX_EN_PIN = 6;      // DE/RE; harmless if tied to 3.3V instead
static constexpr int DMX_RX_PIN = -1;     // TX only
static constexpr uint16_t DMX_SLOTS = 512;      // full universe (safest for cheap fixtures)
static constexpr uint32_t FRAME_PERIOD_MS = 25; // ~40 Hz refresh
static constexpr uint32_t HOLD_MS = 5000;

// 11CH personality offsets (1-based within the fixture's footprint)
enum : uint8_t {
  CH_PAN = 1, CH_PAN_FINE, CH_TILT, CH_TILT_FINE, CH_COLOR, CH_PATTERN,
  CH_STROBE, CH_DIMMER, CH_SPEED, CH_AUTO, CH_RESET
};
static constexpr uint16_t HEAD_A = 1;   // start address of unit A
static constexpr uint16_t HEAD_B = 12;  // start address of unit B (ignored if both on A001)

// ---------------- DMX buffer ----------------
// dmx[0] = start code (0x00), dmx[1..512] = slots
static uint8_t dmx[DMX_SLOTS + 1] = {0};

static inline void setCh(uint16_t start, uint8_t ch, uint8_t value) {
  uint16_t slot = start + ch - 1;
  if (slot >= 1 && slot <= DMX_SLOTS) dmx[slot] = value;
}

// ---------------- Eye-protection zone ----------------
static constexpr int ZONE_MAX_PTS = 12;
static constexpr int32_t PAN_360 = 43690;            // 360 deg of a 540 deg pan in 16-bit units

struct ZonePt { uint16_t pan, tilt; };
struct Zone {
  uint8_t nFar = 0, nNear = 0;
  ZonePt far[ZONE_MAX_PTS];
  ZonePt near_[ZONE_MAX_PTS];
};
struct ZoneCfg {
  uint8_t  enabled = 1;
  uint16_t margin = 1500;      // tilt units added to both sides of the band
  uint16_t settleMs = 300;     // extra dark time after the head has cleared the zone
  uint16_t panRate = 14000;    // pan units/s the head can move  (~115 deg/s, conservative)
  uint16_t tiltRate = 14000;   // tilt units/s
};
static constexpr int POLY_MAX = 6;          // polygons per head
static constexpr int POLY_PTS = 12;         // vertices per polygon
struct Poly { uint8_t n = 0; ZonePt v[POLY_PTS]; };
struct HeadPolys { Poly p[POLY_MAX]; };
static HeadPolys polys[2];
static Zone zones[2];
static ZoneCfg zcfg;
static bool allowAuto = false;                 // za 1: let built-in programs run unprotected
static constexpr uint32_t RESET_HOLD_MS = 6000, RESET_DARK_MS = 15000;
static uint32_t resetStart[2] = {0, 0};        // 0 = no reset running
static Preferences prefs;

static const uint16_t HEAD_START[2] = { HEAD_A, HEAD_B };

static bool bandDefined(int h) { return zones[h].nFar > 0 && zones[h].nNear > 0; }
static int  polyCount(int h) { int c = 0; for (auto &q : polys[h].p) c += q.n >= 3; return c; }
static bool zoneDefined(int h) { return bandDefined(h) || polyCount(h) > 0; }
static bool zoneActive(int h)  { return zcfg.enabled && zoneDefined(h); }

static void saveZones() {
  prefs.begin("dmx", false);
  prefs.putBytes("zoneA", &zones[0], sizeof(Zone));
  prefs.putBytes("zoneB", &zones[1], sizeof(Zone));
  prefs.putBytes("polyA", &polys[0], sizeof(HeadPolys));
  prefs.putBytes("polyB", &polys[1], sizeof(HeadPolys));
  ZoneCfg c = zcfg; c.enabled = 1;                  // always boot with protection on
  prefs.putBytes("zcfg", &c, sizeof(ZoneCfg));
  prefs.end();
}

static void loadZones() {
  prefs.begin("dmx", true);
  if (prefs.getBytesLength("zoneA") == sizeof(Zone)) prefs.getBytes("zoneA", &zones[0], sizeof(Zone));
  if (prefs.getBytesLength("zoneB") == sizeof(Zone)) prefs.getBytes("zoneB", &zones[1], sizeof(Zone));
  if (prefs.getBytesLength("polyA") == sizeof(HeadPolys)) prefs.getBytes("polyA", &polys[0], sizeof(HeadPolys));
  if (prefs.getBytesLength("polyB") == sizeof(HeadPolys)) prefs.getBytes("polyB", &polys[1], sizeof(HeadPolys));
  if (prefs.getBytesLength("zcfg") == sizeof(ZoneCfg)) prefs.getBytes("zcfg", &zcfg, sizeof(ZoneCfg));
  prefs.end();
  zcfg.enabled = 1;
}

static void addPoint(ZonePt *pts, uint8_t &n, uint16_t pan, uint16_t tilt) {
  // keep sorted by pan; replace a point at (nearly) the same pan
  for (int i = 0; i < n; i++) {
    if (abs((int)pts[i].pan - (int)pan) < 300) { pts[i].tilt = tilt; return; }
  }
  if (n >= ZONE_MAX_PTS) return;
  int i = n++;
  while (i > 0 && pts[i - 1].pan > pan) { pts[i] = pts[i - 1]; i--; }
  pts[i] = { pan, tilt };
}

// tilt of an edge at a given pan: linear between marked points, flat beyond the ends
static int32_t edgeTilt(const ZonePt *pts, uint8_t n, int32_t pan) {
  if (pan <= pts[0].pan) return pts[0].tilt;
  if (pan >= pts[n - 1].pan) return pts[n - 1].tilt;
  for (int i = 1; i < n; i++) {
    if (pan <= pts[i].pan) {
      int32_t p0 = pts[i - 1].pan, p1 = pts[i].pan, t0 = pts[i - 1].tilt, t1 = pts[i].tilt;
      return t0 + (t1 - t0) * (pan - p0) / (p1 - p0);
    }
  }
  return pts[n - 1].tilt;
}

// The same real direction exists at pan and pan +/- 360 deg; map into the marked range.
static int32_t normalisePan(const Zone &z, int32_t pan) {
  int32_t lo = min(z.far[0].pan, z.near_[0].pan);
  int32_t hi = max(z.far[z.nFar - 1].pan, z.near_[z.nNear - 1].pan);
  if (pan < lo && pan + PAN_360 <= hi) return pan + PAN_360;
  if (pan > hi && pan - PAN_360 >= lo) return pan - PAN_360;
  return pan;
}

static bool inBand(int h, int32_t pan, int32_t tilt) {
  const Zone &z = zones[h];
  pan = normalisePan(z, pan);
  int32_t a = edgeTilt(z.far, z.nFar, pan), b = edgeTilt(z.near_, z.nNear, pan);
  return tilt >= min(a, b) - zcfg.margin && tilt <= max(a, b) + zcfg.margin;
}

// even-odd ray casting
static bool inPoly(const Poly &q, int32_t pan, int32_t tilt) {
  bool c = false;
  for (int i = 0, j = q.n - 1; i < q.n; j = i++) {
    int32_t xi = q.v[i].pan, yi = q.v[i].tilt, xj = q.v[j].pan, yj = q.v[j].tilt;
    if ((yi > tilt) != (yj > tilt) &&
        pan < (int64_t)(xj - xi) * (tilt - yi) / (yj - yi) + xi) c = !c;
  }
  return c;
}

// polygon test with margin (point + 4 offsets) and pan wrap (same direction at +/-360 deg)
static bool inPolys(int h, int32_t pan, int32_t tilt) {
  const int32_t m = zcfg.margin;
  const int32_t off[5][2] = { {0, 0}, {m, 0}, {-m, 0}, {0, m}, {0, -m} };
  for (const Poly &q : polys[h].p) {
    if (q.n < 3) continue;
    for (int32_t w = -PAN_360; w <= PAN_360; w += PAN_360)
      for (auto &o : off)
        if (inPoly(q, pan + w + o[0], tilt + o[1])) return true;
  }
  return false;
}

static bool inZone(int h, int32_t pan, int32_t tilt) {
  return (bandDefined(h) && inBand(h, pan, tilt)) || inPolys(h, pan, tilt);
}

// Model of where each head physically is (motors lag the DMX command).
// Rates are deliberately conservative: a slow estimate keeps the beam dark longer.
static float estPan[2], estTilt[2];
static bool estInit = false;
static uint32_t clearSince[2] = {0, 0};
static bool blanked[2] = {false, false}, blankedReported[2] = {false, false};

static void updateZoneModel(float dt) {
  for (int h = 0; h < 2; h++) {
    uint16_t s = HEAD_START[h];
    float cmdPan  = (dmx[s] << 8) | dmx[s + 1];
    float cmdTilt = (dmx[s + 2] << 8) | dmx[s + 3];
    if (!estInit) { estPan[h] = cmdPan; estTilt[h] = cmdTilt; }
    float speedScale = 1.0f + dmx[s + CH_SPEED - 1] / 32.0f;   // ch9: higher = slower head
    float maxP = zcfg.panRate * dt / speedScale, maxT = zcfg.tiltRate * dt / speedScale;
    float dp = cmdPan - estPan[h], dtl = cmdTilt - estTilt[h];
    estPan[h]  += (dp > maxP) ? maxP : (dp < -maxP ? -maxP : dp);
    estTilt[h] += (dtl > maxT) ? maxT : (dtl < -maxT ? -maxT : dtl);

    if (!zoneActive(h)) { blanked[h] = false; continue; }

    // check the path from where the head is to where it is going
    bool hit = false;
    for (int k = 0; k <= 32 && !hit; k++) {
      float f = k / 32.0f;
      hit = inZone(h, estPan[h] + (cmdPan - estPan[h]) * f, estTilt[h] + (cmdTilt - estTilt[h]) * f);
    }
    uint32_t now = millis();
    if (hit) { clearSince[h] = 0; blanked[h] = true; }
    else {
      if (clearSince[h] == 0) clearSince[h] = now ? now : 1;
      blanked[h] = (now - clearSince[h]) < zcfg.settleMs;
    }
  }
  estInit = true;
}

// ---------------- Frame transmit ----------------
static uint8_t out[DMX_SLOTS + 1];   // what actually goes on the wire (dmx + safety)

// BREAK is made by dropping the baud rate and sending 0x00:
// at 83,333 baud, start bit + 8 zero bits = 9 x 12 us = 108 us low (BREAK >= 88 us),
// then the 2 stop bits give 24 us high (MAB >= 8 us). Baud switch adds a few us more.
static constexpr uint32_t DMX_BAUD   = 250000;
static constexpr uint32_t BREAK_BAUD = 83333;

static void sendFrame() {
  Serial1.updateBaudRate(BREAK_BAUD);
  Serial1.write((uint8_t)0x00);
  Serial1.flush();                       // wait until BREAK + MAB are fully on the wire
  Serial1.updateBaudRate(DMX_BAUD);
  memcpy(out, dmx, sizeof(dmx));
  uint32_t now = millis();
  for (int h = 0; h < 2; h++) {
    uint16_t s = HEAD_START[h];
    if (blanked[h]) out[s + CH_DIMMER - 1] = 0;                        // beam off
    if (zoneActive(h) && !allowAuto && out[s + CH_AUTO - 1] >= 60)
      out[s + CH_AUTO - 1] = 0;                                        // no built-in shows
    if (resetStart[h]) {
      uint32_t t = now - resetStart[h];
      if (t < RESET_DARK_MS) {
        out[s + CH_DIMMER - 1] = 0;                                    // dark while re-homing
        out[s + CH_AUTO - 1] = 0;
        out[s + CH_RESET - 1] = (t < RESET_HOLD_MS) ? 255 : 0;
      } else {
        resetStart[h] = 0;
      }
    }
  }
  Serial1.write(out, sizeof(out));       // start code + 512 slots
  Serial1.flush();                       // ~22.7 ms; blocks so frames never overlap
}

// ---------------- Scenes ----------------
static void setHead(uint16_t start, uint16_t pan16, uint16_t tilt16) {
  setCh(start, CH_PAN,       pan16 >> 8);
  setCh(start, CH_PAN_FINE,  pan16 & 0xFF);
  setCh(start, CH_TILT,      tilt16 >> 8);
  setCh(start, CH_TILT_FINE, tilt16 & 0xFF);
  setCh(start, CH_COLOR,   0);    // first colour slot, assumed white (stay <= 139)
  setCh(start, CH_PATTERN, 0);    // fixed pattern (0-63)
  setCh(start, CH_STROBE,  0);    // no strobe; if beam is dark try 255
  setCh(start, CH_DIMMER,  128);  // ~50%
  setCh(start, CH_SPEED,   0);    // fastest movement
  setCh(start, CH_AUTO,    0);    // 0-59 = DMX control (60+ = built-in programs!)
  setCh(start, CH_RESET,   0);    // never 250-255 (resets after 5 s)
}

static void sceneEndStops() {
  setHead(HEAD_A, 0x0000, 0x0000);  // unit A at one end
  setHead(HEAD_B, 0xFFFF, 0xFFFF);  // unit B at the opposite end (if addressed A012)
}

// ---------------- Modes ----------------
enum class Mode { HOLD, SWEEP, MANUAL };
static Mode mode = Mode::HOLD;
static uint32_t modeStart = 0;

static void updateSweep(uint32_t now) {
  // triangle wave 0..255..0 over ~10 s
  uint32_t t = (now - modeStart) % 10000;
  uint8_t pan = (t < 5000) ? (t * 255 / 5000) : ((10000 - t) * 255 / 5000);
  setCh(HEAD_A, CH_PAN, pan);
  setCh(HEAD_A, CH_PAN_FINE, 0);
  setCh(HEAD_B, CH_PAN, pan);
  setCh(HEAD_B, CH_PAN_FINE, 0);
}

// ---------------- BLE (Nordic UART Service) ----------------
static const char *BLE_NAME     = "DMX-Controller";
static const char *NUS_SERVICE  = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char *NUS_RX       = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"; // phone -> ESP32 (write)
static const char *NUS_TX       = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"; // ESP32 -> phone (notify)

static BLECharacteristic *bleTx = nullptr;
static volatile int bleClients = 0;     // phone + PC can be connected at the same time

// BLE callbacks run in the Bluetooth task, so incoming commands are queued and
// executed in loop() - no DMX buffer access from two tasks at once.
static constexpr size_t CMD_LEN = 200;
struct Cmd { char text[CMD_LEN]; };
static QueueHandle_t cmdQueue;

static void queueCommands(const char *data, size_t len) {
  // one BLE write may hold several commands separated by \n, \r or ';'
  Cmd c; size_t n = 0;
  for (size_t i = 0; i <= len; i++) {
    char ch = (i < len) ? data[i] : '\n';
    if (ch == '\n' || ch == '\r' || ch == ';') {
      if (n) { c.text[n] = 0; xQueueSend(cmdQueue, &c, 0); n = 0; }
    } else if (n < CMD_LEN - 1) {
      c.text[n++] = ch;
    }
  }
}

class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer *) override {
    bleClients++;
    BLEDevice::startAdvertising();           // stay visible so another phone/PC can join
  }
  void onDisconnect(BLEServer *) override {
    if (bleClients > 0) bleClients--;
    BLEDevice::startAdvertising();
  }
};

class RxCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *ch) override {
    auto v = ch->getValue();                 // std::string (core 2.x) or String (core 3.x)
    queueCommands(v.c_str(), v.length());
  }
};

static void bleBegin() {
  cmdQueue = xQueueCreate(16, sizeof(Cmd));
  BLEDevice::init(BLE_NAME);
  BLEServer *srv = BLEDevice::createServer();
  srv->setCallbacks(new ServerCB());
  BLEService *svc = srv->createService(NUS_SERVICE);

  bleTx = svc->createCharacteristic(NUS_TX, BLECharacteristic::PROPERTY_NOTIFY);
#if ESP_ARDUINO_VERSION_MAJOR < 3
  bleTx->addDescriptor(new BLE2902());
#endif
  BLECharacteristic *rx = svc->createCharacteristic(
      NUS_RX, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCB());

  svc->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
}

// Send a reply to USB serial and, if a phone is connected, as BLE notifications
// (chunked to 20 bytes so it works with the default MTU).
static void reply(const char *fmt, ...) {
  char buf[400];
  va_list ap; va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n <= 0) return;
  if (n > (int)sizeof(buf) - 1) n = sizeof(buf) - 1;
  Serial.print(buf);
  if (bleClients > 0 && bleTx) {
    for (int i = 0; i < n; i += 20) {
      int len = (n - i > 20) ? 20 : (n - i);
      bleTx->setValue((uint8_t *)buf + i, len);
      bleTx->notify();
      delay(5);
    }
  }
}

// ---------------- Zone commands ----------------
static int headIndex(char c) { return (c == 'A' || c == 'a') ? 0 : (c == 'B' || c == 'b') ? 1 : -1; }

static void reportZones() {
  for (int h = 0; h < 2; h++) {
    int k = 0;
    for (int i = 0; i < POLY_MAX; i++) if (polys[h].p[i].n >= 3) k++;
    reply("ZPN %c %d\n", 'A' + h, k);
    k = 0;
    for (int i = 0; i < POLY_MAX; i++) {
      const Poly &q = polys[h].p[i];
      if (q.n < 3) continue;
      char line[200]; int p = snprintf(line, sizeof(line), "ZP %c %d", 'A' + h, k++);
      for (int v = 0; v < q.n; v++) p += snprintf(line + p, sizeof(line) - p, " %u,%u", q.v[v].pan, q.v[v].tilt);
      reply("%s\n", line);
    }
    for (int e = 0; e < 2; e++) {
      const ZonePt *pts = e ? zones[h].near_ : zones[h].far;
      uint8_t n = e ? zones[h].nNear : zones[h].nFar;
      char line[150]; int p = snprintf(line, sizeof(line), "Z %c %c", 'A' + h, e ? 'N' : 'F');
      for (int i = 0; i < n && p < (int)sizeof(line) - 14; i++)
        p += snprintf(line + p, sizeof(line) - p, " %u,%u", pts[i].pan, pts[i].tilt);
      reply("%s\n", line);
    }
  }
  reply("ZC %u %u %u %u %u %u\n", zcfg.enabled, zcfg.margin, zcfg.settleMs, zcfg.panRate, zcfg.tiltRate,
        allowAuto ? 1 : 0);
}

static const char *ZONE_HELP =
  "zone cmds:\n"
  " zm A|B F|N  mark far/near edge at head's position\n"
  " zc A|B      clear marked band\n"
  " zp A|B p,t p,t p,t ...  add shape\n"
  " zx A|B [i]  delete shape i / all shapes\n"
  " ze 0|1      zone off (setup) / on\n"
  " za 0|1      block / allow built-in programs\n"
  " zs margin [settleMs [panRate tiltRate]]\n"
  " zg          show zones\n";

static void handleZoneCmd(char *line) {
  char hc = 0, ec = 0; int a, b, c, d;
  char name[4] = {0}; sscanf(line, "%3s", name);
  int nargs = 0; for (char *q = line; *q; q++) if (*q == ' ' && q[1] && q[1] != ' ') nargs++;
  if (sscanf(line, "zm %c %c", &hc, &ec) == 2 && headIndex(hc) >= 0) {
    int h = headIndex(hc); uint16_t s = HEAD_START[h];
    uint16_t pan = (dmx[s] << 8) | dmx[s + 1], tilt = (dmx[s + 2] << 8) | dmx[s + 3];
    if (ec == 'F' || ec == 'f') addPoint(zones[h].far, zones[h].nFar, pan, tilt);
    else if (ec == 'N' || ec == 'n') addPoint(zones[h].near_, zones[h].nNear, pan, tilt);
    else { reply("ERR: zm <A|B> <F|N>\n"); return; }
    saveZones();
    reportZones();
  } else if (sscanf(line, "zp %c", &hc) == 1 && headIndex(hc) >= 0) {
    int h = headIndex(hc), slot = -1;
    for (int i = 0; i < POLY_MAX; i++) if (polys[h].p[i].n < 3) { slot = i; break; }
    if (slot < 0) { reply("ERR: max %d shapes per head\n", POLY_MAX); return; }
    Poly q; char *pp = line + 4;
    while (q.n < POLY_PTS) {
      unsigned pan, tilt; int used = 0;
      if (sscanf(pp, " %u,%u%n", &pan, &tilt, &used) != 2) break;
      q.v[q.n++] = { (uint16_t)min(pan, 65535u), (uint16_t)min(tilt, 65535u) };
      pp += used;
    }
    if (q.n < 3) { reply("ERR: a shape needs at least 3 points\n"); return; }
    polys[h].p[slot] = q;
    saveZones();
    reportZones();
  } else if (sscanf(line, "zx %c %d", &hc, &a) == 2 && headIndex(hc) >= 0) {
    int h = headIndex(hc), k = 0;
    for (int i = 0; i < POLY_MAX; i++) {
      if (polys[h].p[i].n < 3) continue;
      if (k++ == a) { polys[h].p[i] = Poly(); break; }
    }
    saveZones();
    reportZones();
  } else if (sscanf(line, "zx %c", &hc) == 1 && headIndex(hc) >= 0) {
    polys[headIndex(hc)] = HeadPolys();
    saveZones();
    reportZones();
  } else if (sscanf(line, "zc %c", &hc) == 1 && headIndex(hc) >= 0) {
    zones[headIndex(hc)] = Zone();
    saveZones();
    reportZones();
  } else if (sscanf(line, "ze %d", &a) == 1) {
    zcfg.enabled = a ? 1 : 0;
    reply(zcfg.enabled ? "zone ON\n" : "zone OFF (setup mode)\n");
    reportZones();
  } else if (sscanf(line, "za %d", &a) == 1) {
    allowAuto = a != 0;
    reply(allowAuto ? "built-in programs ALLOWED - eye zone cannot protect them\n"
                    : "built-in programs blocked while zone active\n");
    reportZones();
  } else if (strncmp(line, "zs ", 3) == 0 && sscanf(line, "zs %d", &a) == 1) {
    int n = sscanf(line, "zs %d %d %d %d", &a, &b, &c, &d);
    if (n == 3) { reply("zs needs margin, margin settle, or all four values\n"); return; }
    zcfg.margin = constrain(a, 0, 20000);
    if (n >= 2) zcfg.settleMs = constrain(b, 0, 5000);
    if (n == 4) { zcfg.panRate = constrain(c, 500, 65535); zcfg.tiltRate = constrain(d, 500, 65535); }
    saveZones();
    reportZones();
  } else if (strcmp(line, "zg") == 0) {
    reportZones();
  } else {
    // explain what was wrong instead of a bare error
    char key[8]; snprintf(key, sizeof(key), " %s ", name);
    bool known = strstr(" zm zc zp zx ze za zs zg ", key) != nullptr;
    if (!known) reply("unknown zone command '%s'\n", name);
    else if (nargs == 0) reply("'%s' needs arguments\n", name);
    else reply("ERR: couldn't parse '%s'\n", line);
    reply("%s", ZONE_HELP);
  }
}

// ---------------- Commands ----------------
static void handleLine(char *line) {
  int slot, value;
  if (line[0] == 'f' && line[1] == ' ') {          // bulk set, no reply
    char *p = line + 2;
    char *end;
    long start = strtol(p, &end, 10);
    if (end == p || start < 1) return;
    p = end;
    for (long sl = start; sl <= DMX_SLOTS; sl++) {
      long v = strtol(p, &end, 10);
      if (end == p) break;
      dmx[sl] = (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v));
      p = end;
    }
    mode = Mode::MANUAL;
    return;
  }
  if (sscanf(line, "s %d %d", &slot, &value) == 2) {
    if (slot < 1 || slot > DMX_SLOTS || value < 0 || value > 255) {
      reply("ERR: slot 1-512, value 0-255\n");
      return;
    }
    dmx[slot] = (uint8_t)value;
    mode = Mode::MANUAL;
    reply("slot %d = %d\n", slot, value);
  } else if (strcmp(line, "z") == 0) {
    memset(dmx + 1, 0, DMX_SLOTS);
    mode = Mode::MANUAL;
    reply("all slots 0\n");
  } else if (strcmp(line, "d") == 0) {
    for (int head = 0; head < 2; head++) {        // one line per fixture
      char line[80]; int p = 0;
      p += snprintf(line + p, sizeof(line) - p, "%c:", 'A' + head);
      for (int i = 1; i <= 11; i++) p += snprintf(line + p, sizeof(line) - p, " %d", dmx[head * 11 + i]);
      reply("%s\n", line);
    }
  } else if (line[0] == 'z' && line[1] && line[1] != ' ') {
    handleZoneCmd(line);
  } else if (strncmp(line, "reset", 5) == 0) {
    char who[8] = "all"; sscanf(line + 5, " %7s", who);
    bool all = strcmp(who, "all") == 0;
    bool a = all || who[0] == 'A' || who[0] == 'a';
    bool b = all || who[0] == 'B' || who[0] == 'b';
    if (!a && !b) { reply("usage: reset A|B|all\n"); return; }
    uint32_t now = millis(); if (!now) now = 1;
    if (a) resetStart[0] = now;
    if (b) resetStart[1] = now;
    reply("RESET %s: ch11=255 for 6 s, beam dark for 15 s\n", a && b ? "A+B" : a ? "A" : "B");
  } else if (strcmp(line, "help") == 0 || strcmp(line, "?") == 0) {
    reply("cmds:\n s slot val | f slot v1 v2.. | z (all 0) | d (dump)\n"
          " sweep | reset A|B|all | help\n");
    reply("%s", ZONE_HELP);
  } else if (strcmp(line, "sweep") == 0) {
    sceneEndStops();
    mode = Mode::SWEEP;
    modeStart = millis();
    reply("sweep\n");
  } else if (line[0]) {
    reply("unknown '%s' - type help\n", line);
  }
}

static void pollSerial() {
  static char buf[CMD_LEN];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { buf[len] = 0; handleLine(buf); len = 0; }
    else if (len < sizeof(buf) - 1) buf[len++] = c;
  }
}

// ---------------- Arduino ----------------
void setup() {
  Serial.begin(115200);
  pinMode(DMX_EN_PIN, OUTPUT);
  digitalWrite(DMX_EN_PIN, HIGH);        // MAX485 driver enabled (transmit)
  Serial1.begin(DMX_BAUD, SERIAL_8N2, DMX_RX_PIN, DMX_TX_PIN);

  loadZones();
  sceneEndStops();
  modeStart = millis();
  bleBegin();
  Serial.println("DMX TX up: HOLD (end stops, white, 50%), BLE advertising as DMX-Controller");
}

void loop() {
  static uint32_t lastFrame = 0;
  uint32_t now = millis();

  pollSerial();
  Cmd c;
  while (xQueueReceive(cmdQueue, &c, 0) == pdTRUE) handleLine(c.text);   // BLE commands

  if (mode == Mode::HOLD && now - modeStart >= HOLD_MS) {
    mode = Mode::SWEEP;
    modeStart = now;
    Serial.println("SWEEP: pan 0 -> 255 -> 0");
  }
  if (mode == Mode::SWEEP) updateSweep(now);

  if (now - lastFrame >= FRAME_PERIOD_MS) {
    float dt = (now - lastFrame) / 1000.0f;
    if (dt > 0.2f) dt = 0.2f;
    lastFrame = now;
    updateZoneModel(dt);
    sendFrame();
    for (int h = 0; h < 2; h++) {                       // tell the app when blanking changes
      if (blanked[h] != blankedReported[h]) {
        blankedReported[h] = blanked[h];
        reply("ZB %c %d\n", 'A' + h, blanked[h] ? 1 : 0);
      }
    }
  }
}
