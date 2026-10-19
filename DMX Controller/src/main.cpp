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
 */
#include <Arduino.h>
#include <stdarg.h>
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

// ---------------- Frame transmit ----------------
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
  Serial1.write(dmx, sizeof(dmx));       // start code + 512 slots
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
static constexpr size_t CMD_LEN = 128;
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
  char buf[160];
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
  } else if (strcmp(line, "sweep") == 0) {
    sceneEndStops();
    mode = Mode::SWEEP;
    modeStart = millis();
    reply("sweep\n");
  } else if (line[0]) {
    reply("cmds: s <slot> <val> | z | d | sweep\n");
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
    lastFrame = now;
    sendFrame();
  }
}
