/*
  Quag Cortex v2 (now called ENVOY) — ESP32 BLE MIDI Controller
  ==========================================
  Master switch + 4 encoders (E1..E4), RGB mode LED, one status LED.
  Sends BLE MIDI. Pair from Windows and receive with MidiBerry.

  Built from the v2 "Recode map". Everything below that differs from the v1
  firmware is marked "v2:" so you can see what moved.

  ---- WHAT CHANGED FROM v1 ------------------------------------------------
  v2: Master PRESS now cycles the mode AND immediately fires that mode's CC.
      In v1 you had to hold the master 500ms to send it. Press 1 lands on
      AMP1 and fires CC101, press 2 -> AMP2/CC102, ... press 8 wraps back to
      ADMIN and fires CC100. Master hold no longer does anything.
  v2: ADMIN E1 rotation is suppressed - the noise gate is toggle-only, so
      turning E1 in ADMIN sends nothing (v1 sent CC10).
  v2: Each CC remembers its own position. In v1 all four encoders shared one
      value each across every mode, so AMP1 Gain and AMP2 Gain fought over the
      same number. Now ccValue[] is indexed by CC, and every named parameter
      on the map keeps its own spot. Set PER_CC_VALUES to 0 for the old feel.
  v2: Encoder rotation uses a real quadrature state table instead of a bare
      CLK-edge compare, so bounce no longer produces phantom steps. Emission
      rate is unchanged (2 CC steps per detent) - see STEPS_PER_DETENT.
  v2: Encoder-switch debounce is a proper 30ms filter with a separate 500ms
      hold. v1's 250ms lockout meant an enc4 tap shorter than 250ms was
      swallowed and only 250-500ms registered as a short press.
  v2: No delay() anywhere in the run path. The BLE connect/disconnect
      callbacks now only set a flag, and RGB flashes time out in the loop.

  ---- TWO THINGS TO CHECK BEFORE YOU FLASH --------------------------------
  1. PIN MAP. These are still the v1 pin numbers. The recode map defines
     behaviour, not wiring, so verify each one against the v2 board.
  2. E3's SWITCH. The v2 panel drawing labels Mini 3 "no switch", but the
     recode map gives PREPEDALS an "E3 SW -> Delay 1 ON/OFF CC#93" action.
     Both can't be true. The code implements the map, so if E3 really has no
     button that one toggle is simply unreachable and nothing else breaks -
     but the Delay 1 on/off will need to live somewhere else.
*/

#include <Arduino.h>
#include <BLEMIDI_Transport.h>
#include <hardware/BLEMIDI_ESP32.h>
#include <MIDI.h>

// ─── BLE MIDI SETUP ──────────────────────────────────────────────────────────
BLEMIDI_CREATE_INSTANCE("Ryan's MIDI Controller", MIDI)

#define MIDI_CHANNEL 1

bool bleConnected = false;

// Touched from the BLE task, read from loop(). Keep the callbacks trivial.
volatile bool bleEvtConnect    = false;
volatile bool bleEvtDisconnect = false;

// ─── PIN DEFINITIONS ─────────────────────────────────────────────────────────
// v1 numbers, carried over. Confirm against the v2 board before flashing.
#define ENC1_CLK 23
#define ENC1_DT  22
#define ENC1_SW  13

#define ENC2_CLK 19
#define ENC2_DT  18
#define ENC2_SW  12   // MTDI strapping pin: fine with INPUT_PULLUP (the pull-up
                      // only comes on after boot), but never fit an EXTERNAL
                      // pull-up here or the ESP32 picks 1.8V flash and won't start.

#define ENC3_CLK 14
#define ENC3_DT  32
#define ENC3_SW  2    // strapping pin, see the pinMode dance in setup()

#define ENC4_CLK 5
#define ENC4_DT  4
#define ENC4_SW  15

#define MASTER_SW 33
#define LED_PIN   21
#define RGB_R     25
#define RGB_G     26
#define RGB_B     27

// Set to 1 if the RGB LED's common leg goes to 3V3 instead of GND.
#define RGB_COMMON_ANODE 0

// ─── TIMING ──────────────────────────────────────────────────────────────────
#define DEBOUNCE_MS     30    // v2: was a 250ms lockout, which ate short taps
#define HOLD_TIME       500
#define BLINK_INTERVAL  400
#define RGB_FLASH_MS    300

// Quadrature steps to count before emitting one CC step. A 24-detent encoder
// gives 4 transitions per detent, so 2 = two CC steps per detent, which is what
// v1's edge-compare produced. Use 4 for exactly one CC step per detent.
#define STEPS_PER_DETENT 2

// v2: 1 = every CC remembers its own position, 0 = v1 behaviour (one shared
// value per encoder, carried across modes).
#define PER_CC_VALUES 1

// ─── MASTER MODES ────────────────────────────────────────────────────────────
#define NUM_MASTER_MODES 8

enum {
  MODE_ADMIN = 0,
  MODE_AMP1,
  MODE_AMP2,
  MODE_AMP3,
  MODE_AMP4,
  MODE_PREPEDALS,
  MODE_POSTPEDALS,
  MODE_EQ
};

const uint8_t modeColors[NUM_MASTER_MODES][3] = {
  { 255,   0,   0 },   // ADMIN      red
  { 255,  80,   0 },   // AMP1       orange
  { 255, 200,   0 },   // AMP2       yellow
  {   0, 255,   0 },   // AMP3       green
  {   0, 220, 255 },   // AMP4       light blue
  {   0,   0, 255 },   // PREPEDALS  dark blue
  { 180,   0, 255 },   // POSTPEDALS purple
  { 255,  20, 147 },   // EQ         pink
};

// Fired on the master press that lands on each mode: CC 100 + mode.
#define CC_MASTER_MODE_BASE 100

const uint8_t modeValues[NUM_MASTER_MODES] = {
  127,  // ADMIN
  1,    // AMP1
  35,   // AMP2
  65,   // AMP3
  120,  // AMP4
  127,  // PREPEDALS
  127,  // POSTPEDALS
  127,  // EQ
};

// ─── CC TABLES ───────────────────────────────────────────────────────────────
#define CC_NONE 0xFF   // v2: marks a rotation that sends nothing

// Rotation CC per mode, per encoder.
const uint8_t ccNumbers[NUM_MASTER_MODES][4] = {
  { CC_NONE, 11, 12, 13 },   // ADMIN      -, Input, Transpose, Output
  {      20, 21, 22, 23 },   // AMP1       Gain, Bass, Mid, Treble
  {      30, 31, 32, 33 },   // AMP2       Gain*, Bass, Mid, Treble
  {      40, 41, 42, 43 },   // AMP3       Gain*, Bass, Mid, Treble
  {      50, 51, 52, 53 },   // AMP4       Gain, Bass, Mid, Treble
  {      60, 61, 62, 63 },   // PREPEDALS  Comp, OD1, Delay1, OD2 levels
  {      70, 71, 72, 73 },   // POSTPEDALS Delay2 Mix, Rev Mix, Rev Op1, Rev Op2
  {      80, 81, 82, 83 },   // EQ         75Hz*, 350Hz, 1.5kHz, 4.5kHz
};                           // * = E1 is overridden by the mini-mode below

// E1 rotation CC for mini-modes 1/2/3. Only AMP2, AMP3 and EQ have them.
const uint8_t miniModeCC[NUM_MASTER_MODES][3] = {
  { CC_NONE, CC_NONE, CC_NONE },
  { CC_NONE, CC_NONE, CC_NONE },
  {      30,      34,      35 },   // AMP2  Gain / Presence / Depth
  {      40,      44,      45 },   // AMP3  Gain / Resonance / Presence
  { CC_NONE, CC_NONE, CC_NONE },
  { CC_NONE, CC_NONE, CC_NONE },
  { CC_NONE, CC_NONE, CC_NONE },
  {      80,      84,      85 },   // EQ    75Hz / HPF / LPF
};

// ─── EFFECT TOGGLES ──────────────────────────────────────────────────────────
enum {
  TG_NOISEGATE = 0,
  TG_COMPRESSOR,
  TG_OD1,
  TG_DELAY1,
  TG_OD2,
  TG_DELAY2,
  TG_REVERB,
  NUM_TOGGLES
};

const uint8_t toggleCC[NUM_TOGGLES] = {
  90,  // noise gate
  91,  // compressor
  92,  // OD1
  93,  // delay 1
  94,  // OD2
  95,  // delay 2
  96,  // reverb
};

bool toggleState[NUM_TOGGLES] = { false };

// ─── ENCODER SWITCH ACTIONS ──────────────────────────────────────────────────
enum SwAction : uint8_t {
  SW_NONE = 0,
  SW_TOGGLE,     // flips an effect and sends its CC as 127 / 0
  SW_MINIMODE    // E1 only: cycles mini-mode 1 -> 2 -> 3 -> 1
};

struct SwitchSpec {
  uint8_t action;
  uint8_t toggleIdx;   // only meaningful for SW_TOGGLE
};

const SwitchSpec switchMap[NUM_MASTER_MODES][4] = {
  // E1                          E2                        E3                          E4
  { { SW_TOGGLE, TG_NOISEGATE }, { SW_NONE, 0 },           { SW_NONE, 0 },             { SW_NONE, 0 }           }, // ADMIN
  { { SW_NONE,   0 },            { SW_NONE, 0 },           { SW_NONE, 0 },             { SW_NONE, 0 }           }, // AMP1
  { { SW_MINIMODE, 0 },          { SW_NONE, 0 },           { SW_NONE, 0 },             { SW_NONE, 0 }           }, // AMP2
  { { SW_MINIMODE, 0 },          { SW_NONE, 0 },           { SW_NONE, 0 },             { SW_NONE, 0 }           }, // AMP3
  { { SW_NONE,   0 },            { SW_NONE, 0 },           { SW_NONE, 0 },             { SW_NONE, 0 }           }, // AMP4
  { { SW_TOGGLE, TG_COMPRESSOR },{ SW_TOGGLE, TG_OD1 },    { SW_TOGGLE, TG_DELAY1 },   { SW_TOGGLE, TG_OD2 }    }, // PREPEDALS
  { { SW_TOGGLE, TG_DELAY2 },    { SW_TOGGLE, TG_REVERB }, { SW_NONE, 0 },             { SW_NONE, 0 }           }, // POSTPEDALS
  { { SW_MINIMODE, 0 },          { SW_NONE, 0 },           { SW_NONE, 0 },             { SW_NONE, 0 }           }, // EQ
};

// Which toggle the status LED mirrors in each mode, or -1 for none. Modes with
// mini-modes drive the LED from the mini-mode instead and are always -1 here.
const int8_t ledMirrorToggle[NUM_MASTER_MODES] = {
  TG_NOISEGATE,   // ADMIN
  -1,             // AMP1
  -1,             // AMP2  (mini-mode)
  -1,             // AMP3  (mini-mode)
  -1,             // AMP4
  TG_COMPRESSOR,  // PREPEDALS
  TG_DELAY2,      // POSTPEDALS
  -1,             // EQ    (mini-mode)
};

// ─── PC MODE ─────────────────────────────────────────────────────────────────
// Carried over from v1 - the recode map doesn't mention it, so it is left
// exactly as it behaved. Hold E4 for 500ms to enter, hold again to leave.
// Master press flips sub-mode A (white, PC 1-4) / B (pink, PC 5-8).
// Encoder switches send their PC number, all rotations send CC 13.
#define CC_VOLUME 13

bool    pcModeActive = false;
uint8_t pcSubMode    = 0;

// ─── STATE ───────────────────────────────────────────────────────────────────
uint8_t masterMode   = MODE_ADMIN;
uint8_t enc1MiniMode = 0;          // 0/1/2 == mini-mode 1/2/3 on the map

#if PER_CC_VALUES
uint8_t ccValue[128];              // one remembered position per CC number
#endif

uint32_t lastBlinkTime = 0;
bool     blinkLEDState = false;

uint32_t rgbFlashUntil = 0;        // 0 = no flash pending

// ─── DEBOUNCED SWITCH ────────────────────────────────────────────────────────
struct Switch {
  uint8_t  pin;
  bool     raw;
  bool     stable;
  uint32_t lastEdge;
  uint32_t pressStart;
  bool     holdFired;
};

Switch masterSw;
Switch encSw[4];

// Feeds one sample in. Returns true on the frame where the debounced level
// changes; read sw.stable for the new level (LOW == pressed).
bool switchUpdate(Switch &sw, uint32_t now) {
  bool r = digitalRead(sw.pin);
  if (r != sw.raw) {
    sw.raw = r;
    sw.lastEdge = now;
  }
  if (r != sw.stable && (now - sw.lastEdge) >= DEBOUNCE_MS) {
    sw.stable = r;
    return true;
  }
  return false;
}

void switchInit(Switch &sw, uint8_t pin) {
  sw.pin = pin;
  pinMode(pin, INPUT_PULLUP);
  sw.raw = sw.stable = digitalRead(pin);
  sw.lastEdge = 0;
  sw.pressStart = 0;
  sw.holdFired = false;
}

// ─── ENCODER ─────────────────────────────────────────────────────────────────
struct Encoder {
  uint8_t clkPin, dtPin;
  uint8_t prevAB;
  int8_t  subSteps;
  int     value;      // only used when PER_CC_VALUES == 0
};

Encoder encoders[4] = {
  { ENC1_CLK, ENC1_DT, 0, 0, 64 },
  { ENC2_CLK, ENC2_DT, 0, 0, 64 },
  { ENC3_CLK, ENC3_DT, 0, 0, 64 },
  { ENC4_CLK, ENC4_DT, 0, 0, 64 },
};

// Standard quadrature table, indexed by (prevAB << 2) | newAB.
// 0 = no movement or an illegal jump (contact bounce), +1/-1 = one step.
const int8_t QUAD_TABLE[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

// ─── HELPERS ─────────────────────────────────────────────────────────────────
bool hasMiniModes() {
  return miniModeCC[masterMode][0] != CC_NONE;
}

void setRGB(uint8_t r, uint8_t g, uint8_t b) {
#if RGB_COMMON_ANODE
  analogWrite(RGB_R, 255 - r);
  analogWrite(RGB_G, 255 - g);
  analogWrite(RGB_B, 255 - b);
#else
  analogWrite(RGB_R, r);
  analogWrite(RGB_G, g);
  analogWrite(RGB_B, b);
#endif
}

// The colour the panel should be showing right now, flashes aside.
void restoreRGB() {
  if (pcModeActive) {
    if (pcSubMode == 0) setRGB(255, 255, 255);   // white = PC 1-4
    else                setRGB(255,  20, 147);   // pink  = PC 5-8
  } else {
    setRGB(modeColors[masterMode][0],
           modeColors[masterMode][1],
           modeColors[masterMode][2]);
  }
}

// v2: replaces the delay()-based flashes. Colour reverts from the loop.
void flashRGB(uint8_t r, uint8_t g, uint8_t b, uint16_t ms) {
  setRGB(r, g, b);
  rgbFlashUntil = millis() + ms;
}

void updateLED() {
  if (pcModeActive) {
    digitalWrite(LED_PIN, LOW);
    return;
  }
  if (hasMiniModes()) {
    // mini-mode 1 = on, 2 = off, 3 = blink (handled in the loop)
    if (enc1MiniMode == 0)      digitalWrite(LED_PIN, HIGH);
    else if (enc1MiniMode == 1) digitalWrite(LED_PIN, LOW);
    else {
      lastBlinkTime = millis();
      blinkLEDState = true;
      digitalWrite(LED_PIN, HIGH);
    }
    return;
  }
  int8_t t = ledMirrorToggle[masterMode];
  digitalWrite(LED_PIN, (t >= 0 && toggleState[t]) ? HIGH : LOW);
}

// Rotation CC for one encoder in the current mode, or CC_NONE.
uint8_t getCCNumber(uint8_t encIdx) {
  if (encIdx == 0 && hasMiniModes())
    return miniModeCC[masterMode][enc1MiniMode];
  return ccNumbers[masterMode][encIdx];
}

// ─── MIDI OUT ────────────────────────────────────────────────────────────────
void sendCC(uint8_t cc, uint8_t value) {
  if (cc == CC_NONE) return;          // v2: suppressed rotations stop here
  if (!bleConnected) return;
  MIDI.sendControlChange(cc, value, MIDI_CHANNEL);
}

void sendPC(uint8_t program) {
  if (!bleConnected) return;
  MIDI.sendProgramChange(program, MIDI_CHANNEL);
}

void fireToggle(uint8_t idx) {
  toggleState[idx] = !toggleState[idx];
  sendCC(toggleCC[idx], toggleState[idx] ? 127 : 0);
  updateLED();
}

// ─── ACTIONS ─────────────────────────────────────────────────────────────────
void masterPress() {
  if (pcModeActive) {
    pcSubMode = (pcSubMode + 1) % 2;
    restoreRGB();
    return;
  }
  // v2: cycle then immediately fire the CC for the mode we landed on.
  masterMode = (masterMode + 1) % NUM_MASTER_MODES;
  enc1MiniMode = 0;
  restoreRGB();
  updateLED();
  sendCC(CC_MASTER_MODE_BASE + masterMode, modeValues[masterMode]);
}

void encoderShortPress(uint8_t i) {
  if (pcModeActive) {
    sendPC((pcSubMode == 0) ? (i + 1) : (i + 5));
    return;
  }
  const SwitchSpec &spec = switchMap[masterMode][i];
  switch (spec.action) {
    case SW_TOGGLE:
      fireToggle(spec.toggleIdx);
      break;
    case SW_MINIMODE:
      enc1MiniMode = (enc1MiniMode + 1) % 3;
      updateLED();
      break;
    default:
      break;
  }
}

void encoder4Hold() {
  pcModeActive = !pcModeActive;
  if (pcModeActive) {
    pcSubMode = 0;
    restoreRGB();                       // white, sub-mode A
  } else {
    flashRGB(100, 100, 100, 200);       // grey blip, then back to the mode colour
  }
  updateLED();
}

void encoderRotate(uint8_t i, int8_t dir) {
  uint8_t cc = pcModeActive ? CC_VOLUME : getCCNumber(i);
  if (cc == CC_NONE) return;

#if PER_CC_VALUES
  int v = constrain((int)ccValue[cc] + dir, 0, 127);
  if (v == ccValue[cc]) return;         // already pinned at an end
  ccValue[cc] = (uint8_t)v;
  sendCC(cc, (uint8_t)v);
#else
  int v = constrain(encoders[i].value + dir, 0, 127);
  if (v == encoders[i].value) return;
  encoders[i].value = v;
  sendCC(cc, (uint8_t)v);
#endif
}

// ─── SETUP ───────────────────────────────────────────────────────────────────
void setup() {
  // GPIO2 is a strapping pin. Park it low before switching to a pulled-up
  // input so a held-down encoder button can't confuse the boot ROM.
  pinMode(ENC3_SW, OUTPUT);
  digitalWrite(ENC3_SW, LOW);

  Serial.begin(115200);

#if PER_CC_VALUES
  for (int i = 0; i < 128; i++) ccValue[i] = 64;
#endif

  BLEMIDI.setHandleConnected([]() {
    bleConnected  = true;
    bleEvtConnect = true;               // v2: no delay() inside the callback
  });

  BLEMIDI.setHandleDisconnected([]() {
    bleConnected     = false;
    bleEvtDisconnect = true;
  });

  MIDI.begin();

  for (int i = 0; i < 4; i++) {
    pinMode(encoders[i].clkPin, INPUT_PULLUP);
    pinMode(encoders[i].dtPin,  INPUT_PULLUP);
    encoders[i].prevAB = (digitalRead(encoders[i].clkPin) << 1) |
                          digitalRead(encoders[i].dtPin);
    encoders[i].subSteps = 0;
  }

  switchInit(encSw[0], ENC1_SW);
  switchInit(encSw[1], ENC2_SW);
  switchInit(encSw[2], ENC3_SW);
  switchInit(encSw[3], ENC4_SW);
  switchInit(masterSw, MASTER_SW);

  pinMode(LED_PIN, OUTPUT);
  pinMode(RGB_R, OUTPUT);
  pinMode(RGB_G, OUTPUT);
  pinMode(RGB_B, OUTPUT);

  restoreRGB();
  updateLED();
}

// ─── LOOP ────────────────────────────────────────────────────────────────────
void loop() {
  MIDI.read();                     // required, keeps the BLE stack serviced
  uint32_t now = millis();

  // ── BLE connect / disconnect blips ────────────────────────────────────────
  if (bleEvtConnect) {
    bleEvtConnect = false;
    flashRGB(0, 255, 0, RGB_FLASH_MS);
  }
  if (bleEvtDisconnect) {
    bleEvtDisconnect = false;
    flashRGB(255, 0, 0, RGB_FLASH_MS);
  }
  if (rgbFlashUntil && (int32_t)(now - rgbFlashUntil) >= 0) {
    rgbFlashUntil = 0;
    restoreRGB();
  }

  // ── MASTER SWITCH ─────────────────────────────────────────────────────────
  if (switchUpdate(masterSw, now) && masterSw.stable == LOW) {
    masterPress();                 // v2: fires on press, no hold needed
  }

  // ── ENCODER SWITCHES ──────────────────────────────────────────────────────
  for (int i = 0; i < 4; i++) {
    Switch &sw = encSw[i];
    if (switchUpdate(sw, now)) {
      if (sw.stable == LOW) {
        sw.pressStart = now;
        sw.holdFired  = false;
      } else if (!sw.holdFired) {
        encoderShortPress(i);      // released before the hold threshold
      }
    }
    // E4 is the only switch with a hold action (enter / leave PC mode).
    if (i == 3 && sw.stable == LOW && !sw.holdFired &&
        (now - sw.pressStart) >= HOLD_TIME) {
      sw.holdFired = true;
      encoder4Hold();
    }
  }

  // ── ENCODER ROTATION ──────────────────────────────────────────────────────
  for (int i = 0; i < 4; i++) {
    Encoder &enc = encoders[i];
    uint8_t ab = (digitalRead(enc.clkPin) << 1) | digitalRead(enc.dtPin);
    if (ab == enc.prevAB) continue;

    int8_t step = QUAD_TABLE[(enc.prevAB << 2) | ab];
    enc.prevAB = ab;
    if (step == 0) continue;       // both lines moved at once: bounce, ignore

    enc.subSteps += step;
    if (enc.subSteps >= STEPS_PER_DETENT) {
      enc.subSteps -= STEPS_PER_DETENT;
      encoderRotate(i, +1);
    } else if (enc.subSteps <= -STEPS_PER_DETENT) {
      enc.subSteps += STEPS_PER_DETENT;
      encoderRotate(i, -1);
    }
  }

  // ── STATUS LED BLINK (mini-mode 3) ────────────────────────────────────────
  if (!pcModeActive && hasMiniModes() && enc1MiniMode == 2) {
    if (now - lastBlinkTime >= BLINK_INTERVAL) {
      lastBlinkTime = now;
      blinkLEDState = !blinkLEDState;
      digitalWrite(LED_PIN, blinkLEDState ? HIGH : LOW);
    }
  }
}
