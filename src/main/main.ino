#include <TheThingsNetwork.h>
#include "HAN_IoT_Shield.h"
#include "SensingModule.h"

// =============================================================================
// MODE SELECT — comment one out
// =============================================================================
#define DEMO_MODE
//#define PRODUCTION_MODE

#if defined(DEMO_MODE) && defined(PRODUCTION_MODE)
  #error "Pick one: DEMO_MODE or PRODUCTION_MODE"
#endif
#if !defined(DEMO_MODE) && !defined(PRODUCTION_MODE)
  #error "Define either DEMO_MODE or PRODUCTION_MODE"
#endif

// TTN Credentials
const char *appEui = "1234123412341234";
const char *appKey = "CC33FF50DEAF68B3C9B80F465233A745";

// ── Node identity — set per physical device before flashing ──────────────────
// Sent once on startup (port 0) so the dashboard knows which node this is.
// Coordinates in degrees × 10000 (int32), fits 4 bytes, ~10m precision.
#define NODE_ID    0x01          // unique per node: 0x01, 0x02, 0x03 ...
#define NODE_LAT   519849L       // 51.9849 N  — XXM3+XH Arnhem (Gildemeesterplein) (× 10000)
#define NODE_LON    59539L       //  5.9539 E  — XXM3+XH Arnhem (Gildemeesterplein) (× 10000)

#define loraSerial Serial1
#define debugSerial Serial
#define freqPlan TTN_FP_EU868

TheThingsNetwork ttn(loraSerial, debugSerial, freqPlan);

// ── IoT Shield instances ──────────────────────────────────────────────────────
iotShieldPotmeter potmeter1(PIN_POT_RED,   -175, 300);  // simulates temperature
iotShieldPotmeter potmeter2(PIN_POT_WHITE,  -50, 100);  // simulates humidity
iotShieldCO2Sensor co2Sensor;
iotShieldButton redButton(PIN_SWITCH_RED);
iotShieldButton blackButton(PIN_SWITCH_BLACK);
iotShieldLED leftRedLED(PIN_LED_1_RED);
iotShieldLED rightRedLED(PIN_LED_2_RED);
iotShieldLED leftGreenLED(PIN_LED_3_GRN);
iotShieldLED rightGreenLED(PIN_LED_4_GRN);

// ── State machine ─────────────────────────────────────────────────────────────
#define ESCALATE_COUNT   3
#define DEESCALATE_COUNT 5

enum alarmState { safe, watch, warning, alarm };
alarmState currentState = safe;

uint8_t escalateCount   = 0;
uint8_t deescalateCount = 0;
bool    alarmActive     = false;

// Payload version — upper nibble of byte 0.
// Bump if format changes so the backend can reject stale decoders.
#define PAYLOAD_VERSION 0x01

// Trigger reason flags packed into alarm payload
#define TRIGGER_SCORE   0x01   // composite score threshold (3x hysteresis)
#define TRIGGER_RATE    0x02   // dT/dt > 10 C/s fast-rise bypass
#define TRIGGER_MANUAL  0x04   // red button pressed
uint8_t lastTriggerReason = 0;

// ── Sensor state ──────────────────────────────────────────────────────────────
float temperature = 20.0f;  // sensible boot default
float humidity    = 50.0f;  // neutral — hScore = 0, no false score inflation on boot
float co2         = 400.0f; // ambient CO2 baseline

unsigned long lastTempRead     = 0;
unsigned long lastHumidityRead = 0;
unsigned long lastCO2Read      = 0;
bool tempUpdated     = false;
bool humidityUpdated = false;
bool co2Updated      = false;

// ── TX timing ─────────────────────────────────────────────────────────────────
unsigned long lastTx        = 0;
unsigned long lastHeartbeat = 0;

#ifdef DEMO_MODE
  // Demo: short intervals so behaviour is visible during presentation.
  // Heartbeat 30 s and watch 30 s exceed TTN Fair Use Policy (30 s airtime/day).
  // EU868 duty cycle itself is fine — at SF7 (~50 ms/frame) these rates are well
  // within the 1% per-channel limit. Acceptable on a dev device in a lab.
  #define HEARTBEAT_INTERVAL   65000UL      // 30 s
  #define WATCH_TX_INTERVAL    60000UL      // silence cap in watch state
  #define WATCH_MAX_SILENCE    60000UL      // force watch uplink every 30 s
  #define WARNING_TX_INTERVAL  30000UL      // 10 s
  #define ALARM_TX_INTERVAL     5000UL      // 2 s
#else
  // Production: EU868 duty cycle compliant at all states.
  // SF7 airtime ~50 ms/frame; 3-channel hopping (868.1/868.3/868.5 MHz) gives a
  // minimum gap of ~1.65 s before duty cycle kicks in, so 2 s alarm is compliant.
  // TTN Fair Use Policy (30 s airtime/day) is exceeded in alarm — acceptable
  // trade-off: in an active fire, getting data out matters more than TTN policy.
  // Use a private LoRaWAN server (e.g. ChirpStack) to avoid FUP entirely.
  #define HEARTBEAT_INTERVAL   86400000UL   // 24 h — negligible airtime
  #define WATCH_TX_INTERVAL    120000UL     // silence cap: 2 min
  #define WATCH_MAX_SILENCE    120000UL     // force watch uplink every 2 min
  #define WARNING_TX_INTERVAL  10000UL      // 10 s
  #define ALARM_TX_INTERVAL     2000UL      // 2 s — EU868 compliant, TTN FUP violated
#endif

// ── Watch change-detection TX ─────────────────────────────────────────────────
// Only uplink in watch state when score shifts by this much, OR silence cap expires
float         lastSentScore    = -1.0f;     // -1 = no uplink sent yet this session
unsigned long lastWatchTx      = 0;
#define WATCH_SCORE_THRESHOLD  3.0f         // minimum score delta to trigger uplink

// ── Blink (alarm LED) ─────────────────────────────────────────────────────────
unsigned long lastBlink = 0;
#define BLINK_INTERVAL 500UL
bool blinkState = false;

// ── Rate-of-rise buffer (short-term, dT/dt computation) ──────────────────────
#define TEMP_BUFFER_SIZE 5
float         tempBuffer[TEMP_BUFFER_SIZE];
unsigned long timeBuffer[TEMP_BUFFER_SIZE];
uint8_t       tempIndex = 0;

// ── Baseline circular buffer (slow fire / pre-anomaly context) ────────────────
// Stores safe-state samples every 5 min — 12 entries = 1 hour of baseline.
// On safe→watch escalation, delta_score (current minus baseline avg) is sent
// so the backend can distinguish a 40-minute smolder from a 30-second spike.
struct SensorSample {
  float         temperature;   // 4 bytes
  float         co2;           // 4 bytes
  float         score;         // 4 bytes — pre-computed at sample time
  unsigned long timestamp;     // 4 bytes (millis())
};                             // 16 bytes × 12 = 192 bytes total

#define SAMPLE_BUFFER_SIZE  12
#define SAMPLE_INTERVAL     300000UL   // 5 min between baseline samples

SensorSample  sampleBuffer[SAMPLE_BUFFER_SIZE];
uint8_t       sampleHead     = 0;
uint8_t       sampleCount    = 0;
unsigned long lastSampleTime = 0;

void addBaselineSample(float temp, float co2ppm, float sc) {
  sampleBuffer[sampleHead] = { temp, co2ppm, sc, millis() };
  sampleHead  = (sampleHead + 1) % SAMPLE_BUFFER_SIZE;
  if (sampleCount < SAMPLE_BUFFER_SIZE) sampleCount++;
}

// Returns average score over the baseline buffer.
// Falls back to current score if buffer is empty.
float getBaselineScore(float fallback) {
  if (sampleCount == 0) return fallback;
  float sum = 0;
  for (uint8_t i = 0; i < sampleCount; i++) sum += sampleBuffer[i].score;
  return sum / sampleCount;
}

// ── Sensor clamp ranges — must match Node-RED decoder ────────────────────────
// Values outside these ranges are physically implausible or the sensor is broken.
// Clamping here prevents garbage from corrupting the score and payload encoding.
#define TEMP_MIN  -20.0f    // °C
#define TEMP_MAX  120.0f    // °C  (node hardware will fail above ~150°C anyway)
#define CO2_MIN   300.0f    // ppm (below ambient CO2 is impossible indoors)
#define CO2_MAX   5000.0f   // ppm (heavy smoke / combustion upper bound)
#define HUM_MIN   0.0f      // %
#define HUM_MAX   100.0f    // %
#define DTDT_MAX  15.0f     // °C/s (alarm threshold is 10, 15 is hard cap)

// ── Normalisation constants ───────────────────────────────────────────────────
#define TEMPERATURE_SHIFT     30
#define TEMPERATURE_DIV_RANGE 70
#define HUMIDITY_SHIFT        50
#define HUMIDITY_DIV_RANGE    50

// =============================================================================
// HELPERS
// =============================================================================

void encodeInt16(byte* buf, int offset, int16_t val) {
  buf[offset]     = (val >> 8) & 0xFF;
  buf[offset + 1] =  val       & 0xFF;
}

float computeScore(float temp, float hum, float co2ppm) {
  float tScore = constrain((temp - TEMPERATURE_SHIFT) / (float)TEMPERATURE_DIV_RANGE, 0, 1.0f) * 100;
  float hScore = constrain((HUMIDITY_SHIFT - hum)     / (float)HUMIDITY_DIV_RANGE,    0, 1.0f) * 100;
  float cScore = constrain((co2ppm - 400.0f) / 600.0f,                               0, 1.0f) * 100;
  return (tScore * 0.4f) + (hScore * 0.3f) + (cScore * 0.3f);
}

void addTempSample(float temp) {
  tempBuffer[tempIndex] = temp;
  timeBuffer[tempIndex] = millis();
  tempIndex = (tempIndex + 1) % TEMP_BUFFER_SIZE;
}

float computeTempRate() {
  int oldest = (tempIndex + 1)                    % TEMP_BUFFER_SIZE;
  int newest = (tempIndex + TEMP_BUFFER_SIZE - 1) % TEMP_BUFFER_SIZE;
  float dT = tempBuffer[newest] - tempBuffer[oldest];
  float dt = (timeBuffer[newest] - timeBuffer[oldest]) / 1000.0f;
  if (dt <= 0) return 0;
  return constrain(dT / dt, -DTDT_MAX, DTDT_MAX);
}

// =============================================================================
// INTERVAL HELPERS
// =============================================================================

unsigned long getTempInterval(alarmState s) {
  switch (s) {
    case safe:    return 5000UL;
    case watch:
    case warning: return 1000UL;
    case alarm:   return  500UL;
  }
  return 1000UL;
}

unsigned long getCO2Interval(alarmState s) {
  switch (s) {
    case safe:    return 5000UL;
    case watch:
    case warning: return 2000UL;
    case alarm:   return  500UL;
  }
  return 1000UL;
}

unsigned long getHumidityInterval(alarmState s) {
  switch (s) {
    case watch:
    case warning: return 2000UL;
    case alarm:   return  500UL;
    default:      return 0;
  }
}

unsigned long getTxInterval(alarmState s) {
  switch (s) {
    case safe:    return HEARTBEAT_INTERVAL;
    case watch:   return WATCH_TX_INTERVAL;
    case warning: return WARNING_TX_INTERVAL;
    case alarm:   return ALARM_TX_INTERVAL;
  }
  return WARNING_TX_INTERVAL;
}

// =============================================================================
// STATE-DIFFERENTIATED UPLINK FUNCTIONS
// =============================================================================

/*
 * ANNOUNCE — sent once on startup, port 0, unconfirmed. 9 bytes.
 * [0]   node ID (uint8)
 * [1-4] latitude  × 10000 (int32, big-endian)
 * [5-8] longitude × 10000 (int32, big-endian)
 * Node-RED stores this on first sight and labels the device permanently.
 */
void sendAnnounce() {
  byte payload[9];
  payload[0] = NODE_ID;
  int32_t lat = NODE_LAT;
  int32_t lon = NODE_LON;
  payload[1] = (lat >> 24) & 0xFF;
  payload[2] = (lat >> 16) & 0xFF;
  payload[3] = (lat >>  8) & 0xFF;
  payload[4] =  lat        & 0xFF;
  payload[5] = (lon >> 24) & 0xFF;
  payload[6] = (lon >> 16) & 0xFF;
  payload[7] = (lon >>  8) & 0xFF;
  payload[8] =  lon        & 0xFF;
  ttn.sendBytes(payload, sizeof(payload), 0, false);
  debugSerial.println("-- ANNOUNCE node:" + String(NODE_ID) +
                      " lat:" + String(NODE_LAT / 10000.0, 4) +
                      " lon:" + String(NODE_LON / 10000.0, 4));
}

/*
 * SAFE — confirmed uplink, 1 byte (state=0x00).
 * Network ACK serves as watchdog / linkcheck. No sensor data needed.
 * Port 1, confirmed.
 */
void sendHeartbeat() {
  rightGreenLED.setState(LED_ON);
  byte payload[1] = { (PAYLOAD_VERSION << 4) | 0x00 };   // 0x10
  ttn.sendBytes(payload, 1, 1, true);
  rightGreenLED.setState(LED_OFF);
  debugSerial.println("-- HEARTBEAT (linkcheck)");
}

/*
 * WATCH — anomaly detected. 7 bytes. Port 2, unconfirmed.
 * "Something is developing — initial signature + trend vs baseline."
 *
 * [0] ver|state=0x21  [1] score(uint8)
 * [2-3] temp*10(int16)  [4-5] CO2 ppm(uint16)
 * [6] delta_score(int8): current score minus 1-hour baseline average
 *     Positive = rising above normal. Large positive = fast development.
 *     Backend uses this to distinguish slow smolder from sudden spike.
 */
void sendWatchPayload(float temp, float co2ppm, float score) {
  rightGreenLED.setState(LED_ON);
  byte payload[7];
  payload[0] = (PAYLOAD_VERSION << 4) | 0x01;            // 0x11
  payload[1] = (uint8_t)constrain(score, 0, 100);
  encodeInt16(payload, 2, (int16_t)(temp * 10));
  uint16_t co2Enc = (uint16_t)constrain(co2ppm, 0, 65535);
  payload[4] = (co2Enc >> 8) & 0xFF;
  payload[5] =  co2Enc       & 0xFF;
  float    baseline   = getBaselineScore(score);
  int8_t   deltaScore = (int8_t)constrain(score - baseline, -127, 127);
  payload[6] = (byte)deltaScore;
  ttn.sendBytes(payload, sizeof(payload), 2, false);
  rightGreenLED.setState(LED_OFF);
  debugSerial.println("-- TX WATCH | score:" + String(score) +
                      " baseline:" + String(baseline) +
                      " delta:" + String(deltaScore));
}

/*
 * WARNING — threat developing. 10 bytes. Port 3, unconfirmed.
 * "Track this actively — full picture including rate-of-rise."
 *
 * [0] state=0x02  [1] score  [2-3] temp*10  [4] humidity%
 * [5-6] CO2 ppm  [7-8] dTdt*100(int16)  [9] reserved
 */
void sendWarningPayload(float temp, float hum, float co2ppm, float score, float dTdt) {
  rightGreenLED.setState(LED_ON);
  byte payload[10];
  payload[0] = (PAYLOAD_VERSION << 4) | 0x02;            // 0x12  (warning)
  payload[1] = (uint8_t)constrain(score, 0, 100);
  encodeInt16(payload, 2, (int16_t)(temp * 10));
  payload[4] = (uint8_t)constrain(hum, 0, 100);
  uint16_t co2Enc = (uint16_t)constrain(co2ppm, 0, 65535);
  payload[5] = (co2Enc >> 8) & 0xFF;
  payload[6] =  co2Enc       & 0xFF;
  encodeInt16(payload, 7, (int16_t)(dTdt * 100));
  payload[9] = 0x00;
  ttn.sendBytes(payload, sizeof(payload), 3, true);   // confirmed — ACK failure = link degraded
  rightGreenLED.setState(LED_OFF);
  debugSerial.println("-- TX WARNING | score:" + String(score) + " dT/dt:" + String(dTdt));
}

/*
 * ALARM — emergency. 11 bytes. Port 4, unconfirmed, max rate.
 * "Fire confirmed or imminent — blast data out before the node dies."
 * Intentionally exceeds EU868 1% duty cycle in emergency conditions.
 *
 * [0] state=0x03  [1] score  [2-3] temp*10  [4] humidity%
 * [5-6] CO2 ppm  [7-8] dTdt*100
 * [9] trigger(SCORE=0x01|RATE=0x02|MANUAL=0x04)  [10] alarmActive
 */
void sendAlarmPayload(float temp, float hum, float co2ppm, float score, float dTdt, uint8_t reason) {
  rightGreenLED.setState(LED_ON);
  byte payload[11];
  payload[0]  = (PAYLOAD_VERSION << 4) | 0x03;           // 0x13  (alarm)
  payload[1]  = (uint8_t)constrain(score, 0, 100);
  encodeInt16(payload, 2, (int16_t)(temp * 10));
  payload[4]  = (uint8_t)constrain(hum, 0, 100);
  uint16_t co2Enc = (uint16_t)constrain(co2ppm, 0, 65535);
  payload[5]  = (co2Enc >> 8) & 0xFF;
  payload[6]  =  co2Enc       & 0xFF;
  encodeInt16(payload, 7, (int16_t)(dTdt * 100));
  payload[9]  = reason;
  payload[10] = alarmActive ? 0x01 : 0x00;
  ttn.sendBytes(payload, sizeof(payload), 4, false);
  rightGreenLED.setState(LED_OFF);
  debugSerial.println("-- TX ALARM | score:" + String(score) + " reason:0x" + String(reason, HEX));
}

// =============================================================================
// STATE MACHINE
// =============================================================================

// Sends a raw AT command to the RN2483 and drains the response from the buffer.
// MUST be used for any direct serial commands — if the "ok"/"invalid_param"
// response is not consumed, the TTN library reads it as its own response and
// corrupts the next operation.
// Note: ADR is disabled and SF7 (DR5) is locked by the TheThingsNetwork library
// itself during join — no manual mac commands needed.

void updateState(float score, float dTdt) {
  // Fast path: dT/dt bypass — no hysteresis, immediate alarm
  if (dTdt > 10.0f) {
    lastTriggerReason = TRIGGER_RATE;
    currentState = alarm;
    debugSerial.println("-- FAST RISE: " + String(dTdt) + " C/s -> ALARM");
    return;
  }

  alarmState newState;
  if      (score >= 70) newState = alarm;
  else if (score >= 45) newState = warning;
  else if (score >= 25) newState = watch;
  else                  newState = safe;

  if (newState > currentState) {
    escalateCount++;
    deescalateCount = 0;
    if (escalateCount >= ESCALATE_COUNT) {
      currentState = newState;
      escalateCount = 0;
      lastTriggerReason = TRIGGER_SCORE;
      lastSentScore = -1.0f;
      debugSerial.println("-- State UP -> " + String(currentState));
    }
  } else if (newState < currentState) {
    deescalateCount++;
    escalateCount = 0;
    if (deescalateCount >= DEESCALATE_COUNT) {
      if (!alarmActive) {
        currentState = newState;
        deescalateCount = 0;
        debugSerial.println("-- State DOWN -> " + String(currentState));
      }
    }
  } else {
    escalateCount   = 0;
    deescalateCount = 0;
  }
}

void updateLEDs() {
  leftGreenLED.setState(LED_OFF);
  leftRedLED.setState(LED_OFF);
  rightRedLED.setState(LED_OFF);

  switch (currentState) {
    case safe:    break;
    case watch:   leftGreenLED.setState(LED_ON); break;
    case warning: leftGreenLED.setState(LED_ON); rightRedLED.setState(LED_ON); break;
    case alarm:
      if (millis() - lastBlink >= BLINK_INTERVAL) {
        blinkState = !blinkState;
        leftRedLED.setState(blinkState ? LED_ON : LED_OFF);
        lastBlink = millis();
      }
      break;
  }
}

// =============================================================================
// SETUP & LOOP
// =============================================================================

void setup() {
  loraSerial.begin(57600);
  debugSerial.begin(9600);
  while (!debugSerial && millis() < 10000);
#ifdef DEMO_MODE
  debugSerial.println("-- MODE: DEMO (short intervals, duty cycle not enforced)");
#else
  debugSerial.println("-- MODE: PRODUCTION (24h heartbeat, EU868 compliant)");
#endif
  debugSerial.println("-- STATUS");
  ttn.showStatus();
  debugSerial.println("-- JOIN");
  ttn.join(appEui, appKey);
  sendAnnounce();
}

void loop() {
  unsigned long now = millis();

  // ── Button handling ──────────────────────────────────────────────────────
  if (redButton.isPressed()) {
    lastTriggerReason = TRIGGER_MANUAL;
    alarmActive  = true;
    currentState = alarm;
    byte manualPayload[2] = { 0x03, TRIGGER_MANUAL };
    ttn.sendBytes(manualPayload, sizeof(manualPayload), 4, false);
    debugSerial.println("-- MANUAL ALARM");
    lastTx = now;
  }

  if (blackButton.isPressed() && alarmActive) {
    alarmActive  = false;
    currentState = safe;
    leftRedLED.setState(LED_OFF);
#ifdef DEMO_MODE
    debugSerial.println("-- ALARM RESET (demo)");
#else
    debugSerial.println("-- ALARM RESET (node should be inspected and replaced)");
#endif
  }

  // ── Sensor sampling ──────────────────────────────────────────────────────
  if (now - lastTempRead >= getTempInterval(currentState)) {
    temperature  = constrain(potmeter1.getValue(), TEMP_MIN, TEMP_MAX);
    lastTempRead = now;
    tempUpdated  = true;
    addTempSample(temperature);
  }

  if (now - lastCO2Read >= getCO2Interval(currentState)) {
    float dTdt = computeTempRate();
    co2         = constrain(co2Sensor.getCO2ppm(temperature, dTdt), CO2_MIN, CO2_MAX);
    lastCO2Read = now;
    co2Updated  = true;
  }

  if (currentState != safe && now - lastHumidityRead >= getHumidityInterval(currentState)) {
    // Humidity inversely derived from temperature — realistic fire behavior.
    // At 25°C → ~60% RH. At 100°C → ~5% RH.
    humidity = constrain(80.0f - ((temperature - 25.0f) * 0.7f), HUM_MIN, HUM_MAX);
    lastHumidityRead = now;
    humidityUpdated  = true;
  }
    // ── Process new data ─────────────────────────────────────────────────────
  bool shouldProcess = (currentState == safe) ? (tempUpdated || co2Updated) : tempUpdated;
  float score = 0, dTdt = 0;

  if (shouldProcess) {
    score = computeScore(temperature, humidity, co2);
    dTdt  = computeTempRate();
    updateState(score, dTdt);

    debugSerial.print("T:");     debugSerial.print(temperature);
    debugSerial.print(" CO2:");  debugSerial.print(co2);
    debugSerial.print(" H:");    debugSerial.print(humidity);
    debugSerial.print(" Score:"); debugSerial.print(score);
    debugSerial.print(" dT/dt:"); debugSerial.println(dTdt);

    tempUpdated = humidityUpdated = co2Updated = false;
  }

  // ── Baseline sampling (safe state only, every 5 min) ─────────────────────
  // Builds the 1-hour rolling baseline used for delta_score in watch uplinks.
  // Not sampled during elevated states — buffer represents "what normal looks like."
  if (currentState == safe && now - lastSampleTime >= SAMPLE_INTERVAL) {
    float sc = computeScore(temperature, 50.0f, co2);   // neutral humidity baseline
    addBaselineSample(temperature, co2, sc);
    lastSampleTime = now;
    debugSerial.println("-- Baseline sample: T=" + String(temperature) +
                        " CO2=" + String(co2) + " score=" + String(sc));
  }

  // ── Safe: confirmed heartbeat = watchdog linkcheck ───────────────────────
  if (currentState == safe && now - lastHeartbeat >= HEARTBEAT_INTERVAL) {
    sendHeartbeat();
    lastHeartbeat = now;
    lastTx        = now;
  }

  // ── Watch: change-detection TX ───────────────────────────────────────────
  // Uplink only when score shifts by WATCH_SCORE_THRESHOLD, OR silence cap expires.
  // Avoids flooding the channel with identical payloads every 30 s.
  if (currentState == watch) {
    score = computeScore(temperature, humidity, co2);
    bool scoreChanged  = (lastSentScore < 0) ||
                         (fabs(score - lastSentScore) >= WATCH_SCORE_THRESHOLD);
    bool silenceTooLong = (now - lastWatchTx >= WATCH_MAX_SILENCE);

    if (scoreChanged || silenceTooLong) {
      sendWatchPayload(temperature, co2, score);
      lastSentScore = score;
      lastWatchTx   = now;
      lastTx        = now;
    }
  }

  // ── Warning / Alarm: fixed-interval TX ───────────────────────────────────
  if ((currentState == warning || currentState == alarm) &&
       now - lastTx >= getTxInterval(currentState)) {
    score = computeScore(temperature, humidity, co2);
    dTdt  = computeTempRate();
    if (currentState == warning)
      sendWarningPayload(temperature, humidity, co2, score, dTdt);
    else
      sendAlarmPayload(temperature, humidity, co2, score, dTdt, lastTriggerReason);
    lastTx = now;
  }

  // ── LEDs ─────────────────────────────────────────────────────────────────
  updateLEDs();
}
