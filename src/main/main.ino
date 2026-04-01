#include <TheThingsNetwork.h>
#include <CayenneLPP.h>
#include "HAN_IoT_Shield.h"
#include "SensingModule.h"

// TTN Credentials
const char *appEui = "1234123412341234";
const char *appKey = "CC33FF50DEAF68B3C9B80F465233A745";

#define loraSerial Serial1
#define debugSerial Serial
#define freqPlan TTN_FP_EU868

TheThingsNetwork ttn(loraSerial, debugSerial, freqPlan);
CayenneLPP lpp(51);

// IoT Shield
iotShieldPotmeter potmeter1(PIN_POT_RED, -175, 300);
iotShieldPotmeter potmeter2(PIN_POT_WHITE, -50, 100);
iotShieldCO2Sensor co2Sensor;
iotShieldButton redButton(PIN_SWITCH_RED);
iotShieldButton blackButton(PIN_SWITCH_BLACK);
iotShieldLED leftRedLED(PIN_LED_1_RED);
iotShieldLED rightRedLED(PIN_LED_2_RED);
iotShieldLED leftGreenLED(PIN_LED_3_GRN);
iotShieldLED rightGreenLED(PIN_LED_4_GRN);

// Global variables
#define ESCALATE_COUNT 3 // consecutive high readings to escalate state
#define DEESCALATE_COUNT 5 // consecutive low readings to deescalate state

#define TEMPERATURE_SHIFT 30
#define TEMPERATURE_DIV_RANGE 70
#define HUMIDITY_SHIFT 50
#define HUMIDITY_DIV_RANGE 50
unsigned long lastTx = 0;

enum alarmState { safe, watch, warning, alarm};
alarmState currentState = safe;

uint8_t escalateCount = 0;
uint8_t deescalateCount = 0;

unsigned long lastBlink = 0;
#define BLINK_INTERVAL 500UL

unsigned long lastTick = 0;
#define TICK_INTERVAL 100UL

// Sensor timing
unsigned long lastTempRead = 0;
unsigned long lastHumidityRead = 0;
unsigned long lastCO2Read = 0;

float temperature = 0;
float humidity = 0;
float co2 = 0;

// Flags 
bool tempUpdated = false;
bool humidityUpdated = false;
bool co2Updated = false;

bool alarmActive = false;
bool blinkState = false;

// Heartbeat
unsigned long lastHeartbeat = 0;
#define HEARTBEAT_INTERVAL 30000UL // 30 seconds for demo purposes, adjust as needed

// Temp buffer (for dT/dt)
#define TEMP_BUFFER_SIZE 5
float tempBuffer[TEMP_BUFFER_SIZE];
unsigned long timeBuffer[TEMP_BUFFER_SIZE];
uint8_t tempIndex = 0;



float computeScore(float temp, float humidity, float co2) {
  // shifts the temperature, anything below will be counted as 'normal'. (anything below 30 degrees)
  // divide by the range from the shift to the highest (100 - 30 = 70)
  float tScore = constrain((temp - TEMPERATURE_SHIFT) / TEMPERATURE_DIV_RANGE, 0, 1.0) * 100; 

  // lower humidity percentage indicates a fire, so it is inversed from temperature
  // anyhting below 50 concerning
  float hScore = constrain((HUMIDITY_SHIFT - humidity) / HUMIDITY_DIV_RANGE, 0, 1.0) * 100;

  float cScore = constrain((co2 - 400.0) / 600.0, 0, 1.0) * 100;

  return (tScore * 0.4) + (hScore * 0.3) + (cScore * 0.3);
}

unsigned long getTempInterval(alarmState state) {
  switch(state) {
    case safe: return 5000UL;
    case watch: 
    case warning: return 1000UL;
    case alarm: return 500UL;
  }
  return 1000UL;
}

unsigned long getCO2Interval(alarmState state) {
  switch(state) {
    case safe: return 5000UL;
    case watch: 
    case warning: return 2000UL;
    case alarm: return 500UL;
  }
  return 1000UL;
}

unsigned long getHumidityInterval(alarmState state) {
  switch(state) {
    case watch: 
    case warning: return 2000UL;
    case alarm: return 500UL;
    default: return 0;
  }
}
unsigned long getIntervalTx(alarmState state) {
  switch(state) {
    case safe:     return 30000UL;
    case watch:
    case warning:  return 5000UL;
    case alarm:    return 2000UL;
  }
  return 5000UL;
}

void addTempSample(float temp) {
  tempBuffer[tempIndex] = temp;
  timeBuffer[tempIndex] = millis();
  tempIndex = (tempIndex + 1) % TEMP_BUFFER_SIZE;
}

float computeTempRate() {
  int oldest = (tempIndex + 1) % TEMP_BUFFER_SIZE;
  int newest = (tempIndex + TEMP_BUFFER_SIZE - 1) % TEMP_BUFFER_SIZE;

  float dT = tempBuffer[newest] - tempBuffer[oldest];
  float dt = (timeBuffer[newest] - timeBuffer[oldest]) / 1000.0; //seconds

  if (dt <= 0){
    return 0; // avoid division by zero
  }
  return dT / dt; // degC/s
}


void updateState(float score, float dTdt) {
  alarmState newState;

  if (dTdt > 10.0) {
    currentState = alarm;
    debugSerial.println(" -- FAST TEMP RISE DETECTED: " + String(dTdt) + " degC/s -> ALARM");
    return; 
  }

  if (score >= 70) {
    newState = alarm;
  } else if (score >= 45) {
    newState = warning;
  } else if (score >= 25) {
    newState = watch;
  } else {
    newState = safe;
  }

  if (newState > currentState) {
    escalateCount++;
    deescalateCount = 0;
    if (escalateCount >= ESCALATE_COUNT) {
      currentState = newState;
      escalateCount = 0;

      debugSerial.println("--State UP: " + String(currentState));

      // Immediate uplink on Stage 1 -> 2 
      if (currentState == watch){
        sendPayload(temperature, humidity, score, co2);
      }
    }

  } else if (newState < currentState) {
      deescalateCount++;
      escalateCount = 0;
      if (deescalateCount >= DEESCALATE_COUNT) {
        if (!alarmActive) { // only clear if it wasn't a manual alarm press
          currentState = newState;
          deescalateCount = 0;
          debugSerial.println("--State DOWN: " + String(currentState));
        }
      }
    } else {
      escalateCount = 0;
      deescalateCount = 0;
    }
}

void updateLEDs() {
  leftGreenLED.setState(LED_OFF);
  leftRedLED.setState(LED_OFF);
  rightRedLED.setState(LED_OFF);

  switch (currentState) {
    case safe:
      break;
    case watch:
      leftGreenLED.setState(LED_ON);
      break;
    case warning:
      leftGreenLED.setState(LED_ON);
      rightRedLED.setState(LED_ON);
      break;
    case alarm:
      if (millis() - lastBlink >= BLINK_INTERVAL) {
        blinkState = !blinkState;
        leftRedLED.setState(blinkState ? LED_ON : LED_OFF);
        lastBlink = millis();
      }
      
      break;
  }
}

void sendPayload(float temperature, float humidity, float score, float co2) {
  rightGreenLED.setState(LED_ON);

  lpp.reset();
  lpp.addTemperature(1, temperature);
  lpp.addRelativeHumidity(2, humidity);
  lpp.addAnalogInput(3, score);
  lpp.addPresence(4, alarmActive ? 1 : 0);
  lpp.addDigitalInput(5, (uint8_t)currentState);
  lpp.addAnalogInput(6, co2);

  ttn.sendBytes(lpp.getBuffer(), lpp.getSize());

  rightGreenLED.setState(LED_OFF);
  debugSerial.println("-- payload sent | State: " + String(currentState) + " | Score: " + String(score));
}

void setup()
{
  loraSerial.begin(57600);
  debugSerial.begin(9600);
  while (!debugSerial && millis() < 10000)
    ;
  debugSerial.println("-- STATUS");
  ttn.showStatus();
  debugSerial.println("-- JOIN");
  ttn.join(appEui, appKey);
}

void loop()
{
  unsigned long now = millis();

  // BUTTON HANDLING 

  if (redButton.isPressed()) {
    alarmActive = true;
    currentState = alarm;

    byte alarmPayload[2];
    alarmPayload[0] = 0xFF;
    alarmPayload[1] = 0x01;

    ttn.sendBytes(alarmPayload, sizeof(alarmPayload));
    debugSerial.println("-- MANUAL ALARM SENT");

    lastTx = now;
  }

  if (blackButton.isPressed()) {
    if (alarmActive) {
      alarmActive  = false;
      currentState = safe;
      leftRedLED.setState(LED_OFF);
      debugSerial.println("-- ALARM RESET");
    } else {
      ttn.sendBytes(NULL, 0);
      debugSerial.println("-- POLL SENT");
    }
  }

  // SENSOR SAMPLING

  if (now - lastTempRead >= getTempInterval(currentState)) {
    temperature = potmeter1.getValue();
    lastTempRead = now;
    tempUpdated = true;

    addTempSample(temperature);
  }

  if (now - lastCO2Read >= getCO2Interval(currentState)) {
    co2 = co2Sensor.getCO2ppm();
    lastCO2Read = now;
    co2Updated = true;
  }

  if (currentState != safe &&
      now - lastHumidityRead >= getHumidityInterval(currentState)) {
    humidity = potmeter2.getValue();
    lastHumidityRead = now;
    humidityUpdated = true;
  }

  // PROCESS ONLY NEW DATA 

  bool shouldProcess = false;

  switch (currentState) {
    case safe:
      shouldProcess = tempUpdated || co2Updated;
      break;

    case watch:
    case warning:
    case alarm:
      shouldProcess = tempUpdated; // fast reaction
      break;
  }

  float score = 0;
  float dTdt = 0;

  if (shouldProcess) {

    score = computeScore(temperature, humidity, co2);
    dTdt = computeTempRate();

    updateState(score, dTdt);

    // DEBUG 
    debugSerial.print("T: "); debugSerial.print(temperature);
    debugSerial.print(" | CO2: "); debugSerial.print(co2);
    debugSerial.print(" | H: "); debugSerial.print(humidity);
    debugSerial.print(" | Score: "); debugSerial.print(score);
    debugSerial.print(" | dT/dt: "); debugSerial.println(dTdt);

    // DEMO TX 
    if (currentState != safe) {
      sendPayload(temperature, humidity, score, co2);
      lastTx = now;
    }

    tempUpdated = false;
    co2Updated = false;
    humidityUpdated = false;
  }

  // HEARTBEAT (Stage 1 only) 
  if (currentState == safe &&
      now - lastHeartbeat >= HEARTBEAT_INTERVAL) {

    score = computeScore(temperature, humidity, co2);
    sendPayload(temperature, humidity, score, co2);

    lastHeartbeat = now;
  }

  // PERIODIC TX 
  if (now - lastTx >= getIntervalTx(currentState)) {
    score = computeScore(temperature, humidity, co2);
    sendPayload(temperature, humidity, score, co2);
    lastTx = now;
  }

  // LED UPDATE
  updateLEDs();
}