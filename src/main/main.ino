#include <TheThingsNetwork.h>
#include <CayenneLPP.h>
#include "HAN_IoT_Shield.h"

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
iotShieldButton redButton(PIN_SWITCH_RED);
iotShieldButton blackButton(PIN_SWITCH_BLACK);
iotShieldLED leftRedLED(PIN_LED_1_RED);
iotShieldLED rightRedLED(PIN_LED_2_RED);
iotShieldLED leftGreenLED(PIN_LED_3_GRN);
iotShieldLED rightGreenLED(PIN_LED_4_GRN);

// Global variables
#define SAFE_INTERVAL 60000UL
#define WATCH_INTERVAL 30000UL
#define WARNING_INTERVAL 15000UL
#define ALARM_INTERVAL 5000UL

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
bool alarmActive = false;

unsigned long lastBlink = 0;
bool blinkState = false;
#define BLINK_INTERVAL 500UL

unsigned long lastTick = 0;
#define TICK_INTERVAL 100UL

float computeScore(float temp, float humidity) {
  // shifts the temperature, anything below will be counted as 'normal'. (anything below 30 degrees)
  // divide by the range from the shift to the highest (100 - 30 = 70)
  float tScore = constrain((temp - TEMPERATURE_SHIFT) / TEMPERATURE_DIV_RANGE, 0, 1.0) * 100; 

  // lower humidity percentage indicates a fire, so it is inversed from temperature
  // anyhting below 50 concerning
  float hScore = constrain((HUMIDITY_SHIFT - humidity) / HUMIDITY_DIV_RANGE, 0, 1.0) * 100;

  return (tScore * 0.6) + (hScore * 0.4);
}

unsigned long getIntervalTx(alarmState state) {
  switch(state) {
    case watch: return WATCH_INTERVAL;
    case warning: return WARNING_INTERVAL;
    case alarm: return ALARM_INTERVAL;
    default: return SAFE_INTERVAL;
  }
}

void updateState(float score) {
  alarmState newState;

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

void sendPayload(float temperature, float humidity, float score) {
  rightGreenLED.setState(LED_ON);

  lpp.reset();
  lpp.addTemperature(1, temperature);
  lpp.addRelativeHumidity(2, humidity);
  lpp.addAnalogInput(3, score);
  lpp.addPresence(4, alarmActive ? 1 : 0);
  lpp.addDigitalInput(5, (uint8_t)currentState);

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
  // Red button = immediate alarm
  if (redButton.isPressed()) {
    alarmActive = true;
    currentState = alarm; 
    byte alarm[2];
    alarm[0] = 0xFF;
    alarm[1] = 0x01;
    ttn.sendBytes(alarm, sizeof(alarm));
    debugSerial.println("-- MANUAL ALARM SENT");
    lastTx = millis(); // reset timer so next periodic isn't immediate
  }

  // Black button = poll for downlink
  if (blackButton.isPressed()) {
  if (alarmActive) {
    // Reset alarm
    alarmActive  = false;
    currentState = safe;
    leftRedLED.setState(LED_OFF);
    debugSerial.println("-- ALARM RESET");
  } else {
    // No alarm active = poll for downlink
    ttn.sendBytes(NULL, 0);
    debugSerial.println("-- POLL SENT");
  }
}


  if (millis() - lastTick >= TICK_INTERVAL) {
    // Read sensors
    float temperature = potmeter1.getValue();
    float humidity    = potmeter2.getValue();
    float score       = computeScore(temperature, humidity);

    debugSerial.println("Temp: "     + String(temperature));
    debugSerial.println("Humidity: " + String(humidity));
    debugSerial.println("Score: " + String(score));

    updateState(score);

    updateLEDs();

    // Periodic send
    if (millis() - lastTx >= getIntervalTx(currentState)) {
      sendPayload(temperature, humidity, score);
      lastTx = millis();
    }

    lastTick = millis();
  }

}