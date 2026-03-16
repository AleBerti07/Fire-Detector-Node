/*--------------------------------------------------------------------
  This file is part of the HAN IoT shield library.

  This code is free software:
  you can redistribute it and/or modify it under the terms of a Creative
  Commons Attribution-NonCommercial 4.0 International License
  (http://creativecommons.org/licenses/by-nc/4.0/) by
  Remko Welling (https://ese.han.nl/~rwelling/) E-mail: remko.welling@han.nl

  The program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  --------------------------------------------------------------------*/

/*!
 * \file HAN_IoT_Shield.cpp
 * \author Remko Welling (remko.welling@han.nl
 */

#include "HAN_IoT_Shield.h"

// Functions for class: iotShieldPotmeter
iotShieldPotmeter::iotShieldPotmeter(uint8_t hardwarePin, 
                                     int minimumValue, 
                                     int maximumValue):
  _pin(hardwarePin),
  _aRange(maximumValue),
  _bRange(minimumValue)
{  
}

iotShieldPotmeter::~iotShieldPotmeter(){};

float iotShieldPotmeter::getValue()
{
  int rawValue    = analogRead(_pin);
  int mappedValue = map(rawValue, 0, 1023, _aRange, _bRange);
  return static_cast<float>(mappedValue);
}

// Functions for class: iotShieldButton
iotShieldButton::iotShieldButton(uint8_t hardwarePin):
  _pin(hardwarePin)
{  
}

iotShieldButton::~iotShieldButton(){};

buttonState_t iotShieldButton::getState()
{
  uint8_t state = digitalRead(_pin);
  buttonState_t returnValue = BUTTON_RELEASED;
  
  // test if red pushbutton is pressed.
  if(state == PRESSED)
  {
    returnValue = BUTTON_PRESSED;
  }
  return returnValue;
}

bool iotShieldButton::isPressed()
{
  uint8_t state = digitalRead(_pin);
  bool returnValue = false;
  
  // test if red pushbutton is pressed.
  if(state == PRESSED)
  {
    returnValue = true;
  }
  return returnValue;
};

// Functions for class: iotShieldLED
iotShieldLED::iotShieldLED(uint8_t hardwarePin):
  _pin(hardwarePin)
{  
   pinMode(_pin, OUTPUT);
}

iotShieldLED::~iotShieldLED(){};

void iotShieldLED::setState(ledState_t state)
{
  if( state == LED_ON)
  {
    digitalWrite(_pin, HIGH);   // turn the LED on (HIGH is the voltage level)
  }
  else
  {
    digitalWrite(_pin, LOW);   // turn the LED on (HIGH is the voltage level)
  }
}

// Functions for class: iotShieldTempsensor
iotShieldTempSensor::iotShieldTempSensor(uint8_t hardwarePin):
  _temperature(0.0),
  _oneWireInterface(hardwarePin),
  _sensors(&_oneWireInterface)
{
  // Start up the Dallas Temperature library
  _sensors.begin();
}

iotShieldTempSensor::~iotShieldTempSensor(){};

float iotShieldTempSensor::getTemperatureCelsius()
{
  _sensors.requestTemperatures();              // Send the command to get temperatures from single wire sensors
  _temperature = _sensors.getTempCByIndex(0);  // Read the temperature from the first sensor 
  return _temperature;
}
