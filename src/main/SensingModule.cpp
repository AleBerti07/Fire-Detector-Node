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
 * \file SensingModule.cpp
 * \author Remko Welling (remko.welling@han.nl)
 */

#include "SensingModule.h"

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

/*/ Functions for class: iotShieldTempsensor
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
*/

// Functions for class: iotShieldCO2Sensor
iotShieldCO2Sensor::iotShieldCO2Sensor():
  _startTime(millis())
{
}

iotShieldCO2Sensor::~iotShieldCO2Sensor(){};

float iotShieldCO2Sensor::getCO2ppm()
{

  unsigned long elapsed = millis() - _startTime;
  float co2 = 400.0 + (elapsed / 10000.0) * 100.0;
  return constrain(co2, 400.0, 2000.0);
}
