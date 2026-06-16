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
 * \file SensingModule.h
 * \brief Module for sensing components on the HAN IoT Shield.
 * \author Remko Welling (remko.welling@han.nl)
 */

#ifndef _SENSING_MODULE_H_
#define _SENSING_MODULE_H_

#include <Arduino.h>
//#include <OneWire.h>
//#include <DallasTemperature.h>

// defines for potentiometers.
#define PIN_POT_RED      A0    ///< Potmeter 1 with red knob is connected to Arduino pin A0
#define PIN_POT_WHITE    A1    ///< Potmeter 1 with white knob is connected to Arduino pin A1

// defines for Dallas one wire sensor
#define PIN_DALLAS       2     ///< Dallas DS18S20 one-wire temperature sensor is connected to Arduino pin 2

class iotShieldPotmeter
{
private:
  uint8_t _pin;                 ///< Hardware pin to which the potentiometer is connected.
  int16_t _aRange;              ///< Minimum value that the potentiometer will give
  int16_t _bRange;              ///< Maximum value that the potentiometer will give.

public:
  /// \brief constructor
  /// \pre requires a analog input pin to which the potentiometer is connected
  /// The potentiometer shall apply 0 to VCC to the analog input.
  /// \param hardwarePin Arduino pin to which the potentiometer is connected
  /// \param minimumValue of the range within the potentiometer will generate values. Default is 0
  /// \param maximumValue of the range within the potentiometer will generate values. Default is 1023.
  iotShieldPotmeter(uint8_t hardwarePin, int minimumValue = 0, int maximumValue = 1023);

  /// \brief Default destructor
  ~iotShieldPotmeter();

  /// \brief read value from potentiometer
  /// This function will transpose raw analog value from 0 to 1023 into the range
  /// specified by the minimum and maximum value given at creation of the object.
  /// \return float transposed value from potentiometer into the specified range.
  float getValue(void);
};

class iotShieldTempSensor
{
private:
  float   _temperature;         ///< Local value to hold temperature read from Dallas sensor
  //OneWire _oneWireInterface;    ///< Object of onewire interface
  //DallasTemperature _sensors;   ///< Object of one wire sensors on one wire bus

public:
  /// \pre requires a digital input pin to which the sensor is connected
  /// \param hardwarePin Arduino pin to which the sensor is connected
  iotShieldTempSensor(uint8_t hardwarePin = PIN_DALLAS);

  /// \brief Default destructor
  ~iotShieldTempSensor();

  /// \brief get temperature in Celsius
  /// \return float temperature in Celsius
  float getTemperatureCelsius();
};

class iotShieldCO2Sensor
{
private:
  float _co2;   // persistent state between calls — replaces _startTime

public:
  iotShieldCO2Sensor();

  ~iotShieldCO2Sensor();

  float getCO2ppm(float temperature, float dTdt);  // <-- updated signature
};

#endif // _SENSING_MODULE_H_
