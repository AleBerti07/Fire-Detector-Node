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
 * \file HAN_IoT_Shield.h
 * \brief functions to use hardware peripherals on the HAN IoT Shield.
 * \author Remko Welling (remko.welling@han.nl)
 * \date 17-12-2018
 * \version 1.1
 * 
 * Version|Date        |Note
 * -------|------------|----
 * 1.2    |  4- 2-2019 | Modified include "arduino.h" to "Arduino.h" to be linux compliant
 * 1.1    | 27-12-2018 | Updates on documentation
 * 1.0    | 15-12-2018 | Initial version
 * 
 * 
 */

 /*!
  \todo make documentation complete

  # HAN_IoT Shield Documentation

  ## Shield layout

  
  \verbatim
  
    +-----------------------------+
    | [gr-1] [gr-2] [gr-2] [gr-2] | <- 4 grove connectors
    |                             |
    |   +--------+   +--------+   |  
    |   |        |   |        |   |  
    |   |   P1   |   |   P2   |   |  <- 2 potentiometers
    |   |        |   |        |   |  
    |   +--------+   +--------+   |  
    |                             |
    |     +----+       +----+     |  
    |     |Red |       |Blck|     |  <- 2 push button switches
    |     +----+       +----+     |  
    |                             |
    | [led1] [led3] [led2] [led4] |  <- 4 LEDs
    |                             |
    | [Dallas]                    |  <- Dallas one wire temperature sensor
    +-\                 /---------+
       \---------------/

       
  \endverbatim

  
 */


#ifndef _HAN_IOT_SHIELD_H_
#define _HAN_IOT_SHIELD_H_

#include <Arduino.h> 

#include <OneWire.h>            // one wire library for Dallas one wire sensor
#include <DallasTemperature.h>  // Dallas one wire sensor library

// Defines for LEDs
#define PIN_LED_1_RED    3     ///< Led 1 is red and connected to arduino pin 3
#define PIN_LED_2_RED    4     ///< Led 2 is red and connected to arduino pin 4
#define PIN_LED_3_GRN    5     ///< Led 3 is green and connected to arduino pin 5
#define PIN_LED_4_GRN    6     ///< Led 4 is green and connected to arduino pin 6

// Defines for pushbutton
#define PIN_SWITCH_BLACK 8     ///< Black switch is connected to pin Arduino pin 8
#define PIN_SWITCH_RED   9     ///< Red switch is connected to pin Arduino pin 9
#define RELEASED HIGH          ///< represent activity of pushbutton, Released or not pressed is HIGH or '1'
#define PRESSED  LOW           ///< represent activity of pushbutton, Pressed is LOW or '0'

// defines for potentiometers.
#define PIN_POT_RED      A0    ///< Potmeter 1 with red knob is connected to Arduino pin A0
#define PIN_POT_WHITE    A1    ///< Potmeter 1 with white knob is connected to Arduino pin A1

// defines for Dallase one wire sensor
#define PIN_DALLAS       2     ///< Dallas DS18S20 one-wire temperature sensor is connected to Arduino pin 2

class iotShieldPotmeter
{
private:
uint8_t _pin;                 ///< Hardware pin to which the potentiometer is connected.
  int16_t _aRange;            ///< Minimum value that the potentiiometer will give
  int16_t _bRange;            ///< Maximum valie that the potentiometer will give.

public:
  /// \brief constructor
  /// \pre requires a analog input pin to which the potentiometer is connected
  /// The potentiometer shall apply 0 to VCC to the anlog input.
  /// \param hardwarePin Arduino pin to which the potentiometeris connected
  /// \param minimumValue of the range within the potentiometer will generate values. Default is 0
  /// \param maximumValue of the range within the potentiometer will generate values. Default is 1023.
  iotShieldPotmeter(uint8_t hardwarePin, int minimumValue = 0, int maximumValue = 1023);

  /// \brief Default destructor
  ~iotShieldPotmeter();

  /// \brief read value from potentiometer
  /// This function will transpose raw analog value form 0 to 1023 in to the range
  /// specified by the minimum and maximumvalue given at creation of the object.
  /// \return float transposed value from potentiometer in to the sepcified range.
  float getValue(void);
};

typedef enum { BUTTON_PRESSED,   ///< Button is pressed
               BUTTON_RELEASED   ///< Button is released
             } buttonState_t;
                 
class iotShieldButton
{
private:
  uint8_t _pin;                 ///< Hardware pin to which the potentiometer is connected.

public:
  /// \pre requires a digital input pin to which the button is connected
  /// The button shall deliver HIGH when RELEASED and LOW when PRESSED
  /// \param hardwarePin Arduino pin to which the button connected
  iotShieldButton(uint8_t hardwarePin);

  /// \brief Default destructor
  ~iotShieldButton();

  /// \brief get state of pushbutton enum
  /// functions returns actual state of pushbutton. Either PRESSED or RELEASED
  /// \return buttonState_t enum of state of pushbutton.
  buttonState_t getState();

  /// \brief get state of pushbutton as boolean
  /// functions returns actual state of pushbutton. Either PRESSED or RELEASED
  /// \return bool state of pushbutton presed = true, released = false.
  bool isPressed();
};

typedef enum { LED_ON,   ///< LED is ON
               LED_OFF   ///< LED is OFF
             } ledState_t;

class iotShieldLED
{
private:
  uint8_t _pin;                 ///< Hardware pin to which the potentiometer is connected.

public:
  /// \pre requires a digital input pin to which the LED is connected
  /// \param hardwarePin Arduino pin to which the LED connected
  iotShieldLED(uint8_t hardwarePin);

  /// \brief Default destructor
  ~iotShieldLED();

  /// \brief set state of LED
  /// \param state set LED state using ledState_t enum.
  void setState(ledState_t state);

};

class iotShieldTempSensor
{
private:
  float   _temperature;         ///< Local value to hold temperature read form Dallas sensor
  OneWire _oneWireInterface;    ///< Object of onewire interface
  DallasTemperature _sensors;   ///< Object of one wire sensors on one wire bus
  
public:
  /// \pre requires a digital input pin to which the LED is connected
  /// \param hardwarePin Arduino pin to which the LED connected
  iotShieldTempSensor(uint8_t hardwarePin = PIN_DALLAS);

  /// \brief Default destructor
  ~iotShieldTempSensor();

  /// \brief set state of LED
  /// \return state set LED state using ledState_t enum.
  float getTemperatureCelsius();

};

#endif // _HAN_IOT_SHIELD_H_
