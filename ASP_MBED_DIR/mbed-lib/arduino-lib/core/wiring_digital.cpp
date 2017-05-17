/* wiring_digital.cpp */
/* Copyright (C) 2016 Nozomu Fujita, MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#include <Arduino.h>
#include <wiring_private.h>
#include <platform.h>
#include <gpio_api.h>
#include <pins_arduino.h>

void pinMode(PinName pinName, uint8_t mode)
{
    if (isValidPinName(pinName)) {
        switch (mode) {
        case OUTPUT:
            setArduinoPinMode(pinName, ArduinoPinModeOutput);
            break;
        case OUTPUT_OPENDRAIN:
            setArduinoPinMode(pinName, ArduinoPinModeOutputOpenDrain);
            break;
        case INPUT:
            setArduinoPinMode(pinName, ArduinoPinModeInput);
            break;
        case INPUT_PULLUP:
            setArduinoPinMode(pinName, ArduinoPinModeInputPullUp);
            break;
        default:
            break;
        }
    }
}

void setArduinoPinModeOutput(PinName pinName)
{
    if (isValidPinName(pinName)) {
        gpio_init_out(&arduinoPinStatus[pinName].gpio, pinName);
    }
}

/*
void resetArduinoPinModeOutput(PinName pinName)
{
}
*/

void setArduinoPinModeOutputOpenDrain(PinName pinName)
{
    if (isValidPinName(pinName)) {
        gpio_init_inout(&arduinoPinStatus[pinName].gpio, pinName, PIN_OUTPUT, OpenDrain, 0);
    }
}

/*
void resetArduinoPinModeOutputOpenDrain(PinName pinName)
{
}
*/

void setArduinoPinModeInput(PinName pinName)
{
    if (isValidPinName(pinName)) {
        gpio_init_in_ex(&arduinoPinStatus[pinName].gpio, pinName, PullNone);
    }
}

/*
void resetArduinoPinModeInput(PinName pinName)
{
}
*/

void setArduinoPinModeInputPullUp(PinName pinName)
{
    if (isValidPinName(pinName)) {
        gpio_init_in_ex(&arduinoPinStatus[pinName].gpio, pinName, PullUp);
    }
}

/*
void resetArduinoPinModeInputPullUp(PinName pinName)
{
}
*/

void pinMode(uint8_t pin, uint8_t mode)
{
    PinName pinName = int2PinName(pin);
    if (pinName != NC) {
        pinMode(pinName, mode);
    }
}

static inline void _digitalWrite(PinName pinName, uint8_t value)
{
    if (isValidPinName(pinName)) {
        gpio_write(&arduinoPinStatus[pinName].gpio, value);
    }
}

void digitalWrite(PinName pinName, uint8_t value)
{
    _digitalWrite(pinName, value);
}

void digitalWrite(uint8_t pin, uint8_t value)
{
    PinName pinName = int2PinName(pin);
    if (pinName != NC) {
        _digitalWrite(pinName, value);
    }
}

static inline int _digitalRead(PinName pinName)
{
    if (isValidPinName(pinName)) {
        return gpio_read(&arduinoPinStatus[pinName].gpio);
    } else {
        return LOW;
    }
}

int digitalRead(PinName pinName)
{
    return _digitalRead(pinName);
}

int digitalRead(uint8_t pin)
{
    PinName pinName = int2PinName(pin);
    if (pinName != NC) {
        return _digitalRead(pinName);
    } else {
        return LOW;
    }
}
