/* wiring_analog.cpp */
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
#include "wiring_private.h"
#include "pins_arduino.h"
#include "pwmout_api.h"

static uint8_t currentAnalogReference = DEFAULT;

void analogReference(uint8_t mode)
{
    currentAnalogReference = mode;
}

int analogRead(PinName pinName)
{
    if (isValidPinName(pinName)) {
        int val = analogin_read_u16(&arduinoPinStatus[pinName].adc) >> 4;
        switch (currentAnalogReference) {
        case DEFAULT:
            val = val * (1024 * 33) / (4096 * 50);
            break;
        case INTERNAL:
            val = val * (1024 * 33) / (4096 * 11);
            if (val > 1023) {
                val = 1023;
            }
            break;
        case EXTERNAL:
            val = val * 1024 / 4096;
            break;
        case RAW12BIT:
            break;
        }
        return val;
    } else {
        return 0;
    }
}

int analogRead(uint8_t pin)
{
    PinName pinName = int2PinName(pin);
    if (pinName != NC) {
        return analogRead(pinName);
    } else {
        return 0;
    }
}

void setArduinoPinModeAnalogRead(PinName pinName)
{
    if (isValidPinName(pinName)) {
        analogin_init(&arduinoPinStatus[pinName].adc, pinName);
    }
}

/*
void resetArduinoPinModeAnalogRead(PinName pinName)
{
    if (isValidPinName(pinName)) {
        analogin_free(&arduinoPinStatus[pin].adc, pin);
    }
}
*/

void analogWrite(PinName pinName, int val)
{
    if (isValidPinName(pinName)) {
        setArduinoPinMode(pinName, ArduinoPinModeAnalogWrite);
        pwmout_write(&arduinoPinStatus[pinName].pwm, (1.0f / 255.0f) * val);
    }
}

void analogWrite(uint8_t pin, int val)
{
    PinName pinName = int2PinName(pin);
    if (pinName != NC) {
        analogWrite(pinName, val);
    }
}

void setArduinoPinModeAnalogWrite(PinName pinName)
{
    if (isValidPinName(pinName)) {
        pwmout_init(&arduinoPinStatus[pinName].pwm, pinName);
    }
}

void resetArduinoPinModeAnalogWrite(PinName pinName)
{
    if (isValidPinName(pinName)) {
        pwmout_free(&arduinoPinStatus[pinName].pwm);
    }
}
