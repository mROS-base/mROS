/* Tone.cpp */
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

#define MTU2_PWM_NUM            22
#define MTU2_PWM_OFFSET         0x20

static struct {
    bool sw;
    PinName pinName;
    uint32_t duration;
} toneDurationTable[MTU2_PWM_NUM] = {
    {false, NC, 0},
};

void tone(PinName pinName, unsigned int frequency, unsigned long duration)
{
    if (isValidPinName(pinName)) {
        setArduinoPinMode(pinName, ArduinoPinModeTone);
        PWMName pwmName = arduinoPinStatus[pinName].pwm.pwm;
        if (pwmName >= MTU2_PWM_OFFSET && pwmName < MTU2_PWM_OFFSET + MTU2_PWM_NUM) {
            int pwmIndex = (int)(pwmName - MTU2_PWM_OFFSET);
            toneDurationTable[pwmIndex].sw = false;
            pwmout_period_us(&arduinoPinStatus[pinName].pwm, 1000 * 1000 / frequency);
            pwmout_write(&arduinoPinStatus[pinName].pwm, 0.5f);
            if (duration > 0) {
                toneDurationTable[pwmIndex].pinName = pinName;
                toneDurationTable[pwmIndex].duration = duration;
                toneDurationTable[pwmIndex].sw = true;
            }
        }
    }
}

void tone(uint8_t pin, unsigned int frequency, unsigned long duration)
{
    PinName pinName = int2PinName(pin);
    if (pinName != NC) {
        tone(pinName, frequency, duration);
    }
}

void noTone(PinName pinName)
{
    if (isValidPinName(pinName)) {
        resetArduinoPinMode(pinName);
    }
}

void noTone(uint8_t pin)
{
    PinName pinName = int2PinName(pin);
    if (pinName != NC) {
        noTone(pinName);
    }
}

void setArduinoPinModeTone(PinName pinName)
{
    if (isValidPinName(pinName)) {
        arduinoPinStatus[pinName].pwm.pwm = (PWMName)(MTU2_PWM_OFFSET - 1);
        pwmout_init(&arduinoPinStatus[pinName].pwm, pinName);
        PWMName pwmName = arduinoPinStatus[pinName].pwm.pwm;
        if (pwmName < MTU2_PWM_OFFSET) {
            changeArduinoPinMode(pinName, ArduinoPinModeUnused);
        }
    }
}

void resetArduinoPinModeTone(PinName pinName)
{
    if (isValidPinName(pinName)) {
        pwmout_free(&arduinoPinStatus[pinName].pwm);
    }
}

void updateToneDuration(void)
{
    for (int i = 0; i < MTU2_PWM_NUM; i++) {
        if (toneDurationTable[i].sw && toneDurationTable[i].duration > 0) {
            if (--toneDurationTable[i].duration == 0) {
                noTone(toneDurationTable[i].pinName);
            }
        }
    }
}
