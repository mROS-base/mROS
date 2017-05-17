/* wiring_private.cpp */
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
#include <wiring_private.h>
#include <pins_arduino.h>

#include <RZ_A1_Init.h>
#include <MBRZA1H.h>

/*
 Use the unused OSTM1 in the mbed library to measure micros() and millis().
 OSTM1 is run in interval timer mode, and the timer interrupt is generated in 1m sec intervals.
 OSTM1 count clock P0Φ operates at either 32MHz or 100/3MHz. 
 The 1ms interval timer interrupt is realized by setting the OSTMnCMP pin of OSTM1
 to 32000-1 when operating at 32MHz, and 33333-1 when operating at 100/3MHz.
 Although a 0.001% error of calculation occurs when operating at 100/3MHz, 
 this can be ignored as it is smaller than the allowable deviation of ±0.005% 
 specified by SG-8003CGPCB, the GR-PEACH external clock.
  
*/

#if 1 // Use OSTM1
#define WrapAroundIrq OSTMI1TINT_IRQn
#define CpgStbcr5Bit  0b00000001
#define OsTimer       OSTM1
#else // Use OSTM0.
#define WrapAroundIrq OSTMI0TINT_IRQn
#define CpgStbcr5Bit  0b00000010
#define OsTimer       OSTM0
#endif

static uint32_t phy0ClockPeriod;
static volatile uint32_t msecCount;

void update1msecCounter(void)
{
    msecCount++;
}

uint32_t get1msecCounter(void)
{
    return msecCount;
}

void start1msecInterrupt(void (*func)(void))
{
    static bool started = false;
    if (!started) {
        CPG.STBCR5 &= ~CpgStbcr5Bit;            // OsTimer power supply
        OsTimer.OSTMnTT = 0b00000001;           // Timer stop

        msecCount = 0;
        InterruptHandlerRegister(WrapAroundIrq, (IRQHandler)func);
        GIC_SetPriority(WrapAroundIrq, 5);
        GIC_EnableIRQ(WrapAroundIrq);

        if (RZ_A1_IsClockMode0()) {
            phy0ClockPeriod = CM0_RENESAS_RZ_A1_P0_CLK / 1000;
        } else {
            phy0ClockPeriod = CM1_RENESAS_RZ_A1_P0_CLK / 1000;
        }
        OsTimer.OSTMnCMP = phy0ClockPeriod - 1; // Interrupt generation timing 
        OsTimer.OSTMnCTL = 0b00000000;          // Interval timer mode 
                                                // Interrupts disabled at count start
        OsTimer.OSTMnTS  = 0b00000001;          // Count start

        started = true;
    }
}

inline uint32_t getPhy0Clock(void)
{
    return phy0ClockPeriod - 1 - OsTimer.OSTMnCNT;
}

uint64_t getPhy0Clock64(void)
{
    int was_masked = __disable_irq();
    uint32_t currentPhy0Clock = getPhy0Clock();
    uint32_t currentMsecCount = msecCount;
    if (GIC_GetIRQStatus(WrapAroundIrq) & 1) {
        currentMsecCount = ++msecCount;
        currentPhy0Clock = 0;
        GIC_ClearPendingIRQ(WrapAroundIrq);
    }
    if (!was_masked) {
        __enable_irq();
    }
    return phy0ClockPeriod * (uint64_t)currentMsecCount + currentPhy0Clock;
}

uint32_t getPhy0ClockPeriod(void)
{
    return phy0ClockPeriod;
}

void delayPhy0Clock(uint64_t remainPhy0Clock)
{
    uint32_t startPhy0Clock = getPhy0Clock();
    while (remainPhy0Clock > 0) {
        uint32_t currentPhy0Clock = getPhy0Clock();
        uint32_t distancePhy0Clock = currentPhy0Clock - startPhy0Clock;
        if (currentPhy0Clock < startPhy0Clock) {
            distancePhy0Clock += phy0ClockPeriod;
        }
        if (remainPhy0Clock <= distancePhy0Clock) {
            break;
        }
        remainPhy0Clock -= distancePhy0Clock;
        startPhy0Clock = currentPhy0Clock;
    }
}

// Pin number→PinName Conversion table
const PinName PinNames[NUM_DIGITAL_PINS] = {
    P2_15,  P2_14,  P4_7,   P4_6,   P4_5,   P4_4,   P8_13,  P8_11,  P8_15,  P8_14,
    P10_13, P10_14, P10_15, P10_12, P1_8,   P1_9,   P1_10,  P1_11,  P1_13,  P1_15,
    P10_0,  P1_1,   P5_7,   P5_6,   P5_5,   P5_4,   P5_3,   P5_2,   P5_1,   P5_0,
    P7_15,  P8_1,   P2_9,   P2_10,  NC,     NC,     P2_0,   P2_1,   P2_2,   P2_3,
    P2_4,   P2_5,   P2_6,   P2_7,   P3_8,   P3_9,   P3_10,  P3_11,  P3_12,  P3_13,
    P3_14,  P3_15,  P1_0,   P1_12,  P1_7,   P1_6,   P6_6,   P7_7,   P8_7,   P7_6,
    P6_5,   P6_4,   P6_8,   P7_5,   P7_4,   P6_7,   P7_2,   P1_3,   P1_2,   P2_13,
    P4_0,   P8_3,   P8_4,   P8_5,   P8_6,   P7_8,   P3_2,   P8_8,   P11_12, P11_13,
    P11_14, P11_15, P6_3,   P6_2,   P6_13,  P6_14,  P6_15,  P6_12,  P6_0,
};

/*
 One troublesome aspect of the Arduino library is that pin initialization and 
 completion processing are handled separately. 
 As a solution, we have unified the interface for pin initialization and 
 completion processing in the library.
 Using “ArduinoPinModeXXXX” as a symbol to indicate the mode for each pin operation, 
 you can implement initialization function “setArduinoPinModeXXXX(PinName pinName)”
 and completion “resetArduinoPinModeXXXX(PinName pinName)” function as needed.
 Use the following to set a pin operation to a specific mode:

 setArduinoPinMode(PinName pinName, ArduinoPinMode mode)

 If the pin is already initialized to a different mode, function
 “resetArduinoPinModeXXXX(PinName pinName)” 
 will be called to end that mode, and then function setArduinoPinModeXXXX(PinName pinName)
 will be called to initialize the pin to the desired mode.
*/

// Pin Status Table
ArduinoPinStatus arduinoPinStatus[MaxPinName + 1] = {
    {ArduinoPinModeUnused},
};

void setArduinoPinMode(PinName pinName, ArduinoPinMode mode)
{
    if (isValidPinName(pinName)) {
        if (getArduinoPinMode(pinName) != mode) {
            resetArduinoPinMode(pinName);
            switch (mode) {
            case ArduinoPinModeUnused:
                if (setArduinoPinModeUnused != NULL) {
                    setArduinoPinModeUnused(pinName);
                }
                goto change;
            case ArduinoPinModeInput:
                if (setArduinoPinModeInput != NULL) {
                    setArduinoPinModeInput(pinName);
                }
                goto change;
            case ArduinoPinModeOutput:
                if (setArduinoPinModeOutput != NULL) {
                    setArduinoPinModeOutput(pinName);
                }
                goto change;
            case ArduinoPinModeInputPullUp:
                if (setArduinoPinModeInputPullUp != NULL) {
                    setArduinoPinModeInputPullUp(pinName);
                }
                goto change;
            case ArduinoPinModeOutputOpenDrain:
                if (setArduinoPinModeOutputOpenDrain != NULL) {
                    setArduinoPinModeOutputOpenDrain(pinName);
                }
                goto change;
            case ArduinoPinModeAnalogRead:
                if (setArduinoPinModeAnalogRead != NULL) {
                    setArduinoPinModeAnalogRead(pinName);
                }
                goto change;
            case ArduinoPinModeAnalogWrite:
                if (setArduinoPinModeAnalogWrite != NULL) {
                    setArduinoPinModeAnalogWrite(pinName);
                }
                goto change;
            case ArduinoPinModeInterrupt:
                if (setArduinoPinModeInterrupt != NULL) {
                    setArduinoPinModeInterrupt(pinName);
                }
                goto change;
            case ArduinoPinModeTone:
                if (setArduinoPinModeTone != NULL) {
                    setArduinoPinModeTone(pinName);
                }
                goto change;
            case ArduinoPinModeServo:
                if (setArduinoPinModeServo != NULL) {
                    setArduinoPinModeServo(pinName);
                }
                goto change;
            case ArduinoPinModeDac:
                if (setArduinoPinModeDac != NULL) {
                    setArduinoPinModeDac(pinName);
                }
                goto change;
            case ArduinoPinModeOther:
                if (setArduinoPinModeOther != NULL) {
                    setArduinoPinModeOther(pinName);
                }
              change:
                changeArduinoPinMode(pinName, mode);
                break;
            default:
                break;
            }
        }
    }
}

ArduinoPinMode getArduinoPinMode(PinName pinName)
{
    return isValidPinName(pinName) ? arduinoPinStatus[pinName].mode : ArduinoPinModeError;
}

void changeArduinoPinMode(PinName pinName, ArduinoPinMode mode)
{
    if (isValidPinName(pinName)) {
        arduinoPinStatus[pinName].mode = mode;
    }
}

void resetArduinoPinMode(PinName pinName)
{
    if (isValidPinName(pinName)) {
        ArduinoPinMode mode = getArduinoPinMode(pinName);
        switch (mode) {
        case ArduinoPinModeUnused:
            break;
        case ArduinoPinModeInput:
            if (resetArduinoPinModeInput != NULL) {
                resetArduinoPinModeInput(pinName);
            }
            goto change;
        case ArduinoPinModeOutput:
            if (resetArduinoPinModeOutput != NULL) {
                resetArduinoPinModeOutput(pinName);
            }
            goto change;
        case ArduinoPinModeInputPullUp:
            if (resetArduinoPinModeInputPullUp != NULL) {
                resetArduinoPinModeInputPullUp(pinName);
            }
            goto change;
        case ArduinoPinModeOutputOpenDrain:
            if (resetArduinoPinModeOutputOpenDrain != NULL) {
                resetArduinoPinModeOutputOpenDrain(pinName);
            }
            goto change;
        case ArduinoPinModeAnalogRead:
            if (resetArduinoPinModeAnalogRead != NULL) {
                resetArduinoPinModeAnalogRead(pinName);
            }
            goto change;
        case ArduinoPinModeAnalogWrite:
            if (resetArduinoPinModeAnalogWrite != NULL) {
                resetArduinoPinModeAnalogWrite(pinName);
            }
            goto change;
        case ArduinoPinModeInterrupt:
            if (resetArduinoPinModeInterrupt != NULL) {
                resetArduinoPinModeInterrupt(pinName);
            }
            goto change;
        case ArduinoPinModeTone:
            if (resetArduinoPinModeTone != NULL) {
                resetArduinoPinModeTone(pinName);
            }
            goto change;
        case ArduinoPinModeServo:
            if (resetArduinoPinModeServo != NULL) {
                resetArduinoPinModeServo(pinName);
            }
            goto change;
        case ArduinoPinModeDac:
            if (resetArduinoPinModeDac != NULL) {
                resetArduinoPinModeDac(pinName);
            }
            goto change;
        case ArduinoPinModeOther:
            if (resetArduinoPinModeOther != NULL) {
                resetArduinoPinModeOther(pinName);
            }
          change:
            changeArduinoPinMode(pinName, ArduinoPinModeUnused);
            break;
        default:
            break;
        }
    }
}
