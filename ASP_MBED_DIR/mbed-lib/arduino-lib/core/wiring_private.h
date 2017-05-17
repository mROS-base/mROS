/*
  wiring_private.h - Internal header file.
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2005-2006 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA
*/

#ifndef WiringPrivate_h
#define WiringPrivate_h

#ifndef GRPEACH
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdarg.h>
#endif/*GRPEACH*/

#include "Arduino.h"

#ifdef __cplusplus
extern "C"{
#endif

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

uint32_t countPulseASM(volatile uint8_t *port, uint8_t bit, uint8_t stateMask, unsigned long maxloops);

#define EXTERNAL_INT_0 0
#define EXTERNAL_INT_1 1
#define EXTERNAL_INT_2 2
#define EXTERNAL_INT_3 3
#define EXTERNAL_INT_4 4
#define EXTERNAL_INT_5 5
#define EXTERNAL_INT_6 6
#define EXTERNAL_INT_7 7

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega128RFA1__) || defined(__AVR_ATmega256RFR2__) || \
    defined(__AVR_AT90USB82__) || defined(__AVR_AT90USB162__) || defined(__AVR_ATmega32U2__) || defined(__AVR_ATmega16U2__) || defined(__AVR_ATmega8U2__)
#define EXTERNAL_NUM_INTERRUPTS 8
#elif defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)
#define EXTERNAL_NUM_INTERRUPTS 3
#elif defined(__AVR_ATmega32U4__)
#define EXTERNAL_NUM_INTERRUPTS 5
#else
#define EXTERNAL_NUM_INTERRUPTS 2
#endif

typedef void (*voidFuncPtr)(void);

#ifdef GRPEACH
#include <pins_arduino.h>
#include <gpio_api.h>
#include <analogin_api.h>
#include <gpio_irq_api.h>
#include <pwmout_api.h>

void start1msecInterrupt(void (*func)(void));
void update1msecCounter(void) __attribute__ ((weak));
uint32_t get1msecCounter(void);
uint32_t getPhy0ClockPeriod(void);
uint32_t getPhy0Clock(void);
uint64_t getPhy0Clock64(void);
void delayPhy0Clock(uint64_t clocks);

#define MaxPinName (P11_15)

static inline int isValidPinName(PinName pinName)
{
    return (pinName >= 0 && pinName <= MaxPinName);
}

extern const PinName PinNames[NUM_DIGITAL_PINS];

static inline PinName int2PinName(int pin)
{
    return (pin >= 0 && pin < NUM_DIGITAL_PINS) ? PinNames[pin] : NC;
}

typedef enum {
    ArduinoPinModeUnused = 0,
    ArduinoPinModeInput,
    ArduinoPinModeOutput,
    ArduinoPinModeInputPullUp,
    ArduinoPinModeOutputOpenDrain,
    ArduinoPinModeAnalogRead,
    ArduinoPinModeAnalogWrite,
    ArduinoPinModeInterrupt,
    ArduinoPinModeTone,
    ArduinoPinModeServo,
    ArduinoPinModeDac,
    ArduinoPinModeOther,
    ArduinoPinModeError,
} ArduinoPinMode;

typedef struct {
    ArduinoPinMode mode;
    gpio_t gpio;
    union {
        pwmout_t pwm;
        analogin_t adc;
        gpio_irq_t gpio_irq;
    };
} ArduinoPinStatus;

extern ArduinoPinStatus arduinoPinStatus[MaxPinName + 1];

void setArduinoPinMode(PinName pinName, ArduinoPinMode mode);
ArduinoPinMode getArduinoPinMode(PinName pinName);
void changeArduinoPinMode(PinName pinName, ArduinoPinMode mode);
void resetArduinoPinMode(PinName pinName);
void setArduinoPinModeUnused(PinName pinName) __attribute__ ((weak));
void setArduinoPinModeInput(PinName pinName) __attribute__ ((weak));
void resetArduinoPinModeInput(PinName pinName) __attribute__ ((weak));
void setArduinoPinModeOutput(PinName pinName) __attribute__ ((weak));
void resetArduinoPinModeOutput(PinName pinName) __attribute__ ((weak));
void setArduinoPinModeInputPullUp(PinName pinName) __attribute__ ((weak));
void resetArduinoPinModeInputPullUp(PinName pinName) __attribute__ ((weak));
void setArduinoPinModeOutputHigh(PinName pinName) __attribute__ ((weak));
void resetArduinoPinModeOutputHigh(PinName pinName) __attribute__ ((weak));
void setArduinoPinModeOutputOpenDrain(PinName pinName) __attribute__ ((weak));
void resetArduinoPinModeOutputOpenDrain(PinName pinName) __attribute__ ((weak));
void setArduinoPinModeAnalogRead(PinName pinName) __attribute__ ((weak));
void resetArduinoPinModeAnalogRead(PinName pinName) __attribute__ ((weak));
void setArduinoPinModeAnalogWrite(PinName pinName) __attribute__ ((weak));
void resetArduinoPinModeAnalogWrite(PinName pinName) __attribute__ ((weak));
void setArduinoPinModeInterrupt(PinName pinName) __attribute__ ((weak));
void resetArduinoPinModeInterrupt(PinName pinName) __attribute__ ((weak));
void setArduinoPinModeTone(PinName pinName) __attribute__ ((weak));
void resetArduinoPinModeTone(PinName pinName) __attribute__ ((weak));
void setArduinoPinModeServo(PinName pinName) __attribute__ ((weak));
void resetArduinoPinModeServo(PinName pinName) __attribute__ ((weak));
void setArduinoPinModeDac(PinName pinName) __attribute__ ((weak));
void resetArduinoPinModeDac(PinName pinName) __attribute__ ((weak));
void setArduinoPinModeOther(PinName pinName) __attribute__ ((weak));
void resetArduinoPinModeOther(PinName pinName) __attribute__ ((weak));
void setArduinoPinModeHardwarePWM(PinName pinName, int period, int term, unsigned long length);
void changeArduinoPinModeHardwarePWM(PinName pinName, int period, int term, unsigned long length);
void resetArduinoPinModeHardwarePWM(PinName pinName);
void setArduinoPinModeSoftwarePWM(PinName pinName, int period, int term, unsigned long length);
void changeArduinoPinModeSoftwarePWM(PinName pinName, int period, int term, unsigned long length);
void resetArduinoPinModeSoftwarePWM(PinName pinName);
#endif/*GRPEACH*/

#ifdef __cplusplus
} // extern "C"
#endif

#endif/*WiringPrivate_h*/
