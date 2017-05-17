/* WInterrupts.cpp */
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
#include <gpio_irq_api.h>
#include <intc_iodefine.h>

#define IrqChannels 8
typedef void (*Handler)(void);
static Handler handlers[IrqChannels] = {NULL};
static const int nIRQn_h = 32;

static void irqCommon(int ch)
{
    if (INTCIRQRR & (1 << ch)) {
        if (handlers[ch] != NULL) {
            handlers[ch]();
        }
        do {
            INTCIRQRR &= ~(1 << ch);
        } while (INTCIRQRR & (1 << ch));
    }
}

static void irq0(void) {irqCommon(0);}
static void irq1(void) {irqCommon(1);}
static void irq2(void) {irqCommon(2);}
static void irq3(void) {irqCommon(3);}
static void irq4(void) {irqCommon(4);}
static void irq5(void) {irqCommon(5);}
static void irq6(void) {irqCommon(6);}
static void irq7(void) {irqCommon(7);}

void attachInterrupt(PinName pinName, Handler userFunc, int mode)
{
    if (isValidPinName(pinName)) {
        setArduinoPinMode(pinName, ArduinoPinModeInterrupt);
        int event = -1;
        switch (mode) {
        case LOW:
            event = 0b00;
            break;
        case CHANGE:
            event = 0b11;
            break;
        case RISING:
            event = 0b10;
            break;
        case FALLING:
            event = 0b01;
            break;
        default:
            break;
        }
        if (event >= 0) {
            int ch = arduinoPinStatus[pinName].gpio_irq.ch;
            GIC_DisableIRQ((IRQn_Type)(nIRQn_h + ch));
            handlers[ch] = userFunc;
            static const IRQHandler irqTbl[IrqChannels] = {
                &irq0, &irq1, &irq2, &irq3, &irq4, &irq5, &irq6, &irq7,
            };
            InterruptHandlerRegister((IRQn_Type)(nIRQn_h + ch), irqTbl[ch]);
            INTCICR1 = (INTCICR1 & ~(0b11 << (2 * ch))) | (event << (2 * ch));
            GIC_EnableIRQ((IRQn_Type)(nIRQn_h + ch));
        }
    }
}

void detachInterrupt(PinName pinName)
{
    if (isValidPinName(pinName)) {
        resetArduinoPinMode(pinName);
    }
}

void attachInterrupt(uint8_t irq, PinName pinName, void (*userFunc)(void), int mode)
{
    (void)irq;
    attachInterrupt(pinName, userFunc, mode);
}

void detachInterrupt(uint8_t irq, PinName pinName)
{
    (void)irq;
    detachInterrupt(pinName);
}

static PinName irq2PinName(uint8_t irq)
{
    static const PinName t[] = {
        P2_14 /*IRQ0*/,
        P2_15 /*IRQ1*/,
        P1_8  /*IRQ2*/,
        P1_9  /*IRQ3*/,
        P1_10 /*IRQ4*/,
        P1_11 /*IRQ5*/,
        P5_6  /*IRQ6*/,
        P1_7  /*IRQ7*/,
    };
    if (irq < uint8_t(sizeof(t) / sizeof(*t))) {
        return t[irq];
    } else {
        return NC;
    }
}

void attachInterrupt(uint8_t irq, void (*userFunc)(void), int mode)
{
    PinName pinName = irq2PinName(irq);
    if (pinName != NC) {
        attachInterrupt(pinName, userFunc, mode);
    }
}

void attachInterrupt(uint8_t irq, uint8_t pin, void (*userFunc)(void), int mode)
{
    (void)irq;
    PinName pinName = int2PinName(pin);
    if (pinName != NC) {
        attachInterrupt(pinName, userFunc, mode);
    }
}

void detachInterrupt(uint8_t irq)
{
    PinName pinName = irq2PinName(irq);
    if (pinName != NC) {
        detachInterrupt(pinName);
    }
}

void detachInterrupt(uint8_t irq, uint8_t pin)
{
    (void)irq;
    PinName pinName = int2PinName(pin);
    if (pinName != NC) {
        detachInterrupt(pinName);
    }
}

void setArduinoPinModeInterrupt(PinName pinName)
{
    if (isValidPinName(pinName)) {
        gpio_irq_init(&arduinoPinStatus[pinName].gpio_irq, pinName, 0, 0);
        gpio_init_in(&arduinoPinStatus[pinName].gpio, pinName);
    }
}

void resetArduinoPinModeInterrupt(PinName pinName)
{
    if (isValidPinName(pinName)) {
        gpio_irq_disable(&arduinoPinStatus[pinName].gpio_irq);
        gpio_irq_free(&arduinoPinStatus[pinName].gpio_irq);
    }
}
