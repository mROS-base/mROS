/*
  wiring_shift.c - shiftOut() function
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

#include <Arduino.h>
#include <wiring_private.h>
#include <pins_arduino.h>

uint8_t shiftIn(PinName dataPinName, PinName clockPinName, uint8_t bitOrder)
{
    return (uint8_t)shiftInEx(dataPinName, clockPinName, bitOrder, 8);
}

uint32_t shiftInEx(PinName dataPinName, PinName clockPinName, uint8_t bitOrder, int len)
{
    uint32_t value = 0;

    for (int i = 0; i < len; i++) {
        digitalWrite(clockPinName, HIGH);
        if (bitOrder == LSBFIRST) {
            value |= digitalRead(dataPinName) << i;
        } else {
            value |= digitalRead(dataPinName) << (len - 1 - i);
        }
        digitalWrite(clockPinName, LOW);
    }
    return value;
}

void shiftOut(PinName dataPinName, PinName clockPinName, uint8_t bitOrder, int len, uint8_t val)
{
    shiftOutEx(dataPinName, clockPinName, bitOrder, 8, val);
}

void shiftOutEx(PinName dataPinName, PinName clockPinName, uint8_t bitOrder, int len, uint32_t val)
{
    for (int i = 0; i < len; i++)  {
        if (bitOrder == LSBFIRST) {
            digitalWrite(dataPinName, !!(val & (1 << i)));
        } else {
            digitalWrite(dataPinName, !!(val & (1 << (len - 1 - i))));
        }
        digitalWrite(clockPinName, HIGH);
        digitalWrite(clockPinName, LOW);
    }
}

uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder)
{
    PinName dataPinName = int2PinName(dataPin);
    PinName clockPinName = int2PinName(clockPin);
    if (dataPinName != NC && clockPinName != NC) {
        return (uint8_t)shiftInEx(dataPinName, clockPinName, bitOrder, 8);
    } else {
        return 0;
    }
}

uint32_t shiftInEx(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, int len)
{
    PinName dataPinName = int2PinName(dataPin);
    PinName clockPinName = int2PinName(clockPin);
    if (dataPinName != NC && clockPinName != NC) {
        return shiftInEx(dataPinName, clockPinName, bitOrder, len);
    } else {
        return 0;
    }
}

void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val)
{
    PinName dataPinName = int2PinName(dataPin);
    PinName clockPinName = int2PinName(clockPin);
    if (dataPinName != NC && clockPinName != NC) {
        shiftOutEx(dataPinName, clockPinName, bitOrder, 8, val);
    }
}

void shiftOutEx(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, int len, uint32_t val)
{
    PinName dataPinName = int2PinName(dataPin);
    PinName clockPinName = int2PinName(clockPin);
    if (dataPinName != NC && clockPinName != NC) {
        shiftOutEx(dataPinName, clockPinName, bitOrder, len, val);
    }
}
