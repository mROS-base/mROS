/*
  TwoWire.h - TWI/I2C library for Arduino & Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 2012 by Todd Krein (todd@krein.org) to implement repeated starts
  Modified 2014 by Yuuki Okamiya for software I2C GR-SAKURA
  Modified 2014 by Yuuki Okamiya for add setFrequency for Hardware Wire
  Modified 10 Jun 2014 by Nozomu Fujita : Wire Ç÷ÇÃ TwoWire::setFrequency(int) ëŒâû
  Modified 10 Jun 2014 by Nozomu Fujita : Wire3ÅAWire7 ëŒâû
  Modified 22 Jun 2016 by Nozomu Fujita : for GR-PEACH
*/

#ifndef TwoWire_h
#define TwoWire_h

#include "Stream.h"
#include <i2c_api.h>
#include <PinNames.h>

#define BUFFER_LENGTH 32

class TwoWire : public Stream
{
  private:
    bool initialized;
    i2c_t i2c;
    PinName sda;
    PinName scl;

    uint8_t rxBuffer[BUFFER_LENGTH];
    uint8_t rxBufferIndex;
    uint8_t rxBufferLength;

    uint8_t wire_channel;
    int wire_frequency;
    static void (*user_onRequest)(void);
    static void (*user_onReceive)(int);
    static void onRequestService(void);
    static void onReceiveService(uint8_t*, int);

  public:
    TwoWire(uint8_t _channel, PinName _sda, PinName _scl);
    virtual ~TwoWire(){};
    void begin();
    void begin(uint8_t);
    void begin(int);
    void beginTransmission(uint8_t);
    void beginTransmission(int);
    uint8_t endTransmission(void);
    uint8_t endTransmission(uint8_t);
    uint8_t requestFrom(uint8_t, uint8_t);
    uint8_t requestFrom(uint8_t, uint8_t, uint8_t);
    uint8_t requestFrom(int, int);
    uint8_t requestFrom(int, int, int);
    virtual size_t write(uint8_t);
    virtual size_t write(const uint8_t *, size_t);
    virtual int available(void);
    virtual int read(void);
    virtual int peek(void);
    virtual void flush(void);
    void setFrequency(int freq);
    void onReceive( void (*)(int) );
    void onRequest( void (*)(void) );

    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n) { return write((uint8_t)n); }
    inline size_t write(unsigned int n) { return write((uint8_t)n); }
    inline size_t write(int n) { return write((uint8_t)n); }
    inline size_t write(const char* string) { return write((const uint8_t*)string, strlen(string)); }
    using Print::write;
};

extern TwoWire Wire0;
extern TwoWire Wire1;
extern TwoWire Wire3;
extern TwoWire& Wire;

#endif
