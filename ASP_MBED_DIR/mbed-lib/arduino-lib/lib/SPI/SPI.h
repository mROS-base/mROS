/* SPI.h */
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
#ifndef _SPI_H_
#define _SPI_H_

#include <Arduino.h>
#include <spi_api.h>

#ifndef LSBFIRST
#define LSBFIRST 0
#endif
#ifndef MSBFIRST
#define MSBFIRST 1
#endif

#define SPI_CLOCK_DIV2   0x00  // 8MHz
#define SPI_CLOCK_DIV4   0x01  // 4MHz
#define SPI_CLOCK_DIV8   0x03  // 2MHz
#define SPI_CLOCK_DIV16  0x07  // 1MHz
#define SPI_CLOCK_DIV32  0x0F  //
#define SPI_CLOCK_DIV64  0x1F
#define SPI_CLOCK_DIV128 0x3F
#define SPI_CLOCK(div) ((16 * 1000 * 1000) / (2 * ((div) + 1)))

#define SPI_MODE0 0x0
#define SPI_MODE1 0x1
#define SPI_MODE2 0x2
#define SPI_MODE3 0x3

class SPISettings {
    friend class SPIClass;
public:
    SPISettings(uint32_t clock = SPI_CLOCK(SPI_CLOCK_DIV4), uint8_t bitOrder = MSBFIRST, uint8_t dataMode = SPI_MODE0);
private:
    uint32_t clock;
    uint8_t bitOrder;
    uint8_t dataMode;
};

class SPIClass {
    PinName mosi;
    PinName miso;
    PinName sclk;
    PinName ssel;
    uint8_t bitOrder;
    boolean initialized;
    spi_t spi;
public:
    SPIClass(PinName mosi, PinName miso, PinName sclk, PinName ssel);
    SPIClass(int mosi, int miso, int sclk, int ssel);
    ~SPIClass();
    void begin();
    void usingInterrupt(uint8_t interruptNumber);
    void notUsingInterrupt(uint8_t interruptNumber);
    void beginTransaction(SPISettings settings);
    uint8_t transfer(uint8_t data);
    uint16_t transfer16(uint16_t data);
    void transfer(void *buf, size_t count);
    void endTransaction(void);
    void end();
    void setBitOrder(uint8_t bitOrder);
    void setDataMode(uint8_t dataMode);
    void setClockDivider(uint8_t clockDiv);
    void attachInterrupt();
    void detachInterrupt();
};

extern SPIClass& SPI;
extern SPIClass SPI0;
extern SPIClass SPI1;
extern SPIClass SPI2;
extern SPIClass SPI3;

#endif/*_SPI_H_*/
