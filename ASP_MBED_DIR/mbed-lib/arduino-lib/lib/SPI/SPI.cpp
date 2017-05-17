/* SPI.cpp */
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
#include <SPI.h>
#include <spi_api.h>
#include <wiring_private.h>

SPISettings::SPISettings(uint32_t _clock, uint8_t _bitOrder, uint8_t _dataMode)
{
    clock = _clock;
    bitOrder = _bitOrder;
    dataMode = _dataMode;
}

SPIClass::SPIClass(PinName _mosi, PinName _miso, PinName _sclk, PinName _ssel)
{
    mosi = _mosi;
    miso = _miso;
    sclk = _sclk;
    ssel = _ssel;
    initialized = false;
}

SPIClass::SPIClass(int _mosi, int _miso, int _sclk, int _ssel)
{
    SPIClass(int2PinName(_mosi), int2PinName(_miso), int2PinName(_sclk), int2PinName(_ssel));
}

SPIClass::~SPIClass()
{
    if (initialized) {
        end();
    }
}

void SPIClass::begin(void)
{
    if (!initialized) {
        spi_init(&spi, mosi, miso, sclk, ssel);
        initialized = true;
    }
    setClockDivider(SPI_CLOCK_DIV4);
    setBitOrder(MSBFIRST);
    setDataMode(SPI_MODE0);
}

//void SPIClass::usingInterrupt(uint8_t interruptNumber){}
//void SPIClass::notUsingInterrupt(uint8_t interruptNumber){}
void SPIClass::beginTransaction(SPISettings settings)
{
    if (!initialized) {
        spi_init(&spi, mosi, miso, sclk, ssel);
        initialized = true;
    }
    spi_frequency(&spi, settings.clock);
    setBitOrder(settings.bitOrder);
    setDataMode(settings.dataMode);
}

static inline uint8_t reverse(uint8_t _data)
{
    return ((_data >> 7) & 0x01)
         | ((_data >> 5) & 0x02)
         | ((_data >> 3) & 0x04)
         | ((_data >> 1) & 0x08)
         | ((_data << 1) & 0x10)
         | ((_data << 3) & 0x20)
         | ((_data << 5) & 0x40)
         | ((_data << 7) & 0x80);
}

uint8_t SPIClass::transfer(uint8_t _data)
{
    if (bitOrder == LSBFIRST) {
        return reverse(spi_master_write(&spi, reverse(_data)));
    } else {
        return spi_master_write(&spi, _data);
    }
}

uint16_t SPIClass::transfer16(uint16_t _data)
{
    if (bitOrder == LSBFIRST) {
        uint16_t l = transfer((uint8_t)(_data >> 0));
        uint16_t h = transfer((uint8_t)(_data >> 8));
        return (l << 8) | (h >> 8);
    } else {
        uint16_t h = transfer((uint8_t)(_data >> 8));
        uint16_t l = transfer((uint8_t)(_data >> 0));
        return (h << 8) | (l >> 8);
    }
}

void SPIClass::transfer(void* _buf, size_t _count)
{
    uint8_t* buf = (uint8_t*)_buf;
    for (size_t i = 0; i < _count; i++) {
        buf[i] = transfer(buf[i]);
    }
}

void SPIClass::endTransaction(void)
{
}

void SPIClass::end(void)
{
    if (initialized) {
        spi_free(&spi);
        initialized = false;
    }
}

void SPIClass::setBitOrder(uint8_t _bitOrder)
{
    bitOrder = _bitOrder;
}

void SPIClass::setDataMode(uint8_t _dataMode)
{
    uint8_t mode = 0b00;
    switch (_dataMode) {
    case SPI_MODE0:
        mode = 0b00;
        break;
    case SPI_MODE1:
        mode = 0b01;
        break;
    case SPI_MODE2:
        mode = 0b10;
        break;
    case SPI_MODE3:
        mode = 0b11;
        break;
    }
    spi_format(&spi, 8, mode, false);
}

void SPIClass::setClockDivider(uint8_t _clockDiv)
{
    spi_frequency(&spi, SPI_CLOCK(_clockDiv));
}

//void SPIClass::attachInterrupt(void){}
//void SPIClass::detachInterrupt(void){}

SPIClass& SPI = SPI0;
SPIClass SPI0(P10_14, P10_15, P10_12, P10_13);
SPIClass SPI1(P4_6, P4_7, P4_4, P4_5);
//SPIClass SPI1(P11_14, P11_15, P11_12, P11_13);
SPIClass SPI2(P8_5, P8_6, P8_3, P8_4);
SPIClass SPI3(P5_2, P5_3, P5_0, P5_1);
