/* Wire.cpp */
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
#include <Wire.h>
#include <i2c_api.h>
#include <PinNames.h>
#include <Arduino.h>

TwoWire::TwoWire(uint8_t _channel, PinName _sda, PinName _scl)
{
    initialized = false;
    wire_channel = _channel;
    sda = _sda;
    scl = _scl;
}

void TwoWire::begin()
{
    begin(-1);
}

void TwoWire::begin(int _address)
{
    if (!initialized) {
        i2c_init(&i2c, sda, scl);
        initialized = true;
    }
    if (_address < 0) {
        i2c_slave_mode(&i2c, false);
    } else {
        i2c_slave_mode(&i2c, true);
        i2c_slave_address(&i2c, 0, _address << 1, 0);
    }
}

void TwoWire::beginTransmission(int _address)
{
    begin();
    i2c_start(&i2c);
    i2c_byte_write(&i2c, _address << 1);
}

uint8_t TwoWire::endTransmission()
{
    return endTransmission(true);
}

uint8_t TwoWire::endTransmission(uint8_t stop)
{
    if (stop) {
        i2c_stop(&i2c);
    }
    return 0;
}

uint8_t TwoWire::requestFrom(uint8_t _address, uint8_t quantity)
{
    return requestFrom(int(_address), int(quantity));
}

uint8_t TwoWire::requestFrom(uint8_t _address, uint8_t quantity, uint8_t stop)
{
    return requestFrom(int(_address), int(quantity), int(stop));
}

uint8_t TwoWire::requestFrom(int _address, int quantity)
{
    return requestFrom(_address, quantity, true);
}

uint8_t TwoWire::requestFrom(int _address, int quantity, int stop)
{
    // clamp to buffer length
    if (quantity > BUFFER_LENGTH) {
        quantity = BUFFER_LENGTH;
    }

    // perform blocking read into buffer
    i2c_read(&i2c, _address << 1, (char*)rxBuffer, quantity, stop);

    // set rx buffer iterator vars
    rxBufferIndex = 0;
    rxBufferLength = quantity;

    return quantity;
}

size_t TwoWire::write(uint8_t _data)
{
    return i2c_byte_write(&i2c, _data);
}

size_t TwoWire::write(const uint8_t* _data, size_t _size)
{
    for (size_t i = 0; i < _size; i++) {
        write(_data[i]);
    }
    return _size;
}

int TwoWire::available()
{
    return rxBufferLength - rxBufferIndex;
}

int TwoWire::read()
{
    int value = -1;

    // get each successive byte on each call
    if (rxBufferIndex < rxBufferLength) {
        value = rxBuffer[rxBufferIndex++];
    }

    return value;
}

int TwoWire::peek()
{
    int value = -1;

    // get each successive byte on each call
    if (rxBufferIndex < rxBufferLength) {
        value = rxBuffer[rxBufferIndex];
    }

    return value;
}

void TwoWire::flush()
{
}

void TwoWire::setFrequency(int freq)
{
    i2c_frequency(&i2c, freq);
}

void (*TwoWire::user_onRequest)(void);
void (*TwoWire::user_onReceive)(int);
void TwoWire::onRequestService(void){}
void TwoWire::onReceiveService(uint8_t*, int){}

TwoWire Wire0(0, P1_1, P1_0);
TwoWire Wire1(1, P1_3, P1_2);
//TwoWire Wire2(2, P1_5, P1_4);
TwoWire Wire3(3, P1_7, P1_6);
TwoWire& Wire = Wire1;
