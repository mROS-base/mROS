/* wiring.c */
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

uint32_t micros(void)
{
    return (uint32_t)(1000 * getPhy0Clock64() / getPhy0ClockPeriod());
}

uint32_t millis(void)
{
    return (uint32_t)(getPhy0Clock64() / getPhy0ClockPeriod());
}

void delayMicroseconds(unsigned int us)
{
    if (us > 0) {
        delayPhy0Clock(getPhy0ClockPeriod() * (uint64_t)us / 1000);
    }
}

void delay(unsigned long ms)
{
    if (ms > 0) {
        delayPhy0Clock(getPhy0ClockPeriod() * (uint64_t)ms);
    }
}

#if 0
void wait(float s)
{
    delayPhy0Clock(1000.0f * getPhy0ClockPeriod() * s);
}
#endif

void wait_ms(int ms)
{
    delay(ms);
}

void wait_us(int us)
{
    delayMicroseconds(us);
}

uint32_t us_ticker_read(void)
{
    return micros();
}
