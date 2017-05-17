/* MsTimer2.cpp */
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
#include <MsTimer2.h>
#include <MsTimer2_private.h>

unsigned long MsTimer2::msecs = 0;
void (*MsTimer2::func)() = 0;
volatile unsigned long MsTimer2::count = 0;
bool MsTimer2::sw = false;

void MsTimer2::set(unsigned long ms, void (*f)())
{
    MsTimer2::sw = false;
    MsTimer2::msecs = ms;
    MsTimer2::func = f;
    MsTimer2::count = 0;
}

void MsTimer2::start()
{
    if (MsTimer2::func != 0) {
        MsTimer2::sw = true;
    }
}

void MsTimer2::stop()
{
    MsTimer2::sw = false;
}

void updateMsTimer2(void)
{
    if (MsTimer2::sw) {
        if (++MsTimer2::count >= MsTimer2::msecs) {
            MsTimer2::count = 0;
            if (MsTimer2::func != 0) {
                MsTimer2::func();
            }
        }
    }
}
