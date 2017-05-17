/*
  avrlib.c - Substitute of the AVRlib functions
  Copyright (c) 2014 Nozomu Fujita.  All right reserved.

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
*/

#include <stdio.h>
#include "avrlib.h"

char* itoa(int val, char* s, int radix)
{
	return ltoa(val, s, radix);
}

char* ltoa(long val, char* s, int radix)
{
	if (val >= 0) {
		ultoa(val, s, radix);
	} else {
		*s = '-';
		ultoa(-val, s + 1, radix);
	}
	return s;
}

char* utoa(unsigned int val, char* s, int radix)
{
	return ultoa(val, s, radix);
}

char* ultoa(unsigned long val, char* s, int radix)
{
	char* _s = s;
	if (radix >= 2 && radix <= 36) {
		char buf[sizeof(val) * __CHAR_BIT__];
		int i = 0;
		do {
			int mod = val % radix;
			buf[i++] = (mod < 10) ? '0' + mod : 'a' + mod - 10;
			val /= radix;
		} while (val > 0);
		do {
			*_s++ = buf[--i];
		} while (i > 0);
	}
	*_s = '\0';
	return s;
}

#if 0 /* under constructions */
long random(void)
{
	return 0;
}

void srandom(unsigned long seed)
{
	(void)seed;
}

long random_r(unsigned long* ctx)
{
	(void)ctx;
	return 0;
}
#endif

char* dtostre(double val, char* s, unsigned char prec, unsigned char flags)
{
	char* _s = s;
	if (prec > 7) {
		prec = 7;
	}
	if (flags & DTOSTRPLUSSIGN) {
		if (val >= 0) {
			*_s++ = '+';
		}
	} else if (flags & DTOSTRALWAYSSIGN) {
		if (val >= 0) {
			*_s++ = ' ';
		}
	}
	const char* fmt = "%.*le";
	if (flags & DTOSTRUPPERCASE) {
		fmt = "%.*lE";
	}
	sprintf(_s, fmt, prec, val);
	return s;
}

char* dtostrf(double val, signed char width, unsigned char prec, char* s)
{
	sprintf(s, "%*.*lf", width, prec, val);
	return s;
}
