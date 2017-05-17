/*
  avrlib.h - Substitute of the AVRlib functions
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

#ifndef _AVRLIB_H
#define _AVRLIB_H

#ifdef __cplusplus
extern "C" {
#endif

char* itoa(int val, char* s, int radix);
char* ltoa(long val, char* s, int radix);
char* utoa(unsigned int val, char* s, int radix);
char* ultoa(unsigned long val, char* s, int radix);

#if 0 /* under constructions */
#define RANDOMMAX 0x7FFFFFFF
long random(void);
void srandom(unsigned long seed);
long random_r(unsigned long* ctx);
#endif

#define DTOSTRALWAYSSIGN 1
#define DTOSTRPLUSSIGN 2
#define DTOSTRUPPERCASE 4
char* dtostre(double val, char* s, unsigned char prec, unsigned char flags);
char* dtostrf(double val, signed char width, unsigned char prec, char* s);

#ifdef __cplusplus
};
#endif

#endif/*_AVRLIB_H*/
