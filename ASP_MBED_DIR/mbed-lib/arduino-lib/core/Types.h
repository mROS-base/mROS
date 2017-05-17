/***************************************************************************
PURPOSE
     RX63N Library for Arduino compatible framework

TARGET DEVICE
     RX63N

AUTHOR
     Renesas Solutions Corp.
     AND Technology Research Ltd.

***************************************************************************
Copyright (C) 2014 Renesas Electronics. All rights reserved.

This library is free software; you can redistribute it and/or modify it under
the terms of the GNU Lesser General Public License as published by the Free
Software Foundation; either version 2.1 of the License, or (at your option) any
later version.

See file LICENSE.txt for further informations on licensing terms.

***************************************************************************
 Copyright   : (c) AND Technology Research Ltd, 2013
 Address     : 4 Forest Drive, Theydon Bois, Essex, CM16 7EY
 Tel         : +44 (0) 1992 81 4655
 Fax         : +44 (0) 1992 81 3362
 Email       : ed.king@andtr.com
 Website     : www.andtr.com

 Project     : Arduino
 Module      : Core
 File        : Types.h
 Author      : E King
 Start Date  : 30/07/13
 Description : Global type definitions.
 @version    : 1.00
 @since      : 1.00

 ******************************************************************************/
/*
 *  Modified 9 May 2014 by Yuuki Okamiya, for remove warnings
 *  Modified 13 July 2014 by Nozomu Fujita, for add bool type
 */

#ifndef TYPES_HPP_
#define TYPES_HPP_


//#include "rx63n/typedefine.h"
#include <stdint.h>
#include <stdbool.h>

// DEFINITIONS ****************************************************************/

/** Definitions of register bits. */
#define BIT_00    0x01
#define BIT_01    0x02
#define BIT_02    0x04
#define BIT_03    0x08
#define BIT_04    0x10
#define BIT_05    0x20
#define BIT_06    0x40
#define BIT_07    0x80

/** Further min/max definitions. */
#define U8_MAX (255U)
#define S8_MAX (127)
#define S8_MIN (-128)

#define U16_MAX (65535U)
#define S16_MAX (32767)
#define S16_MIN (-32768)

#define U32_MAX (4294967295L)
#define S32_MAX (2147483647L)
#define S32_MIN (-2147483648L)

/** Definitions of USB modes. */
#define USB_MODE_DEVICE  0x00
#define USB_MODE_HOST    0x01

// DECLARATIONS ***************************************************************/

/** Standard Arduino type definitions. */
typedef bool            boolean;
typedef unsigned int    word;
typedef unsigned char   byte;
typedef void (*fInterruptFunc_t)(void);
typedef void (*fITInterruptFunc_t)(unsigned long u32timer_millis);

/** Strongly-typed void callback function pointer. */
typedef void (*callback_func_t)(void);



#endif // TYPES_HPP_
