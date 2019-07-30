/* Copyright (C) 2012 mbed.org, MIT License
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
#ifndef _MROS_COMM_TARGET_H_
#define _MROS_COMM_TARGET_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mros_types.h"
#include "lwip/sockets.h"
#include "lwip/init.h"
#include "lwip/netdb.h"

typedef struct sockaddr 	mRosSockAddrType;
typedef struct sockaddr_in	mRosSockAddrInType;
typedef struct hostent 		mRosHostEntType;
typedef ip_addr_t 			mRosIpAaddrType;
typedef fd_set				mRosFdSetType;
typedef struct timeval		mRosTimeValType;

#define MROS_FD_ZERO(arg)				FD_ZERO(arg)
#define MROS_FD_SET(arg1, arg2)			FD_SET(arg1, arg2)
#define MROS_FD_SETSIZE					FD_SETSIZE
#define MROS_FD_ISSET(arg1, arg2)		FD_ISSET(arg1, arg2)

#define MROS_SOCK_STREAM     SOCK_STREAM
#define MROS_SOCK_DGRAM      SOCK_DGRAM
#define MROS_SOCK_RAW        SOCK_RAW

#define MROS_SOCK_UNSPEC	AF_UNSPEC
#define MROS_SOCK_AF_INET	AF_INET


#ifdef __cplusplus
}
#endif


#endif /* _MROS_COMM_TARGET_H_ */
