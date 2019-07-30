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
#ifndef _MROS_COMM_SOCKET_CIMPL_H_
#define _MROS_COMM_SOCKET_CIMPL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mros_comm_cimpl.h"

typedef struct {
	mros_int32		sock_fd;
	mros_uint32		timeout;
	mros_boolean	blocking;
	mros_int32		comm_type;
} mRosCommSocketType;

static inline void mros_comm_set_timeval(mros_uint32 tmo_msec, mRosTimeValType *tv)
{
    mros_uint32 tv_sec = tmo_msec / 1000;
    mros_uint32 tv_usec = (tmo_msec - (tv_sec * 1000)) * 1000;
	mros_comm_timeval_set(tv_sec, tv_usec, tv);
    return;
}

typedef enum {
	MROS_COMM_SOCKET_TYPE_TCP = 0,
	MROS_COMM_SOCKET_TYPE_UDP,
} mRosCommSocketEnumType;

#define MROS_COMM_DEFAULT_TIMEOUT		1500

extern mRosReturnType mros_comm_socket_init(mRosCommSocketType *socket, mRosCommSocketEnumType type);
extern mRosReturnType mros_comm_socket_open(mRosCommSocketType *socket);
extern void mros_comm_socket_close(mRosCommSocketType *socket);
extern mRosReturnType mros_comm_socket_set_blocking(mRosCommSocketType *socket, mros_boolean blocking, mros_uint32 timeout);

extern mRosReturnType mros_comm_socket_wait_readable(mRosCommSocketType *socket, mros_uint32 timeout);
extern mRosReturnType mros_comm_socket_wait_writable(mRosCommSocketType *socket, mros_uint32 timeout);

#ifdef __cplusplus
}
#endif

#endif /* _MROS_COMM_SOCKET_CIMPL_H_ */
