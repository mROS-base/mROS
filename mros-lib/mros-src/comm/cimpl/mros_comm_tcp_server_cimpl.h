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
#ifndef _MROS_COMM_TCP_SERVER_CIMPL_H_
#define _MROS_COMM_TCP_SERVER_CIMPL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mros_comm_tcp_client_cimpl.h"

typedef struct {
	mRosCommSocketType socket;
} mRosCommTcpServerType;

extern mRosReturnType mros_comm_tcp_server_init(mRosCommTcpServerType *server);
extern mRosReturnType mros_comm_tcp_server_bind(mRosCommTcpServerType *server, mros_int32 port);
#define MROS_COMM_TCP_SERVER_LISTEN_MAX_DEFAULT_VALUE	1
extern mRosReturnType mros_comm_tcp_server_listen(mRosCommTcpServerType *server, mros_int32 max);
extern mRosReturnType mros_comm_tcp_server_accept(mRosCommTcpServerType *server, mRosCommTcpClientType *client);

#ifdef __cplusplus
}
#endif

#endif /* _MROS_COMM_TCP_SERVER_CIMPL_H_ */
