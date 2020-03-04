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
#ifndef _MROS_COMM_TCP_CLIENT_CIMPL_H_
#define _MROS_COMM_TCP_CLIENT_CIMPL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mros_comm_socket_cimpl.h"

typedef struct {
	mRosCommSocketType	socket;
	mRosSockAddrInType	remote;
	mros_boolean		connected;
} mRosCommTcpClientType;


extern mRosReturnType mros_comm_tcp_client_init(mRosCommTcpClientType *client, const char* host, mros_int32 port);
extern mRosReturnType mros_comm_tcp_client_ip32_init(mRosCommTcpClientType *client, mros_uint32 ipaddr, mros_int32 port);

extern void mros_comm_tcp_client_close(mRosCommTcpClientType *client);
extern mRosReturnType mros_comm_tcp_client_connect(mRosCommTcpClientType *client);
extern mRosReturnType mros_comm_tcp_client_connect_ip32(mRosCommTcpClientType *client, mros_uint32 ipaddr, mros_int32 port);
extern mros_boolean mros_comm_tcp_client_is_connected(mRosCommTcpClientType *client);
extern mRosReturnType mros_comm_tcp_client_send(mRosCommTcpClientType *client, const char* data, mRosSizeType length, mRosSizeType *res);
extern mRosReturnType mros_comm_tcp_client_send_all(mRosCommTcpClientType *client, const char* data, mRosSizeType length, mRosSizeType *res);
extern mRosReturnType mros_comm_tcp_client_receive(mRosCommTcpClientType *client, char* data, mRosSizeType length, mRosSizeType *res);
extern mRosReturnType mros_comm_tcp_client_receive_all(mRosCommTcpClientType *client, char* data, mRosSizeType length, mRosSizeType *res);


#ifdef __cplusplus
}
#endif

#endif /* _MROS_COMM_TCP_CLIENT_CIMPL_H_ */
