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
#include "mros_comm_tcp_server_cimpl.h"

mRosReturnType mros_comm_tcp_server_init(mRosCommTcpServerType *server)
{
	return mros_comm_socket_init(&server->socket, MROS_COMM_SOCKET_TYPE_TCP);
}

mRosReturnType mros_comm_tcp_server_bind(mRosCommTcpServerType *server, mros_int32 port)
{
	mRosReturnType ret;
    mRosSockAddrInType addr;

    mros_comm_inet_local_sockaddr_init(&addr, port);
    ret = mros_comm_bind(server->socket.sock_fd, (mRosSockAddrType*)&addr, sizeof(mRosSockAddrInType));
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return MROS_E_SYSERR;
	}
	return MROS_E_OK;
}
mRosReturnType mros_comm_tcp_server_listen(mRosCommTcpServerType *server, mros_int32 max)
{
	mRosReturnType ret;
    if (server->socket.sock_fd < 0) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, MROS_E_SYSERR);
		return MROS_E_SYSERR;
    }

    ret = mros_comm_listen(server->socket.sock_fd, max);
    if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return MROS_E_SYSERR;
    }
	return MROS_E_OK;
}

mRosReturnType mros_comm_tcp_server_accept(mRosCommTcpServerType *server, mRosCommTcpClientType *client)
{
	mRosReturnType ret;
	mRosSockAddrInType addr;

    if (server->socket.sock_fd < 0) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, MROS_E_SYSERR);
		return MROS_E_SYSERR;
    }
    if (server->socket.blocking == MROS_FALSE) {
    	ret = mros_comm_socket_wait_readable(&server->socket, server->socket.timeout);
    	if (ret != MROS_E_OK) {
    		if (ret != MROS_E_NOENT) {
        		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
    		}
    		return ret;
    	}
    }

    mRosSizeType len = sizeof(mRosSockAddrInType);
    client->socket.sock_fd = mros_comm_accept(server->socket.sock_fd, (mRosSockAddrType*)&addr, &len);
    if (client->socket.sock_fd < 0) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, MROS_E_SYSERR);
		return MROS_E_SYSERR;
    }
    client->socket.blocking = MROS_TRUE;
    client->socket.timeout = MROS_COMM_DEFAULT_TIMEOUT;
    client->connected = MROS_TRUE;

	return MROS_E_OK;
}
