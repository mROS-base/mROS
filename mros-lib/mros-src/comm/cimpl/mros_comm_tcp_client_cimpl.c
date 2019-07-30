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
#include "mros_comm_tcp_client_cimpl.h"

mRosReturnType mros_comm_tcp_client_init(mRosCommTcpClientType *client, const char* host, mros_int32 port)
{
	client->connected = MROS_FALSE;
	mros_comm_inet_remote_sockaddr_init(&client->remote, port, host);
	return mros_comm_socket_init(&client->socket, MROS_COMM_SOCKET_TYPE_TCP);
}
mRosReturnType mros_comm_tcp_client_ip32_init(mRosCommTcpClientType *client, mros_uint32 ipaddr, mros_int32 port)
{
	client->connected = MROS_FALSE;
	mros_comm_inet_remote_sockaddr_ip32_init(&client->remote, port, ipaddr);
	return mros_comm_socket_init(&client->socket, MROS_COMM_SOCKET_TYPE_TCP);
}

void mros_comm_tcp_client_close(mRosCommTcpClientType *client)
{
	client->connected = MROS_FALSE;
	mros_comm_socket_close(&client->socket);
	return;
}

mRosReturnType mros_comm_tcp_client_connect(mRosCommTcpClientType *client)
{
	mRosReturnType ret;

	if (client->connected == MROS_TRUE) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, MROS_E_INVAL);
		return MROS_E_INVAL;
	}
	if (client->socket.sock_fd < 0) {
		ret = mros_comm_socket_init(&client->socket, MROS_COMM_SOCKET_TYPE_TCP);
		if (ret != MROS_E_OK) {
			ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
			return ret;
		}
	}

	ret = mros_comm_connect(client->socket.sock_fd, (const mRosSockAddrType *)&client->remote, sizeof(mRosSockAddrInType));
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return MROS_E_NOTCONN;
	}
	client->connected = MROS_TRUE;
	return MROS_E_OK;
}



mros_boolean mros_comm_tcp_client_is_connected(mRosCommTcpClientType *client)
{
	return client->connected;
}
mRosReturnType mros_comm_tcp_client_send(mRosCommTcpClientType *client, const char* data, mRosSizeType length, mRosSizeType *res)
{
	mRosReturnType ret;
	mros_int32 snd_size;

    if ((client->socket.sock_fd < 0) || (client->connected == MROS_FALSE)) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, MROS_E_INVAL);
        return MROS_E_INVAL;
    }

    if (client->socket.blocking == MROS_FALSE) {
    	ret = mros_comm_socket_wait_writable(&client->socket, client->socket.timeout);
    	if (ret != MROS_E_OK) {
    		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
    		return ret;
    	}
    }
    snd_size = mros_comm_send(client->socket.sock_fd, data, length, 0);
    if (snd_size == 0) {
		client->connected = MROS_FALSE;
    }
    *res = snd_size;
	return MROS_E_OK;
}
mRosReturnType mros_comm_tcp_client_send_all(mRosCommTcpClientType *client, const char* data, mRosSizeType length, mRosSizeType *res)
{
	mRosReturnType ret;
    mros_int32 writtenLen = 0;

    if ((client->socket.sock_fd < 0) || (client->connected == MROS_FALSE)) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, MROS_E_INVAL);
        return MROS_E_INVAL;
    }

    while (writtenLen < length) {
        if (client->socket.blocking == MROS_FALSE) {
        	ret = mros_comm_socket_wait_writable(&client->socket, client->socket.timeout);
        	if (ret != MROS_E_OK) {
        		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
        		return ret;
        	}
        }

        mros_int32 retlen = mros_comm_send(client->socket.sock_fd, data + writtenLen, length - writtenLen, 0);
        if (retlen > 0) {
            writtenLen += retlen;
            continue;
        } else if (retlen == 0) {
    		client->connected = MROS_FALSE;
            *res = writtenLen;
    		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, MROS_E_SYSERR);
            return MROS_E_SYSERR;
        } else {
            *res = writtenLen;
    		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, MROS_E_SYSERR);
            return MROS_E_SYSERR;
        }
    }
    *res = writtenLen;
	return MROS_E_OK;
}
mRosReturnType mros_comm_tcp_client_receive(mRosCommTcpClientType *client, char* data, mRosSizeType length, mRosSizeType *res)
{
	mRosReturnType ret;
	mros_int32 rcv_size;

    if ((client->socket.sock_fd < 0) || (client->connected == MROS_FALSE)) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, MROS_E_INVAL);
        return MROS_E_INVAL;
    }
    if (client->socket.blocking == MROS_FALSE) {
    	ret = mros_comm_socket_wait_readable(&client->socket, client->socket.timeout);
    	if (ret != MROS_E_OK) {
    		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
    		return ret;
    	}
    }
    rcv_size = mros_comm_recv(client->socket.sock_fd, data, length, 0);
    if (rcv_size == 0) {
		client->connected = MROS_FALSE;
    }
    *res = rcv_size;

	return MROS_E_OK;
}

mRosReturnType mros_comm_tcp_client_receive_all(mRosCommTcpClientType *client, char* data, mRosSizeType length, mRosSizeType *res)
{
	mRosReturnType ret;
    mros_int32 readLen = 0;

    if ((client->socket.sock_fd < 0) || (client->connected == MROS_FALSE)) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, MROS_E_INVAL);
        return MROS_E_INVAL;
    }

    while (readLen < length) {
        if (client->socket.blocking == MROS_FALSE) {
        	ret = mros_comm_socket_wait_writable(&client->socket, client->socket.timeout);
        	if (ret != MROS_E_OK) {
        		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
        		return ret;
        	}
        }
        mros_int32 retlen = mros_comm_recv(client->socket.sock_fd, data + readLen, length - readLen, 0);
        if (retlen > 0) {
        	readLen += retlen;
            continue;
        } else if (retlen == 0) {
    		client->connected = MROS_FALSE;
            *res = readLen;
    		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, MROS_E_SYSERR);
            return MROS_E_SYSERR;
        } else {
            *res = readLen;
    		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, MROS_E_SYSERR);
            return MROS_E_SYSERR;
        }
    }
    *res = readLen;
	return MROS_E_OK;
}
