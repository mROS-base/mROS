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
#include "mros_comm_socket_cimpl.h"
#include "mros_comm_cimpl.h"

mRosReturnType mros_comm_socket_init(mRosCommSocketType *socket, mRosCommSocketEnumType type)
{
	switch (type) {
	case MROS_COMM_SOCKET_TYPE_TCP:
		socket->comm_type = MROS_SOCK_STREAM;
		break;
	case MROS_COMM_SOCKET_TYPE_UDP:
		socket->comm_type = MROS_SOCK_DGRAM;
		break;
	default:
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, MROS_E_INVAL);
		return MROS_E_INVAL;
	}
	socket->sock_fd = mros_comm_socket(MROS_SOCK_AF_INET, socket->comm_type, 0);
	if (socket->sock_fd < 0) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, MROS_E_INVAL);
		return MROS_E_INVAL;
	}

    socket->blocking = MROS_TRUE;
    socket->timeout = MROS_COMM_DEFAULT_TIMEOUT;
	return MROS_E_OK;
}

mRosReturnType mros_comm_socket_open(mRosCommSocketType *socket)
{
	socket->sock_fd = mros_comm_socket(MROS_SOCK_AF_INET, socket->comm_type, 0);
    if (socket->sock_fd < 0) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, MROS_E_INVAL);
    	return MROS_E_INVAL;
    }
	return MROS_E_OK;
}

void mros_comm_socket_close(mRosCommSocketType *socket)
{
	mros_comm_close(socket->sock_fd);
	socket->sock_fd = -1;
	return;
}

static mRosReturnType mros_comm_socket_select(mRosCommSocketType *socket, mros_uint32 timeout, mros_boolean read, mros_boolean write) {

	mRosFdSetType fd_set;
	mRosTimeValType tmo;

	MROS_FD_ZERO(&fd_set);
    MROS_FD_SET(socket->sock_fd, &fd_set);

    mRosFdSetType* r_set = MROS_NULL;
    mRosFdSetType* w_set = MROS_NULL;

    if (read == MROS_TRUE) {
    	r_set = &fd_set;
    }
    if (write == MROS_TRUE) {
    	w_set = &fd_set;
    }
    mros_comm_set_timeval(timeout, &tmo);
    mRosReturnType ret = mros_comm_select(MROS_FD_SETSIZE, r_set, w_set, MROS_NULL, &tmo);
    if (ret < 0) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
    	return MROS_E_SYSERR;
    }
    if ((ret == 0 || !MROS_FD_ISSET(socket->sock_fd, &fd_set))) {
    	return MROS_E_NOENT;
    }
    return MROS_E_OK;
}


mRosReturnType mros_comm_socket_set_blocking(mRosCommSocketType *socket, mros_boolean blocking, mros_uint32 timeout)
{
	socket->blocking = blocking;
	socket->timeout = timeout;
	return MROS_E_OK;
}


mRosReturnType mros_comm_socket_wait_readable(mRosCommSocketType *socket, mros_uint32 timeout)
{
	return mros_comm_socket_select(socket, timeout, MROS_TRUE, MROS_FALSE);
}

mRosReturnType mros_comm_socket_wait_writable(mRosCommSocketType *socket, mros_uint32 timeout)
{
	return mros_comm_socket_select(socket, timeout, MROS_FALSE, MROS_TRUE);
}
