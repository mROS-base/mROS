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
#include "mros_comm_tcp_client_factory_cimpl.h"
#include "mros_sys_config.h"

static mRosCommTcpClientEntryHeadType mros_comm_tcp_client_factory MROS_MATTR_BSS_NOCLR;
static mRosCommTcpClientListReqEntryType mros_comm_tcp_client_entries[MROS_TOPIC_TCP_CLIENT_MAX_NUM] MROS_MATTR_BSS_NOCLR;

mRosReturnType mros_comm_tcp_client_factory_init(void)
{

	List_Init(&mros_comm_tcp_client_factory, mRosCommTcpClientListReqEntryType, MROS_TOPIC_TCP_CLIENT_MAX_NUM, mros_comm_tcp_client_entries);
	return MROS_E_OK;
}

mRosCommTcpClientListReqEntryType *mros_comm_tcp_client_alloc(void)
{
	mRosCommTcpClientListReqEntryType *p;
	ListEntry_Alloc(&mros_comm_tcp_client_factory, mRosCommTcpClientListReqEntryType, &p);
	if (p == MROS_NULL) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, MROS_E_NOMEM);
		return MROS_NULL;
	}
	ListEntry_AddEntry(&mros_comm_tcp_client_factory, p);
	return p;
}
mRosCommTcpClientListReqEntryType *mros_comm_tcp_client_alloc_copy(mRosCommTcpClientType *client)
{
	mRosCommTcpClientListReqEntryType *p;

	p = mros_comm_tcp_client_alloc();
	if (p == MROS_NULL) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, MROS_E_NOMEM);
		return MROS_NULL;
	}
	p->data.client = *client;

	return p;
}
void mros_comm_tcp_client_free(mRosCommTcpClientListReqEntryType *client)
{
	ListEntry_RemoveEntry(&mros_comm_tcp_client_factory, client);
	ListEntry_Free(&mros_comm_tcp_client_factory, client);
	return;
}
