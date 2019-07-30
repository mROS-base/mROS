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
#ifndef _MROS_COMM_TCP_CLIENT_FACTORY_CIMPL_H_
#define _MROS_COMM_TCP_CLIENT_FACTORY_CIMPL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mros_comm_tcp_client_cimpl.h"
#include "mros_memory.h"
#include "mros_wait_queue.h"

typedef struct {
	mRosTopicIdType 		topic_id;
	mros_uint32				ipaddr;
	mros_int32				port;
	void*					api_reqp;
	mRosWaitListEntryType	waitobj;
} mRosRquestObjectType;

typedef struct {
	mRosReturnType (*topic_data_send) (mRosCommTcpClientType *client, const char *data, mRosSizeType datalen);
	mRosReturnType (*topic_data_receive) (mRosCommTcpClientType *client, mRosMemoryManagerType *mempool, mRosMemoryListEntryType **retp);
	void (*free) (void* reqp);
} mRosCommOperationType;

typedef struct {
	mRosCommTcpClientType 	client;
	mRosCommOperationType	op;
	mRosRquestObjectType	reqobj;
} mRosCommTcpClientReqEntryType;
typedef ListEntryType(mRosCommTcpClientListReqEntryType, mRosCommTcpClientReqEntryType) mRosCommTcpClientListReqEntryType;
typedef ListHeadType(mRosCommTcpClientListReqEntryType) mRosCommTcpClientEntryHeadType;

extern mRosReturnType mros_comm_tcp_client_factory_init(void);
extern mRosCommTcpClientListReqEntryType *mros_comm_tcp_client_alloc(void);
extern mRosCommTcpClientListReqEntryType *mros_comm_tcp_client_alloc_copy(mRosCommTcpClientType *client);
extern void mros_comm_tcp_client_free(mRosCommTcpClientListReqEntryType *client);


#ifdef __cplusplus
}
#endif

#endif /* _MROS_COMM_TCP_CLIENT_FACTORY_CIMPL_H_ */
