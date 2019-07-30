#ifndef _MROS_PROTOCOL_OPERATION_CIMPL_H_
#define _MROS_PROTOCOL_OPERATION_CIMPL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mros_memory.h"
#include "mros_comm_tcp_client_cimpl.h"

extern mRosReturnType mros_protocol_topic_data_send(mRosCommTcpClientType *client, const char *data, mRosSizeType datalen);
extern mRosReturnType mros_protocol_topic_data_receive(mRosCommTcpClientType *client, mRosMemoryManagerType *mempool, mRosMemoryListEntryType **retp);

extern mRosSizeType mros_protocol_get_buffersize(mRosSizeType body_size);
extern char* mros_protocol_get_body(char *buffer);

/*
 * reqp: mRosCommTcpClientListReqEntryType
 */
extern void mros_protocol_client_obj_free(void* reqp);


#ifdef __cplusplus
}
#endif
#endif /* _MROS_PROTOCOL_OPERATION_CIMPL_H_ */
