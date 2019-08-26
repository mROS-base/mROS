#ifndef _MROS_PROTOCOL_SERVER_PROC_CIMPL_H_
#define _MROS_PROTOCOL_SERVER_PROC_CIMPL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mros_packet_cimpl.h"
#include "mros_comm_tcp_client_cimpl.h"
#include "mros_protocol_client_rpc_cimpl.h"
#include "mros_node_cimpl.h"
#include "mros_topic_connector_factory_cimpl.h"

extern mRosReturnType mros_proc_init(void);
extern mRosReturnType mros_proc_receive(mRosCommTcpClientType *client, mRosPacketType *packet);
extern mRosReturnType mros_proc_tcpros_receive(mRosCommTcpClientType *client, mRosPacketType *packet);
extern mRosReturnType mros_proc_slave(mRosCommTcpClientType *client, mRosPacketType *packet, mros_uint32 self_ipaddr);
extern mRosReturnType mros_proc_pub_tcpros(mRosCommTcpClientType *client, mRosPacketType *packet);
extern mRosReturnType mros_proc_request_outer_node_addition(mRosTopicIdType topic_id, mRosRequestTopicResType *rpc_response, void *api_reqp);
typedef struct {
	mros_uint32 ipaddr;
	mros_int32	port;
} mRosTopicOuterTcpConnectionType;
extern mRosNodeIdType mros_proc_connector_get_first(mRosTopicIdType topic_id, mRosTopicConnectorEnumType type, mRosNodeEnumType nodeType, mRosTopicOuterTcpConnectionType *tcp_conn);


#ifdef __cplusplus
}
#endif
#endif /* _MROS_PROTOCOL_SERVER_PROC_CIMPL_H_ */
