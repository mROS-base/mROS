#include "mros_protocol_subscribe_cimpl.h"
#include "mros_protocol_client_rpc_cimpl.h"
#include "mros_protocol_server_proc_cimpl.h"
#include "mros_topic_cimpl.h"
#include "mros_topic_connector_factory_cimpl.h"
#include "mros_topic_runner_cimpl.h"
#include "mros_node_cimpl.h"
#include "mros_comm_tcp_client_factory_cimpl.h"
#include "mros_packet_config.h"
#include "mros_exclusive_area.h"
#include "mros_wait_queue.h"
#include "mros_sys_config.h"

typedef union {
	char buffer;
	char buffer1[MROS_PACKET_MAXSIZE_REQ_TCPROS];
	char buffer2[MROS_PACKET_MAXSIZE_RES_TCPROS];
} mRosSubscribePacketTcpRosBufferType;


typedef struct {
	mRosProtocolSubscribeStateEnumType 	state;
	mRosPacketType						tcpros_packet;
	mRosTopicConnectorManagerType 		*pub_mgrp;// for outer pub
} mRosProtocolSubscribeType;

static mRosProtocolSubscribeType mros_protocol_subscribe MROS_MATTR_BSS_NOCLR;
static mRosSubscribePacketTcpRosBufferType mros_subscribe_packet_tcpros_buffer MROS_MATTR_BSS_NOCLR;

mRosReturnType mros_protocol_subscribe_init(void)
{
	mros_protocol_subscribe.tcpros_packet.total_size = sizeof(mRosSubscribePacketTcpRosBufferType);
	mros_protocol_subscribe.tcpros_packet.data = &mros_subscribe_packet_tcpros_buffer.buffer;
	mros_protocol_subscribe.state = MROS_PROTOCOL_SUBSCRIBE_STATE_WAITING;
	mros_protocol_subscribe.pub_mgrp = mros_topic_connector_factory_get(MROS_TOPIC_CONNECTOR_PUB);
	return MROS_E_OK;
}

void mros_protocol_subscribe_run(void)
{
	mRosReturnType ret;
	mRosTopicConnectorType connector;
	mRosCommTcpClientListReqEntryType *client_req;
	mRosRcpRosReqType req;
	mRosTcpRosResType res;
	mRosContainerObjType cobj;
	mROsExclusiveUnlockObjType unlck_obj;
	mRosTopicConnectorType sub_connector;

	req.req_packet = &mros_protocol_subscribe.tcpros_packet;
	res.reply_packet = &mros_protocol_subscribe.tcpros_packet;

	mros_exclusive_lock(&mros_exclusive_area, &unlck_obj);
	while (MROS_TRUE) {
		mros_protocol_subscribe.state = MROS_PROTOCOL_SUBSCRIBE_STATE_WAITING;
		mRosWaitListEntryType *wait_entry = mros_server_queue_wait(&mros_subscribe_wait_queue);
		if (wait_entry == MROS_NULL) {
			mros_topic_data_publisher_run();
			mros_topic_data_subscriber_run();
			continue;
		}
		mros_protocol_subscribe.state = MROS_PROTOCOL_SUBSCRIBE_STATE_PUB_CONNECTING;
		client_req = (mRosCommTcpClientListReqEntryType*)wait_entry->data.reqp;

		ret = mros_comm_tcp_client_ip32_init(&client_req->data.client, client_req->data.reqobj.ipaddr, client_req->data.reqobj.port);
		if (ret != MROS_E_OK) {
			ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
			mros_comm_tcp_client_free(client_req);
			continue;
		}
		ret = mros_comm_tcp_client_connect(&client_req->data.client);
		if (ret != MROS_E_OK) {
			ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
			mros_comm_tcp_client_free(client_req);
			continue;
		}
		ROS_INFO("INFO: sub connected");

		connector.topic_id = client_req->data.reqobj.topic_id;
		connector.func_id = (mRosFuncIdType)MROS_ID_NONE;
		ret = mros_node_create_outer(&connector.node_id);
		if (ret != MROS_E_OK) {
			ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
			mros_comm_tcp_client_free(client_req);
			continue;
		}
		ret = mros_topic_connector_add(mros_protocol_subscribe.pub_mgrp, &connector, MROS_OUTER_CONNECTOR_QUEUE_MAXLEN, &ros_outer_topic_publisher_mempool);
		if (ret != MROS_E_OK) {
			ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
			(void)mros_node_remove(connector.node_id);
			mros_comm_tcp_client_free(client_req);
			continue;
		}

		mros_protocol_subscribe.state = MROS_PROTOCOL_SUBSCRIBE_STATE_PUB_REQUESTING;
		sub_connector.node_id = mros_proc_connector_get_first(connector.topic_id, MROS_TOPIC_CONNECTOR_SUB, MROS_NODE_TYPE_INNER, MROS_NULL);
		if (sub_connector.node_id == MROS_ID_NONE) {
			ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
			(void)mros_node_remove(connector.node_id);
			mros_comm_tcp_client_free(client_req);
			continue;
		}

		req.node_name = mros_node_name(sub_connector.node_id);
		req.topic_name = mros_topic_get_topic_name(connector.topic_id);
		req.topic_typename = mros_topic_get_topic_typename(connector.topic_id);
		(void)mros_topic_get_md5sum(connector.topic_id, &req.md5sum);
		ret = mros_rpc_tcpros(&client_req->data.client, &req, &res);
		if (ret != MROS_E_OK) {
			ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
			mros_topic_connector_remove(mros_protocol_subscribe.pub_mgrp, &connector);
			(void)mros_node_remove(connector.node_id);
			mros_comm_tcp_client_free(client_req);
			continue;
		}

		cobj = mros_topic_connector_get_obj(mros_protocol_subscribe.pub_mgrp, &connector);
		(void)mros_topic_connector_set_connection(cobj, client_req);
		//wakeup api requester
		if (client_req->data.reqobj.api_reqp != MROS_NULL) {
			mros_client_wakeup((mRosWaitListEntryType*)client_req->data.reqobj.api_reqp);
			client_req->data.reqobj.api_reqp = NULL;
		}
	}
	mros_exclusive_unlock(&unlck_obj);
	return;
}
