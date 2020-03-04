#include "mros_protocol_master_cimpl.h"
#include "mros_protocol_client_rpc_cimpl.h"
#include "mros_protocol_server_proc_cimpl.h"
#include "mros_topic_cimpl.h"
#include "mros_topic_connector_factory_cimpl.h"
#include "mros_protocol_operation_cimpl.h"
#include "mros_packet_decoder_cimpl.h"
#include "mros_packet_config.h"
#include "mros_exclusive_area.h"
#include "mros_wait_queue.h"
#include "mros_sys_config.h"

typedef union {
	char buffer;
	char buffer1[MROS_PACKET_MAXSIZE_REQ_REGISTER_PUBLISHER];
	char buffer2[MROS_PACKET_MAXSIZE_RES_REGISTER_PUBLISHER];
	char buffer3[MROS_PACKET_MAXSIZE_REQ_REGISTER_SUBSCRIBER];
	char buffer4[MROS_PACKET_MAXSIZE_RES_REGISTER_SUBSCRIBER];
	char buffer5[MROS_PACKET_MAXSIZE_REQ_REQUEST_TOPIC];
	char buffer6[MROS_PACKET_MAXSIZE_RES_REQUEST_TOPIC];
} mRosMasterPacketBufferType;

static mRosMasterPacketBufferType mros_master_packet_buffer;

typedef struct {
	mRosProtocolMasterStateEnumType 	state;
	mRosPacketType						register_packet;
	mRosPacketType						reqtopic_packet;
	mRosCommTcpClientType				master_comm;
	mRosWaitListEntryType				*api_reqp;
	mros_uint32							self_ipaddr;
} mRosProtocolMasterType;

static mRosProtocolMasterType mros_protocol_master MROS_MATTR_BSS_NOCLR;
static mRosReturnType mros_protocol_master_register_publisher(mRosProtocolMasterRequestType *pub_req);
static mRosReturnType mros_protocol_master_register_subscriber(mRosProtocolMasterRequestType *sub_req);

mRosReturnType mros_protocol_master_init(void)
{
	mRosReturnType ret = mros_comm_tcp_client_init(&mros_protocol_master.master_comm, MROS_MASTER_IPADDR, MROS_MASTER_PORT_NO);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}
	mros_protocol_master.register_packet.total_size = sizeof(mros_master_packet_buffer);
	mros_protocol_master.register_packet.data = &mros_master_packet_buffer.buffer;
	mros_protocol_master.reqtopic_packet.total_size = sizeof(mros_master_packet_buffer);
	mros_protocol_master.reqtopic_packet.data = &mros_master_packet_buffer.buffer;
	mros_protocol_master.state = MROS_PROTOCOL_MASTER_STATE_WAITING;
	mros_protocol_master.api_reqp = NULL;
	ret = mros_comm_inet_get_ipaddr((const char *)mros_comm_config.mros_node_ipaddr, &mros_protocol_master.self_ipaddr);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}
	return MROS_E_OK;
}

void mros_protocol_master_run(void)
{
	mROsExclusiveUnlockObjType unlck_obj;
	mros_exclusive_lock(&mros_exclusive_area, &unlck_obj);
	while (MROS_TRUE) {
		mros_protocol_master.api_reqp = NULL;
		mRosWaitListEntryType *wait_entry = mros_server_queue_wait(&mros_master_wait_queue);
		if (wait_entry == MROS_NULL) {
			continue;
		}
		mros_protocol_master.api_reqp = wait_entry;
		mRosProtocolMasterRequestType *req = (mRosProtocolMasterRequestType*)wait_entry->data.reqp;
		switch (req->req_type) {
		case MROS_PROTOCOL_MASTER_REQ_REGISTER_PUBLISHER:
			mros_protocol_master_register_publisher(req);
			mros_client_wakeup(mros_protocol_master.api_reqp);
			break;
		case MROS_PROTOCOL_MASTER_REQ_REGISTER_SUBSCRIBER:
			mros_protocol_master_register_subscriber(req);
			//register subscriber api_reqp wakeup is done on subscriber_run.
			break;
		default:
			mros_client_wakeup(mros_protocol_master.api_reqp);
			break;
		}
	}
	mros_exclusive_unlock(&unlck_obj);
	return;
}

static mRosReturnType mros_protocol_master_register(mRosProtocolMasterRequestType *req, mRosTopicConnectorEnumType type, mRosRegisterTopicResType *rpc_response)
{
	mRosReturnType ret;
	mRosTopicConnectorType connector;
	mRosRegisterTopicReqType rpc_request;

	ret = mros_topic_connector_get(req->connector_obj,  &connector);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, MROS_E_INVAL);
		return MROS_E_INVAL;
	}

	rpc_response->reply_packet =  &mros_protocol_master.register_packet;

	rpc_request.node_name = mros_node_name(connector.node_id);
	rpc_request.req_packet = &mros_protocol_master.register_packet;
	rpc_request.topic_name = mros_topic_get_topic_name(connector.topic_id);
	rpc_request.topic_typename = mros_topic_get_topic_typename(connector.topic_id);
	if (type == MROS_TOPIC_CONNECTOR_PUB) {
		ret = mros_rpc_register_publisher(&mros_protocol_master.master_comm, &rpc_request, rpc_response);
	}
	else {
		ret = mros_rpc_register_subscriber(&mros_protocol_master.master_comm, &rpc_request, rpc_response);
	}
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}
	return ret;
}

static mRosReturnType mros_protocol_master_request_topic(mRosCommTcpClientType *client, mRosProtocolMasterRequestType *req, mRosRequestTopicResType *rpc_response)
{
	mRosReturnType ret;
	mRosTopicConnectorType connector;
	mRosRequestTopicReqType rpc_request;

	ret = mros_topic_connector_get(req->connector_obj,  &connector);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, MROS_E_INVAL);
		return MROS_E_INVAL;
	}

	rpc_response->reply_packet =  &mros_protocol_master.reqtopic_packet;

	rpc_request.node_name = mros_node_name(connector.node_id);
	rpc_request.req_packet = &mros_protocol_master.reqtopic_packet;
	rpc_request.topic_name = mros_topic_get_topic_name(connector.topic_id);
	ret = mros_rpc_request_topic(client, &rpc_request, rpc_response);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}
	ret = mros_xmlpacket_reqtopicres_result(rpc_response->reply_packet);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}

	return mros_proc_request_outer_node_addition(connector.topic_id, rpc_response, mros_protocol_master.api_reqp);
}

static mRosReturnType mros_protocol_master_register_publisher(mRosProtocolMasterRequestType *pub_req)
{
	mRosReturnType ret;
	mRosRegisterTopicResType rpc_response;

	mros_protocol_master.state = MROS_PROTOCOL_MASTER_STATE_REGISTER_PUBLISHER;

	ret = mros_comm_tcp_client_connect(&mros_protocol_master.master_comm);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		ROS_ERROR("ERROR: Unable to communicate with master!");
		goto done;
	}
	ROS_INFO("INFO: master pub: connected");
	ret = mros_protocol_master_register(pub_req, MROS_TOPIC_CONNECTOR_PUB, &rpc_response);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		goto done;
	}
	ret = mros_xmlpacket_pubres_result(rpc_response.reply_packet);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		goto done;
	}

done:
	mros_comm_tcp_client_close(&mros_protocol_master.master_comm);
	mros_protocol_master.state = MROS_PROTOCOL_MASTER_STATE_WAITING;

	return ret;
}


static mRosReturnType mros_protocol_master_register_subscriber(mRosProtocolMasterRequestType *sub_req)
{
	mRosReturnType ret;
	mRosRegisterTopicResType rpc_regc_res;
	mRosRequestTopicResType rpc_topic_res;
	mros_uint32 ipaddr;
	mros_int32 port;
	mRosPtrType ptr;
	mros_boolean is_outer_node = MROS_FALSE;

	mros_protocol_master.state = MROS_PROTOCOL_MASTER_STATE_REGISTER_SUBSCRIBER;

	ret = mros_comm_tcp_client_connect(&mros_protocol_master.master_comm);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		ROS_ERROR("ERROR: Unable to communicate with master!");
		goto done;
	}
	ROS_INFO("INFO: master sub: connected");

	ret = mros_protocol_master_register(sub_req, MROS_TOPIC_CONNECTOR_SUB, &rpc_regc_res);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		goto done;
	}
	mros_comm_tcp_client_close(&mros_protocol_master.master_comm);
	ret = mros_xmlpacket_subres_result(rpc_regc_res.reply_packet);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		goto done;
	}

	mros_protocol_master.state = MROS_PROTOCOL_MASTER_STATE_REQUESTING_TOPIC;
	//TODO まだ出版ノードが存在しない場合は，非同期でマスタから情報をもらう
	ptr = mros_xmlpacket_subres_get_first_uri(rpc_regc_res.reply_packet, &ipaddr, &port);
	if (ptr == MROS_NULL) {
		mRosTopicConnectorType connector;
		(void)mros_topic_connector_get(sub_req->connector_obj,  &connector);
		ROS_WARN("WARNING: topic [%s] does not appear to be published yet", mros_topic_get_topic_name(connector.topic_id));
	}
	while (ptr != MROS_NULL) {
		mRosCommTcpClientType client;
		if (ipaddr == mros_protocol_master.self_ipaddr) {
			ptr = mros_xmlpacket_subres_get_next_uri(ptr, rpc_regc_res.reply_packet, &ipaddr, &port);
			continue;
		}

		ret = mros_comm_tcp_client_ip32_init(&client, ipaddr, port);
		if (ret != MROS_E_OK) {
			ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
			goto done;
		}
		ret = mros_comm_tcp_client_connect(&client);
		if (ret != MROS_E_OK) {
			ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
			goto done;
		}
		ret = mros_protocol_master_request_topic(&client, sub_req, &rpc_topic_res);
		mros_comm_tcp_client_close(&client);
		if (ret != MROS_E_OK) {
			ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
			goto done;
		}
		is_outer_node = MROS_TRUE;
		ptr = mros_xmlpacket_subres_get_next_uri(ptr, rpc_regc_res.reply_packet, &ipaddr, &port);
	}

done:
	if (is_outer_node == MROS_FALSE) {
		mros_client_wakeup(mros_protocol_master.api_reqp);
	}
	mros_comm_tcp_client_close(&mros_protocol_master.master_comm);
	mros_protocol_master.state = MROS_PROTOCOL_MASTER_STATE_WAITING;

	return ret;
}
