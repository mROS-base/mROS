#include "mros_protocol_server_proc_cimpl.h"
#include "mros_protocol_operation_cimpl.h"
#include "mros_comm_tcp_client_cimpl.h"
#include "mros_topic_cimpl.h"
#include "mros_topic_connector_factory_cimpl.h"
#include "mros_packet_encoder_cimpl.h"
#include "mros_packet_decoder_cimpl.h"
#include "mros_exclusive_area.h"
#include "mros_wait_queue.h"
#include "mros_sys_config.h"
#include <string.h>

static mRosPacketDecodedRequestType mros_proc_slave_decoded_requst MROS_MATTR_BSS_NOCLR;

mRosReturnType mros_proc_init(void)
{
	return MROS_E_OK;
}

mRosReturnType mros_proc_receive(mRosCommTcpClientType *client, mRosPacketType *packet)
{
	mRosReturnType ret;
	mRosSizeType res;
	mros_boolean is_end;

	packet->data_size = 0;
	do {
		ret = mros_comm_tcp_client_receive(client, &packet->data[packet->data_size], (packet->total_size - packet->data_size),  &res);
		if (ret != MROS_E_OK) {
			ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
			break;
		}
		packet->data_size += res;
		if ((packet->data_size + 1) >= packet->total_size) {
			ret = MROS_E_NOMEM;
			ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
			break;
		}
		is_end = mros_xmlpacket_has_request_end(packet);
	} while (is_end == MROS_FALSE);
	packet->data[packet->data_size] = '\0';
	return ret;
}
mRosReturnType mros_proc_tcpros_receive(mRosCommTcpClientType *client, mRosPacketType *packet)
{
	mRosReturnType ret;
	mRosSizeType res;
	mRosPacketType header_packet;
	mros_int8 rawdata[MROS_TCPROS_RAWDATA_HEADER_SIZE];

	ret = mros_comm_tcp_client_receive_all(client, rawdata, MROS_TCPROS_RAWDATA_HEADER_SIZE, &res);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}

	header_packet.total_size = MROS_TCPROS_RAWDATA_HEADER_SIZE;
	header_packet.data_size = MROS_TCPROS_RAWDATA_HEADER_SIZE;
	header_packet.data = rawdata;
	ret = mros_tcprospacket_get_body_size(&header_packet, &res);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}
	packet->data_size = 0;
	packet->total_size = res;
	ret = mros_comm_tcp_client_receive_all(client, packet->data, packet->total_size, &res);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}
	packet->data_size = res;
	return ret;
}

mRosNodeIdType mros_proc_connector_get_first(mRosTopicIdType topic_id, mRosTopicConnectorEnumType type, mRosNodeEnumType nodeType, mRosTopicOuterTcpConnectionType *tcp_conn)
{
	mRosContainerObjType obj;
	mRosContainerObjType topic_obj;
	mRosTopicConnectorManagerType *mgrp;
	mRosCommTcpClientListReqEntryType *connection;
	mRosTopicConnectorType connector;

	mgrp = mros_topic_connector_factory_get(type);
	if (mgrp == MROS_NULL) {
		return MROS_ID_NONE;
	}
	topic_obj = mros_topic_connector_get_topic_obj(mgrp, topic_id);
	if (topic_obj == MROS_COBJ_NULL) {
		return MROS_ID_NONE;
	}
	obj = mros_topic_connector_get_first(mgrp, nodeType, topic_obj);
	while (obj != MROS_COBJ_NULL) {
		(void)mros_topic_connector_get(obj,  &connector);
		if (tcp_conn != MROS_NULL) {
			 (void)mros_topic_connector_get_connection(obj, &connection);
			 if (connection != MROS_NULL) {
				 if ( (connection->data.client.remote.sin_port == tcp_conn->port) &&
						 (connection->data.client.remote.sin_addr.s_addr == tcp_conn->ipaddr) ) {
					return connector.node_id;
				 }
			 }
		}
		else {
			return connector.node_id;
		}
		obj = mros_topic_connector_get_next(mgrp, topic_obj, obj);
	}
	return MROS_ID_NONE;
}

static mRosReturnType mros_proc_slave_request_topic(mRosCommTcpClientType *client, mRosPacketType *packet)
{
	mRosReturnType ret;
	mRosNodeIdType node_id;
	mRosTopicIdType topic_id;
	mRosSizeType res;

	if (mros_proc_slave_decoded_requst.request.topic.topic_name.res.len >= MROS_TOPIC_NAME_MAXLEN) {
		return MROS_E_INVAL;
	}
	mros_proc_slave_decoded_requst.request.topic.topic_name.res.head[mros_proc_slave_decoded_requst.request.topic.topic_name.res.len] = '\0';
	ret = mros_topic_get((const char*)&mros_proc_slave_decoded_requst.request.topic.topic_name.res.head[0], &topic_id);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}
	node_id = mros_proc_connector_get_first(topic_id, MROS_TOPIC_CONNECTOR_PUB, MROS_NODE_TYPE_INNER, NULL);
	if (node_id == MROS_ID_NONE) {
		//TODO error reply...
		// original code does not support this case...
	}
	else {
		mRosEncodeArgType arg;
		arg.type = MROS_PACKET_DATA_REQUEST_TOPIC_RES;
		arg.args_int = 1;
		arg.argi[0] = MROS_PUBLISHER_PORT_NO;
		arg.args_char = 2;
		arg.argv[0] ="TCPROS";
		arg.argv[1] = MROS_NODE_IPADDR;
		ret = mros_packet_encode(&arg, packet);
		if (ret != MROS_E_OK) {
			ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
			return ret;
		}
		ret = mros_comm_tcp_client_send_all(client, packet->data, packet->data_size, &res);
	}

	return ret;
}

mRosReturnType mros_proc_request_outer_node_addition(mRosTopicIdType topic_id, mRosRequestTopicResType *rpc_response, void *api_reqp)
{
	mRosReturnType ret = MROS_E_OK;
	mRosPtrType ptr;
	mRosTopicOuterTcpConnectionType tcp_conn;

	ptr = mros_xmlpacket_reqtopicres_get_first_uri(rpc_response->reply_packet, &tcp_conn.ipaddr, &tcp_conn.port);
	while (ptr != MROS_NULL) {
		mRosCommTcpClientListReqEntryType *req = mros_comm_tcp_client_alloc();
		if (req == MROS_NULL) {
			ret = MROS_E_NOMEM;
			ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
			goto done;
		}
		if (mros_proc_connector_get_first(topic_id, MROS_TOPIC_CONNECTOR_PUB, MROS_NODE_TYPE_OUTER, &tcp_conn) != MROS_ID_NONE) {
			mros_comm_tcp_client_free(req);
			ptr = mros_xmlpacket_reqtopicres_get_next_uri(ptr, rpc_response->reply_packet, &tcp_conn.ipaddr, &tcp_conn.port);
			continue;
		}
		req->data.reqobj.ipaddr = tcp_conn.ipaddr;
		req->data.reqobj.port = tcp_conn.port;
		req->data.reqobj.topic_id = topic_id;
		req->data.op.free = mros_protocol_client_obj_free;
		req->data.op.topic_data_receive = mros_protocol_topic_data_receive;
		req->data.op.topic_data_send = mros_protocol_topic_data_send;
		mros_client_wait_entry_init(&req->data.reqobj.waitobj, req);
		req->data.reqobj.api_reqp = api_reqp;

		mros_client_put_request(&mros_subscribe_wait_queue, &req->data.reqobj.waitobj);

		ptr = mros_xmlpacket_reqtopicres_get_next_uri(ptr, rpc_response->reply_packet, &tcp_conn.ipaddr, &tcp_conn.port);
	}

done:
	return ret;
}

static mRosReturnType mros_proc_slave_publisher_update_do_request_topic(mRosTopicIdType topic_id, mRosTopicOuterTcpConnectionType *tcp_conn, mRosPacketType *packet)
{
	mRosReturnType ret;
	mRosRequestTopicReqType rpc_request;
	mRosRequestTopicResType rpc_response;
	mRosCommTcpClientType slave_client;
	mRosTopicConnectorType sub_connector;

	sub_connector.topic_id = topic_id;
	sub_connector.node_id = mros_proc_connector_get_first(topic_id, MROS_TOPIC_CONNECTOR_SUB, MROS_NODE_TYPE_INNER, MROS_NULL);
	if (sub_connector.node_id == MROS_ID_NONE) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, MROS_E_NOENT);
		return MROS_E_NOENT;
	}

	rpc_request.req_packet = packet;
	rpc_response.reply_packet =  packet;

	rpc_request.node_name = mros_node_name(sub_connector.node_id);
	rpc_request.topic_name = mros_topic_get_topic_name(topic_id);

	ret = mros_comm_tcp_client_ip32_init(&slave_client, tcp_conn->ipaddr, tcp_conn->port);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}
	ret = mros_comm_tcp_client_connect(&slave_client);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}
	ret = mros_rpc_request_topic(&slave_client, &rpc_request, &rpc_response);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}
	ret = mros_xmlpacket_reqtopicres_result(rpc_response.reply_packet);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}
	mros_comm_tcp_client_close(&slave_client);
	ret = mros_proc_request_outer_node_addition(topic_id, &rpc_response, MROS_NULL);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}
	return MROS_E_OK;
}

static mRosReturnType mros_proc_slave_publisher_update(mRosCommTcpClientType *client, mRosPacketType *packet, mros_uint32 self_ipaddr)
{
	mRosReturnType ret;
	mRosTopicIdType topic_id;
	mRosPtrType ptr;
	mRosTopicOuterTcpConnectionType tcp_conn;

	if (mros_proc_slave_decoded_requst.request.topic.topic_name.res.len >= MROS_TOPIC_NAME_MAXLEN) {
		return MROS_E_INVAL;
	}
	mros_proc_slave_decoded_requst.request.topic.topic_name.res.head[mros_proc_slave_decoded_requst.request.topic.topic_name.res.len] = '\0';
	ret = mros_topic_get((const char*)&mros_proc_slave_decoded_requst.request.topic.topic_name.res.head[0], &topic_id);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}

	ret = MROS_E_OK;
	ptr = mros_xmlpacket_pubupreq_get_first_uri(&mros_proc_slave_decoded_requst.request.publisher_update.topic_name.res.tail[1], &tcp_conn.ipaddr, &tcp_conn.port);
	while (ptr != MROS_NULL) {
		if (tcp_conn.ipaddr == self_ipaddr) {
			ptr = mros_xmlpacket_pubupreq_get_next_uri(ptr, packet, &tcp_conn.ipaddr, &tcp_conn.port);
			continue;
		}
		if (mros_proc_connector_get_first(topic_id, MROS_TOPIC_CONNECTOR_PUB, MROS_NODE_TYPE_OUTER, &tcp_conn) != MROS_ID_NONE) {
			ptr = mros_xmlpacket_pubupreq_get_next_uri(ptr, packet, &tcp_conn.ipaddr, &tcp_conn.port);
			continue;
		}
		ret = mros_proc_slave_publisher_update_do_request_topic(topic_id, &tcp_conn, packet);
		if (ret != MROS_E_OK) {
			ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
			return ret;
		}
		ptr = mros_xmlpacket_pubupreq_get_next_uri(ptr, packet, &tcp_conn.ipaddr, &tcp_conn.port);
	}

	return ret;
}

mRosReturnType mros_proc_slave(mRosCommTcpClientType *client, mRosPacketType *packet, mros_uint32 self_ipaddr)
{
	mRosReturnType ret = MROS_E_INVAL;

	mRosPacketDataEnumType type = mros_xmlpacket_slave_request_decode(packet, &mros_proc_slave_decoded_requst);
	switch (type) {
	case MROS_PACKET_DATA_REQUEST_TOPIC_REQ:
		ret = mros_proc_slave_request_topic(client, packet);
		break;
	case MROS_PACKET_DATA_PUBLISHER_UPDATE_REQ:
		ret = mros_proc_slave_publisher_update(client, packet, self_ipaddr);
		break;
	default:
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		break;
	}

	return ret;
}

static mRosReturnType mros_proc_add_outersub_connector(mRosCommTcpClientType *client, mRosTopicIdType topic_id)
{
	mRosReturnType ret = MROS_E_INVAL;
	mRosTopicConnectorType connector;
	mRosContainerObjType cobj;
	mRosTopicConnectorManagerType *sub_mgrp = mros_topic_connector_factory_get(MROS_TOPIC_CONNECTOR_SUB);
	if (sub_mgrp == MROS_NULL) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}
	connector.topic_id = topic_id;
	connector.func_id = (mRosFuncIdType)MROS_ID_NONE;
	ret = mros_node_create_outer(&connector.node_id);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return MROS_E_NOENT;
	}
	ret = mros_topic_connector_add(sub_mgrp, &connector, MROS_OUTER_CONNECTOR_QUEUE_MAXLEN, MROS_NULL);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}
	cobj = mros_topic_connector_get_obj(sub_mgrp, &connector);

	mRosCommTcpClientListReqEntryType *client_entry = mros_comm_tcp_client_alloc_copy(client);
	if (client_entry == MROS_NULL) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, MROS_E_NOENT);
		return MROS_E_NOENT;
	}
	client_entry->data.op.free = mros_protocol_client_obj_free;
	client_entry->data.op.topic_data_receive = mros_protocol_topic_data_receive;
	client_entry->data.op.topic_data_send = mros_protocol_topic_data_send;
	ret = mros_topic_connector_set_connection(cobj, client_entry);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}
	return MROS_E_OK;
}


mRosReturnType mros_proc_pub_tcpros(mRosCommTcpClientType *client, mRosPacketType *packet)
{
	mRosReturnType ret;
	mRosNodeIdType node_id;
	mRosTopicIdType topic_id;
	mRosSizeType res;
	mRosTcpRosPacketType tcpros_packet;

	ret = mros_tcprospacket_decode(packet, &tcpros_packet);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}
	ret = mros_topic_get((const char*)&tcpros_packet.topic[0], &topic_id);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}
	node_id = mros_proc_connector_get_first(topic_id, MROS_TOPIC_CONNECTOR_PUB, MROS_NODE_TYPE_INNER, NULL);
	if (node_id == MROS_ID_NONE) {
		//TODO error reply...
		// original code does not support this case...
		ROS_ERROR("%s %u : not supported reply error", __FUNCTION__, __LINE__);
	}
	else {
		mRosEncodeArgType arg;
		const char* md5sum;
		(void)mros_topic_get_md5sum(topic_id, &md5sum);
		arg.type = MROS_PACKET_DATA_TCPROS_TOPIC_RES;
		arg.args_int = 0;
		arg.args_char = 4;
		arg.argv[0] = mros_node_name(node_id);
		arg.argv[1] = mros_topic_get_topic_name(topic_id);
		arg.argv[2] = mros_topic_get_topic_typename(topic_id);
		arg.argv[3] = md5sum;
		ret = mros_packet_encode(&arg, packet);
		if (ret != MROS_E_OK) {
			ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
			return ret;
		}
		ret = mros_comm_tcp_client_send_all(client, packet->data, packet->data_size, &res);
		if (ret != MROS_E_OK) {
			ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
			return ret;
		}
		(void)mros_proc_add_outersub_connector(client, topic_id);
	}

	return ret;
}

