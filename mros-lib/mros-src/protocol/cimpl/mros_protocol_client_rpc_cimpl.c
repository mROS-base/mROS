#include "mros_protocol_client_rpc_cimpl.h"
#include "mros_packet_encoder_cimpl.h"
#include "mros_packet_decoder_cimpl.h"
#include "mros_sys_config.h"

static mRosReturnType mros_rpc_sendreply_xmlpacket(mRosEncodeArgType *arg, mRosCommTcpClientType *client, mRosPacketType *req, mRosPacketType *res)
{
	mRosReturnType ret;
	mros_boolean is_end;
	mRosSizeType rlen;

	ret = mros_packet_encode(arg, req);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}
	ret = mros_comm_tcp_client_send_all(client, req->data, req->data_size, &rlen);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}
	res->data_size = 0;
	do {
		ret = mros_comm_tcp_client_receive(client, &res->data[res->data_size], (res->total_size - res->data_size),  &rlen);
		if (ret != MROS_E_OK) {
			return ret;
		}
		res->data_size += rlen;
		if ((res->data_size + 1) >= res->total_size) {
			ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, MROS_E_NOMEM);
			return MROS_E_NOMEM;
		}
		is_end = mros_xmlpacket_has_response_end(res);
	} while (is_end == MROS_FALSE);
	res->data[res->data_size] = '\0';
	return MROS_E_OK;
}

mRosReturnType mros_rpc_register_publisher(mRosCommTcpClientType *client, mRosRegisterTopicReqType *req, mRosRegisterTopicResType *res)
{
	mRosEncodeArgType arg;

	arg.type = MROS_PACKET_DATA_REGISTER_PUBLISHER_REQ;
	arg.args_int = 0;
	arg.args_char = 5;
	arg.argv[0] = "registerPublisher";
	arg.argv[1] = req->node_name;
	arg.argv[2] = req->topic_name;
	arg.argv[3] = req->topic_typename;
	arg.argv[4] = MROS_URI_SLAVE;
	return mros_rpc_sendreply_xmlpacket(&arg, client, req->req_packet, res->reply_packet);
}

mRosReturnType mros_rpc_register_subscriber(mRosCommTcpClientType *client, mRosRegisterTopicReqType *req, mRosRegisterTopicResType *res)
{
	mRosEncodeArgType arg;

	arg.type = MROS_PACKET_DATA_REGISTER_SUBSCRIBER_REQ;
	arg.args_int = 0;
	arg.args_char = 5;
	arg.argv[0] = "registerSubscriber";
	arg.argv[1] = req->node_name;
	arg.argv[2] = req->topic_name;
	arg.argv[3] = req->topic_typename;
	arg.argv[4] = MROS_URI_SLAVE;
	return mros_rpc_sendreply_xmlpacket(&arg, client, req->req_packet, res->reply_packet);
}

mRosReturnType mros_rpc_request_topic(mRosCommTcpClientType *client, mRosRequestTopicReqType *req, mRosRequestTopicResType *res)
{
	mRosEncodeArgType arg;

	arg.type = MROS_PACKET_DATA_REQUEST_TOPIC_REQ;
	arg.args_int = 0;
	arg.args_char = 4;
	arg.argv[0] = "requestTopic";
	arg.argv[1] = req->node_name;
	arg.argv[2] = req->topic_name;
	arg.argv[3] = "TCPROS";

	return mros_rpc_sendreply_xmlpacket(&arg, client, req->req_packet, res->reply_packet);
}

static mRosReturnType mros_rpc_sendreply_tcpros(mRosEncodeArgType *arg, mRosCommTcpClientType *client, mRosRcpRosReqType *req, mRosTcpRosResType *res)
{
	mRosPacketType packet;
	mRosReturnType ret;
	mRosSizeType len;
	mRosSizeType rlen;
	mros_int8 rawdata[MROS_TCPROS_RAWDATA_HEADER_SIZE];

	ret = mros_packet_encode(arg, req->req_packet);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}

	ret = mros_comm_tcp_client_send_all(client, req->req_packet->data, req->req_packet->data_size, &rlen);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}

	ret = mros_comm_tcp_client_receive_all(client, rawdata, MROS_TCPROS_RAWDATA_HEADER_SIZE, &rlen);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}
	packet.total_size = MROS_TCPROS_RAWDATA_HEADER_SIZE;
	packet.data_size = MROS_TCPROS_RAWDATA_HEADER_SIZE;
	packet.data = rawdata;
	ret = mros_tcprospacket_get_body_size(&packet, &len);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}
	ret = mros_comm_tcp_client_receive_all(client, res->reply_packet->data, len, &rlen);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}
	res->reply_packet->data_size = len;
	res->result = MROS_E_OK;
	return MROS_E_OK;
}

mRosReturnType mros_rpc_tcpros(mRosCommTcpClientType *client, mRosRcpRosReqType *req, mRosTcpRosResType *res)
{
	mRosEncodeArgType arg;

	arg.type = MROS_PACKET_DATA_TCPROS_TOPIC_REQ;
	arg.args_int = 0;
	arg.args_char = 4;
	arg.argv[0] = req->node_name;
	arg.argv[1] = req->topic_name;
	arg.argv[2] = req->topic_typename;
	arg.argv[3] = req->md5sum;

	return mros_rpc_sendreply_tcpros(&arg, client, req, res);
}
