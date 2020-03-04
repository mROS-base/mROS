#include "mros_protocol_operation_cimpl.h"
#include "mros_comm_tcp_client_factory_cimpl.h"
#include "mros_packet_decoder_cimpl.h"
#include "mros_packet_encoder_cimpl.h"
#include "mros_sys_config.h"
#include <string.h>

mRosSizeType mros_protocol_get_buffersize(mRosSizeType body_size)
{
	return (body_size + MROS_TOPIC_RAWDATA_HEADER_SIZE);
}

char* mros_protocol_get_body(char *buffer)
{
	return &buffer[MROS_TOPIC_RAWDATA_HEADER_SIZE];
}


mRosReturnType mros_protocol_topic_data_send(mRosCommTcpClientType *client, const char *data, mRosSizeType datalen)
{
	mRosReturnType ret;
	mRosSizeType res;
	mRosPacketType packet;
	mRosEncodeArgType arg;

	packet.total_size = MROS_TOPIC_RAWDATA_HEADER_SIZE;
	packet.data_size = 0;

	packet.data = (char*)data;
	arg.type = MROS_PACKET_DATA_TOPIC;
	arg.args_int = 1;
	arg.argi[0] = datalen;
	arg.args_char = 0;
	ret = mros_packet_encode(&arg, &packet);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}
	//send header & body
	return mros_comm_tcp_client_send_all(client, data, datalen, &res);
}

mRosReturnType mros_protocol_topic_data_receive(mRosCommTcpClientType *client, mRosMemoryManagerType *mempool, mRosMemoryListEntryType **retp)
{
	mRosPacketType packet;
	mRosSizeType len;
	mRosSizeType res;
	mRosReturnType ret;
	mRosMemoryListEntryType *mem_entryp;
	mros_int8 rawdata[MROS_TOPIC_RAWDATA_HEADER_SIZE];
	*retp = MROS_NULL;

	ret = mros_comm_socket_wait_readable(&client->socket, 0);
	if (ret != MROS_E_OK) {
		//ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}
	//receive header
	ret = mros_comm_tcp_client_receive_all(client, rawdata, MROS_TOPIC_RAWDATA_HEADER_SIZE, &res);
	if (ret != MROS_E_OK) {
		//ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}
	//decode header
	packet.total_size = MROS_TOPIC_RAWDATA_HEADER_SIZE;
	packet.data_size = MROS_TOPIC_RAWDATA_HEADER_SIZE;
	packet.data = rawdata;
	ret = mros_topicpacket_get_body_size(&packet, &len);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}
	//receive body
	ret = mros_mem_alloc(mempool, len + MROS_TOPIC_RAWDATA_HEADER_SIZE, &mem_entryp);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}
	memcpy(&mem_entryp->data.memp[0], rawdata, MROS_TOPIC_RAWDATA_HEADER_SIZE);

	ret = mros_comm_tcp_client_receive_all(client, &mem_entryp->data.memp[MROS_TOPIC_RAWDATA_HEADER_SIZE], len, &res);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}
	*retp = mem_entryp;
	return MROS_E_OK;
}

void mros_protocol_client_obj_free(void* reqp)
{
	mros_comm_tcp_client_free((mRosCommTcpClientListReqEntryType *)reqp);
	return;
}


