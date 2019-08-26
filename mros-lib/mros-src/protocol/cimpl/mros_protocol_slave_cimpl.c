#include "mros_protocol_slave_cimpl.h"
#include "mros_protocol_server_proc_cimpl.h"
#include "mros_comm_tcp_client_cimpl.h"
#include "mros_comm_tcp_server_cimpl.h"
#include "mros_packet_encoder_cimpl.h"
#include "mros_exclusive_area.h"
#include "mros_sys_config.h"

typedef union {
	char buffer;
	char buffer1[MROS_PACKET_MAXSIZE_REQ_REQUEST_TOPIC];
	char buffer2[MROS_PACKET_MAXSIZE_RES_REQUEST_TOPIC];
	char buffer3[MROS_PACKET_MAXSIZE_REQ_PUBLISHER_UPDATE];
} mRosSlavePacketBufferType;
static mRosSlavePacketBufferType mros_slave_packet_buffer MROS_MATTR_BSS_NOCLR;

typedef struct {
	mRosProtocolSlaveStateEnumType 		state;
	mRosEncodeArgType 					arg;
	mRosPacketType						packet;
	mRosCommTcpServerType				server_comm;
	mRosCommTcpClientType				client_comm;
	mros_uint32							self_ipaddr;
} mRosProtocolSlaveType;

static mRosProtocolSlaveType mros_protocol_slave MROS_MATTR_BSS_NOCLR;

mRosReturnType mros_protocol_slave_init(void)
{
	mRosReturnType ret = mros_comm_tcp_server_init(&mros_protocol_slave.server_comm);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}

	mros_protocol_slave.packet.total_size = sizeof(mRosSlavePacketBufferType);
	mros_protocol_slave.packet.data = &mros_slave_packet_buffer.buffer;
	mros_protocol_slave.state = MROS_PROTOCOL_SLAVE_STATE_WAITING;
	ret =  mros_comm_tcp_server_bind(&mros_protocol_slave.server_comm, MROS_SLAVE_PORT_NO);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}
	ret = mros_comm_tcp_server_listen(&mros_protocol_slave.server_comm, MROS_COMM_TCP_SERVER_LISTEN_MAX_DEFAULT_VALUE);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}
	ret = mros_comm_inet_get_ipaddr((const char *)MROS_NODE_IPADDR, &mros_protocol_slave.self_ipaddr);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return ret;
	}
	return MROS_E_OK;
}

void mros_protocol_slave_run(void)
{
	mRosReturnType ret;
	mROsExclusiveUnlockObjType unlck_obj;

	while (MROS_TRUE) {
		mros_protocol_slave.state = MROS_PROTOCOL_SLAVE_STATE_WAITING;
		ret = mros_comm_tcp_server_accept(&mros_protocol_slave.server_comm, &mros_protocol_slave.client_comm);
		if (ret != MROS_E_OK) {
			continue;
		}
		mros_protocol_slave.state = MROS_PROTOCOL_SLAVE_STATE_REPLYING_REQUEST_TOPIC;
		ret = mros_proc_receive(&mros_protocol_slave.client_comm, &mros_protocol_slave.packet);
		if (ret != MROS_E_OK) {
			ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
			mros_comm_tcp_client_close(&mros_protocol_slave.client_comm);
			continue;
		}
		mros_exclusive_lock(&mros_exclusive_area, &unlck_obj);
		ret = mros_proc_slave(&mros_protocol_slave.client_comm, &mros_protocol_slave.packet, mros_protocol_slave.self_ipaddr);
		mros_exclusive_unlock(&unlck_obj);
		if (ret != MROS_E_OK) {
			ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
			continue;
		}
		mros_comm_tcp_client_close(&mros_protocol_slave.client_comm);
	}
	return;
}
