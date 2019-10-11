#ifndef _MROS_PACKET_ENCODER_CIMPL_H_
#define _MROS_PACKET_ENCODER_CIMPL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mros_packet_cimpl.h"

/*************************************
 * Encoder
 *************************************/
#define MROS_ENCODE_ARGS_MAX	5
typedef struct {
	mRosPacketDataEnumType type;
	mRosSizeType	args_char;
	const char* 	argv[MROS_ENCODE_ARGS_MAX];
	mRosSizeType	args_int;
	mros_uint32		argi[MROS_ENCODE_ARGS_MAX];
} mRosEncodeArgType;

extern mRosReturnType mros_packet_encoder_init(void);

/*
 * 	arg.type = MROS_PACKET_DATA_REGISTER_PUBLISHER_REQ;
 *	arg.argv[0] = "registerPublisher";
 *	arg.argv[1] = req->node_name;
 *	arg.argv[2] = req->topic_name;
 *	arg.argv[3] = req->topic_typename;
 *	arg.argv[4] = MROS_URI_SLAVE;
 */
/*
 *	arg.type = MROS_PACKET_DATA_REGISTER_SUBSCRIBER_REQ;
 *	arg.argv[0] = "registerSubscriber";
 *	arg.argv[1] = req->node_name;
 *	arg.argv[2] = req->topic_name;
 *	arg.argv[3] = req->topic_typename;
 *	arg.argv[4] = MROS_URI_SLAVE;
 */
/*
 *	arg.type = MROS_PACKET_DATA_REQUEST_TOPIC_REQ;
 *	arg.argv[0] = "requestTopic";
 *	arg.argv[1] = req->node_name;
 *	arg.argv[2] = req->topic_name;
 *	arg.argv[3] = "TCPROS";
 */
/*
 *	arg.type = MROS_PACKET_DATA_REQUEST_TOPIC_RES;
 *	arg.argi[0] = MROS_PUBLISHER_PORT_NO;
 *	arg.argv[0] ="TCPROS";
 *	arg.argv[1] = MROS_NODE_IPADDR;
 */
extern mRosReturnType mros_packet_encode(mRosEncodeArgType *arg, mRosPacketType *packet);

#ifdef __cplusplus
}
#endif

#endif /* _MROS_PACKET_ENCODER_CIMPL_H_ */
