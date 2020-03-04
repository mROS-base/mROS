#ifndef _MROS_PACKET_DECODER_CIMPL_H_
#define _MROS_PACKET_DECODER_CIMPL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mros_packet_cimpl.h"

extern mRosReturnType mros_packet_decoder_init(void);


/************************************************
 * XML RPC PACKET
 ************************************************/
/*
 * XML request
 */
extern mros_boolean mros_xmlpacket_has_request_end(mRosPacketType *packet);
/*
 * XML response
 */
extern mros_boolean mros_xmlpacket_has_response_end(mRosPacketType *packet);

//MASTER
/*
 * ReigsterPublish response
 */
extern mRosReturnType mros_xmlpacket_pubres_result(mRosPacketType *packet);

/*
 * ReigsterSubscribe response
 */
extern mRosReturnType mros_xmlpacket_subres_result(mRosPacketType *packet);
extern mRosPtrType mros_xmlpacket_subres_get_first_uri(mRosPacketType *packet, mros_uint32 *ipaddr, mros_int32 *port);
extern mRosPtrType mros_xmlpacket_subres_get_next_uri(mRosPtrType ptr, mRosPacketType *packet, mros_uint32 *ipaddr, mros_int32 *port);


//SLAVE
typedef struct {
	struct {
		char *start_key;
		char *end_key;
	} req;
	struct {
		char *head;
		char *tail;
		mRosSizeType len;
	} res;
} mRosPacketMemberInfoType;

typedef struct {
	mRosPacketDataEnumType		packet_type;
	mRosPacketMemberInfoType method;
	union {
		struct {
			mRosPacketMemberInfoType node_name;
			mRosPacketMemberInfoType topic_name;
		} topic;
		struct {
			mRosPacketMemberInfoType name;
			mRosPacketMemberInfoType topic_name;
		} publisher_update;
	} request;
} mRosPacketDecodedRequestType;
extern mRosPacketDataEnumType mros_xmlpacket_slave_request_decode(mRosPacketType *packet, mRosPacketDecodedRequestType *decoded_infop);


/*
 * RequestTopic request
 */
extern mRosReturnType mros_xmlpacket_slave_reqtopic_get_topic_name(mRosPacketType *packet, char* topic_name, mros_uint32 len);

/*
 * RequestTopic response
 */
extern mRosReturnType mros_xmlpacket_reqtopicres_result(mRosPacketType *packet);
extern mRosPtrType mros_xmlpacket_reqtopicres_get_first_uri(mRosPacketType *packet, mros_uint32 *ipaddr, mros_int32 *port);
extern mRosPtrType mros_xmlpacket_reqtopicres_get_next_uri(mRosPtrType ptr, mRosPacketType *packet, mros_uint32 *ipaddr, mros_int32 *port);

/*
 * publisherUpdate request
 */
extern mRosPtrType mros_xmlpacket_pubupreq_get_first_uri(char *packet_data, mros_uint32 *ipaddr, mros_int32 *port);
extern mRosPtrType mros_xmlpacket_pubupreq_get_next_uri(mRosPtrType ptr, mRosPacketType *packet, mros_uint32 *ipaddr, mros_int32 *port);

/****************************************************
 * TCPROS
 ****************************************************/
extern mRosReturnType mros_tcprospacket_get_body_size(mRosPacketType *packet, mRosSizeType *len);
typedef struct {
	mros_uint16 callerid_len;
	mros_uint16 tcp_nodelay_len;
	mros_uint16 topic_len;
	mros_uint16 type_len;
	mros_uint16 md5sum_len;
	char *callerid;
	char *tcp_nodely;
	char *topic;
	char* type;
	char *md5sum;
} mRosTcpRosPacketType;
extern mRosReturnType mros_tcprospacket_decode(mRosPacketType *packet, mRosTcpRosPacketType *decoded_packet);


/****************************************************
 * TOPIC DATA
 ****************************************************/
extern mRosReturnType mros_topicpacket_get_body_size(mRosPacketType *packet, mRosSizeType *len);



#ifdef __cplusplus
}
#endif

#endif /* _MROS_PACKET_DECODER_CIMPL_H_ */
