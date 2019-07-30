#include "mros_packet_encoder_cimpl.h"
#include "mros_packet_fmt_xml.h"
#include "mros_packet_fmt_http.h"
#include "mros_packet_fmt_tcpros.h"
#include "mros_packet_config.h"
#include <string.h>
#include <stdio.h>

static mRosReturnType encode_register_publisher_req(mRosEncodeArgType *arg, mRosPacketType *packet);
static mRosReturnType encode_register_subscriber_req(mRosEncodeArgType *arg, mRosPacketType *packet);
static mRosReturnType encode_request_topic_req(mRosEncodeArgType *arg, mRosPacketType *packet);
static mRosReturnType encode_request_topic_res(mRosEncodeArgType *arg, mRosPacketType *packet);
static mRosReturnType encode_tcpros_topic_req(mRosEncodeArgType *arg, mRosPacketType *packet);
static mRosReturnType encode_tcpros_topic_res(mRosEncodeArgType *arg, mRosPacketType *packet);
static mRosReturnType encode_topic_data(mRosEncodeArgType *arg, mRosPacketType *packet);
typedef mRosReturnType (*encode_table_type) (mRosEncodeArgType*, mRosPacketType*);

static encode_table_type encode_table[MROS_PACKET_DATA_NUM] = {
		encode_register_publisher_req, 			//MROS_PACKET_DATA_REGISTER_PUBLISHER_REQ
		encode_register_subscriber_req,			//MROS_PACKET_DATA_REGISTER_SUBSCRIBER_REQ
		encode_request_topic_req,				//MROS_PACKET_DATA_REQUEST_TOPIC_REQ
		encode_request_topic_res,				//MROS_PACKET_DATA_REQUEST_TOPIC_RES
		MROS_NULL,								//MROS_PACKET_DATA_PUBLISHER_UPDATE_REQ
		encode_tcpros_topic_req,				//MROS_PACKET_DATA_TCPROS_TOPIC_REQ
		encode_tcpros_topic_res,				//MROS_PACKET_DATA_TCPROS_TOPIC_RES
		encode_topic_data,						//MROS_PACKET_DATA_TOPIC
};

typedef struct {
	const char* fmt;
	mros_uint32 len;
} mRosFmtType;


static mRosFmtType mros_xml_fmt_table[MROS_PACKET_DATA_NUM] MROS_MATTR_BSS_NOCLR;
static mRosFmtType mros_http_post_fmt MROS_MATTR_BSS_NOCLR;
static mRosFmtType mros_http_ok_fmt MROS_MATTR_BSS_NOCLR;
static mRosFmtType mros_tcpros_topic_req_fmt MROS_MATTR_BSS_NOCLR;
static mRosFmtType mros_tcpros_topic_res_fmt MROS_MATTR_BSS_NOCLR;

mRosReturnType mros_packet_encoder_init(void)
{
	mros_xml_fmt_table[MROS_PACKET_DATA_REGISTER_PUBLISHER_REQ].fmt = MROS_PACKET_FMT_XML_REGISTER_REQ;
	mros_xml_fmt_table[MROS_PACKET_DATA_REGISTER_PUBLISHER_REQ].len = strlen(MROS_PACKET_FMT_XML_REGISTER_REQ) + 1;

	mros_xml_fmt_table[MROS_PACKET_DATA_REGISTER_SUBSCRIBER_REQ].fmt = MROS_PACKET_FMT_XML_REGISTER_REQ;
	mros_xml_fmt_table[MROS_PACKET_DATA_REGISTER_SUBSCRIBER_REQ].len = strlen(MROS_PACKET_FMT_XML_REGISTER_REQ) + 1;

	mros_xml_fmt_table[MROS_PACKET_DATA_REQUEST_TOPIC_REQ].fmt = MROS_PACKET_FMT_XML_REQUEST_TOPIC_REQ;
	mros_xml_fmt_table[MROS_PACKET_DATA_REQUEST_TOPIC_REQ].len = strlen(MROS_PACKET_FMT_XML_REQUEST_TOPIC_REQ) + 1;

	mros_xml_fmt_table[MROS_PACKET_DATA_REQUEST_TOPIC_RES].fmt = MROS_NULL;
	mros_xml_fmt_table[MROS_PACKET_DATA_REQUEST_TOPIC_RES].len = 0;

	mros_xml_fmt_table[MROS_PACKET_DATA_TCPROS_TOPIC_REQ].fmt = MROS_NULL;
	mros_xml_fmt_table[MROS_PACKET_DATA_TCPROS_TOPIC_REQ].len = 0;

	mros_xml_fmt_table[MROS_PACKET_DATA_TCPROS_TOPIC_RES].fmt = MROS_NULL;
	mros_xml_fmt_table[MROS_PACKET_DATA_TCPROS_TOPIC_RES].len = 0;

	mros_http_post_fmt.fmt = MROS_PACKET_FMT_HTTP_POST;
	mros_http_post_fmt.len = strlen(MROS_PACKET_FMT_HTTP_POST) + 1;

	mros_http_ok_fmt.fmt = MROS_PACKET_FMT_HTTP_OK;
	mros_http_ok_fmt.len = strlen(MROS_PACKET_FMT_HTTP_OK) + 1;

	mros_tcpros_topic_req_fmt.fmt = MROS_PACKET_FMT_TCPROS_TOPIC_REQ;
	mros_tcpros_topic_req_fmt.len = strlen(MROS_PACKET_FMT_TCPROS_TOPIC_REQ) + 1;

	mros_tcpros_topic_res_fmt.fmt = MROS_PACKET_FMT_TCPROS_TOPIC_RES;
	mros_tcpros_topic_res_fmt.len = strlen(MROS_PACKET_FMT_TCPROS_TOPIC_RES) + 1;


	return MROS_E_OK;
}

static void add_len(unsigned char *buf, mros_uint32 len)
{
    buf[0] = (unsigned char) (len);
    buf[1] = (unsigned char) (len >> 8);
    buf[2] = (unsigned char) (len >> 16);
    buf[3] = (unsigned char) (len >> 24);
}
static mros_uint32 get_digit(mros_uint32 value)
{
	mros_uint32 digit = 0;
	while (value != 0) {
		value = value / 10;
		digit++;
	}
	return digit;
}

static mRosSizeType get_arglen(mRosEncodeArgType *arg)
{
	mRosSizeType len = 0;
	mRosSizeType i;
	for (i = 0; i < arg->args_char; i++) {
		len += strlen(arg->argv[i]) + 1;
	}
	for (i = 0; i < arg->args_int; i++) {
		len += get_digit(arg->argi[i]) + 1;
	}
	return len;
}



mRosReturnType mros_packet_encode(mRosEncodeArgType *arg, mRosPacketType *packet)
{
	if (arg->type >= MROS_PACKET_DATA_NUM) {
		return MROS_E_RANGE;
	}
	if (encode_table[arg->type] == MROS_NULL) {
		return MROS_E_INVAL;
	}
	return encode_table[arg->type](arg, packet);
}


static mRosReturnType encode_register_publisher_req(mRosEncodeArgType *arg, mRosPacketType *packet)
{
	mRosSizeType len;
	mRosSizeType off = 0;
	mRosSizeType xml_len;

	len = get_arglen(arg);
	len += mros_http_post_fmt.len;
	len += mros_xml_fmt_table[arg->type].len;

	if (len > packet->total_size) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, MROS_E_NOMEM);
		return MROS_E_NOMEM;
	}
	xml_len = snprintf(&packet->data[off], packet->total_size,
			MROS_PACKET_FMT_XML_REGISTER_REQ,
			arg->argv[0],
			arg->argv[1],
			arg->argv[2],
			arg->argv[3],
			arg->argv[4]);

	packet->data_size = snprintf(&packet->data[off], packet->total_size,
			MROS_PACKET_FMT_HTTP_POST MROS_PACKET_FMT_XML_REGISTER_REQ,
			xml_len,
			arg->argv[0],
			arg->argv[1],
			arg->argv[2],
			arg->argv[3],
			arg->argv[4]);

	return MROS_E_OK;
}

static mRosReturnType encode_register_subscriber_req(mRosEncodeArgType *arg, mRosPacketType *packet)
{
	mRosSizeType len;
	mRosSizeType off = 0;
	mRosSizeType xml_len;

	len = get_arglen(arg);
	len += mros_http_post_fmt.len;
	len += mros_xml_fmt_table[arg->type].len;

	if (len > packet->total_size) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, MROS_E_NOMEM);
		return MROS_E_NOMEM;
	}
	xml_len = snprintf(&packet->data[off], packet->total_size,
			MROS_PACKET_FMT_XML_REGISTER_REQ,
			arg->argv[0],
			arg->argv[1],
			arg->argv[2],
			arg->argv[3],
			arg->argv[4]);

	packet->data_size = snprintf(&packet->data[off], packet->total_size,
			MROS_PACKET_FMT_HTTP_POST MROS_PACKET_FMT_XML_REGISTER_REQ,
			xml_len,
			arg->argv[0],
			arg->argv[1],
			arg->argv[2],
			arg->argv[3],
			arg->argv[4]);
	return MROS_E_OK;
}

static mRosReturnType encode_request_topic_req(mRosEncodeArgType *arg, mRosPacketType *packet)
{
	mRosSizeType len;
	mRosSizeType off = 0;
	mRosSizeType xml_len;

	len = get_arglen(arg);
	len += mros_http_post_fmt.len;
	len += mros_xml_fmt_table[arg->type].len;

	if (len > packet->total_size) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, MROS_E_NOMEM);
		return MROS_E_NOMEM;
	}
	xml_len = snprintf(&packet->data[off], packet->total_size,
			MROS_PACKET_FMT_XML_REQUEST_TOPIC_REQ,
			arg->argv[0],
			arg->argv[1],
			arg->argv[2],
			arg->argv[3]);

	packet->data_size = snprintf(&packet->data[off], packet->total_size,
			MROS_PACKET_FMT_HTTP_POST MROS_PACKET_FMT_XML_REQUEST_TOPIC_REQ,
			xml_len,
			arg->argv[0],
			arg->argv[1],
			arg->argv[2],
			arg->argv[3]);

	return MROS_E_OK;
}
mRosReturnType encode_request_topic_res(mRosEncodeArgType *arg, mRosPacketType *packet)
{
	mRosSizeType len;
	mRosSizeType off = 0;
	mRosSizeType xml_len;

	len = get_arglen(arg);
	len += mros_http_post_fmt.len;
	len += mros_xml_fmt_table[arg->type].len;

	if (len > packet->total_size) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, MROS_E_NOMEM);
		return MROS_E_NOMEM;
	}
	xml_len = snprintf(&packet->data[off], packet->total_size,
			MROS_PACKET_FMT_XML_REQUEST_TOPIC_RES,
			arg->argv[0],
			arg->argv[1],
			arg->argi[0]);

	packet->data_size = snprintf(&packet->data[off], packet->total_size,
			MROS_PACKET_FMT_HTTP_OK MROS_PACKET_FMT_XML_REQUEST_TOPIC_RES,
			xml_len,
			arg->argv[0],
			arg->argv[1],
			arg->argi[0]);

	return MROS_E_OK;
}

static mRosReturnType encode_tcpros_topic_req(mRosEncodeArgType *arg, mRosPacketType *packet)
{
	mRosSizeType len;
	mRosSizeType off = 0;
	mRosSizeType len_callerid;
	mRosSizeType len_tcpnodelay;
	mRosSizeType len_topic;
	mRosSizeType len_type;
	mRosSizeType len_md5sum;

	len = get_arglen(arg);
	len += mros_tcpros_topic_req_fmt.len;

	if (len > packet->total_size) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, MROS_E_NOMEM);
		return MROS_E_NOMEM;
	}

	len_callerid = snprintf(&packet->data[off], packet->total_size,
			MROS_PACKET_FMT_TCPROS_CALLER_ID, arg->argv[0]);

	len_tcpnodelay = snprintf(&packet->data[off], packet->total_size,
			MROS_PACKET_FMT_TCPROS_TCPNODELAY, "1");

	len_topic = snprintf(&packet->data[off], packet->total_size,
			MROS_PACKET_FMT_TCPROS_TOPIC, arg->argv[1]);

	len_type = snprintf(&packet->data[off], packet->total_size,
			MROS_PACKET_FMT_TCPROS_TYPE, arg->argv[2]);

	len_md5sum = snprintf(&packet->data[off], packet->total_size,
			MROS_PACKET_FMT_TCPROS_MD5SUM, arg->argv[3]);

	packet->data_size = snprintf(&packet->data[off], packet->total_size,
			MROS_PACKET_FMT_TCPROS_TOPIC_REQ,
			arg->argv[0],
			arg->argv[1],
			arg->argv[2],
			arg->argv[3]);

	off = 0;
	add_len((unsigned char*)&packet->data[off], packet->data_size - 4);

	off += 4U;
	add_len((unsigned char*)&packet->data[off], len_callerid);

	off += (4U + len_callerid);
	add_len((unsigned char*)&packet->data[off], len_tcpnodelay);

	off += (4U + len_tcpnodelay);
	add_len((unsigned char*)&packet->data[off], len_topic);

	off += (4U + len_topic);
	add_len((unsigned char*)&packet->data[off], len_type);

	off += (4U + len_type);
	add_len((unsigned char*)&packet->data[off], len_md5sum);

	return MROS_E_OK;
}

static mRosReturnType encode_tcpros_topic_res(mRosEncodeArgType *arg, mRosPacketType *packet)
{
	mRosSizeType len;
	mRosSizeType off = 0;
	mRosSizeType len_callerid;
	mRosSizeType len_topic;
	mRosSizeType len_type;
	mRosSizeType len_md5sum;

	len = get_arglen(arg);
	len += mros_tcpros_topic_res_fmt.len;

	if (len > packet->total_size) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, MROS_E_NOMEM);
		return MROS_E_NOMEM;
	}

	len_callerid = snprintf(&packet->data[off], packet->total_size,
			MROS_PACKET_FMT_TCPROS_CALLER_ID, arg->argv[0]);

	len_topic = snprintf(&packet->data[off], packet->total_size,
			MROS_PACKET_FMT_TCPROS_TOPIC, arg->argv[1]);

	len_type = snprintf(&packet->data[off], packet->total_size,
			MROS_PACKET_FMT_TCPROS_TYPE, arg->argv[2]);

	len_md5sum = snprintf(&packet->data[off], packet->total_size,
			MROS_PACKET_FMT_TCPROS_MD5SUM, arg->argv[3]);

	packet->data_size = snprintf(&packet->data[off], packet->total_size,
			MROS_PACKET_FMT_TCPROS_TOPIC_RES,
			arg->argv[0],
			arg->argv[1],
			arg->argv[2],
			arg->argv[3]);

	off = 0;
	add_len((unsigned char*)&packet->data[off], packet->data_size - 4);

	off += 4U;
	add_len((unsigned char*)&packet->data[off], len_callerid);

	off += (4U + len_callerid);
	add_len((unsigned char*)&packet->data[off], len_topic);

	off += (4U + len_topic);
	add_len((unsigned char*)&packet->data[off], len_type);

	off += (4U + len_type);
	add_len((unsigned char*)&packet->data[off], len_md5sum);

	return MROS_E_OK;
}

static mRosReturnType encode_topic_data(mRosEncodeArgType *arg, mRosPacketType *packet)
{
	if (packet->total_size < MROS_TOPIC_RAWDATA_HEADER_SIZE) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, MROS_E_NOMEM);
		return MROS_E_NOMEM;
	}
    add_len((unsigned char*)&packet->data[0], arg->argi[0] - MROS_TOPIC_RAWDATA_HEADER_SIZE);
    packet->data_size = MROS_TOPIC_RAWDATA_HEADER_SIZE;

	return MROS_E_OK;
}
