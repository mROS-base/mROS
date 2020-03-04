#ifndef _MROS_PACKET_FMT_TCPROS_H_
#define _MROS_PACKET_FMT_TCPROS_H_

#ifdef __cplusplus
extern "C" {
#endif

#define MROS_PACKET_FMT_TCPROS_CALLER_ID	"callerid=%s"
#define MROS_PACKET_FMT_TCPROS_TOPIC		"topic=%s"
#define MROS_PACKET_FMT_TCPROS_TYPE			"type=%s"
#define MROS_PACKET_FMT_TCPROS_MD5SUM		"md5sum=%s"
#define MROS_PACKET_FMT_TCPROS_TCPNODELAY	"tcp_nodelay=%s"

/*
 * ARG1: callerid
 * ARG2: topic
 * ARG3: type
 * ARG4: md5sum
 */
#define MROS_PACKET_FMT_TCPROS_TOPIC_REQ		\
	"SIZE"	\
	"SIZE"	\
	"callerid=%s"	\
	"SIZE"	\
	"tcp_nodelay=1"	\
	"SIZE"	\
	"topic=%s"	\
	"SIZE"	\
	"type=%s"		\
	"SIZE"	\
	"md5sum=%s"


/*
 * ARG1: callerid
 * ARG2: topic
 * ARG3: type
 * ARG4: md5sum
 */
#define MROS_PACKET_FMT_TCPROS_TOPIC_RES	\
	"SIZE"	\
	"SIZE"	\
	"callerid=%s"	\
	"SIZE"	\
	"topic=%s"	\
	"SIZE"	\
	"type=%s"		\
	"SIZE"	\
	"md5sum=%s"


#ifdef __cplusplus
}
#endif


#endif /* _MROS_PACKET_FMT_TCPROS_H_ */
