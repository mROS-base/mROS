#include "mros_protocol_client_rpc_cimpl.h"
#include "mros_comm_tcp_server_cimpl.h"

/*****************************************
 * TASK CYCLE
 *****************************************/
#define MROS_SLAVE_TIMEOUT			100

void do_test_server(void)
{
	mRosCommTcpServerType server;
	mRosCommTcpClientType client;
	mRosReturnType ret = mros_comm_tcp_server_init(&server);
	if (ret != MROS_E_OK) {
		syslog(LOG_NOTICE, "mros_comm_tcp_server_init()=%d", ret);
		return;
	}
	(void)mros_comm_socket_set_blocking(&server.socket, MROS_FALSE, MROS_SLAVE_TIMEOUT);
	ret =  mros_comm_tcp_server_bind(&server, MROS_SLAVE_PORT_NO);
	if (ret != MROS_E_OK) {
		syslog(LOG_NOTICE, "mros_comm_tcp_server_bind()=%d", ret);
		return;
	}
	ret = mros_comm_tcp_server_listen(&server, MROS_COMM_TCP_SERVER_LISTEN_MAX_DEFAULT_VALUE);
	if (ret != MROS_E_OK) {
		syslog(LOG_NOTICE, "mros_comm_tcp_server_bind()=%d", ret);
		return;
	}
	while (MROS_TRUE) {
		ret = mros_comm_tcp_server_accept(&server, &client);
		if (ret != MROS_E_OK) {
			syslog(LOG_NOTICE, "mros_comm_tcp_server_accept()=%d", ret);
			continue;
		}
		syslog(LOG_NOTICE, "connected: ip=0x%x port=%u", client.remote.sin_addr, client.remote.sin_port);

		mros_comm_tcp_client_close(&client);
	}
}

void do_test_register_publisher(void)
{
	mRosPacketType packet;
	mRosReturnType ret;
	static char buffer[1024];
	mRosCommTcpClientType client;
	syslog(LOG_NOTICE, "START: TEST REGISTER PUBLISHER");

	ret = mros_comm_tcp_client_init(&client, MROS_MASTER_IPADDR, MROS_MASTER_PORT_NO);
	if (ret != MROS_E_OK) {
		syslog(LOG_NOTICE, "mros_comm_tcp_client_init()=%d", ret);
		return;
	}
	packet.data = buffer;
	packet.total_size = sizeof(buffer);
	packet.data_size = 0;

	ret = mros_comm_tcp_client_connect(&client);
	if (ret != MROS_E_OK) {
		syslog(LOG_NOTICE, "mros_comm_tcp_client_connect()=%d", ret);
		return;
	}
	mRosRegisterTopicReqType req;
	mRosRegisterTopicResType res;
	req.node_name = "node1";
	req.req_packet = &packet;
	req.topic_name = "/test_string";
	req.topic_typename = "std_msgs/String";
	res.reply_packet = &packet;

	ret = mros_rpc_register_publisher(&client, &req, &res);
	if (ret != MROS_E_OK) {
		syslog(LOG_NOTICE, "mros_rpc_register_publisher()=%d", ret);
		return;
	}
	mros_comm_tcp_client_close(&client);
	syslog(LOG_NOTICE, "data_size=%d", packet.data_size);
	syslog(LOG_NOTICE, "packet=%s", packet.data);
	syslog(LOG_NOTICE, "END: TEST REGISTER PUBLISHER");

	return;
}

void do_test_register_subscriber(void)
{
	mRosPacketType packet;
	mRosReturnType ret;
	static char buffer[1024];
	mRosCommTcpClientType client;

	syslog(LOG_NOTICE, "START: TEST REGISTER SUBSCRIBER");


	ret = mros_comm_tcp_client_init(&client, MROS_MASTER_IPADDR, MROS_MASTER_PORT_NO);
	if (ret != MROS_E_OK) {
		syslog(LOG_NOTICE, "mros_comm_tcp_client_init()=%d", ret);
		return;
	}
	packet.data = buffer;
	packet.total_size = sizeof(buffer);
	packet.data_size = 0;

	ret = mros_comm_tcp_client_connect(&client);
	if (ret != MROS_E_OK) {
		syslog(LOG_NOTICE, "mros_comm_tcp_client_connect()=%d", ret);
		return;
	}
	mRosRegisterTopicReqType req;
	mRosRegisterTopicResType res;
	req.node_name = "node1";
	req.req_packet = &packet;
	req.topic_name = "/test_string";
	req.topic_typename = "std_msgs/String";
	res.reply_packet = &packet;

	ret = mros_rpc_register_subscriber(&client, &req, &res);
	if (ret != MROS_E_OK) {
		syslog(LOG_NOTICE, "mros_rpc_register_publisher()=%d", ret);
		return;
	}
	syslog(LOG_NOTICE, "data_size=%d", packet.data_size);
	syslog(LOG_NOTICE, "packet=%s", packet.data);

	mros_uint32 ipaddr = -1;
	mros_int32 port = -1;
	mRosPtrType ptr = mros_xmlpacket_subres_get_first_uri(res.reply_packet, &ipaddr, &port);
	syslog(LOG_NOTICE, "ptr=0x%x ipaddr=0x%x port=%d", ptr, ipaddr, port);

	syslog(LOG_NOTICE, "END: TEST REGISTER SUBSCRIBER");

	return;
}

static mros_uint32 test_request_topic_ipaddr;
static mros_int32 test_request_topic_port;

void do_test_request_topic(void)
{
	mRosPacketType packet;
	mRosReturnType ret;
	static char buffer[1024];
	mRosCommTcpClientType client;

	syslog(LOG_NOTICE, "START: TEST REQUEST TOPIC");


	ret = mros_comm_tcp_client_init(&client, MROS_MASTER_IPADDR, MROS_MASTER_PORT_NO);
	if (ret != MROS_E_OK) {
		syslog(LOG_NOTICE, "mros_comm_tcp_client_init()=%d", ret);
		return;
	}
	packet.data = buffer;
	packet.total_size = sizeof(buffer);
	packet.data_size = 0;

	ret = mros_comm_tcp_client_connect(&client);
	if (ret != MROS_E_OK) {
		syslog(LOG_NOTICE, "mros_comm_tcp_client_connect()=%d", ret);
		return;
	}
	mRosRegisterTopicReqType req;
	mRosRegisterTopicResType res;
	req.node_name = "node1";
	req.req_packet = &packet;
	req.topic_name = "/test_string";
	req.topic_typename = "std_msgs/String";
	res.reply_packet = &packet;
	ret = mros_rpc_register_subscriber(&client, &req, &res);
	if (ret != MROS_E_OK) {
		syslog(LOG_NOTICE, "mros_rpc_register_publisher()=%d", ret);
		return;
	}
	mros_comm_tcp_client_close(&client);
	mros_uint32 ipaddr = -1;
	mros_int32 port = -1;
	mRosPtrType ptr = mros_xmlpacket_subres_get_first_uri(res.reply_packet, &ipaddr, &port);
	syslog(LOG_NOTICE, "ptr=0x%x ipaddr=0x%x port=%d", ptr, ipaddr, port);
	if (ptr == MROS_NULL) {
		syslog(LOG_NOTICE, "no publisher");
		return;
	}
	mRosRequestTopicReqType topic_req;
	mRosRequestTopicResType topic_res;
	topic_req.node_name = "node1";
	topic_req.req_packet = &packet;
	topic_req.topic_name = "/test_string";
	topic_res.reply_packet = &packet;
	if (ptr != MROS_NULL) {
		ret = mros_comm_tcp_client_ip32_init(&client, ipaddr, port);
		if (ret != MROS_E_OK) {
			syslog(LOG_NOTICE, "mros_comm_tcp_client_connect()=%d", ret);
			return;
		}
		ret = mros_comm_tcp_client_connect(&client);
		if (ret != MROS_E_OK) {
			syslog(LOG_NOTICE, "mros_comm_tcp_client_connect()=%d", ret);
			return;
		}
		ret = mros_rpc_request_topic(&client, &topic_req, &topic_res);
		mros_comm_tcp_client_close(&client);
		if (ret != MROS_E_OK) {
			syslog(LOG_NOTICE, "mros_protocol_master_request_topic()=%d", ret);
			return;
		}
		ptr = mros_xmlpacket_reqtopicres_get_first_uri(topic_res.reply_packet, &test_request_topic_ipaddr, &test_request_topic_port);
		syslog(LOG_NOTICE, "ptr=0x%x ipaddr=0x%x port=%d", ptr, test_request_topic_ipaddr, test_request_topic_port);
	}
	syslog(LOG_NOTICE, "data_size=%d", packet.data_size);
	syslog(LOG_NOTICE, "packet=%s", packet.data);
	mros_comm_tcp_client_close(&client);


	syslog(LOG_NOTICE, "END: TEST REQUEST TOPIC");

	return;
}

void do_test_tcpros_topic(void)
{
	mRosPacketType packet;
	mRosReturnType ret;
	static char buffer[1024];
	mRosCommTcpClientType client;
	mRosRcpRosReqType req;
	mRosTcpRosResType res;


	do_test_request_topic();

	syslog(LOG_NOTICE, "START: TEST TCPROS TOPIC");
	packet.data = buffer;
	packet.total_size = sizeof(buffer);
	packet.data_size = 0;

	req.node_name = "node1";
	req.req_packet = &packet;
	req.topic_name = "/test_string";
	req.topic_typename = "std_msgs/String";
	res.reply_packet = &packet;

	ret = mros_comm_tcp_client_ip32_init(&client, test_request_topic_ipaddr, test_request_topic_port);
	if (ret != MROS_E_OK) {
		syslog(LOG_NOTICE, "mros_comm_tcp_client_connect()=%d", ret);
		return;
	}
	ret = mros_comm_tcp_client_connect(&client);
	if (ret != MROS_E_OK) {
		syslog(LOG_NOTICE, "mros_comm_tcp_client_connect()=%d", ret);
		return;
	}
	syslog(LOG_NOTICE, "mros_comm_tcp_client_connect(): SUCCESS");

	ret = mros_rpc_tcpros(&client, &req, &res);
	if (ret != MROS_E_OK) {
		syslog(LOG_NOTICE, "mros_rpc_tcpros()=%d", ret);
		return;
	}
	syslog(LOG_NOTICE, "data_size=%d", packet.data_size);
	mRosTcpRosPacketType tcpros_packet;
	ret = mros_tcprospacket_decode(&packet, &tcpros_packet);
	if (ret != MROS_E_OK) {
		syslog(LOG_NOTICE, "mros_tcprospacket_decode()=%d", ret);
		return;
	}
	syslog(LOG_NOTICE, "callerid=%s", tcpros_packet.callerid);
	syslog(LOG_NOTICE, "tcp_nodely=%s", tcpros_packet.tcp_nodely);
	syslog(LOG_NOTICE, "topic=%s", tcpros_packet.topic);
	syslog(LOG_NOTICE, "type=%s", tcpros_packet.type);
	syslog(LOG_NOTICE, "md5sum=%s", tcpros_packet.md5sum);
	mros_comm_tcp_client_close(&client);
	syslog(LOG_NOTICE, "END: TEST TCPROS TOPIC");

	return;
}
