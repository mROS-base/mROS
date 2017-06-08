#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include "syssvc/serial.h"
#include "syssvc/syslog.h"
#include "kernel_cfg.h"
#include "ros_emb.h"

#include "mbed.h"
//#include "DisplayBace.h"
//#include "rtos.h"
//#include "JPEG_Converter.h"
#include "EthernetInterface.h"


#include "syssvc/logtask.h"





/*初期化（最初のタスク：masterに接続？）
 *マスタに対する名前解決とかもかな？ */
/****Network setteing **/
#define USE_DHCP (1)
#if(USE_DHCP == 0)
	#define IP_ADDRESS  	("")	/*IP address */
	#define SUBNET_MASK		("")	/*Subset mask */
	#define DEFAULT_GATEWAY	("")	/*Default gateway */
#endif

EthernetInterface network;
xmlNode *node;

void network_init(){

/*TCP/IPでマスタのPCと接続*/
#if (USE_DHCP == 1)
	if(network.init() != 0) {
#else
	if(network.init(IP_ADDRESS, SUBNET_MASK, DEFAULT_GATEWAY) != 0) {
#endif
		syslog(LOG_NOTICE, "Network Initialize Error\r\n");
	return;
	}
		syslog(LOG_NOTICE, "Network Initialized successfully");
	while (network.connect(5000) != 0){
		//syslog(LOG_NOTICE, "LOG_NOTICE: Network Connect Error");
	}
		syslog(LOG_NOTICE,"MAC Address is %s\r\n", network.getMACAddress());
		syslog(LOG_NOTICE,"IP Address is %s\r\n", network.getIPAddress());
		syslog(LOG_NOTICE,"NetMask is %s\r\n", network.getNetworkMask());
		syslog(LOG_NOTICE,"Gateway Address is %s\r\n", network.getGateway());

		//SnapshotHandler::attach_req(&snapshot_req);
		//HTTPServerAddHandler<FSHandler>("/");
		//HTTPServerAddHandler<RPCHandler>("/rpc");
		//HTTPServerStart(80);
}

//マスタへのXML-RPCクライアント通信OK
TCPSocketConnection mas_sock;

const char *m_ip = "192.168.0.15";	//ros master IP
const int m_port = 11311;	//ros master xmlrpc port
int n_port,tcp_port;
/* masterとの通信タスク*/
/* HTTPクライアントが必要 */
void connect_master(){
	if(mas_sock.connect(m_ip,m_port) == -1){
		exit(1);
	}
}

//
void xml2master(){
	string xml;
	//テスト用にとりあえずサブスクライバの登録だけ
	//xml = registerPublisher("/mros_node","/test_string","std_msgs/String","http://192.168.0.10:40040");
	xml = registerSubscriber("/mros_node","/test_string","std_msgs/String","http://192.168.0.10:40040");
	char *snd_buff;
	snd_buff = (char *)malloc(1024*sizeof(char));
	strcpy(snd_buff,xml.c_str());
	int n;
	n = mas_sock.send(snd_buff,strlen(snd_buff));
	//syslog(LOG_NOTICE,"LOG_INFO: strlen: %d , sizeof: %d",strlen(snd_buff),sizeof(snd_buff));
	free(snd_buff);
	//syslog(LOG_NOTICE, "LOG_INFO: data send\n%s",snd_buff);
	if(n < 0){
		exit(1);
	}
	char *rcv_buff;
	rcv_buff = (char *)malloc(1024*sizeof(char));
	n = mas_sock.receive(rcv_buff,1024);
	string tmp;
	tmp = rcv_buff;
	n_port = get_port(tmp);
	syslog(LOG_NOTICE,"LOG_INFO: Node port %d",n_port);
	if(n < 0){
		free(rcv_buff);
		exit(1);
	}else{
		syslog(LOG_NOTICE, "LOG_INFO: data receive\n%s",rcv_buff);
		free(rcv_buff);
	}
}


void node_server(int port){
	TCPSocketConnection csock;
	TCPSocketServer svr;
	nodeServerStart(svr,csock,port);
}

void request_topic(){
	TCPSocketConnection subsock;
	subsock.connect(m_ip,n_port);
	string rpc;
	rpc = requestTopic("requestTopic","/mros_node","TCPROS");
	char *snd_buff;
		snd_buff = (char *)malloc(1024*sizeof(char));
		strcpy(snd_buff,rpc.c_str());
		int n;
		n = subsock.send(snd_buff,strlen(snd_buff));
		//syslog(LOG_NOTICE,"LOG_INFO: strlen: %d , sizeof: %d",strlen(snd_buff),sizeof(snd_buff));
		free(snd_buff);
		//syslog(LOG_NOTICE, "LOG_INFO: data send\n%s",snd_buff);
		if(n < 0){
			exit(1);
		}
		char *rcv_buff;
		rcv_buff = (char *)malloc(1024*sizeof(char));
		n = subsock.receive(rcv_buff,1024);
		string tmp;
		tmp = rcv_buff;
		tcp_port = get_port2(tmp);
		syslog(LOG_NOTICE,"LOG_INFO: TCP port %d",tcp_port);
		if(n < 0){
			free(rcv_buff);
			exit(1);
		}else{
			syslog(LOG_NOTICE, "LOG_INFO: data receive\n%s",rcv_buff);
			free(rcv_buff);
		}
}

void rostcp(){
	TCPSocketConnection tcpsock;
	tcpsock.connect(m_ip,tcp_port);
	string rpc;
	//rpc = requestTopic("requestTopic","/mros_node","TCPROS");
	char *snd_buff;
		snd_buff = (char *)malloc(1024*sizeof(char));
		strcpy(snd_buff,rpc.c_str());
		int n;
		n = tcpsock.send(snd_buff,strlen(snd_buff));
		//syslog(LOG_NOTICE,"LOG_INFO: strlen: %d , sizeof: %d",strlen(snd_buff),sizeof(snd_buff));
		free(snd_buff);
		if(n < 0){
			exit(1);
		}
		char *rcv_buff;
		rcv_buff = (char *)malloc(1024*sizeof(char));
		n = tcpsock.receive(rcv_buff,1024);
		string tmp;
		tmp = rcv_buff;
		if(n < 0){
			free(rcv_buff);
			exit(1);
		}else{
			syslog(LOG_NOTICE, "LOG_INFO: data receive\n%s",rcv_buff);
			free(rcv_buff);
		}
}

/* mROS communication test
 * [registration as Publisher] mROS ->(XML-RPC)-> master node
 * [request topic] mROS <- (XML-RPC) <- Subscriber node
 */

void main_task(){
	syslog(LOG_NOTICE, "**********mROS START******************");
	syslog(LOG_NOTICE, "LOG_INFO: network initialize...");
	network_init();
	syslog(LOG_NOTICE, "LOG_INFO: SUCCESS INITIALIZATION");

	//MASTER CLIENT TEST//
	syslog(LOG_NOTICE, "LOG_INFO: connecting master...");
	connect_master();
	syslog(LOG_NOTICE, "LOG_INFO: CONNECTED master");
	syslog(LOG_NOTICE, "LOG_INFO: DO RPC CALL");
	xml2master();


	//SERVER TEST//
	syslog(LOG_NOTICE, "LOG_INFO: starting server...");
	//node_server(40040);

	//SUBSCRIBER TEST//
	request_topic();

	syslog(LOG_NOTICE,"LOG_INFO: starting TCPROS...");
	//RECEIVE TCPROS//
	//node_server(8080);

	//TCPROS TEST//
	rostcp();

	syslog(LOG_NOTICE, "**********mROS FINISH***********");

}

