#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include "syssvc/serial.h"
#include "syssvc/syslog.h"
#include "kernel_cfg.h"
#include <string>
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


void connect_host(){
	printf("Network Setting up...\r\n");

/*TCP/IPでマスタのPCと接続*/
#if (USE_DHCP == 1)
	if(network.init() != 0) {
#else
	if(network.init(IP_ADDRESS, SUBNET_MASK, DEFAULT_GATEWAY) != 0) {
#endif
		//	syslog(LUG_NOTICE, "Network Initialize Error\r\n");
	return;
	}
	//	syslog(LOG_NOTICE, "Network Initialized successfully");
	while (network.connect(5000) != 0){
		//	syslog(LOG_NOTICE, "LOG_NOTICE: Network Connect Error");
	}
		printf("MAC Address is %s\r\n", network.getMACAddress());
		printf("IP Address is %s\r\n", network.getIPAddress());
		printf("NetMask is %s\r\n", network.getNetworkMask());
		printf("Gateway Address is %s\r\n", network.getGateway());

		//	syslog(LOG_INFO, "LOG_INFO: Network Setup OK");
		printf("Network Setup OK\r\n");
	    //SnapshotHandler::attach_req(&snapshot_req);
		//HTTPServerAddHandler<FSHandler>("/");
		//HTTPServerAddHandler<RPCHandler>("/rpc");
		//HTTPServerStart(80);
}

TCPSocketConnection mas_sock;
const char *m_ip = "";	//ros master IP
const int m_port = 11311;	//ros master xmlrpc port

/* masterとの通信タスク*/
/* HTTPクライアントが必要 */
void connect_master(){
	if(mas_sock.connect(m_ip,m_port) == -1){
		printf("connect_master error");
		exit(1);
	}
}

//master への　rpc call とりあえずrpc_numでメソッドの仕分け
void rpc_call(int rpc_num){

}


/*トピックの購読
 * ポーリングとか？この辺はまた調べる*/
/*
void sub_topic(){

}
*/

/*トピックのデータ処理*/
/*
void topic_func(){

}

*/

/*トピックの出版
 * とりあえず出す*/
 /*
void pub_topic(){

}
*/

void xml2master(){
	string xml;
	//テスト用にとりあえずサブスクライバの登録だけ
	xml = registerSubscriber("/mros_node","/test_int","int","/");
	char *buff = xml.c_str();
	int n;
	n = mas_sock.send(buff,strlen(buff));
	if(n < 0){
		printf("fail register\n");
		exit(1);
	}
	n = mas_sock.receive_all(buff,512);
	if(n < 0){
		printf("fail receive\n");
		exit(1);
	}else{
		printf("****receive data****\n");
		printf("%s",buff);
	}

}


void main_task(){
	printf("********mROS START********\n");
	printf("<test start>\n");

	printf("...connecting host PC\n");
	connect_host();
	printf("success connect host PC\n");
	
	printf("...connecting ros master\n");
	connect_master();
	printf("success connect ros master\n");
	
	printf("...registering this node as subscriver\n");
	xml2master();
	printf("<test end>\n");

}
