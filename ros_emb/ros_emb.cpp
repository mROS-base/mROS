#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include "syssvc/serial.h"
#include "syssvc/syslog.h"
#include "kernel_cfg.h"
#include "mbed.h"
#include "EthernetInterface.h"
#include "syssvc/logtask.h"

#include "ros_emb.h"
#include "SoftPWM.h"

#define SUBSCRIBER true
#define PUBLISHER false



/*ネットワーク初期化
 *マスタに対する名前解決とかもかな？ */
#define USE_DHCP (1)
#if(USE_DHCP == 0)
	#define IP_ADDRESS  	("")	/*IP address */
	#define SUBNET_MASK		("")	/*Subset mask */
	#define DEFAULT_GATEWAY	("")	/*Default gateway */
#endif

EthernetInterface network;
//xmlNode *node;


void network_init(){

/*TCP/IPでホストPCと接続*/
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
		syslog(LOG_NOTICE, "LOG_NOTICE: Network Connect Error");
	}
		syslog(LOG_NOTICE,"MAC Address is %s\r\n", network.getMACAddress());
		syslog(LOG_NOTICE,"IP Address is %s\r\n", network.getIPAddress());
		syslog(LOG_NOTICE,"NetMask is %s\r\n", network.getNetworkMask());
		syslog(LOG_NOTICE,"Gateway Address is %s\r\n", network.getGateway());

}

//マスタへのソケット
TCPSocketConnection mas_sock;

const char *m_ip = "192.168.0.15";	//ros master IP
const int m_port = 11311;	//ros master xmlrpc port
int n_port,tcp_port;


void connect_master(){
	if(mas_sock.connect(m_ip,m_port) == -1){
		exit(1);
	}
}

//マスタとのXML-RPC通信クライアント
void xml2master(bool mode){
	string xml;
	if(mas_sock.connect(m_ip,m_port) == -1){
			exit(1);
		}
	//テスト用にとりあえずサブスクライバの登録だけ mode = false -> サブスクライバ
	if(!mode){
		xml = registerPublisher("/mros_node","/mros_msg","std_msgs/String","http://192.168.0.10:40040");
	}else{
		xml = registerSubscriber("/mros_node","/test_string","std_msgs/String","http://192.168.0.10");
	}
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
	string tmp;
	bool f = true;
	while(f){
	n = mas_sock.receive(rcv_buff,1024);
	syslog(LOG_NOTICE,"LOGINFO : RECIEVING now..");
	tmp = rcv_buff;
	if(tmp.find("OK") != -1){
		f = !f;
	}
	}
	//ポートの取得　サブスクライバの時だけでいい
	if(mode){
		n_port = get_port(tmp);
		syslog(LOG_NOTICE,"LOG_INFO: Node port %d",n_port);
	}
	if(n < 0){
		free(rcv_buff);
		exit(1);
	}else{
		//syslog(LOG_NOTICE, "LOG_INFO: data receive\n%s",rcv_buff);
		free(rcv_buff);
	}
	mas_sock.close();
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
		//ポート番号の切り出しは動く
		//XML-RPCはここで終わり
		tcp_port = get_port2(tmp);
		syslog(LOG_NOTICE,"LOG_INFO: TCP port %d",tcp_port);
		if(n < 0){
			free(rcv_buff);
			exit(1);
		}else{
			//syslog(LOG_NOTICE, "LOG_INFO: data receive\n%s",rcv_buff);
			free(rcv_buff);
		}
}

//デモ用

static DigitalOut ledu(P6_12);                                  // LED-User
static SoftPWM ledr(P6_13);                                     // LED-Red
static SoftPWM ledg(P6_14);                                     // LED-Green
static SoftPWM ledb(P6_15);                                     // LED-Blue

void rostcp(){
	//TCPROSを行ってデータを受信するところ
//デモ用
	ledu = 0;
	ledr.period_ms(10);
	ledr = 0.0f;
	ledg.period_ms(10);
	ledg = 0.0f;
	ledb.period_ms(10);
	ledb = 0.0f;
//
	TCPSocketConnection tcpsock;
	tcpsock.connect(m_ip,tcp_port);
	char *snd_buff;
	snd_buff = (char *)malloc(1024*sizeof(char));
	int len = genSubTcpRosH(snd_buff); //tcprosヘッダの作成
	int n = tcpsock.send(snd_buff,len);
	free(snd_buff);
	if(n < 0){
		exit(1);
	}
	char *rcv_buff;
	char *rcv_p;
	rcv_buff = (char *)malloc(1024*sizeof(char));
	rcv_p = &rcv_buff[8]; //tcprosのヘッダの部分は避ける
	while(1){
		n = tcpsock.receive(rcv_buff,256);
		rcv_buff[0] = rcv_buff[0] + '0';
		rcv_buff[1] = rcv_buff[1] + '0';
		rcv_buff[2] = rcv_buff[2] + '0';
		rcv_buff[3] = rcv_buff[3] + '0';
		rcv_buff[4] = rcv_buff[4] + '0';
		rcv_buff[5] = rcv_buff[5] + '0';
		rcv_buff[6] = rcv_buff[6] + '0';
		rcv_buff[7] = rcv_buff[7] + '0';
		if(n < 0){
			free(rcv_buff);
			exit(1);
		}else{
			//データリード
			//データレングス取得
			int len = rcv_buff[4] - '0';
			len = len + (rcv_buff[5] - '0') * 256;
			len = len + (rcv_buff[6] - '0') * 65536;
			len = len + (rcv_buff[7] - '0') * 16777216;
			rcv_p[len] = '\0';
			syslog(LOG_NOTICE, "mROS_INFO:Subsclibed %d [%s]\n",len,rcv_p);

			ledu = !ledu; //あんまり関係ないスイッチングしてるだけ
			string str = rcv_p;

			//LED云々するプログラム
			//loopを入れるかどうかあとで
			if(str.find("red") != -1){
				ledr = (float)100/128;		//LED RED
			}else if(str.find("blue") != -1){
				ledb = (float)100/128;		//LED BLUE
			}else if(str.find("green") != -1){
				ledg = (float)100/128;		//LED GREEN
			}else if(str.find("reset") != -1){
				ledr =0;
				ledg =0;
				ledb =0;
			}else if(str.find("end") != -1){
				return;
			}
            free(rcv_buff);
		}
	}
}
/* mROS communication test
 * [registration as Publisher] mROS ->(XML-RPC)-> master node
 * [request topic] mROS <- (XML-RPC) <- Subscriber node
 */

//ETWEST用デモプログラム
//userタスク

void main_task(){
	syslog(LOG_NOTICE, "**********mROS START******************");
	syslog(LOG_NOTICE, "LOG_INFO: network initialize...");
	network_init();
	syslog(LOG_NOTICE, "LOG_INFO: SUCCESS INITIALIZATION");

	//act_tsk(PUB_TASK);
	//act_tsk(SUB_TASK);
	char c;
	bool loop = true;
	while(loop){
	    serial_rea_dat(TASK_PORTID, &c, 1);
		switch(c){ //SUBSCRIBERのほうが優先度高い
		case 'p':
			act_tsk(PUB_TASK);
			break;
		case 's':
			act_tsk(SUB_TASK);
			break;
		case '1': //SUBSCRIBERのほうを高くする
			chg_pri(1,ROS_SUB_TASK_PRI);
			chg_pri(2,ROS_PUB_TASK_PRI);
			break;
		case '2': //PUBLISHERのほうを高くする
			chg_pri(2,ROS_SUB_TASK_PRI);
			chg_pri(1,ROS_PUB_TASK_PRI);
			break;
		case 'q':
			loop = false;
			ter_tsk(PUB_TASK);
			ter_tsk(SUB_TASK);
			break;
		case 'e':
			ter_tsk(PUB_TASK);
		default:
			break;
		}
	}
	syslog(LOG_NOTICE, "**********mROS FINISH***********");

}

void pub_task(){
#ifndef _PUB_
#define _PUB_
	xml2master(PUBLISHER);
	syslog(LOG_NOTICE, "========Activate mROS PUBLISHER========");
#endif //_PUB_

	node_server(40040); //xmlrpc受付
	node_server(40400);	//TCPの受付＆トピック出版
}

void sub_task(){
#ifndef _SUB_
#define _SUB_
	syslog(LOG_NOTICE, "========Activate mROS SUBSCRIBER========");
	xml2master(SUBSCRIBER);
#endif //_SUB_

	request_topic();
	rostcp();
	//rosfinish();
}




