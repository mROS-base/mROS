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

/***** congiuration ros master ******/
const char *m_ip = "192.168.0.15";	//ros master IP
const int m_port = 11311;	//ros master xmlrpc port

/***** タスク間共有メモリ *****/
char *mem = (char *)malloc(sizeof(char)*2048);

/* ネットワーク初期化 */
#define USE_DHCP (1)
#if(USE_DHCP == 0)
	#define IP_ADDRESS  	("192.168.0.10")	/*IP address */
	#define SUBNET_MASK		("255.0.0.0")	/*Subset mask */
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

int n_port,tcp_port;

//マスタとのXML-RPC通信クライアント
void xml2master(bool mode){
	string xml;
	if(mas_sock.connect(m_ip,m_port) == -1){
			exit(1);
		}
	if(!mode){
		xml = registerPublisher("/mros_node","/mros_msg","std_msgs/String","http://192.168.10:40040");
	}else{
		xml = registerSubscriber("/mros_node","/test_string","std_msgs/String","http://192.168.10");
	}
	char *snd_buff;
	snd_buff = (char *)malloc(1024*sizeof(char));
	strcpy(snd_buff,xml.c_str());
	int n;
	n = mas_sock.send(snd_buff,strlen(snd_buff));
	free(snd_buff);
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
		free(snd_buff);
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
			free(rcv_buff);
		}
}

//デモ用

void rostcp(){
	//TCPROSを行ってデータを受信するところ
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
	rcv_p = &rcv_buff[8]; //tcprosのヘッダの部分は避ける 詳しくはETWESTブランチを見る
    free(rcv_buff);
}


void main_task(){
	syslog(LOG_NOTICE, "**********mROS main task start**********");
	syslog(LOG_NOTICE, "LOG_INFO: network initialize...");
	network_init();
	syslog(LOG_NOTICE, "LOG_INFO: SUCCESS INITIALIZATION");
	syslog(LOG_NOTICE, "Activated Main task");

//mROS通信ライブラリタスク起動
//初期化
	act_task(pub_task);
	act_task(sub_task);
	act_task(xml_slv_task);
	act_task(xml_mas_task);
//ユーザタスク起動
	act_task(usr_task1);

	syslog(LOG_NOTICE,"**********mROS main task finish**********")
}

//パブリッシュタスク
//mbedのEathernetConnectionはマルチキャストできないからポートに対して一つの接続
void pub_task(){
#ifndef _PUB_
#define _PUB_
	syslog(LOG_NOTICE, "========Activate mROS PUBLISH========");
	struct mros_pair {
		char ID;
		TCPSocketServer *p_sock;	
		mros_pair(char c,TCPSocketServer *s){
			ID = c;
			p_sock = s;
		}	
	};
	struct mros_list{
	private:
		std::vetcor<mros_pair> mros_vec;
	public:
		int find(char c){
			for(int i=0;i < this->mros_vec.size();i++){
				if(c == this->mros_vec[i].ID){
					return i;
				}
			}
			return -1;
		}
		void add(mros_pair){this->mros_vec.push_back(mros_pair);}
	};
	static mros_list lst;
	//1ワード -> 32bit -> 4B
	//pdq[0]      : mROSID -> 参照してリストかなんかになかったら初期化と判断
	//pdq[1]  : address 
	//pdq[2][3] : data length ->初期化の場合はポート番号を入れる
	static char *pdq;
	pdq = (char*)malloc(sizeof(char)*4);
	char *addr;
#endif //_PUB_
	//ソケットの接続受付をどうするか？周期的に見る？
	while(1){
		rcv_dtq(PUB_DTQ,pdq);		//ER ercd = rcv_dtq(ID dtqid, intptr_t *p_data)
		int num = lst.find(pdq[0]); 	//ID取得
		if(num == -1){ //初期化
		//指定されたポート番号を開通させ，mROSIDと紐づけ
			int port;
			port = atoi(pdq[2]);
			port = size + atoi(pdq[3])*256;
			static TCPSocketServer *sock;
			sock.bind(port);	//ポートバインド
			lst.add(pdq[0],sock);
		}else{	//パブリッシュ
		//mROSIDに紐づけられたポートで出版
		//データ長
		int size;
			size = atoi(pdq[2]);
			size = size + atoi(pdq[3])*256;
		//メモリアドレス どこまで必要かはわからんがとりあえず1B分の処理
		int offset = atoi(pdq[1]);
		//offset = offset + atoi(pdq[2])*256;
		//offset = offset + atoi(pdq[3])*65536;
		//offset = offset + atoi(pdq[4])*16777216
		addr = 	&mem[offset];		//共有メモリのアドレスを入れる
		//データコピー
		char *buff;
		memcpy(buff,addr,size);
		//出版
		lst.mros_vec[num].p_sock.send(buff,size);
		}
	}
	/*
	node_server(40040); //xmlrpc受付
	node_server(40400);	//TCPの受付＆トピック出版
	*/
}

//サブスクライブタスク
//ユーザタスクがほしいときにとりにくる実装
void sub_task(){
#ifndef _SUB_
#define _SUB_
	syslog(LOG_NOTICE, "========Activate mROS SUBSCRIBE========");
	
#endif //_SUB_
	request_topic();
	rostcp();
}

//XML-RPCスレーブタスク
//周期ハンドラ回して確認する実装にする
void xml_slv_task(){
#ifndef _XML_SLAVE_
#define _XML_SLAVE_
	syslog(LOG_NOTICE,"========Activate mROS XML-RPC Slave========");
#endif
}

//XML-RPCマスタタスク
void xml_mas_task(){
#ifndef _XML_MASTER_
#define _XML_MASTER_
	syslog(LOG_NOTICE,"========Activate mROS XML-RPC Master========");
#endif
	while(1){
		//register
		if(node_type == PUBLISHER){
			xml2master(PUBLISHER);
		}else if(node_type == SUBSCRIBER){
			xml2master(SUBSCRIBER);
		}
	}
}

void usr_task1(){
#ifndef _USR_TASK_
#define _USR_TASK_
	syslog(LOG_NOTICE,"========Activate user task========");
#endif
	//ros::init,nodeHandleとかを定義しないといけない別ファイル
	//共有メモリ先頭ポインタ mem
/****	参考コード
	ros::init(argc,argv,"mros_talker");
	ros::NodeHandel n;			//ノードハンドラ
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);		//パブリッシャとして登録XML-RPCをする
	ros::Rate loop_rate(10);
	while(ros::ok()){
		chatter_pub.publish(data);			//TCPROSでデータ出版
		ros::spineOnce();
		loop_rate.sleep();
	}
*/
}
