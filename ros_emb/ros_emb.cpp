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
bool node_type;
/***** congiuration ros master ******/
const char *m_ip = "192.168.0.15";	//ros master IP
const int m_port = 11311;	//ros master xmlrpc port

/***** タスク間共有メモリ *****/
//sizeは未定
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


/******************************************************************************************************************************************
 * ここから過去の遺産
 * ****************************************************************************************************************************************
//マスタへのソケット
TCPSocketConnection mas_sock;

int n_port,tcp_port;

//マスタとのXML-RPC通信クライアント
void xml2master(bool mode){
	string xml;
	if(mas_sock.connect(m_ip,m_port) == -1){
			exit(1);
		}
	//network.getIPAddress();からIPを自動でとる
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

 * ****************************************************************************************************************************************
 * ここまで過去の遺産
 * ****************************************************************************************************************************************/

void main_task(){
	syslog(LOG_NOTICE, "**********mROS main task start**********");
	syslog(LOG_NOTICE, "LOG_INFO: network initialize...");
	network_init();
	syslog(LOG_NOTICE, "LOG_INFO: SUCCESS INITIALIZATION");
	syslog(LOG_NOTICE, "Activated Main task");

//mROS通信ライブラリタスク起動
//初期化
	act_tsk(PUB_TASK);
	//act_tsk(SUB_TASK);
	act_tsk(XML_SLV_TASK);
	act_tsk(XML_MAS_TASK);
//ユーザタスク起動
	act_tsk(USR_TASK);

	syslog(LOG_NOTICE,"**********mROS main task finish**********");
}

//パブリッシュタスク
//mbedのEathernetConnectionはマルチキャストできないからポートに対して一つの接続
void pub_task(){
#ifndef _PUB_
#define _PUB_
	syslog(LOG_NOTICE, "========Activate mROS PUBLISH========");
	/*struct mros_pair { 							//IDとソケットのペアを考えたけど元からベクトル2本持てばいいことに気づいた
		char ID;
		TCPSocketServer *p_sock;					//サーバの口は1つでよくてパブのソケットが複数いる
		mros_pair(char c,TCPSocketServer *s){
			ID = c;
			p_sock = s;
		}	
	};
	*/
	struct pub_list{
	public:
		std::vector<TCPSocketConnection> sock_vec;
		std::vector<char> id_vec;
		int find(char c){
			for(int i=0;i < this->id_vec.size();i++){
				if(c == this->id_vec[i]){
					return i;
				}
			}
			return -1;
		}
		void add(TCPSocketConnection sock,char ID){this->sock_vec.push_back(sock);this->id_vec.push_back(ID);} //ペア追加
		void del(int num){this->sock_vec.erase(this->sock_vec.begin() + num - 1);this->id_vec.erase(this->id_vec.begin() + num - 1);}
	};
	static TCPSocketServer *sock; 					//サーバの入り口
	sock->bind(11411);								//ポートバインド：：とりあえず固定ポート11411
	static pub_list lst;
	//1ワード -> 32bit -> 4B
	//pdq[0]      : mROSID -> 参照してリストかなんかになかったら初期化と判断
	//pdq[1]  : address 
	//pdq[2][3] : data length
	char *pdq;
	//pdq = (char*)malloc(sizeof(char)*4);
	intptr_t *dq;	//たぶん32bitのはず(未確認)
	char *addr;
#endif 	//_PUB_
														//ソケットの接続受付をどうするか？周期的に見る？listenのタイミング
	while(1){
		rcv_dtq(PUB_DTQ,dq);							//ER ercd = rcv_dtq(ID dtqid, intptr_t *p_data) よくわからなi
		pdq = (char *)dq;
		int num = lst.find(pdq[0]); 					//ID取得
		if(num == -1){ 									//初期化
			static TCPSocketConnection sock;
			lst.add(sock,pdq[0]);
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
		addr = 	&mem[offset];							//共有メモリのアドレスを入れる
		//データコピー
		char *buff;
		memcpy(buff,addr,size);
		//出版
		lst.sock_vec[num].send(buff,size);
		}
	}
}

//サブスクライブタスク
//ユーザタスクがほしいときにとりにくる実装
//周期ハンドラかな？
void sub_task(){
#ifndef _SUB_
#define _SUB_
	syslog(LOG_NOTICE, "========Activate mROS SUBSCRIBE========");
	typedef struct sub_list{
	public:
		std::vector<TCPSocketConnection> sock_vec; 	//TCPソケット
		std::vector<char> id_vec; 					//mROSID
		std::vector<intptr_t*> addr_vec; 				//共有メモリアドレス
		std::vector<intptr_t*> func_vec; 			//関数ポインタ
		int find(char c){
					for(int i=0;i < this->id_vec.size();i++){
						if(c == this->id_vec[i]){
							return i;
						}
					}
					return -1;
				}
		void add(TCPSocketConnection sock,char ID,intptr_t addr,intptr_t func){this->sock_vec.push_back(sock);this->id_vec.push_back(ID);this->addr_vec.push_back(addr);this->func_vec.push_back(func);}
	};
	sub_list lst;
	intptr_t *dq;
	char *sdq,*ip,*tmp,*buf,*addr; //data queue pointer,IP address,temporary buffer,receive buffer,memory address
	int port;	//port number
#endif //_SUB_

	while(1){
		trcv_dtq(SUB_DTQ,dq,0);
		sdq = (char *)dq;
		//初期登録の確認
		//ID，コールバックの関数ポインタ，共有メモリアドレス，ソケット
		if(lst.find(atoi(sdq[0])) == -1){
			static TCPSocketConnection sock;
			intptr_t *funcp;
			int size = atoi(sdq[2]);
			size = size + atoi(sdq[3])*256;
			int offset = atoi(sdq[1]);
			addr = &mem[offset];
			//ソケットの接続と各種アドレスの取得を行う
			//必要なモノ：相手のURI，ポート，コールバック関数ポインタ，結果格納用共有メモリアドレス -> xml_mas_taskでやっちゃったほうがいいのでは？つながったソケットを返す？
			memcpy(tmp,addr,size);				//get date
			//IP,port,関数ポインタの切り出し

			//
			sock.connect(ip,port);
			lst.add(sock,sdq[0],addr,funcp);
		}
		//サブスクライブとコールバックの実行
		for(int i=0;i < lst.id_vec.size();i++){
			//データレシーブ
			lst.sock_vec[i].receive(buf,2048);
			//コールバック関数実行

			}
		slp_tsk();
		}

	/*
	request_topic();
	rostcp();
	*/
}


//XML-RPCスレーブタスク
//周期ハンドラ回して確認する実装にする
void xml_slv_task(){
#ifndef _XML_SLAVE_
#define _XML_SLAVE_
	syslog(LOG_NOTICE,"========Activate mROS XML-RPC Slave========");
	TCPSocketConnection xml_slv_sock;
	TCPSocketServer xml_slv_srv;
	int port = 11411;
#endif
	slp_tsk();
}

typedef struct node{
		bool node_type;			//true->subscriber false->publisher
		char ID;				//タスク用
		std::string topic_name;		//ROS用
		std::string topic_type;
		std::string callerid;
		std::string message_definition;
		std::string uri;
		std::string port;
		intptr_t func;			//タスク用
	};

void get_node(node *node,std::string xml,bool type){
	char *c;
	if(type){
		node->node_type = type;
		node->topic_type = get_ttype(&xml);
		node->topic_name = get_tname(&xml);
		node->callerid = get_cid(&xml);
		node->message_definition = get_msgdef(&xml);
		sprintf(c,"http://%s",network.getIPAddress());
		node->uri = c;
		node->func = get_faddr(&xml);
	}else{
		node->node_type = type;
		node->topic_type = get_ttype(&xml);
		node->topic_name = get_tname(&xml);
		node->callerid = get_cid(&xml);
		node->message_definition = get_msgdef(&xml);
		sprintf(c,"http://%s",network.getIPAddress());
		node->uri = c;
	}
}

//XML-RPCマスタタスク
void xml_mas_task(){
#ifndef _XML_MASTER_
#define _XML_MASTER_
	syslog(LOG_NOTICE,"========Activate mROS XML-RPC Master========");
	TCPSocketConnection xml_mas_sock;
	xml_mas_sock.connect(m_ip,m_port);
	std::vector<node> node_lst;
	intptr_t dq,sdata;
	char data[3];
	char *xdq,*buf,*addr,*rbuf;
	rbuf = (char *)malloc(sizeof(char)*512);
#endif
	while(1){
		//usr_taskから呼ばれる，もしくはsub_taskから呼ばれる
		//あらかじめデータフィールドを決めておいて，TCPROSチックにやり取りする？ ｛[data length + data]*｝　みたいな
		//XMLを使った感じでいい？
		rcv_dtq(XML_DTQ,dq);
		syslog(LOG_NOTICE,"XML_MAS_TASK: receive dtq data[%s]",dq);
		xdq = (char *)dq;
		int size = atoi(xdq[2]);
		size = size + atoi(xdq[3])*256;
		int offset = atoi(xdq[1]);
		addr = &mem[offset];
		memcpy(buf,addr,size);				//get date
		//メソッドの識別
		std::string str,meth;
		str = buf;
		int mh,mt;
		mh = (int)str.find("<methodCall>");
		mt = (int)str.find("</methodCall>");
		for(int i = mh + sizeof("<methodCall>") -1;i < mt ; i++){
			meth = meth + str[i];
		}

		//メソッドごとによる送信XMLの生成
		if(meth == "registerSubscriber"){
			node sub;
			std::string xml;
			get_node(&sub,str,true);														//データを取得
			node_lst.push_back(sub);
			xml = registerSubscriber(sub.callerid,sub.topic_name,sub.topic_type,sub.uri); 	//ROSmaster用のXML生成
			xml_mas_sock.send(xml.c_str(),strlen(xml.c_str()));						 		//ROSmasterにデータ送信
			xml_mas_sock.receive(rbuf,sizeof(char)*512);
			syslog(LOG_NOTICE,"XML_MAS_TASK:receive data [%s]",rbuf);
			sub.port = get_port(rbuf);	//IPは考慮していない
			//subタスクにデータ送信
			xml = registerSubtask(sub.func,sub.port);
			memcpy(mem,xml.c_str(),strlen(xml.c_str()));
			data[0] = sub.ID;
			data[1] = "0"; //とりあえず先頭から入れる
			data[2] = strlen(xml.c_str());
			data[3] = strlen(xml.c_str())/256;
			sdata = data;
			syslog(LOG_NOTICE,"XML_MAS_TASK:send data [%d]",sdata);
			snd_dtq(SUB_DTQ,sdata);
		}else if(meth == "registerPublisher"){
			node pub;
			std::string xml;
			get_node(&pub,str,false);														//データ取得
			pub.uri += ":11411";
			node_lst.push_back(pub);
			xml = registerPublisher(pub.callerid,pub.topic_name,pub.topic_type,pub.uri); 	//ROSmaster用のXML生成 uriにxml_slvのポートを入れないといけない
			xml_mas_sock.send(xml.c_str(),strlen(xml.c_str())); 							//ROSmasterにデータ送信
			//pubタスクにデータ送信

		}else if(str == "requestTopic"){

		}else{

		}

	} //end while loop
}

//周期ハンドラ
//XML-RPCスレーブタスクとサブスクライブタスクを起動させる？
void cyclic_handler(intptr_t exinf){
	wup_tsk(SUB_TASK);
	wup_tsk(XML_SLV_TASK);
}

void Callback(const std_msgs::String::ConstPtr *msg){	//ConstPtr& msgの意味が分からん
	ros_info("I heard: [%s]", msg->data.c_str()); //ROS＿INFOだとなんかうまくいかない
}


void usr_task1(){
#ifndef _USR_TASK_
#define _USR_TASK_
	syslog(LOG_NOTICE,"========Activate user task========");
#endif
	//テストコード
	/*
	int argc = 0;
	char *argv = NULL;
	ros::init(argc,argv,"/mros_node");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscriber("test_string",10,Callback);
	*/
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
	string str;
	str += "<methodCall>registerSubscriber</methodCall>";
	str += "<ID>01</ID>";
	str += "<topic_name>/test_string</topic_name>";
	str += "<topic_type>std_msgs/String</topic_type>";
	str += "<caller_id>/mros_node</caller_id>";
	str += "<message_definition>std_msgs/String</message_definition>";
	str += "<function_pointer>192.168.11</function_pointer>";
	syslog(LOG_NOTICE,"USR_TASK: data [%s]",str.c_str());
	intptr_t dq = 0;
	memcpy(mem,str.c_str(),strlen(str.c_str()));
	char buf[3];
	buf[0] = 1;
	buf[1] = 0;
	int size = strlen(str.c_str());
	buf[2] = size;
	buf[3] = size/256;
	for(int i=0;i<4;i++){
	syslog(LOG_NOTICE,"USR_TASK: data [%x]",buf[i]);
	}

	syslog(LOG_NOTICE,"USR_TASK:data [%x]",&buf);
	snd_dtq(XML_DTQ,&buf);
}

