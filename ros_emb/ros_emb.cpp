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
const char *m_ip = "192.168.0.20";	//ros master IP
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
	syslog(LOG_NOTICE, "**********Activate Main task**********");

//mROS通信ライブラリタスク起動
//初期化
	//act_tsk(PUB_TASK);
	act_tsk(SUB_TASK);
	//act_tsk(XML_SLV_TASK);
	act_tsk(XML_MAS_TASK);
//ユーザタスク起動
	act_tsk(USR_TASK);

	syslog(LOG_NOTICE,"**********mROS main task finish**********");
}

 /******************
 * パブリッシュタスク            ||
  ******************/
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
	dq = (intptr_t *)malloc(sizeof(char)*512);
	char *addr;
#endif 	//_PUB_
														//ソケットの接続受付をどうするか？周期的に見る？listenのタイミング
	while(1){
syslog(LOG_NOTICE,"PUB_TASK:enter loop");
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
			size = pdq[2];
			size = size + pdq[3]*256;
		//メモリアドレス どこまで必要かはわからんがとりあえず1B分の処理
		int offset = pdq[1];
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

/******************
* サブスクライブタスク        ||
 ******************/
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
		std::vector<intptr_t*> func_vec; 			//関数ポインタ
		int find(char c){
					for(int i=0;i < this->id_vec.size();i++){
						if(c == this->id_vec[i]){
							return i;
						}
					}
					return -1;
				}
		void add(TCPSocketConnection sock,char ID,intptr_t func){this->sock_vec.push_back(sock);this->id_vec.push_back(ID);this->func_vec.push_back(func);}
	};
	sub_list lst;
	intptr_t *dqp,*snd_dqp;
	dqp = (intptr_t *)malloc(sizeof(char)*4);	//レシーブ用領域
	char *sdq,*ip,*tmp,*buf,*addr; //data queue pointer,IP address,temporary buffer,receive buffer,memory address
	tmp = (char *)malloc(sizeof(char)*512);
	buf = (char *)malloc(sizeof(char)*512);
	int port;	//port number
#endif //_SUB_

	while(1){
syslog(LOG_NOTICE,"SUB_TASK:enter loop");
	//trcv_dtq(SUB_DTQ,dqp,0);	//登録用のデータレシーブ，待ち時間が0で，キューにパケットがなければスルー
		rcv_dtq(SUB_DTQ,dqp);	//テスト用
syslog(LOG_NOTICE,"SUB_TASK:receive dtq dqp [%8x]",*dqp);
		sdq = (char *)dqp;

		if(lst.find(sdq[0]) == -1){
//initialize
syslog(LOG_NOTICE,"SUB_TASK:new subscriber ID [%x]",sdq[0]);
			static TCPSocketConnection sock;
			intptr_t funcp;
			int size = sdq[2];
			size = size + sdq[3]*256;
			int offset = sdq[1];
			addr = &mem[offset];
syslog(LOG_NOTICE,"SUB_TASK:data size [%d]",size);
syslog(LOG_NOTICE,"SUB_TASK:memory offset [%d]",offset);
		//ソケットの接続と各種アドレスの取得を行う
		//必要なモノ：相手のURI，ポート，コールバック関数ポインタ，結果格納用共有メモリアドレス -> xml_mas_taskでやっちゃったほうがいいのでは？つながったソケットを返す？
			memcpy(tmp,addr,size);				//get date
			syslog(LOG_NOTICE,"SUB_TASK:get data [%s]",tmp);
		//関数ポインタの切り出し
			string str = tmp;
			funcp = (intptr_t)atoi(get_fptr(str).c_str());
			syslog(LOG_NOTICE,"SUB_TASK:fptr [%d]",funcp);
		//requestTopic送信
			str = "<methodCall>requestTopic</methodCall>";
			memcpy(mem,str.c_str(),str.size());
			char buf[3];
			buf[0] = sdq[0];
			buf[1] = 0;
			buf[2] = str.size();
			buf[3] = str.size()/256;
			snd_dqp = (intptr_t) &buf;
			syslog(LOG_NOTICE,"SUB_TASK:send dtq dq [%8x]",*snd_dqp);
			snd_dtq(XML_DTQ,*snd_dqp);
			rcv_dtq(SUB_DTQ,dqp);
			syslog(LOG_NOTICE,"SUB_TASK:receive dtq dq [%8x]",*dqp);
			sdq = (char *)dqp;
			size = sdq[2];
			size = size + sdq[3]*256;
			offset = sdq[1];
			addr = &mem[offset];
			memcpy(tmp,addr,size);				//get date
			syslog(LOG_NOTICE,"SUB_TASK:get data [%s]",tmp);
			str = tmp;
			port = atoi(get_port2(str).c_str());	//これはパブリッシャのポートを入れる とりあえず前に使ってた関数を入れてる
			//ip = get_ip(str).c_str();   //IPないときどうする
			ip = m_ip;
			syslog(LOG_NOTICE,"SUB_TASK:port [%d],IP [%s]",port,ip);
			sock.connect(ip,port);
		//TCPROSコネクションヘッダを送信
			int len = genSubTcpRosH(buf);	//TCPROSの生成もうまくやるように実装する
			sock.send(buf,len);
			/*テスト
			int c=0;
			while(1){
			sock.receive(buf,512);
			syslog(LOG_NOTICE,"SUB_TASK_LOOP:receive [%d][%s]",c,&buf[8]);
			c++;
			}
			*/
			sock.receive(buf,512);
			lst.add(sock,sdq[0],funcp);
		}
//initialize end

		//サブスクライブとコールバックの実行
		/*for(int i=0;i < lst.id_vec.size();i++){
			//データレシーブ
			lst.sock_vec[i].receive(buf,512);
			//コールバック関数実行
			syslog(LOG_NOTICE,"SUB_TASK:Callback [%s]",buf);
			}*/
		//slp_tsk();
		}
}


/******************
* XML-RPCスレーブタスク  ||
 ******************/
//周期ハンドラ回して確認する実装にする
void xml_slv_task(){
#ifndef _XML_SLAVE_
#define _XML_SLAVE_
	syslog(LOG_NOTICE,"========Activate mROS XML-RPC Slave========");
	TCPSocketConnection xml_slv_sock;
	TCPSocketServer xml_slv_srv;
	int port = 11411;
	if(xml_slv_srv.bind(port) == -1){
			syslog(LOG_NOTICE,"Failed bind");
			exit(1);
		}
#endif
	/*
	while(1){
		syslog(LOG_NOTICE,"XML_SLV_TASK:enter loop");
		svr.listen();
		syslog(LOG_NOTICE,"SERVER_INFO: Listening...");
		bool connect_status = false;
		syslog(LOG_NOTICE,"SERVER_INFO: Waiting...");
		if(svr.accept(csock) == 0){
			connect_status = true;
			syslog(LOG_NOTICE,"SERVER_INFO: Connected");
		}
	}
	*/
	slp_tsk();
}

/*****************************************
 * XML_MAS taskで管理するノード構造体                      *
 * ***************************************/
typedef struct node{
		bool node_type;						//true->subscriber false->publisher
		char ID;							//タスク用
		std::string topic_name;				//ROS用
		std::string topic_type;				//ROS
		std::string callerid;				//ROS
		std::string message_definition;		//ROS
		std::string uri;					//ROS
		std::string port;					//サブスクライバ用 パブリッシャのXML-RPCポートを格納
		std::string fptr;					//タスク用　intptr_tだとなんかうまくいかないので文字列として処理 使うところでintptr_tに戻す
public:
		void set_node_type(bool type){this->node_type=type;};
		void set_ID(char c){this->ID = c;};
		void set_topic_name(string t){this->topic_name=t;};
		void set_topic_type(string t){this->topic_type=t;};
		void set_callerid(string t){this->callerid=t;};
		void set_message_definition(string t){this->message_definition=t;};
		void set_uri(string t){this->uri=t;};
		void set_port(string t){this->port=t;};
		void set_fptr(string t){this->fptr=t;};
	}node;

/******************************************
 * ノードのメンバを入れる関数                                              *
 * (メンバ関数的にすればいいのかもしれない)         *
* ****************************************/
void get_node(node *node,std::string *xml,bool type){
	string c;
	syslog(LOG_NOTICE,"XML_MAS_TASK::get_node function body [%s]",xml->c_str());
	node->set_node_type(type);
	node->set_topic_type(get_ttype(*xml));
	syslog(LOG_NOTICE,"XML_MAS_TASK::get_node function::get_ttype [%s]",node->topic_type.c_str());
	node->set_topic_name(get_tname(*xml));
	syslog(LOG_NOTICE,"XML_MAS_TASK::get_node function::get_tname [%s]",node->topic_name.c_str());
	node->set_callerid(get_cid(*xml));
	syslog(LOG_NOTICE,"XML_MAS_TASK::get_node function::get_cid [%s]",node->callerid.c_str());
	node->set_message_definition(get_msgdef(*xml));
	syslog(LOG_NOTICE,"XML_MAS_TASK::get_node function::get_msgdef [%s]",node->message_definition.c_str());
	c += "http://";
	c += network.getIPAddress();
	c += ":11411";	//xml slaveのポートを追加
	node->set_uri(c);
	syslog(LOG_NOTICE,"XML_MAS_TASK::get_node function::get_uri [%s]",node->uri.c_str());

	//なんかうまく取れない…
	//intptr_tでやるとここで変な値が取れるのでstring処理
	if(type){
		node->set_fptr(get_fptr(*xml));
syslog(LOG_NOTICE,"XML_MAS_TASK::get_node function::get_fptr [%s]",node->fptr.c_str());
	}
}
/******************
* XML-RPCマスタタスク  ||
 ******************/
void xml_mas_task(){
#ifndef _XML_MASTER_
#define _XML_MASTER_
	syslog(LOG_NOTICE,"========Activate mROS XML-RPC Master========");
	TCPSocketConnection xml_mas_sock;
	xml_mas_sock.connect(m_ip,m_port);
	std::vector<node> node_lst;
	intptr_t *dq,*sdata;
	dq = (intptr_t *)malloc(sizeof(char)*4);	//データを受信するハコを用意する
	char data[3];
	char *xdq,*addr,*buf,*rbuf;
	rbuf = (char *)malloc(sizeof(char)*512);
	buf = (char *)malloc(sizeof(char)*512);
	int count=1;
#endif
	while(1){
syslog(LOG_NOTICE,"XML_MAS_TASK:enter loop");
		//usr_taskから呼ばれる，もしくはsub_taskから呼ばれる
		//あらかじめデータフィールドを決めておいて，TCPROSチックにやり取りする？ ｛[data length + data]*｝　みたいな
		//XMLを使った感じでいい？
		rcv_dtq(XML_DTQ,dq); //rcvはデータを受け取るポインタを渡す？
		//syslog(LOG_NOTICE,"XML_MAS_TASK: receive dtq data[%8x]",dq);
		//syslog(LOG_NOTICE,"XML_MAS_TASK: receive dtq data[%8x]",&dq);
syslog(LOG_NOTICE,"XML_MAS_TASK: receive dtq data[%8x]",*dq);	//中身取るならこれ
		xdq = (char *)dq;
		for(int i=0;i<4;i++){
syslog(LOG_NOTICE,"XML_MAS_TASK: xdq [%x]",xdq[i]);
			}
		int size = xdq[2];
		size = size + xdq[3]*256;
		int offset = xdq[1];
		addr = &mem[offset];
syslog(LOG_NOTICE,"XML_MAS_TASK:get size [%d]",size);
		memcpy(buf,addr,size);				//get date
syslog(LOG_NOTICE,"XML_MAS_TASK:get data [%s]",buf);
		//メソッドの識別
		std::string str,meth;
		str = buf;
		int mh,mt;
		mh = (int)str.find("<methodCall>");
		mt = (int)str.find("</methodCall>");
		for(int i = mh + sizeof("<methodCall>") -1;i < mt ; i++){
			meth = meth + str[i];
		}
syslog(LOG_NOTICE,"XML_MAS_TASK:method [%s]",meth.c_str());

		//メソッドごとに送信XMLの生成とノードの登録
//===registerSubscriber=========================================================================================================================
		if(meth == "registerSubscriber"){
			static node sub;
			string xml;
syslog(LOG_NOTICE,"XML_MAS_TASK:regiester Subscriber");
			sub.ID = (char)count;
			count++;
			get_node(&sub,&str,true);														//データを取得
			xml = registerSubscriber(sub.callerid,sub.topic_name,sub.topic_type,sub.uri); 	//ROSmaster用のXML生成
syslog(LOG_NOTICE,"XML_MAS_TASK:send data [%s]",xml.c_str());
			xml_mas_sock.send(xml.c_str(),strlen(xml.c_str()));						 		//ROSmasterにデータ送信
			xml_mas_sock.receive(rbuf,sizeof(char)*512);
			xml = rbuf;
syslog(LOG_NOTICE,"XML_MAS_TASK:receive data [%s]",xml.c_str());
			sub.set_port(get_port(xml));	//IPは考慮していない　PUBノードが立っていない場合ここでポートが返ってこない
syslog(LOG_NOTICE,"XML_MAS_TASK:port [%s]",sub.port.c_str());
			//sub.port = "14141";	//ポートが返ってこないので適当にいれる	※要：かえって来ないときの対応
			//subタスクにデータ送信
			node_lst.push_back(sub);
			xml = registerSubtask(sub.fptr,sub.port);
			memcpy(mem,xml.c_str(),strlen(xml.c_str()));
syslog(LOG_NOTICE,"XML_MAS_TASK:mem data [%s]",mem);
			data[0] = sub.ID;
			data[1] = 0; //とりあえず先頭から入れる
			data[2] = strlen(xml.c_str());
			data[3] = strlen(xml.c_str())/256;
			sdata = (intptr_t) &data;
syslog(LOG_NOTICE,"XML_MAS_TASK:send data [%8x]",*sdata);
			snd_dtq(SUB_DTQ,*sdata);	//subtaskに投げる

//===registerPublisher===========================================================================================================================
		}else if(meth == "registerPublisher"){
			node pub;
			std::string xml;
			pub.ID = count;
			count++;
syslog(LOG_NOTICE,"XML_MAS_TASK:regiester Publisher");
			get_node(&pub,&str,false);														//データ取得
			node_lst.push_back(pub);
			xml = registerPublisher(pub.callerid,pub.topic_name,pub.topic_type,pub.uri); 	//ROSmaster用のXML生成 uriにxml_slvのポートを入れないといけない->いれた
			xml_mas_sock.send(xml.c_str(),strlen(xml.c_str())); 							//ROSmasterにデータ送信
			xml_mas_sock.receive(rbuf,sizeof(char)*512);
syslog(LOG_NOTICE,"XML_MAS_TASK:receive data [%s]",rbuf);
			//pubタスクにデータ送信
			//xml = registerPubtask(); 何も送らなくていい 初期登録の場合，データ長を0で送信，IDだけ送る
			data[0] = pub.ID;
			data[1] = 0;
			data[2] = 0;
			data[3] = 0;
			sdata = (intptr_t) &data;
			syslog(LOG_NOTICE,"XML_MAS_TASK:send data [%8x]",*sdata);
			snd_dtq(PUB_DTQ,*sdata);	//pubtaskに投げる

//===requestTopic================================================================================================================================
		}else if(meth == "requestTopic"){	//sub taskからくるリクエストを投げる
syslog(LOG_NOTICE,"XML_MAS_TASK:send request topic");
			for(int i=0;i < node_lst.size();i++){
				if(node_lst[i].ID == xdq[0]){
syslog(LOG_NOTICE,"XML_MAS_TASK:request node ID [%x]",node_lst[i].ID);
					string body = requestTopic(node_lst[i].callerid,node_lst[i].topic_name);
syslog(LOG_NOTICE,"XML_MAS_TASK:body [%s]",body.c_str());
syslog(LOG_NOTICE,"XML_MAS_TASK:port [%d]",atoi(node_lst[i].port.c_str()));
					TCPSocketConnection tmpsock;
					if(tmpsock.connect(m_ip,atoi(node_lst[i].port.c_str())) == -1){
						exit(1);
					}
					tmpsock.send(body.c_str(),strlen(body.c_str()));
					tmpsock.receive(rbuf,sizeof(char)*512);
syslog(LOG_NOTICE,"XML_MAS_TASK:receive data [%s]",rbuf);
					memcpy(mem,rbuf,strlen(rbuf));
					data[0] = node_lst[i].ID;
					data[1] = 0; //とりあえず先頭から入れる
					data[2] = strlen(rbuf);
					data[3] = strlen(rbuf)/256;
					sdata = (intptr_t) &data;
syslog(LOG_NOTICE,"XML_MAS_TASK:send data [%8x]",*sdata);
					tmpsock.close();
					snd_dtq(SUB_DTQ,*sdata);
				}
			}

//===else========================================================================================================================================
		}else{	//なんかに使うかも

		}


	//slp_tsk(); //テスト用 スリープすることはないはず
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

//テスト用コード
	string str;
	str += "<methodCall>registerSubscriber</methodCall>\n";
	str += "<topic_name>/test_string</topic_name>\n";
	str += "<topic_type>std_msgs/String</topic_type>\n";
	str += "<caller_id>/mros_node</caller_id>\n";
	str += "<message_definition>std_msgs/String</message_definition>\n";
	str += "<fptr>12345671</fptr>\n";
	syslog(LOG_NOTICE,"USR_TASK: data [%s]",str.c_str());
	intptr_t *dq;
	memcpy(mem,str.c_str(),strlen(str.c_str()));
	syslog(LOG_NOTICE,"USR_TASK:length [%d]",strlen(str.c_str()));
	char buf[3];
	buf[0] = 0;		//IDここでは考えなくてよさそう
	buf[1] = 0;		//共有メモリオフセット
	int size = strlen(str.c_str());
	buf[2] = size;
	buf[3] = size/256;
	for(int i=0;i<4;i++){
	syslog(LOG_NOTICE,"USR_TASK: data [%x]",buf[i]);
	}
	dq = (intptr_t) &buf;
	syslog(LOG_NOTICE,"USR_TASK:data [%8x]",*dq);
	snd_dtq(XML_DTQ,*dq); //sndはデータ本体を渡す？big-little?なエンディアン 20b1->1b02で渡される
	slp_tsk();
}

