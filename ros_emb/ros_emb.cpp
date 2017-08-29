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
const char *m_ip = "192.168.0.21";	//ros master IP
const int m_port = 11311;	//ros master xmlrpc port

/*********グローバル変数***************/

/***** タスク間共有メモリ *****/
//sizeは未定
char *mem = (char *)malloc(sizeof(char)*2048);

/*****************************************
 * XML_MAS taskで管理するノード構造体                      *
 * ***************************************/
typedef struct node{
		bool node_type;						//true->subscriber false->publisher
		bool stat = true;
		char ID;							//タスク用
		std::string topic_name;				//ROS用
		std::string topic_type;				//ROS
		std::string callerid;				//ROS
		std::string message_definition;		//ROS
		std::string uri;					//ROS
		std::string port;					//サブスクライバ用 パブリッシャのXML-RPCポートを格納
		std::string fptr;					//タスク用　intptr_tだとなんかうまくいかないので文字列として処理 使うところでintptr_tに戻す
		TCPSocketConnection sock;			//タスク用　めんどくさいからもっとけ精神
public:
		void set_node_type(bool type){this->node_type=type;};
		void init(){this->stat = false;};
		void set_ID(char c){this->ID = c;};
		void set_topic_name(string t){this->topic_name=t;};
		void set_topic_type(string t){this->topic_type=t;};
		void set_callerid(string t){this->callerid=t;};
		void set_message_definition(string t){this->message_definition=t;};
		void set_uri(string t){this->uri=t;};
		void set_port(string t){this->port=t;};
		void set_fptr(string t){this->fptr=t;};
		void set_socket(TCPSocketConnection s){this->sock = s;};
		void connect(const char *IP,int port){this->sock.connect(IP,port);}
	}node;

/******ノードリスト***********/
std::vector<node> node_lst;

int find_node(vector<node> list,string topic){for(unsigned int i=0;i < list.size();i++){if(list[i].topic_name == topic){return i;}}return -1;}
int find_id(vector<node> list,char ID){for(unsigned int i=0;i < list.size();i++){if(list[i].ID == ID){return i;}}return -1;}
/* ネットワーク初期化 */
#define USE_DHCP (1)
#if(USE_DHCP == 0)
	#define IP_ADDRESS  	("192.168.0.10")	/*IP address */
	#define SUBNET_MASK		("255.0.0.0")	/*Subset mask */
	#define DEFAULT_GATEWAY	("")	/*Default gateway */
#endif

EthernetInterface network;


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

void main_task(){
	syslog(LOG_NOTICE, "**********mROS main task start**********");
	syslog(LOG_NOTICE, "LOG_INFO: network initialize...");
	network_init();
	syslog(LOG_NOTICE, "LOG_INFO: SUCCESS INITIALIZATION");
	syslog(LOG_NOTICE, "**********Activate Main task**********");

//mROS通信ライブラリタスク起動
//初期化
	act_tsk(PUB_TASK);
	act_tsk(SUB_TASK);
	act_tsk(XML_SLV_TASK);
	act_tsk(XML_MAS_TASK);
//ユーザタスク起動
	act_tsk(USR_TASK2);
	act_tsk(USR_TASK1);
	//syslog(LOG_NOTICE,"**********mROS Main task finish**********");
}


 /******************************************************************************************************************************************************************
 * パブリッシュタスク            ||
  ******************************************************************************************************************************************************************/
//mbedのEathernetConnectionはマルチキャストできないからポートに対して一つの接続
void pub_task(){
#ifndef _PUB_
#define _PUB_
syslog(LOG_NOTICE, "========Activate mROS PUBLISH========");
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
	TCPSocketServer srv; 					//TCPROSの入り口
	srv.bind(11511);								//ポートバインド：：とりあえず固定ポート11511
	pub_list lst;
	//1ワード -> 32bit -> 4B
	//pdq[0]      : mROSID -> 参照してリストかなんかになかったら初期化と判断
	//pdq[1]  : mem offset
	//pdq[2][3] : data length
	char *pdqp,*rbuf,*buf;
	intptr_t *dqp;	//たぶん32bitのはず(未確認)
	dqp = (intptr_t *)malloc(sizeof(char)*4);
	rbuf = (char *)malloc(sizeof(char)*1024);
	buf = (char *)malloc(sizeof(char)*1024);
	char *addr;
#endif 	//_PUB_
														//ソケットの接続受付をどうするか？周期的に見る？listenのタイミング
	while(1){
		//syslog(LOG_NOTICE,"PUB_TASK:enter loop");
		rcv_dtq(PUB_DTQ,dqp);							//ER ercd = rcv_dtq(ID dtqid, intptr_t *p_data) よくわからなi
		pdqp = (char *)dqp;
		int num = lst.find(pdqp[0]); 					//ID取得
		int size;
		size = pdqp[2];
		size = size + pdqp[3]*256;
		int offset = pdqp[1]*256;
		addr = 	&mem[offset];							//共有メモリのアドレスを入れる
		if(num == -1){ 									//初期化
			syslog(LOG_NOTICE,"PUB_TASK:publisher initialization");
			static TCPSocketConnection sock;
			lst.add(sock,pdqp[0]);
		}else if((num != -1) && (size == 0)){	//requestTopicがあるとき
			bool connect_status = false;
			//TCPROSを受け付ける
			//TCPSocketConnection sock2;
			syslog(LOG_NOTICE,"PUB_TASK:request Topic node[%x]",lst.id_vec[num]);
			srv.listen();
			syslog(LOG_NOTICE,"PUB_TASK: Listening...");
			syslog(LOG_NOTICE,"PUB_TASK: Waiting...");
			if(srv.accept(lst.sock_vec[num]) == 0){
				connect_status = true;
				char *snd_buf;
				snd_buf = (char *)malloc(512);
				while(connect_status){
					int stat = lst.sock_vec[num].receive(rbuf,512);
					switch(stat){
					case 0:
					case -1:
						connect_status = false;
						break;
					default:
						//node_listからいろいろ持ってきて送る
						int len = genPubTcpRosH(snd_buf);	//テスト用関数
						lst.sock_vec[num].send(snd_buf,len);
						syslog(LOG_NOTICE,"PUB_TASK: SEND HEADER");
						connect_status = false;
						wup_tsk(USR_TASK1);
						}
				}
			}
		}else{	//パブリッシュ(ユーザタスクから送られてくるデータ)
		//データコピー
		memcpy(rbuf,addr,size);
		rbuf[size] = '\0';	//データを切る
		int l = genMessage(buf,rbuf);
		//出版
		lst.sock_vec[num].send(buf,l);
		wup_tsk(USR_TASK2);
		}
	}
}

/******************************************************************************************************************************************************************************************************
* サブスクライブタスク        ||
 ************************************************************************************************************************************************************************************/
//ユーザタスクがほしいときにとりにくる実装
//周期ハンドラかな？
void sub_task(){
#ifndef _SUB_
#define _SUB_
syslog(LOG_NOTICE, "========Activate mROS SUBSCRIBE========");
	/*
	typedef struct sub_list{
	public:
		std::vector<TCPSocketConnection> sock_vec; 	//TCPソケット
		std::vector<char> id_vec; 					//mROSID
		std::vector<intptr_t> func_vec; 			//関数ポインタ
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
	*/
	intptr_t *dqp,*snd_dqp;
	dqp = (intptr_t *)malloc(sizeof(char)*4);	//レシーブ用領域
	char *sdq,*ip,*tmp,*addr,*buf; //data queue pointer,IP address,temporary buffer,receive buffer,memory address
	tmp = (char *)malloc(sizeof(char)*512);
	buf = (char *)malloc(sizeof(char)*512);
	int port;	//port number
#endif //_SUB_

	while(1){
		//syslog(LOG_NOTICE,"SUB_TASK:enter loop");
		slp_tsk();
	    int t = trcv_dtq(SUB_DTQ,dqp,1);	//登録用のデータレシーブ，待ち時間が0で，キューにパケットがなければスルー
	   // syslog(LOG_NOTICE,"SUB_TASK:ER [%d]",t);
	    //rcv_dtq(SUB_DTQ,dqp);	//テスト用
	    if(t == 0){
		//syslog(LOG_NOTICE,"SUB_TASK:receive dtq dqp [%8x]",*dqp);
		sdq = (char *)dqp;
		int num = find_id(node_lst,sdq[0]);
		if(node_lst[num].stat){
//initialize
			node_lst[num].init();
			//syslog(LOG_NOTICE,"SUB_TASK:new subscriber ID [%x]",sdq[0]);
			intptr_t funcp;
			int size = sdq[2];
			size = size + sdq[3]*256;
			int offset = sdq[1]*256;
			addr = &mem[offset];
			//syslog(LOG_NOTICE,"SUB_TASK:data size [%d]",size);
			//syslog(LOG_NOTICE,"SUB_TASK:memory offset [%d]",offset);
		//ソケットの接続と各種アドレスの取得を行う
		//必要なモノ：相手のURI，ポート，コールバック関数ポインタ，結果格納用共有メモリアドレス -> xml_mas_taskでやっちゃったほうがいいのでは？つながったソケットを返す？
			memcpy(tmp,addr,size);				//get date
			//syslog(LOG_NOTICE,"SUB_TASK:get data [%s]",tmp);
		//関数ポインタの切り出し
			string str = tmp;
			funcp = (intptr_t)atoi(get_fptr(str).c_str());
			//syslog(LOG_NOTICE,"SUB_TASK:fptr [%d]",funcp);
			//lst.add(sock,sdq[0],funcp);
		//requestTopic送信
			str = "<methodCall>requestTopic</methodCall>";
			memcpy(mem,str.c_str(),str.size());
			char buf[3];
			buf[0] = sdq[0];
			buf[1] = 0;
			buf[2] = str.size();
			buf[3] = str.size()/256;
			snd_dqp = (intptr_t) &buf;
			//syslog(LOG_NOTICE,"SUB_TASK:send dtq dq [%8x]",*snd_dqp);
			snd_dtq(XML_DTQ,*snd_dqp);
			rcv_dtq(SUB_DTQ,dqp);
			//syslog(LOG_NOTICE,"SUB_TASK:receive dtq dq [%8x]",*dqp);
			sdq = (char *)dqp;
			size = sdq[2];
			size = size + sdq[3]*256;
			offset = sdq[1]*256;
			addr = &mem[offset];
			memcpy(tmp,addr,size);				//get date
			//syslog(LOG_NOTICE,"SUB_TASK:get data [%s]",tmp);
			str = tmp;
			port = atoi(get_port2(str).c_str());	//これはパブリッシャのポートを入れる とりあえず前に使ってた関数を入れてる
			//ip = get_ip(str).c_str();   //IPないときどうする
			ip = m_ip;
			//syslog(LOG_NOTICE,"SUB_TASK:port [%d],IP [%s]",port,ip);
			node_lst[num].connect(ip,port);
			//sock.connect(ip,port);
		//TCPROSコネクションヘッダを送信
			int len = genSubTcpRosH(buf);	//TCPROSの生成もうまくやるように実装する
			node_lst[num].sock.send(buf,len);
			node_lst[num].sock.receive(buf,512);	//TCPROSヘッダの受信

			//ループを外に出したいけど出すと変なことになる
			syslog(LOG_NOTICE,"SUBSCRIBER LISTEN");
			while(1){
			slp_tsk();
			int n = node_lst[num].sock.receive(buf,512);
			buf[0] = buf[0] + '0';
			buf[1] = buf[1] + '0';
			buf[2] = buf[2] + '0';
			buf[3] = buf[3] + '0';
			buf[4] = buf[4] + '0';
			buf[5] = buf[5] + '0';
			buf[6] = buf[6] + '0';
			buf[7] = buf[7] + '0';
			if(n < 0){
				//syslog(LOG_NOTICE,"SUB_TASK: No data");
			}else{
				int len = buf[4] - '0';
				len = len + (buf[5] - '0') * 256;
				len = len + (buf[6] - '0') * 65536;
				len = len + (buf[7] - '0') * 16777216;
				char *rcv = &buf[8];
				rcv[len] = '\0';

				//コールバック関数
				void (*fp)(string);
				fp = atoi(node_lst[num].fptr.c_str());
				fp(rcv);
				}
			}
	    }
//end initialize
		//サブスクライブとコールバックの実行
		//syslog(LOG_NOTICE,"SUB_TASK:enter sub loop");
		/*
		for(unsigned int i=0;i < node_lst.size();i++){
			//データレシーブ
			syslog(LOG_NOTICE,"SUB_TASK:receive sub loop [%d]",i);
			if(node_lst[i].node_type){
				node_lst[i].sock.receive(buf,512);
				//コールバック関数実行
				syslog(LOG_NOTICE,"SUB_TASK:Callback [%s]",&buf[8]);
				}
			}*/
		//syslog(LOG_NOTICE,"SUB_TASK:sleep");

		}
}
}

/******************************************************************************************************************************************************************
* XML-RPCスレーブタスク  ||
 ******************************************************************************************************************************************************************/
//周期ハンドラ回して確認する実装にする
void xml_slv_task(){
#ifndef _XML_SLAVE_
#define _XML_SLAVE_
	syslog(LOG_NOTICE,"========Activate mROS XML-RPC Slave========");
	TCPSocketConnection xml_slv_sock;
	TCPSocketServer xml_slv_srv;
	xml_slv_srv.set_blocking(true,1000);	//ノンブロッキングにしてタイムアウトをつける? trueがいいのかfalseがいいのかわからない
	int port = 11411;
	char *rbuf;
	char data[3];
	string str,meth;
	intptr_t *dqp;
	bool connect_status = false;
	rbuf = (char *)malloc(sizeof(char)*1024);
	if(xml_slv_srv.bind(port) == -1){
			syslog(LOG_NOTICE,"Failed bind");
			exit(1);
		}
#endif

	while(1){
		//syslog(LOG_NOTICE,"XML_SLV_TASK:enter loop");
		xml_slv_srv.listen();
		//syslog(LOG_NOTICE,"XML_SLV_TASK: Listening...");
		//syslog(LOG_NOTICE,"XML_SLV_TASK: Waiting...");
		if(xml_slv_srv.accept(xml_slv_sock) == 0){ //acceptとかでタスクが待ち状態にならないか？
			connect_status = true;
			syslog(LOG_NOTICE,"XML_SLV_TASK: Connected");
			while(connect_status){
			int stat = xml_slv_sock.receive(rbuf,1024);
			switch(stat){
			case 0:
			case -1:
				connect_status = false;
				syslog(LOG_NOTICE,"XML_SLV_TASK:disconnected");
				break;
			default:
				//syslog(LOG_NOTICE,"XML_SLV_TASK:receive data [%s]",rbuf);
				str =rbuf;
				int mh,mt;
				//メソッドの切り出し
				mh = (int)str.find("<methodName>");
				mt = (int)str.find("</methodName>");
				for(int i = mh + sizeof("<methodName>") -1;i < mt ; i++){
					meth = meth + str[i];
				}
				//syslog(LOG_NOTICE,"XML_SLV_TASK:method [%s]",meth.c_str());

				//メソッドごとの処理
				if(meth == "requestTopic"){
					syslog(LOG_NOTICE,"XML_SLV_TASK:request Topic method");
					string topic_name = req_topic_name(str);
					int num = find_node(node_lst,topic_name);
					data[2] = 0;
					data[3] = 0;
					data[1] = 0;
					data[0] = node_lst[num].ID;
					dqp = (intptr_t) &data;
					string tmp;
					tmp = network.getIPAddress();
					str = test_requestResponse(tmp);				//テスト用関数
					xml_slv_sock.send(str.c_str(),str.size());
					connect_status = false;
					snd_dtq(PUB_DTQ,*dqp);
					xml_slv_sock.close();
					wup_tsk(USR_TASK2);
					}
				}
			}
		}
		slp_tsk();
	}
}


/************************************************************************************************************************************************************************
 * ノードのメンバを入れる関数                                              *
 * (メンバ関数的にすればいいのかもしれない)         *
* **********************************************************************************************************************************************************************/
void get_node(node *node,std::string *xml,bool type){
	string c;
	static TCPSocketConnection sock;
	//syslog(LOG_NOTICE,"XML_MAS_TASK::get_node function body [%s]",xml->c_str());
	node->set_node_type(type);
	node->set_topic_type(get_ttype(*xml));
	//syslog(LOG_NOTICE,"XML_MAS_TASK::get_node function::get_ttype [%s]",node->topic_type.c_str());
	node->set_topic_name(get_tname(*xml));
	//syslog(LOG_NOTICE,"XML_MAS_TASK::get_node function::get_tname [%s]",node->topic_name.c_str());
	node->set_callerid(get_cid(*xml));
	//syslog(LOG_NOTICE,"XML_MAS_TASK::get_node function::get_cid [%s]",node->callerid.c_str());
	node->set_message_definition(get_msgdef(*xml));
	//syslog(LOG_NOTICE,"XML_MAS_TASK::get_node function::get_msgdef [%s]",node->message_definition.c_str());
	c += "http://";
	c += network.getIPAddress();
	c += ":11411";	//xml slaveのポートを追加
	node->set_uri(c);
	//syslog(LOG_NOTICE,"XML_MAS_TASK::get_node function::get_uri [%s]",node->uri.c_str());
	node->set_socket(sock);
	//なんかうまく取れない…
	//intptr_tでやるとここで変な値が取れるのでstring処理
	if(type){
		node->set_fptr(get_fptr(*xml));
		//syslog(LOG_NOTICE,"XML_MAS_TASK::get_node function::get_fptr [%s]",node->fptr.c_str());
	}
}
/******************************************************************************************************************************************************************
* XML-RPCマスタタスク  ||
 ******************************************************************************************************************************************************************/
void xml_mas_task(){
#ifndef _XML_MASTER_
#define _XML_MASTER_
	syslog(LOG_NOTICE,"========Activate mROS XML-RPC Master========");
	intptr_t *dq,*sdata;
	dq = (intptr_t *)malloc(sizeof(char)*4);	//データを受信するハコを用意する
	char data[3];
	char *xdq,*addr,*buf,*rbuf;
	rbuf = (char *)malloc(sizeof(char)*512);
	buf = (char *)malloc(sizeof(char)*512);
	int count=1;
#endif
	while(1){
		TCPSocketConnection xml_mas_sock;	//呼ばれるたびに新しい口じゃないとダメだったような気がする
		xml_mas_sock.connect(m_ip,m_port);
		syslog(LOG_NOTICE,"XML_MAS_TASK:enter loop");
		//usr_taskから呼ばれる，もしくはsub_taskから呼ばれる
		//あらかじめデータフィールドを決めておいて，TCPROSチックにやり取りする？ ｛[data length + data]*｝　みたいな
		//XMLを使った感じでいい？
		rcv_dtq(XML_DTQ,dq); //rcvはデータを受け取るポインタを渡す？
		//syslog(LOG_NOTICE,"XML_MAS_TASK: receive dtq data[%8x]",dq);
		//syslog(LOG_NOTICE,"XML_MAS_TASK: receive dtq data[%8x]",&dq);
		//syslog(LOG_NOTICE,"XML_MAS_TASK: receive dtq data[%8x]",*dq);	//中身取るならこれ
		xdq = (char *)dq;
		for(int i=0;i<4;i++){
			//syslog(LOG_NOTICE,"XML_MAS_TASK: xdq [%x]",xdq[i]);
			}
		int size = xdq[2];
		size = size + xdq[3]*256;
		int offset = xdq[1]*256;
		addr = &mem[offset];
		//syslog(LOG_NOTICE,"XML_MAS_TASK:get addr [%d]",offset);
		memcpy(buf,addr,size);				//get date
		//syslog(LOG_NOTICE,"XML_MAS_TASK:get data [%s]",buf);
		//メソッドの識別
		std::string str,meth;
		str = buf;
		int mh,mt;
		mh = (int)str.find("<methodCall>");
		mt = (int)str.find("</methodCall>");
		for(int i = mh + sizeof("<methodCall>") -1;i < mt ; i++){
			meth = meth + str[i];
		}
		//syslog(LOG_NOTICE,"XML_MAS_TASK:method [%s]",meth.c_str());

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
			//syslog(LOG_NOTICE,"XML_MAS_TASK:send data [%s]",xml.c_str());
			xml_mas_sock.send(xml.c_str(),strlen(xml.c_str()));						 		//ROSmasterにデータ送信
			xml_mas_sock.receive(rbuf,sizeof(char)*512);
			xml = rbuf;
			//syslog(LOG_NOTICE,"XML_MAS_TASK:reg sub receive data [%s]",rbuf);
			sub.set_port(get_port(xml));	//IPは考慮していない　PUBノードが立っていない場合ここでポートが返ってこない
			syslog(LOG_NOTICE,"XML_MAS_TASK:port [%s]",sub.port.c_str());
			//sub.port = "14141";	//ポートが返ってこないので適当にいれる	※要：かえって来ないときの対応
			//subタスクにデータ送信
			node_lst.push_back(sub);
			xml = registerSubtask(sub.fptr,sub.port);
			memcpy(mem,xml.c_str(),strlen(xml.c_str()));
			//syslog(LOG_NOTICE,"XML_MAS_TASK:mem data [%s]",mem);
			data[0] = sub.ID;
			data[1] = 0; //とりあえず先頭から入れる
			data[2] = strlen(xml.c_str());
			data[3] = strlen(xml.c_str())/256;
			sdata = (intptr_t) &data;
			//syslog(LOG_NOTICE,"XML_MAS_TASK:send data [%8x]",*sdata);
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
			//syslog(LOG_NOTICE,"XML_MAS_TASK:reg pub receive data [%s]",rbuf);
			//pubタスクにデータ送信
			//xml = registerPubtask(); 何も送らなくていい 初期登録の場合，データ長を0で送信，IDだけ送る
			data[0] = pub.ID;
			data[1] = 0;
			data[2] = 0;
			data[3] = 0;
			sdata = (intptr_t) &data;
			//syslog(LOG_NOTICE,"XML_MAS_TASK:send data [%8x]",*sdata);
			snd_dtq(PUB_DTQ,*sdata);	//pubtaskに投げる

//===requestTopic================================================================================================================================
		}else if(meth == "requestTopic"){	//sub taskからくるリクエストを投げる
			syslog(LOG_NOTICE,"XML_MAS_TASK:send request topic");
			for(int i=0;i < node_lst.size();i++){
				if(node_lst[i].ID == xdq[0]){
					//syslog(LOG_NOTICE,"XML_MAS_TASK:request node ID [%x]",node_lst[i].ID);
					string body = requestTopic(node_lst[i].callerid,node_lst[i].topic_name);
					//syslog(LOG_NOTICE,"XML_MAS_TASK:body [%s]",body.c_str());
					//syslog(LOG_NOTICE,"XML_MAS_TASK:port [%d]",atoi(node_lst[i].port.c_str()));
					TCPSocketConnection tmpsock;
					if(tmpsock.connect(m_ip,atoi(node_lst[i].port.c_str())) == -1){
						exit(1);
					}
					tmpsock.send(body.c_str(),strlen(body.c_str()));
					tmpsock.receive(rbuf,sizeof(char)*512);
					//syslog(LOG_NOTICE,"XML_MAS_TASK:receive data [%s]",rbuf);
					memcpy(mem,rbuf,strlen(rbuf));
					data[0] = node_lst[i].ID;
					data[1] = 0; //とりあえず先頭から入れる
					data[2] = strlen(rbuf);
					data[3] = strlen(rbuf)/256;
					sdata = (intptr_t) &data;
					//syslog(LOG_NOTICE,"XML_MAS_TASK:send data [%8x]",*sdata);
					tmpsock.close();
					snd_dtq(SUB_DTQ,*sdata);
				}
			}

//===else========================================================================================================================================
		}else{	//なんかに使うかも

		}

		xml_mas_sock.close();
	//slp_tsk(); //テスト用 スリープすることはないはず
	} //end while loop
}

//周期ハンドラ
//XML-RPCスレーブタスクとサブスクライブタスクを起動させる？
void cyclic_handler(intptr_t exinf){

	iwup_tsk(SUB_TASK); 	//周期ハンドラからはiwup_tsk()で起こす．wup_tsk()だとコンテキストエラー
	iwup_tsk(XML_SLV_TASK);
	//syslog(LOG_NOTICE,"CYCLIC_HANDLER:wake up xml slv task ER [%d]",n);
}







/*----------------------------------------------------------------------------------------------------------------------------------------------
 * ユーザ定義領域
 * Subscriber　:LEDのカラーデータをstd_msgs/Stringで購読し，コールバックで光らせる
 * node:mros_node,toipic_name:/test_string
 * Publisher  :超音波センサのデータを取得して，std_msgs/Stringで出版
 * node:mros_node2,topic_name:/mros_msg
 */

static DigitalOut ledu(P6_12);                                  // LED-User
static SoftPWM ledr(P6_13);                                     // LED-Red
static SoftPWM ledg(P6_14);                                     // LED-Green
static SoftPWM ledb(P6_15);                                     // LED-Blue

void led_init(){
	ledu = 0;
	ledr.period_ms(10);
	ledr = 0.0f;
	ledg.period_ms(10);
	ledg = 0.0f;
	ledb.period_ms(10);
	ledb = 0.0f;
}

static DigitalOut USSTriger (P2_14);         //P11 :超音波センサ トリガ出力
Timer ActiveTime;
/* 割り込み処理宣言 */
Ticker TrigerTiming;                //Trigerピン :インターバルタイマ
static InterruptIn USSEcho (P2_15);          //p12 :超音波センサ  エコー入力
unsigned short USSDistance;         //USSDistance:超音波センサ測定距離
static DigitalIn Button (P6_0);		//ユーザボタン

void Triger (){
    USSTriger = 1;
    wait_us(10);
    USSTriger = 0;
}

void RiseEcho(){
    ActiveTime.start();
}

void FallEcho(){
    unsigned long ActiveWidth;
    ActiveTime.stop();
    ActiveWidth = ActiveTime.read_us();
    USSDistance = ActiveWidth * 0.0170;
    ActiveTime.reset();
}

void sensor_init(void){
    TrigerTiming.attach( Triger , 0.060 );      //USSTriger周期 60ms
    USSEcho.rise( RiseEcho );                   //USSEcho立ち上がり時割り込み
    USSEcho.fall( FallEcho );                   //USSEcho立ち下がり時割り込み
}


//コールバック関数
void Callback(string *msg){	//ConstPtr& msgの意味が分からん
	syslog(LOG_NOTICE,"Call back: I heard: [%s]", msg->c_str()); //ROS＿INFOだとなんかうまくいかない

	//Lチカ
	//LED云々するプログラム
	//loopを入れるかどうかあとで
	if(msg->find("red") != -1){
		if(ledr == 0){
			ledr = (float)100/128;//LED RED
		}else{
			ledr = 0;
		}
	}else if(msg->find("blue") != -1){
		if(ledb == 0){
			ledb = (float)100/128;		//LED BLUE
		}else{
			ledb = 0;
		}
	}else if(msg->find("green") != -1){
		if(ledg == 0){
			ledg = (float)100/128;		//LED GREEN
		}else{
			ledg = 0;
		}
	}else if(msg->find("reset") != -1){
		ledr = 0;
		ledg = 0;
		ledb = 0;
	}else if(msg->find("end") != -1){
		syslog(LOG_NOTICE,"GOOD BYE !!");
		exit(1);
	}
}


void usr_task1(){

#ifndef _USR_TASK_
#define _USR_TASK_

	syslog(LOG_NOTICE,"========Activate user task1========");
	//テストコード
	slp_tsk();	//現状sleep　セマフォとかでブロック
	led_init();
	int argc = 0;
	char *argv = NULL;
	ros::init(argc,argv,"/mros_node");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscriber("test_string",1,Callback);
	ros::spine();
#endif
}

void usr_task2(){
#ifndef _USR_TASK_2_
#define _USR_TASK_2_

	syslog(LOG_NOTICE,"========Activate user task2========");
	char *data;
	data = (char *)malloc(sizeof(char)*64);
	int argc = 0;
	char *argv = NULL;
	ros::init(argc,argv,"mros_node2");
	ros::NodeHandle n;			//ノードハンドラ
	ros::Publisher chatter_pub = n.advertise("mros_msg", 1);		//パブリッシャとして登録XML-RPCをする
	ros::Rate loop_rate(500);
	slp_tsk();	//現状スリープしないといけない…セマフォとかでブロックしてやる必要がある**************************************
	//publish loop
	sensor_init();
	bool b = true;
	bool bb = true;
	while(1){
		if(Button.read() == 0 && bb){
			b = !b;
			bb = false;
		}else if(Button.read() == 1){
			bb = true;
		}
		if(b){
			sprintf(data,"Sonic Sensor Distance [%d]cm\0",USSDistance);
			chatter_pub.publish(data);			//TCPROSでデータ出版
			//ros::spineOnce();
			loop_rate.sleep();
		}
	}

#endif
}



