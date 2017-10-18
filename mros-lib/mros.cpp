#include "mros.h"

#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include "syssvc/serial.h"
#include "syssvc/syslog.h"
#include "kernel_cfg.h"
#include "mbed.h"
#include "EthernetInterface.h"
#include "syssvc/logtask.h"


/***** congiuration ros master ******/
const char *m_ip = "192.168.11.4";	//ros master IP
const int m_port = 11311;	//ros master xmlrpc port

/*********グローバル変数***************/

/***** タスク間共有メモリ *****/
//sizeは未定
//char *mem = (char *)malloc(sizeof(char)*1024*1024*2);
char mem[1024*1024*2];
extern std::vector<ID> IDv;
int ros_sem =0;	//mROSの登録資源セマフォ
int count=1;	//ノードのID割り当てカウンタ



/******* ユーザタスクをいろいろする関数 ******/
void sus_all(){
	for(int i =0;i < IDv.size();i++){
		sus_tsk(IDv[i]);
	}
}

void rsm_all(){
	for(int i=0;i < IDv.size();i++){
		int l = rsm_tsk(IDv[i]);
	}
	ros_sem = 0;
}

/*****************************************
 * XML_MAS taskで管理するノード構造体        *
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
}node;

/******mROSノードリスト***********/
std::vector<node> node_lst;

int find_node(vector<node> list,string topic){for(unsigned int i=0;i < list.size();i++){if(list[i].topic_name == topic){return i;}}return -1;}
int find_id(vector<node> list,char ID){for(unsigned int i=0;i < list.size();i++){if(list[i].ID == ID){return i;}}return -1;}

/***評価用時間変数***/
SYSUTM time1;
SYSUTM time2;

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



/*********************************
メインタスク
***********************************/

void main_task(){
	syslog(LOG_NOTICE, "**********mROS main task start**********");
	syslog(LOG_NOTICE, "LOG_INFO: network initialize...");
	network_init();
	syslog(LOG_NOTICE, "LOG_INFO: SUCCESS INITIALIZATION");
	syslog(LOG_NOTICE, "**********Activate Main task**********");

//mROS通信ライブラリタスク起動
//初期化
	syslog(LOG_NOTICE,"MAIN_TASK:global variables mem[%d],ros_sem[%d]",&mem,ros_sem);
	act_tsk(PUB_TASK);
	act_tsk(SUB_TASK);
	act_tsk(XML_SLV_TASK);
	act_tsk(XML_MAS_TASK);
	//	syslog(LOG_NOTICE,"**********mROS Main task finish**********");
}


 /******************************************************************************************************************************************************************
 * パブリッシュタスク            ||
  ******************************************************************************************************************************************************************/
//mbedのEathernetConnectionはマルチキャストできないからポートに対して一つの接続
//SocketServerを複数用意すればなんとかなるっちゃなる
void pub_task(){
#ifndef _PUB_
#define _PUB_
syslog(LOG_NOTICE, "========Activate mROS PUBLISH========");
	//PUB_TASKが管理するソケット情報
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
	srv.bind(11511);		//ポートバインド：：とりあえず固定ポート11511
	pub_list lst;
	char *pdqp,*rbuf,*buf;
	intptr_t *dqp;	//たぶん32bitのはず(未確認)
	dqp = (intptr_t *)malloc(sizeof(char)*4);
	buf = (char *)malloc(sizeof(char)*1024*50);
	rbuf = &buf[8];
	char *addr;
#endif 	//_PUB_
														//ソケットの接続受付をどうするか？周期的に見る？listenのタイミング
	while(1){
		//syslog(LOG_NOTICE,"PUB_TASK:enter loop");
		rcv_dtq(PUB_DTQ,dqp);							//ER ercd = rcv_dtq(ID dtqid, intptr_t *p_data) よくわからなi
		pdqp = (char *)dqp;
		int num = lst.find(pdqp[0]);  			//ID取得
		int size;
		size = pdqp[1];
		size += pdqp[2]*256;
		size += pdqp[3]*65536;
		if(num == -1){ 									//初期化
			syslog(LOG_NOTICE,"PUB_TASK:publisher initialize");
			static TCPSocketConnection sock;
			lst.add(sock,pdqp[0]);
		}else if((num != -1) && (size == 0)){	//requestTopicがあるとき
			bool connect_status = false;
			//TCPROSを受け付ける
			//syslog(LOG_NOTICE,"PUB_TASK:request Topic node[%x]",node_lst[num].ID);
			srv.listen();
			//syslog(LOG_NOTICE,"PUB_TASK: Listening...");
			//syslog(LOG_NOTICE,"PUB_TASK: Waiting...");
			if(srv.accept(lst.sock_vec[num]) == 0){
				connect_status = true;
				char *snd_buf;
				snd_buf = (char *)malloc(512);
				while(connect_status){
					int stat = lst.sock_vec[num].receive(rbuf,512);	//受け取ったデータがヘッダかどうかの判断をしないといけない
					switch(stat){
					case 0:
					case -1:
						connect_status = false;
						break;
					default:
						if(check_head(rbuf)){
						int len = genPubTcpRosH(snd_buf);	//テスト用関数
						int l = lst.sock_vec[num].send(snd_buf,len);
						connect_status = false;
						}else{
							syslog(LOG_NOTICE,"PUB_TASK: not TCPROS connection header");
						}
					}
				}
				free(snd_buf);
				syslog(LOG_NOTICE,"PUB_TASK: publisher connected");
				rsm_all();
			}
		}else{	//パブリッシュ(ユーザタスクから送られてくるデータ)
		//データコピー
		memcpy(rbuf,mem,size);
		rbuf[size] = '\0';	//データを切る
		int l = genMessage(buf,rbuf);
		//出版
		int len = lst.sock_vec[num].send(buf,l);
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
	//SUB_TASKが管理するソケット情報
	struct sub_list{
	public:
		std::vector<TCPSocketConnection> sock_vec; 	//TCPソケット
		std::vector<char> id_vec; 					//mROSID
		std::vector<intptr_t> func_vec; 			//関数ポインタ
		std::vector<bool> stat_vec;					//状態ベクトル
		int find(char c){
					for(int i=0;i < this->id_vec.size();i++){
						if(c == this->id_vec[i]){
							return i;
						}
					}
					return -1;
				}
		void add(TCPSocketConnection sock,char ID,intptr_t func){this->sock_vec.push_back(sock);this->id_vec.push_back(ID);this->func_vec.push_back(func);this->stat_vec.push_back(false);}
		void set_stat(bool stat,int idx){this->stat_vec[idx] = stat;}
	};
	sub_list lst;
	intptr_t *dqp,*snd_dqp;
	dqp = (intptr_t *)malloc(sizeof(char)*4);	//レシーブ用領域
	char *sdq,*ip,*tmp,*rbuf,*rptr; //data queue pointer,IP address,temporary buffer,receive buffer,memory address
	tmp = (char *)malloc(sizeof(char)*256);
	rbuf = (char *)malloc(sizeof(char)*1024*512);
	int port;	//port number
	bool init = false;
	bool rcv_flag = true;
	int len,msg_size,data_size;
#endif //_SUB_

	while(1){
		//syslog(LOG_NOTICE,"SUB_TASK:enter loop");
		slp_tsk();
	    int t = trcv_dtq(SUB_DTQ,dqp,1);	//登録用のデータレシーブ，待ち時間が0で，キューにパケットがなければスルー
	    if(t == 0){
	    	sdq = (char *)dqp;
	    	int num = lst.find(sdq[0]);
	    	if(num == -1){
	    		int idx = lst.id_vec.size();
	    		//initialize
				syslog(LOG_NOTICE,"SUB_TASK:subscriber initialize");
				static TCPSocketConnection sock;
				static intptr_t funcp;
				int size = sdq[1];
				size += sdq[2]*256;
				size += sdq[3]*65536;
				//ソケットの接続と各種アドレスの取得を行う
				//必要なモノ：相手のURI，ポート，コールバック関数ポインタ，結果格納用共有メモリアドレス -> xml_mas_taskでやっちゃったほうがいいのでは？つながったソケットを返す？
				memcpy(tmp,&mem[SUB_ADDR],size);				//get date
				//関数ポインタの切り出し
				string str = tmp;
				funcp = (intptr_t)atoi(get_fptr(str).c_str());
				lst.add(sock,sdq[0],funcp);
				syslog(LOG_NOTICE,"SUB_TASK:fptr [%d]",funcp);
				//requestTopic送信
				str = "<methodCall>requestTopic</methodCall>";
				size = str.size();
				memcpy(&mem[XML_ADDR],str.c_str(),size);
				char buf[3];
				buf[0] = sdq[0];
				buf[1] = size;
				buf[2] = size/256;
				buf[3] = size/65536;
				snd_dqp = (intptr_t) &buf;
				snd_dtq(XML_DTQ,*snd_dqp);
				rcv_dtq(SUB_DTQ,dqp);
				sdq = (char *)dqp;
				size = sdq[1];
				size += sdq[2]*256;
				size += sdq[3]*65536;
				memcpy(tmp,&mem[SUB_ADDR],size);				//get date
				str = tmp;
				port = atoi(get_port2(str).c_str());	//これはパブリッシャのポートを入れる とりあえず前に使ってた関数を入れてる
				ip = m_ip;
				//syslog(LOG_NOTICE,"SUB_TASK:port [%d],IP [%s]",port,ip);
				//TCPROSコネクションヘッダを送信
				size = genSubTcpRosH(tmp);	//TCPROSの生成もうまくやるように実装する

				lst.sock_vec[idx].connect(ip,port);
				lst.sock_vec[idx].send(tmp,size);
				wait_ms(500);
				lst.sock_vec[idx].receive(tmp,256);	//TCPROSヘッダの受信
				lst.set_stat(true,idx);
				init = true;
				syslog(LOG_NOTICE,"SUB_TASK: subscriber connected");
				rsm_all();
			}
	    }

	    //end initialize

//subscribe and callback loop
	    if(init){
		for(unsigned int i=0;i < lst.id_vec.size();i++){
			//データレシーブ
			while(rcv_flag){
				rptr = rbuf;
				if(lst.stat_vec[i]){
					int n = lst.sock_vec[i].receive(rptr,5000);
					//コールバック関数実行
					if(n < 0){
						syslog(LOG_NOTICE,"SUB_TASK: No data");
					}
					if(rcv_flag){
						msg_size = (int)rbuf[0] + (int)rbuf[1]*256 + rbuf[2]*65536 + rbuf[3]*16777216;
						data_size = (int)rbuf[4] + (int)rbuf[5]*256 + rbuf[6]*65536 + rbuf[7]*16777216;
						rcv_flag = false;
					}
					len += n;
					if(len == msg_size +4){
						//データ長がきちんとあったときの処理;
						rbuf[len] = '\0';
						syslog(LOG_NOTICE,"SUB_TASK:data length [%d]",len);
						void (*fp)(string);
						fp = lst.func_vec[i];
						fp(&rbuf[8]);
						rptr = rbuf;
						rcv_flag = true;
						len = 0;
					}else if(len > msg_size + 4){
						//データがオーバーしたときの処理　エラー処理にしたい
						syslog(LOG_NOTICE,"invalid header[%d][%d] [%d]",msg_size,len,n);
						rptr = rbuf;
						rcv_flag = true;
						len = 0;
					}else{
					//データ長がすべて取り切れてないときの処理　レシーブのポインタを続きにする
						rptr = &rbuf[n];
					}
					//syslog(LOG_NOTICE,"SUB_TASK: len:[%d] header:[%d][%d][%d]",len,(int)rbuf[4]+(int)rbuf[5]*256,(int)rbuf[4],(int)rbuf[5]*256);
				}
			}
		}
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
	rbuf = (char *)malloc(sizeof(char)*512);
	if(xml_slv_srv.bind(port) == -1){
			syslog(LOG_NOTICE,"Failed bind");
			exit(1);
		}
#endif

	while(1){
		//syslog(LOG_NOTICE,"XML_SLV_TASK:enter loop");
		slp_tsk();
		xml_slv_srv.listen();
		//syslog(LOG_NOTICE,"XML_SLV_TASK: Listening...");
		//syslog(LOG_NOTICE,"XML_SLV_TASK: Waiting...");
		if(xml_slv_srv.accept(xml_slv_sock) == 0){ //acceptとかでタスクが待ち状態にならないか？
			connect_status = true;
			//syslog(LOG_NOTICE,"XML_SLV_TASK: Connected");
			while(connect_status){
			int stat = xml_slv_sock.receive(rbuf,512);
			switch(stat){
			case 0:
			case -1:
				connect_status = false;
				//syslog(LOG_NOTICE,"XML_SLV_TASK:disconnected");
				break;
			default:
				str =rbuf;
				int mh,mt;
				//メソッドの切り出し
				mh = (int)str.find("<methodName>");
				mt = (int)str.find("</methodName>");
				for(int i = mh + sizeof("<methodName>") -1;i < mt ; i++){
					meth = meth + str[i];
				}
				//メソッドごとの処理
				if(meth == "requestTopic"){
					//syslog(LOG_NOTICE,"XML_SLV_TASK:request Topic method");
					string topic_name = req_topic_name(str);
					int num = find_node(node_lst,topic_name);
					if(num == -1){
						//syslog(LOG_NOTICE,"No Existing node!");
					}else{
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
					}
					xml_slv_sock.close();
					}
				}
			}
		}
	}
}


/************************************************************************************************************************************************************************
 * ノードのメンバを入れる関数                                              *
 * (メンバ関数的にすればいいのかもしれない)         *
* **********************************************************************************************************************************************************************/
void get_node(node *node,std::string *xml,bool type){
	string c;
	node->set_node_type(type);
	node->set_topic_type(get_ttype(*xml));
	node->set_topic_name(get_tname(*xml));
	node->set_callerid(get_cid(*xml));
	node->set_message_definition(get_msgdef(*xml));
	c += "http://";
	c += network.getIPAddress();
	c += ":11411";	//xml slaveのポートを追加
	node->set_uri(c);
	if(type){
		node->set_fptr(get_fptr(*xml));
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
	char *xdq,*addr,*rbuf;
	rbuf = (char *)malloc(sizeof(char)*512);
	int count=1;
#endif
	while(1){
		TCPSocketConnection xml_mas_sock;	//呼ばれるたびに新しい口じゃないとダメだったような気がする
		xml_mas_sock.connect(m_ip,m_port);
		//syslog(LOG_NOTICE,"XML_MAS_TASK:enter loop");
		//usr_taskから呼ばれる，もしくはsub_taskから呼ばれる
		//あらかじめデータフィールドを決めておいて，TCPROSチックにやり取りする？ ｛[data length + data]*｝　みたいな
		//XMLを使った感じでいい？
		rcv_dtq(XML_DTQ,dq);
		xdq = (char *)dq;
		int size = xdq[1];
		size += xdq[2]*256;
		size += xdq[3]*65536;
		memcpy(rbuf,&mem[XML_ADDR],size);				//get date
		//メソッドの識別
		std::string str,meth;
		str = rbuf;
		int mh,mt;
		mh = (int)str.find("<methodCall>");
		mt = (int)str.find("</methodCall>");
		for(int i = mh + sizeof("<methodCall>") -1;i < mt ; i++){
			meth = meth + str[i];
		}
		//メソッドごとに送信XMLの生成とノードの登録
//===registerSubscriber=========================================================================================================================
		if(meth == "registerSubscriber"){
			//sus_all();
			node sub;
			string xml;
			syslog(LOG_NOTICE,"XML_MAS_TASK:regiester Subscriber");
			sub.ID = xdq[0];
			get_node(&sub,&str,true);														//データを取得
			xml = registerSubscriber(sub.callerid,sub.topic_name,sub.topic_type,sub.uri); 	//ROSmaster用のXML生成
			xml_mas_sock.send(xml.c_str(),strlen(xml.c_str()));						 		//ROSmasterにデータ送信
			xml_mas_sock.receive(rbuf,sizeof(char)*512);
			xml = rbuf;
			sub.set_port(get_port(xml));	//IPは考慮していない　PUBノードが立っていない場合ここでポートが返ってこない
			//syslog(LOG_NOTICE,"XML_MAS_TASK:port [%s]",sub.port.c_str());
			//subタスクにデータ送信
			node_lst.push_back(sub);
			xml = registerSubtask(sub.fptr,sub.port);
			int size = strlen(xml.c_str());
			memcpy(&mem[SUB_ADDR],xml.c_str(),size);
			mem[strlen(xml.c_str())+1] == '\0';
			data[0] = sub.ID;
			data[1] = size; //とりあえず先頭から入れる
			data[2] = size/256;
			data[3] = size/65536;
			sdata = (intptr_t) &data;
			snd_dtq(SUB_DTQ,*sdata);	//subtaskに投げる

//===registerPublisher===========================================================================================================================
		}else if(meth == "registerPublisher"){
			//sus_all();
			node pub;
			std::string xml;
			pub.ID = xdq[0];
			syslog(LOG_NOTICE,"XML_MAS_TASK:regiester Publisher");
			get_node(&pub,&str,false);														//データ取得
			node_lst.push_back(pub);
			xml = registerPublisher(pub.callerid,pub.topic_name,pub.topic_type,pub.uri); 	//ROSmaster用のXML生成 uriにxml_slvのポートを入れないといけない->いれた
			xml_mas_sock.send(xml.c_str(),strlen(xml.c_str())); 							//ROSmasterにデータ送信
			xml_mas_sock.receive(rbuf,sizeof(char)*512);
			//pubタスクにデータ送信
			data[0] = pub.ID;
			data[1] = 0;
			data[2] = 0;
			data[3] = 0;
			sdata = (intptr_t) &data;
			snd_dtq(PUB_DTQ,*sdata);	//pubtaskに投げる

//===requestTopic================================================================================================================================
		}else if(meth == "requestTopic"){	//sub taskからくるリクエストを投げる
			syslog(LOG_NOTICE,"XML_MAS_TASK:send request topic");
				int num = find_id(node_lst,xdq[0]);
				//syslog(LOG_NOTICE,"XML_MAS_TASK: node num [%d]",num);
				if(num != -1){
				//syslog(LOG_NOTICE,"XML_MAS_TASK:request node ID [%x]",node_lst[num].ID);
				string body = requestTopic(node_lst[num].callerid,node_lst[num].topic_name);
				TCPSocketConnection tmpsock;
				if(tmpsock.connect(m_ip,atoi(node_lst[num].port.c_str())) == -1){
					exit(1);
				}
				tmpsock.send(body.c_str(),strlen(body.c_str()));
				tmpsock.receive(rbuf,sizeof(char)*512);
				tmpsock.close();
				int size = strlen(rbuf);
				memcpy(&mem[SUB_ADDR],rbuf,size);
				data[0] = node_lst[num].ID;
				data[1] = size;
				data[2] = size/256;
				data[3] = size/65536;
				sdata = (intptr_t) &data;
				snd_dtq(SUB_DTQ,*sdata);
			}else{
				//syslog(LOG_NOTICE,"XML_MAS: Can't find node! [request topic]");

			}

//===else========================================================================================================================================
		}else{	//他のメソッドを処理する場合はここから追加する

		}
		xml_mas_sock.close();
	} //end while loop
}

//周期ハンドラ
//XML-RPCスレーブタスクとサブスクライブタスクを起動させる？
void cyclic_handler(intptr_t exinf){
	//周期ハンドラからはiwup_tsk()で起こす．wup_tsk()だとコンテキストエラー
	iwup_tsk(SUB_TASK);
	iwup_tsk(XML_SLV_TASK);
}



