#include "node_server.h"

//テストサーバ
void nodeServerStart(TCPSocketServer svr,TCPSocketConnection csock,int port){
	//string msg;
	//xmlNode *node;
	if(svr.bind(port) == -1){
		syslog(LOG_NOTICE,"Failed bind");
		exit(1);
	}
	svr.listen();
	syslog(LOG_NOTICE,"SERVER_INFO: Listening...");
		bool connect_status = false;
		syslog(LOG_NOTICE,"SERVER_INFO: Waiting...");
		if(svr.accept(csock) == 0){
			connect_status = true;
			syslog(LOG_NOTICE,"SERVER_INFO: Connected");
			char *srv_buff;
			srv_buff = (char *)malloc(1024);
			bool tcp_first = true;
			while(connect_status && tcp_first){
				int status = csock.receive(srv_buff,1024);
				syslog(LOG_NOTICE,"SERVER_INFO: receive success");
				switch(status){
				case 0:
				case -1:
					connect_status = false;
					break;
				default:
					//syslog(LOG_NOTICE,"SERVER_INFO: receive size:%d\n--------receive data-------\n%s",status,srv_buff);
					//残念ながらパーサを入れるとどっかで死ぬっぽい．パーサの中で死んでるのかレシーブができてないのか不明
					//将来的には動かないといけない
					//SERVER_INFO: receive seccessが出ずに死ぬからパーサまで届いてない気もするが…
					/*
					msg = srv_buff;
					int i = ParseReceiveMessage(msg,node);
					syslog(LOG_NOTICE,"SERVER_INFO: End parsing--------\nparse code:%d,parse data:%s",i,msg.c_str());
					if(i == 0){
						connect_status = false;
					}
					*/

					//リクエストにこたえる用
					if(port == 40040){
						csock.send(test_requestResponse().c_str(),test_requestResponse().size());
					}
					//tcprosの通信する用
					if(port == 40400 && tcp_first){
						//エンコードが必要
						char *snd_head;
						snd_head = (char *)malloc(1024);
						int hlen = getPubTcpRosH(snd_head);
						csock.send(snd_head,hlen);
						tcp_first = false;
						free(snd_head);
					}
					break;
				}
			}
			//message send loop
			if(port == 40400){
				double c=0;
				while(1){
				if(c==5000){
				syslog(LOG_NOTICE,"Hello mROS");
				char *snd_body;
				snd_body = (char *)malloc(512);
				int blen = genMessage(snd_body);
				csock.send(snd_body,blen);
				free(snd_body);
				}else if(c == 5000000){
					c = 0;
				}
				c++;
				}
			}
			free(srv_buff);
		}else{
			syslog(LOG_NOTICE,"SERVER_INFO: Denied connection");
		}
	csock.close();
	svr.close();
}
