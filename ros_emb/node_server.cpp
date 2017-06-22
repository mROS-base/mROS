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
				free(srv_buff);
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

					//msg = srv_buff;
					//int i = ParseReceiveMessage(msg,node);
					//syslog(LOG_NOTICE,"SERVER_INFO: End parsing--------\nparse code:%d,parse data:%s",i,msg.c_str());
					//if(i == 0){
					//	connect_status = false;
					//}

					//リクエストにこたえる用
					if(port == 40040){
						csock.send(test_requestResponse().c_str(),test_requestResponse().size());
					}
					//tcprosの通信する用
					if(port == 40400 && tcp_first){
						//エンコードが必要
						char *snd_buf;
						snd_buf = (char *)malloc(512);
						int len = genPubTcpRosH(snd_buf);
						csock.send(snd_buf,len);
						syslog(LOG_NOTICE,"TCPROS: SEND HEADER");
						tcp_first = false;
					}
					break;
				}
			}

			//message send loop
			if(port == 40400){
				char *buf;
				buf = (char *)malloc(256);
			    long c=0;
			    	while(1){
			    		if(c==5000){
			    			int l = genMessage(buf);
			    			syslog(LOG_NOTICE,"Hello mROS!");
			    			csock.send(buf,l);
			    	}else if(c == 5000000){
			    			c = 0;
			    	}
			    		c++;
			    	}
			    	free(buf);
			}
		}else{
			syslog(LOG_NOTICE,"SERVER_INFO: Denied connection");
		}
	csock.close();
	svr.close();
}


/*
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

					//リクエストにこたえる用
					if(port == 40040){
						csock.send(test_requestResponse().c_str(),test_requestResponse().size());
					}
					//tcprosの通信する用
					if(port == 40400 && tcp_first){
						//エンコードが必要
						char *snd_head = "\x93\x00\x00\x00\x13\x00\x00\x00\x63\x61llerid=/mros_node\x1f\x00\x00\x00message_definition=string data\n\x12\x00\x00\x00topic=/test_string\x14\x00\x00\x00type=std_msgs/String\x27\x00\x00\x00md5sum=992ce8a1687cec8c8bd883ec73ca41d1";
						csock.send(snd_head,151);
						tcp_first = false;
						//free(snd_head);
					}
					break;
				}
			}
			//message send loop
			if(port == 40400){
				long c=0;
				while(1){
				if(c==5000){
					syslog(LOG_NOTICE,"Hello mROS!");
					char *snd_body="\x10\x00\x00\x00\x0c\x00\x00\x00Hello mROS!!";
					csock.send(snd_body,20);
					//free(snd_body);
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
*/
