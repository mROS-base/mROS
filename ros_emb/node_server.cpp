#include "node_server.h"

//超音波センサHC-SR04の関数
//参考（https://developer.mbed.org/users/haru36rr/notebook/hcsr04_operation_check/）
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

void init(void){
    TrigerTiming.attach( Triger , 0.060 );      //USSTriger周期 60ms
    USSEcho.rise( RiseEcho );                   //USSEcho立ち上がり時割り込み
    USSEcho.fall( FallEcho );                   //USSEcho立ち下がり時割り込み
}


//パブリッシャー用TCPサーバ
void nodeServerStart(TCPSocketServer svr,TCPSocketConnection csock,int port){
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

			if(port == 40400){
				char *buf;
				char *msg;
				buf = (char *)malloc(256);
				msg = (char *)malloc(256);
				init();
				bool b = false;
				bool bb = true;
			    	while(1){
			    		if(Button.read() == 0 && bb){
			    			b = !b;
			    			bb = false;
			    		}else if(Button.read() == 1){
			    			bb = true;
			    		}
			    		if(b){
			    			wait_ms(1000);
			    			sprintf(msg,"Distance[%d]cm\0",USSDistance);
			    			int l = genMessage(buf,msg);
			    			csock.send(buf,l);
			    		}
			    	}
			    	free(buf);
			    	free(msg);
			}
		}else{
			syslog(LOG_NOTICE,"SERVER_INFO: Denied connection");
		}
	csock.close();
	svr.close();
}

