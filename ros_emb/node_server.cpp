#include "node_server.h"

void nodeServerStart(TCPSocketServer svr,TCPSocketConnection csock,int port=8000){

	if(svr.bind(port) == -1){
		exit(1);
	}
	//printf("Listning...\n");
	syslog(LOG_NOTICE,"SERVR_INFO: Listening...");
	svr.listen();
	while(1){
		bool connect_stat = false;
		//printf("Waiting...\n");
		syslog(LOG_NOTICE,"SERVER_INFO: Waiting...");
		if(svr.accept(csock) == 0){
			connect_stat = true;
			csouck.set_blocking(true,50000)
			//printf("Connected\n");
			syslog(LOG_NOTICE,"SERVER_INFO: Connected");
			//受信がぶつ切りになるのでwhileをやめてみる
			//while(connect_stat){
				char *buff;
				buff = (char *)malloc(1024);
				int status = csock.receive(buff,strlen(buff));
				switch(status){
				case 0:
				case -1: //disconnect
					connect_stat = false;
					break;
				default: //どこかでdisconnectになるタイミングが必要　->　受け取って返信がいる
					//printf("receive size:%d B\n data:%s\n",status,buff);
					string http;
					http = buff;
					syslog(LOG_NOTICE,"SERVER_INFO: receive size:%d\ndata:%s\nconvert:%s",status,buff,http.c_str());
					xmlNode *node;
					syslog(LOG_NOTICE,"SERVER_INFO: Let's parse");
					int i = ParseReceiveMessage(http,node);
					syslog(LOG_NOTICE,"SERVER_INFO:parse_status_code: %d",i);
					//リクエストに対してお返事を返す
					csock.send((test_requestRespose().c_str(),test_requestResponse().size());
					break;
				}
			//}
			csock.close();
			//printf("Disconnected");
			syslog(LOG_NOTICE,"Disconnected");
		}else{
		//printf("time out\n");
		syslog(LOG_NOTICE,"time out");
		}
	}
	svr.close();
}
