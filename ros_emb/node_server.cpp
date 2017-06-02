#include "node_server.h"

void nodeServerStart(TCPSocketServer svr,TCPSocketConnection csock,int port=8000){

	if(svr.bind(port) == -1){
		exit(1);
	}
	//printf("Listning...\n");
	syslog(LOG_NOTICE,"SERVR_INFO: Listning...");
	svr.listen();
	while(1){
		bool connect_stat = false;
		//printf("Waiting...\n");
		syslog(LOG_NOTICE,"SERVER_INFO: Waiting...");
		if(svr.accept(csock) == 0){
			connect_stat = true;
			//printf("Connected\n");
			syslog(LOG_NOTICE,"SERVER_INFO: Connected");
			while(connect_stat){
				char *buff;
				buff = (char *)malloc(1024);
				int status = csock.receive(buff,strlen(buff));
				switch(status){
				case 0:
				case -1:
					connect_stat = false;
					break;
				default:
					//printf("receive size:%d B\n data:%s\n",status,buff);
					syslog(LOG_NOTICE,"SERVER_INFO: receive size:%d\n data:%s",status,buff);
					break;
				}
			}
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
