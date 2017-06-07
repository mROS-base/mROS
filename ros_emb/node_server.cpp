#include "node_server.h"
/*whileループ切ったバージョン
 * とりあえず動くけどデータは取れない
*/
void nodeServerStart(TCPSocketServer svr,TCPSocketConnection csock,int port=40009){
	if(svr.bind(port) == -1){
		exit(1);
	}
	syslog(LOG_NOTICE,"SERVR_INFO: Listening...");
	svr.listen();
	//while(1){　　//とりあえずループ切る
		bool connect_stat = false;
		syslog(LOG_NOTICE,"SERVER_INFO: Waiting...");
		if(svr.accept(csock) == 0){
			connect_stat = true;
			csock.set_blocking(true,5000);
			syslog(LOG_NOTICE,"SERVER_INFO: Connected");
			//受信がぶつ切りになるのでwhileをやめてみる
			string http;
			char *srv_buff;
			srv_buff = (char *)malloc(512);

			//while(connect_stat){  //とりあえずループ切る
				//syslog(LOG_NOTICE,"srlen(srv_buff):%d,srv_buff:%s",strlen(srv_buff),srv_buff);
				//syslog(LOG_NOTICE,"LOG_INFO:asdfasf");
				int status = csock.receive(srv_buff,256); //サイズを512とか大きくすると死ぬ　strlenでバッファに入ってる文字列全部更新してstringで結合して取り出すか？
				syslog(LOG_NOTICE,"receive success");
				switch(status){
				case 0:
				case -1: //disconnect
					connect_stat = false;
					break;
				default: //どこかでdisconnectになるタイミングが必要　->　受け取って返信がいる
					syslog(LOG_NOTICE,"SERVER_INFO: receive size:%d\nreceive data \n%s",status,srv_buff);
					xmlNode *node;
					string tmp;
					tmp = srv_buff;
					syslog(LOG_NOTICE,"SERVER_INFO: Let's parse");
					http += tmp;
					syslog(LOG_NOTICE,"data body:%s\n",http);
					int i = ParseReceiveMessage(http,node);
					syslog(LOG_NOTICE,"SERVER_INFO:parse_status_code: %d",i);
					//リクエストに対してお返事を返す
					if(i==0){
						csock.send(test_requestResponse().c_str(),test_requestResponse().size());
					}
					break;
				}
				free(srv_buff);	//ループによってこいつの位置もかえる
			}
			csock.close();
			syslog(LOG_NOTICE,"Disconnected");
		//}else{
		syslog(LOG_NOTICE,"Denied");
		//}
	//}
	svr.close();
}
