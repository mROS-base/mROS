#include "mros.h"

//TCPROSのfield lengthを計算して付与する関数
//mallocを使いたくない
char* addtcproshead(char *buf){
#ifndef addtcp
#define addtcp
	char *c;
	c = (char *)malloc(512);
#endif
	int len = strlen(buf);
	c[0] = len - 0;
    c[1] = len/256 - 0;
	c[2] = len/65536 - 0;
	c[3] = len/16777216 - 0;
	strcpy(&c[4],buf);
	//syslog(LOG_NOTICE,"c:%s\n",&c[4]);

	return c;
	
}

//TCPROSコネクションヘッダを作る関数
//Publisherのヘッダを生成
int genPubTcpRosH(char *buf){
    //この辺の文字列をROSのプログラムから取ってくる必要ある

    char *id = "callerid=/mros_node2";
	char *msg_def = "message_definition=string data\n";
	char *topic = "topic=/mros_msg";
	char *type = "type=std_msgs/String";
    char *md5 = "md5sum=992ce8a1687cec8c8bd883ec73ca41d1";

    int lid = strlen(id) + 4;
    id = addtcproshead(id);
    int ldef = strlen(msg_def) + 4;
    msg_def = addtcproshead(msg_def);
    int ltpc = strlen(topic) + 4;
    topic = addtcproshead(topic);
    int ltyp = strlen(type) + 4; 
    type = addtcproshead(type);
    int lmd5 = strlen(md5) + 4; 
    md5 = addtcproshead(md5);

    int p=4;
    for(int i=0;i < lid;i++){
        buf[p] = id[i];
        p++;
    }
    free(id);
    for(int i=0;i < ldef;i++){
        buf[p] = msg_def[i];
        p++;
    }
    free(msg_def);
    for(int i=0;i < ltpc;i++){
        buf[p] = topic[i];
        p++;
    }
    free(topic);
    for(int i=0;i < ltyp;i++){
        buf[p] = type[i];
        p++;
    }
    free(type);
    for(int i=0;i < lmd5;i++){
        buf[p] = md5[i];
        p++;
    }
    free(md5);
    int q = p - 4; 
    buf[0] = q - 0;
    buf[1] = q/256 - 0;
    buf[2] = q/65536 - 0;
    buf[3] = q/16777216 - 0;
    //sock.send(buf,p);
    //for(int i=0;i < p;i++){
    //	syslog(LOG_NOTICE,"bin:%x\n",buf[i]);
    //}
    //free(buf);
    return p;
}

int genSubTcpRosH(char *buf){
	char *id = "callerid=/mros_node";
	//char *msg_def = "message_definition=string data\n";
	char *nodel= "tcp_nodelay=0";
	char *topic = "topic=/test_string";
	char *type = "type=std_msgs/String";
	char *md5 = "md5sum=992ce8a1687cec8c8bd883ec73ca41d1";
	int lid = strlen(id) + 4;
	    id = addtcproshead(id);
	    int ll = strlen(nodel) + 4;
	    nodel = addtcproshead(nodel);
	    int ltpc = strlen(topic) + 4;
	    topic = addtcproshead(topic);
	    int ltyp = strlen(type) + 4;
	    type = addtcproshead(type);
	    int lmd5 = strlen(md5) + 4;
	    md5 = addtcproshead(md5);

	    int p=4;
	    for(int i=0;i < lid;i++){
	        buf[p] = id[i];
	        p++;
	    }
	    free(id);
	    for(int i=0;i < ll;i++){
	        buf[p] = nodel[i];
	        p++;
	    }
	    free(nodel);
	    for(int i=0;i < ltpc;i++){
	        buf[p] = topic[i];
	        p++;
	    }
	    free(topic);
	    for(int i=0;i < ltyp;i++){
	        buf[p] = type[i];
	        p++;
	    }
	    free(type);
	    for(int i=0;i < lmd5;i++){
	        buf[p] = md5[i];
	        p++;
	    }
	    free(md5);
	    int q = p - 4;
	    buf[0] = q - 0;
	    buf[1] = q/256 - 0;
	    buf[2] = q/65536 - 0;
	    buf[3] = q/16777216 - 0;

	    return p;
}

//暫定的に
char* addtcproshead2(char *buf){
	char *c;
	c = &buf[4];
	int len = strlen(&buf[8]);
	c[0] = len - 0;
    c[1] = len/256 - 0;
	c[2] = len/65536 - 0;
	c[3] = len/16777216 - 0;
	
	return c;

}

//TCPROSのボディを作る関数
int genMessage(char *buf,char *msg){
    int len = strlen(msg) + 4;
    addtcproshead2(buf);

    int p = 4 + len;
    /*
    free(msg);
    */
    buf[0] = len - 0;
    buf[1] = (len/256) - 0;
    buf[2] = (len/65536) - 0;
    buf[3] = (len/16777216) - 0;
    //syslog(LOG_NOTICE,"TCPROS header [%x][%x][%x][%x]",buf[0],buf[1],buf[2],buf[3]);
    //free(buf);
    return p;
}

/* 
* TCPROSのヘッダに必要なfieldの要素をどうするか考える．
* structでpub/sub,server/clientで分けるかどうか
* md5sumとかtopicのname,typeとかの参照をどうするか
*/

bool check_head(char *buf){
	int len,l;
	if(buf[0] == '0'){
		syslog(LOG_NOTICE,"Not header!");
		return false;
	}
	len = buf[0] + buf[1]*256 + buf[2]*65536 + buf[3]*16777216;
	for(int p=4;p < len;){
		if(buf[p] == '0'){
			syslog(LOG_NOTICE,"Not header!");
			return false;
		}
		l = buf[p] + buf[p+1]*256 + buf[p+2]*65536 + buf[p+3]*16777216;
		//syslog(LOG_NOTICE,"TCPROS HEADER: [%s]",&buf[p+4]);
		char *c = &buf[p+4];
		char *i = strstr(c,"callerid");
		if(i != NULL){
			return true;
		}
		p = p + l + 4;
	}
	return false;
}


