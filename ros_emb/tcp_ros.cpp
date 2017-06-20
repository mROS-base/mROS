#include "tcp_ros.h"


//TCPROSのfield lengthを計算して付与する関数
char* addtcproshead(char *buf){
	char *c;
    c = (char *)malloc(512);
	int len = strlen(buf);
	c[0] = len - 0;
	c[1] = len/256 - 0;
	c[2] = len/65536 - 0;
	c[3] = len/16777216 - 0;
	strcat(&c[4],buf);
	return c;
	
}

//TCPROSコネクションヘッダを作る関数
//Publisherのヘッダを生成
int genPubTcpRosH(char *buf){    
    //この辺の文字列をROSのプログラムから取ってくる必要ある
	char *id = "/mros_node";
    char *msg_def = "message_definition=string data\n";
    char *topic = "topic=/test_string";
    char *type = "type=std_msgs/String"; 
    char *md5 = "md5sum=992ce8a1687cec8cbd883ec73ca41d1";

    int lid = strlen(id);
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
    buf[0] = p - 0;
    buf[1] = p/256 - 0;
    buf[2] = p/65536 - 0;
    buf[3] = p/16777216 - 0;
    return p;
}


//TCPROSのボディを作る関数
int genMessage(char *buf){
    char *msg = "Hello mROS!!";
    int len = strlen(msg) + 4;
    msg = addtcproshead(msg);

    int p=4;
    for(int i=0;i < len;i++){
        buf[p] = msg[i];
        p++;
    }
    free(msg);
    buf[0] = p - 0;
    buf[1] = p/256 - 0;
    buf[2] = p/65536 - 0;
    buf[3] = p/16777216 - 0;
    return p;
}

/* 
* TCPROSのヘッダに必要なfieldの要素をどうするか考える．
* structでpub/sub,server/clientで分けるかどうか
* md5sumとかtopicのname,typeとかの参照をどうするか
*/

