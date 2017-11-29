#include "mros.h"


void add_len(char *buf,int len){
    buf[0] = len - 0;
    buf[1] = len/256 - 0;
    buf[2] = len/65536 - 0;
    buf[3] = len/16777216 - 0;
}

int pub_gen_header(char *buf,string id,string msg_def,string topic,string type,string md5){
    int len = 0;
    int it=0;
    id = "callerid=" + id;
    msg_def = "message_definition=" + msg_def + "\n";
    topic = "topic=" + topic;
    type = "type=" + type;
    md5 = "md5sum=" + md5;
    len = id.size() + msg_def.size() + topic.size() + type.size() + md5.size() + 24;
    add_len(&buf[it],len-4);
    it = it + 4;
    add_len(&buf[it],id.size());
    memcpy(&buf[it +4],id.c_str(),id.size());
    it = it + 4 + id.size();
    add_len(&buf[it],msg_def.size());
    memcpy(&buf[it +4],msg_def.c_str(),msg_def.size());
    it = it + 4 + msg_def.size();
    add_len(&buf[it],topic.size());
    memcpy(&buf[it +4],topic.c_str(),topic.size());
    it = it + 4 + topic.size();
    add_len(&buf[it],type.size());
    memcpy(&buf[it +4],type.c_str(),type.size());
    it = it + 4 + type.size();
    add_len(&buf[it],md5.size());
    memcpy(&buf[it +4],md5.c_str(),md5.size());
    it = it + 4 + md5.size();
    return len;
}

int sub_gen_header(char *buf,string id,string nodelay,string topic,string type,string md5){
    int len = 0;
    int it=0;
    id = "callerid=" + id;
    nodelay = "tcp_nodelay=0";
    topic = "topic=" + topic;
    type = "type=" + type;
    md5 = "md5sum=" + md5;
    len = id.size() + nodelay.size() + topic.size() + type.size() + md5.size() + 24;
    add_len(&buf[it],len-4);
    it = it + 4;
    add_len(&buf[it],id.size());
    memcpy(&buf[it +4],id.c_str(),id.size());
    it = it + 4 + id.size();
    add_len(&buf[it],nodelay.size());
    memcpy(&buf[it +4],nodelay.c_str(),nodelay.size());
    it = it + 4 + nodelay.size();
    add_len(&buf[it],topic.size());
    memcpy(&buf[it +4],topic.c_str(),topic.size());
    it = it + 4 + topic.size();
    add_len(&buf[it],type.size());
    memcpy(&buf[it +4],type.c_str(),type.size());
    it = it + 4 + type.size();
    add_len(&buf[it],md5.size());
    memcpy(&buf[it +4],md5.c_str(),md5.size());
    it = it + 4 + md5.size();
    return len;
}

//only for std_msgs/String 
int pub_gen_msg(char *buf,char *msg){
    int len = strlen(msg);
    add_len(&buf[4],len);
    add_len(buf,len+4);
    return len+8;
}


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


