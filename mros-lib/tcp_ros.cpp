#include "tcp_ros.h"


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
    len = id.size() + topic.size() + type.size() + md5.size() + 20;
    //len = id.size() + msg_def.size() + topic.size() + type.size() + md5.size() + 24;
    add_len(&buf[it],len-4);
    it = it + 4;
    add_len(&buf[it],id.size());
    memcpy(&buf[it +4],id.c_str(),id.size());
    it = it + 4 + id.size();
    /*
    add_len(&buf[it],msg_def.size());
    memcpy(&buf[it +4],msg_def.c_str(),msg_def.size());
    it = it + 4 + msg_def.size();
    */
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
    add_len(&buf[it],strlen("tcp_nodely=1"));
    memcpy(&buf[it +4],"tcp_nodely=1",strlen("tcp_nodely=1"));
    it = it + 4 + strlen("tcp_nodely=1");
    return len;
}



//only for std_msgs/String 
int pub_gen_msg(char *buf,char *msg){
    int len = strlen(msg);
    add_len(&buf[4],len);
    add_len(buf,len+4);
    return len+8;
}


int pub_gen_img_msg(char *buf,char *msg,int size){
    memcpy(&buf[4],msg,size);
    add_len(buf,size);
    /*
    //seq
    memcpy(&buf[len],&buf[len+4],sizeof(unsigned int));
    len += sizeof(unsigned int);
    //sec
    memcpy(&buf[len],&buf[len+4],sizeof(unsigned int));
    len += sizeof(unsigned int)
    //nsec
    memcpy(&buf[len],&buf[len+4],sizeof(unsigned int));
    len += sizeof(unsigned int);
    //frame_id -> need branch
    //length
    memcpy(&buf[len],&buf[len+sizeof(unsigned int)],sizeof(unsigned int));
    len += 4;
    memcpy(&buf[len],&buf[len+strlen(&buf[len])],);
    //height
    len += 4;
    memcpy(&buf[len],&buf[len+sizeof(unsigned int)],sizeof(unsigned int));//height
    //width
    len += 4;
    memcpy(&buf[len],&buf[len+sizeof(unsigned int)],sizeof(unsigned int));
    //encoding
    len += 4;
    add_len(&buf[len],strlen(&buf[len+4]));
    len += 4;
    memcpy(&buf[len],&buf[len+4],);
    add_len(&buf[4],len);
    add_len(buf,len+4);
    */
    return size+4;
}

