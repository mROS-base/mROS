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
    return len;
}

//only for std_msgs/String 
int pub_gen_msg(char *buf,char *msg){
    int len = strlen(msg);
    add_len(&buf[4],len);
    add_len(buf,len+4);
    return len+8;
}

bool first = true;
int ind;
//rgb8用ダミー関数
int pub_gen_dummy(char *buf){
	int p=4;
	if(first){
    //Header -> sequence
	for(int j= 0;j<4;j++){
		buf[j+p] = 0x00;
	}
	p=p+4;
	//header -> timestamp [sec]
    for(int j= 0;j<4;j++){
    	buf[j+p] = 0x00;
    }
    p = p+4;
	//header -> timestamp [nsec]
    for(int j= 0;j<4;j++){
		buf[j+p] = 0x00;
	}
    p = p+4;
    //header -> frame_id
    add_len(&buf[p],sizeof("mROScam"));
    p = p + 4;
    memcpy(&buf[p],"mROScam",sizeof("mROScam"));
    p = p+ sizeof("mROScam");
    //height
    buf[p] = 0x04;
    buf[p+1] = 0x0;
    buf[p+2] = 0x00;
    buf[p+3] = 0x00;
    p = p+4;
    //width
    buf[p] = 0x02;
	buf[p+1] = 0x00;
	buf[p+2] = 0x00;
	buf[p+3] = 0x00;
	p = p+4;
    //encode
	add_len(&buf[p],4);
	p = p+4;
	buf[p] = 0x72;
	buf[p+1] = 0x67;
	buf[p+2] = 0x62;
	buf[p+3] = 0x38;
	p=p+4;
	//endian,step
	buf[p] = 0x00;
	buf[p+1] = 0x06;
	buf[p+2] = 0x00;
	buf[p+3] = 0x00;
	buf[p+4] = 0x00;
	p=p+5;
	//data
	add_len(&buf[p],24);
	p=p+4;
	ind = p;
	//1-1
	buf[p] = 0x00;
	buf[p+1] = 0x00;
	buf[p+2] = 0xff;
	p=p+3;
	//1-2
	buf[p] = 0x00;
	buf[p+1] = 0xff;
	buf[p+2] = 0x00;
	p=p+3;
	//2-1
	buf[p] = 0xff;
	buf[p+1] = 0x00;
	buf[p+2] = 0x00;
	p=p+3;
	//2-2
	buf[p] = 0x00;
	buf[p+1] = 0x00;
	buf[p+2] = 0x00;
	p=p+3;
	//3-1
	buf[p] = 0xff;
	buf[p+1] = 0xff;
	buf[p+2] = 0xff;
	p=p+3;
	//3-2
	buf[p] = 0x88;
	buf[p+1] = 0x88;
	buf[p+2] = 0x88;
	p=p+3;
	//fixed
	//4-1
	buf[p] = 0x88;
	buf[p+1] = 0x88;
	buf[p+2] = 0x88;
	p=p+3;
	//4-2
	buf[p] = 0x88;
	buf[p+1] = 0x88;
	buf[p+2] = 0x88;
	p=p+3;
	//add length
	add_len(buf,p-4);
	first = false;
	}else{
	pibot(&buf[ind],6);
	p = ind + 24;
	}
    return p;
}

void pibot(char *data,int n){
	char tmp[3];
	memcpy(tmp,&data[0],3);
	for(int i=0;i<5;i++){
		memcpy(&data[i*3],&data[(i+1)*3],3);
	}
	memcpy(&data[(n-1)*3],tmp,3);
}

int pub_gen_img_msg(char *buf,char *msg,int size){
    int len = 4;
    memcpy(&buf[len],msg,size);
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

