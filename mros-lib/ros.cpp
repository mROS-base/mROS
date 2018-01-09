#include "ros.h"

std::vector<ID> IDv;
int ID_find(std::vector<ID> IDv,ID id){for(unsigned int i=0;i < IDv.size();i++){if(IDv[i] == id){return i;}}return -1;}

std::vector<std::string> node_nv;

extern std::vector<node> node_lst;
int find_node(std::vector<node> list,std::string topic){for(unsigned int i=0;i < list.size();i++){if(list[i].topic_name == topic){return i;}}return -1;}
int find_id(std::vector<node> list,char ID){for(unsigned int i=0;i < list.size();i++){if(list[i].ID == ID){return i;}}return -1;}
//同一デバイス通信用
int find_sub(std::vector<node> list,std::string topic){
	for(unsigned int i=0;i < list.size();i++){
		if(list[i].topic_name == topic && list[i].node_type){
			return i; 	//現状一つだけに対応
		}
	}
	return -1;
}




void ros::init(int argc,char *argv,std::string node_name){
	ID id;
	get_tid(&id);
	IDv.push_back(id);
	syslog(LOG_NOTICE,"usr task ID [%d]",id);
	node_nv.push_back(node_name);
}
/*
template<class M,class T> ros::Subscriber ros::NodeHandle::subscriber(std::string topic,int queue_size,void (T::*fp)(M)){
	Subscriber sub;
	return sub;
}
template<class M> ros::Subscriber ros::NodeHandle::subscriber(std::string topic,int queue_size,void (*fp)(M)){
	Subscriber sub;
	return sub;
}
*/


ros::Subscriber ros::NodeHandle::subscriber(std::string topic,int queue_size,void (*fp)(std::string)){
	while(ros_sem != 0){

	}
	state = 1;
	syslog(LOG_NOTICE,"Change state [%d]",state);
	ros_sem++;
	ID id;
	get_tid(&id);
	Subscriber sub;
	sub.node = node_nv[ID_find(IDv,id)].c_str();
	sub.topic = topic.c_str();
	sub.ID = (char) count;
	count++;
	char tmp[8];
	sprintf(tmp,"%d",fp);
	std::string sstr;
	sstr = "<methodCall>registerSubscriber</methodCall>\n";
	sstr += "<topic_name>/";
	sstr += topic;
	sstr += "</topic_name>\n";
	sstr += "<topic_type>std_msgs/String</topic_type>\n";
	sstr += "<caller_id>/";
	sstr +=  node_nv[ID_find(IDv,id)].c_str();
	sstr+=	"</caller_id>\n";
	sstr += "<message_definition>std_msgs/String</message_definition>\n";
	sstr += "<fptr>";
	sstr += tmp;
	sstr += "</fptr>\n";
	intptr_t *sdq;
	int size = strlen(sstr.c_str());
	memcpy(&mem[XML_ADDR],sstr.c_str(),size);
	char sbuf[3];
	sbuf[0] = sub.ID;
	sbuf[1] = size;
	sbuf[2] = size/256;
	sbuf[3] = size/65536;
	sdq = (intptr_t) &sbuf;
	snd_dtq(XML_DTQ,*sdq); //sndはデータ本体を渡す？big-little?なエンディアン 20b1->1b02で渡される
	return sub;
}

ros::Publisher ros::NodeHandle::advertise(string topic,int queue_size){
	//セマフォの確認
	while(ros_sem != 0){

	}
	ros_sem++;
	state = 2;
	syslog(LOG_NOTICE,"Change state [%d]",state);
	ID id;
	get_tid(&id);
	Publisher pub;
	pub.node = node_nv[ID_find(IDv,id)].c_str();
	pub.topic = topic.c_str();
	pub.ID = count;
	count++;
	string pstr;
	pstr = "<methodCall>registerPublisher</methodCall>\n";
	pstr += "<topic_name>/";
	pstr += topic;
	pstr += "</topic_name>\n";
	pstr += "<topic_type>std_msgs/String</topic_type>\n";
	pstr += "<caller_id>/";
	pstr +=	node_nv[ID_find(IDv,id)].c_str();
	pstr += "</caller_id>\n";
	pstr += "<message_definition>string data</message_definition>\n";
	pstr += "<fptr>12345671</fptr>\n";
	intptr_t *pdq;
	memcpy(&mem[XML_ADDR],pstr.c_str(),pstr.size());
	char pbuf[3];
	pbuf[0] = pub.ID;
	int size = strlen(pstr.c_str());
	pbuf[1] = size;
	pbuf[2] = size/256;
	pbuf[3] = size/65536;
	pdq = (intptr_t) &pbuf;
	snd_dtq(XML_DTQ,*pdq);
	slp_tsk();
	return pub;
}

void ros::Publisher::publish(char* data){
	if(ros_sem != 0){
		slp_tsk();
	}
	int size = strlen(data);
	int i = find_sub(node_lst,node_lst[find_id(node_lst,this->ID)].topic_name);

	if(i != -1){
		char sbuf[4];
		memcpy(&mem[PUB_ADDR],data,size);
		intptr_t *pdq;
		sbuf[0] = node_lst[i].ID;
		sbuf[1] = size;
		sbuf[2] = size/256;
		sbuf[3] = size/65536;
		pdq = (intptr_t) &sbuf;
		snd_dtq(SUB_DTQ,*pdq);
	}
	/*
	char pbuf[4];
	memcpy(&mem[PUB_ADDR],data,size);
	intptr_t *pdq;
	pbuf[0] = this->ID;
	pbuf[1] = size;
	pbuf[2] = size/256;
	pbuf[3] = size/65536;
	pdq = (intptr_t) &pbuf;
	snd_dtq(PUB_DTQ,*pdq);
*/
}




/**image data用関数なんかアレ**/
void ros::Publisher::imgpublish(ros_Image *img){
	if(ros_sem != 0){
		slp_tsk();
	}
	//とりあえずまずは共有メモリにぶち込んでいく方法でやる?
	//絶対時間やばい
	//ros_Image構造体のポインタを渡す？
	int size=0;
	char data[1000000];
	//Header
	//sprintf(&data[size],"%u",img->header.seq);
	data[size] = img->header.seq;
	data[size+1] = img->header.seq/256;
	data[size+2] = img->header.seq/65536;
	data[size+3] = 0;
	size += sizeof(img->header.seq);
	//sprintf(&data[size],"%c",img->header.sec);
	data[size] = img->header.sec;
	data[size+1] = img->header.sec/256;
	data[size+2] = img->header.sec/65536;
	data[size+3] = 0;
	size += sizeof(img->header.sec);
	//sprintf(&data[size],"%c",img->header.nsec);
	data[size] = img->header.nsec;
	data[size+1] = img->header.nsec/256;
	data[size+2] = img->header.nsec/65536;
	data[size+3] = 0;
	size += sizeof(img->header.nsec);
	//frame_id
	data[size] = strlen(img->header.frame_id.c_str());
	data[size+1] = strlen(img->header.frame_id.c_str())/256;
	data[size+2] = strlen(img->header.frame_id.c_str())/65536;
	data[size+3] = 0;
	size += 4;
	sprintf(&data[size],"%s",img->header.frame_id.c_str());
	size += strlen(img->header.frame_id.c_str());
	//rosImage
	//height
	//sprintf(&data[size],"%x",img->height);
	data[size] = img->height;
	data[size+1] = img->height/256;
	data[size+2] = img->height/65536;
	data[size+3] = 0;
	size += sizeof(img->height);
	//width
	//sprintf(&data[size],"%u",img->width);
	data[size] = img->width;
	data[size+1] = img->width/256;
	data[size+2] = img->width/65536;
	data[size+3] = 0;
	size += sizeof(img->width);
	//encoding
	data[size] = strlen(img->encoding.c_str());
	data[size+1] = strlen(img->encoding.c_str())/256;
	data[size+2] = strlen(img->encoding.c_str())/65536;
	data[size+3] = 0;
	size += 4;
	sprintf(&data[size],"%s",img->encoding.c_str());
	size += strlen(img->encoding.c_str());
	//endian
	sprintf(&data[size],"%c",img->is_bigendian);
	size += sizeof(img->is_bigendian);
	//step
	//sprintf(&data[size],"%u",img->step);
	data[size] = img->step;
	data[size+1] = img->step/256;
	data[size+2] = img->step/65536;
	data[size+3] = 0;
	size += sizeof(img->step);
	//data
	data[size] = img->step*img->height;
	data[size+1] = img->step*img->height/256;
	data[size+2] = img->step*img->height/65536;
	data[size+3] = 0;
	size += 4; 	//length space
	memcpy(&data[size],img->data,sizeof(img->data));
	size += sizeof(img->data);
	//ROS_INFO("img publish function [%d]",img->data[100]);
	//syslog(LOG_NOTICE,"size [%d]",size);
	memcpy(&mem[PUB_ADDR],data,size);
	char pbuf[4];
	intptr_t *pdq;
	pbuf[0] = this->ID;
	pbuf[1] = size;
	pbuf[2] = size/256;
	pbuf[3] = size/65536;
	pdq = (intptr_t) &pbuf;
	snd_dtq(PUB_DTQ,*pdq);
}

/**検証用ダミー関数**/
void ros::Publisher::publish_dummy(){
	if(ros_sem != 0){
		slp_tsk();
	}
	int size = 50;
	char pbuf[4];
	intptr_t *pdq;
	pbuf[0] = this->ID;
	pbuf[1] = size;
	pbuf[2] = size/256;
	pbuf[3] = size/65536;
	pdq = (intptr_t) &pbuf;
	snd_dtq(PUB_DTQ,*pdq);
}

void ros::Rate::sleep(){
	wait_ms(1000/this->rate);
}

void ros::spin(){
	slp_tsk();
}
