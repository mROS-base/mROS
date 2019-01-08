
#include "ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt16.h"
#include "mros_test/StrMsg.h"
#include "mros_test/LightSensorValues.h"
#include "mros_test/PersonalData.h"

std::vector<ID> IDv;
int ID_find(std::vector<ID> IDv,ID id){for(unsigned int i=0;i < IDv.size();i++){if(IDv[i] == id){return i;}}return -1;}

//node_name vector for TASK
/*　タスクごとにノードが生成可能にしているため必要
 * デバイス上で一つのノードとするなら必要ない?　　 */
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
#if 0
<<<<<<< HEAD
ros::Subscriber ros::NodeHandle::subscriber(std::string topic,std::string type,int queue_size,void (*fp)(std::string)){
	ID id;
	get_tid(&id);
	IDv.push_back(id);
	syslog(LOG_NOTICE,"usr task ID [%d]",id);
	while(ros_sem != 0){
=======
>>>>>>> mori_ws
#endif

template<class T>
ros::Subscriber ros::NodeHandle::subscribe(std::string topic,int queue_size,void (*fp)(T)){
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
#if 0
<<<<<<< HEAD
	sstr += "<topic_type>";
	sstr += type;
	sstr += "</topic_type>\n";
	sstr += "<caller_id>/mros_node2</caller_id>\n";
=======
#endif
	sstr += "<topic_type>";
	sstr += message_traits::DataType<T>().value();
	sstr += "</topic_type>\n";
	std::stringstream ss;
	ss <<  message_traits::DataTypeId<T>().value();
	sstr += "<topic_type_id>";
	sstr += ss.str();
	sstr += "</topic_type_id>\n";
	sstr += "<caller_id>/";
	sstr +=  node_nv[ID_find(IDv,id)].c_str();
	sstr+=	"</caller_id>\n";
//>>>>>>> mori_ws
	sstr += "<message_definition>";
	sstr += message_traits::Definition<T>().value();
	sstr += "</message_definition>\n";
	sstr += "<fptr>";
	sstr += tmp;
	sstr += "</fptr>\n";
	syslog(LOG_NOTICE,"hoge");
	syslog(LOG_NOTICE,sstr.c_str());
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
template ros::Subscriber ros::NodeHandle::subscribe(std::string,int,void (*fp)(std_msgs::String*));
template ros::Subscriber ros::NodeHandle::subscribe(std::string,int,void (*fp)(std_msgs::UInt32*));
template ros::Subscriber ros::NodeHandle::subscribe(std::string,int,void (*fp)(std_msgs::UInt16*));
template ros::Subscriber ros::NodeHandle::subscribe(std::string,int,void (*fp)(std_msgs::UInt8*));
template ros::Subscriber ros::NodeHandle::subscribe(std::string,int,void (*fp)(mros_test::StrMsg*));
template ros::Subscriber ros::NodeHandle::subscribe(std::string,int,void (*fp)(mros_test::LightSensorValues*));
template ros::Subscriber ros::NodeHandle::subscribe(std::string,int,void (*fp)(mros_test::PersonalData*));
#if 0
<<<<<<< HEAD
ros::Publisher ros::NodeHandle::advertise(string topic,string type,int queue_size){
	ID id;
	get_tid(&id);
	IDv.push_back(id);
	syslog(LOG_NOTICE,"usr task ID [%d]",id);
=======
#endif
template <class T>
ros::Publisher ros::NodeHandle::advertise(string topic,int queue_size){
//>>>>>>> mori_ws
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
#if 0
<<<<<<< HEAD
	pstr += "<topic_type>";
	pstr += type;
	pstr += "</topic_type>\n";
	pstr += "<caller_id>/mros_node</caller_id>\n";
=======
#endif
	//pstr += "<topic_type>std_msgs/String</topic_type>\n";
	pstr += "<topic_type>";
	pstr += message_traits::DataType<T*>().value();
	pstr += "</topic_type>\n";
	pstr += "<caller_id>/";
	pstr +=	node_nv[ID_find(IDv,id)].c_str();
	pstr += "</caller_id>\n";
//>>>>>>> mori_ws
	pstr += "<message_definition>";
	pstr += message_traits::DataType<T*>().value();
	pstr += "</message_definition>\n";
	syslog(LOG_NOTICE,message_traits::DataType<T*>().value());
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
template ros::Publisher ros::NodeHandle::advertise<std_msgs::String>(string, int);
template ros::Publisher ros::NodeHandle::advertise<std_msgs::UInt32>(string, int);
template ros::Publisher ros::NodeHandle::advertise<std_msgs::UInt16>(string, int);
template ros::Publisher ros::NodeHandle::advertise<std_msgs::UInt8>(string, int);
template ros::Publisher ros::NodeHandle::advertise<mros_test::StrMsg>(string, int);

template<class T>
void ros::Publisher::publish(T& data){
	ROS_INFO("PUBLISH STRING");
	while(ros_sem != 0){

	}
	int size = data.dataSize();
	int i = find_sub(node_lst,node_lst[find_id(node_lst,this->ID)].topic_name);

	if(i != -1){
		char sbuf[4];
		//memcpy(&mem[PUB_ADDR2],&data.data,size); //for int
		//memcpy(&mem[PUB_ADDR2],data.data.c_str(),size); //for string
		data.memCopy(&mem[PUB_ADDR2]);
		intptr_t *pdq;
		sbuf[0] = node_lst[i].ID;
		sbuf[1] = size;
		sbuf[2] = size/256;
		sbuf[3] = size/65536;
		pdq = (intptr_t) &sbuf;
		snd_dtq(SUB_DTQ,*pdq);
	}

	char pbuf[4];
	//memcpy(&mem[PUB_ADDR],&data.data,size); //for int
	//memcpy(&mem[PUB_ADDR],data.data.c_str(),size); //for string
	data.memCopy(&mem[PUB_ADDR]);
	intptr_t *pdq;
	pbuf[0] = this->ID;
	pbuf[1] = size;
	pbuf[2] = size/256;
	pbuf[3] = size/65536;
	pdq = (intptr_t) &pbuf;
	snd_dtq(PUB_DTQ,*pdq);

}
template void ros::Publisher::publish(std_msgs::String&);
template void ros::Publisher::publish(std_msgs::UInt16&);
template void ros::Publisher::publish(mros_test::StrMsg&);

#if 0
<<<<<<< HEAD
//とりあえずのimageデータ出版関数
//ここでTCPROSのデータに変換している方式
void ros::Publisher::imgpublish(ros_Image *img){
	if(ros_sem != 0){
		slp_tsk();
	}
	int size=0;
	char data[1000000];
=======
#endif



/**image data用関数なんかアレ**/
/*
void ros::Publisher::publish(sensor_msgs::Image& img){

	while(ros_sem != 0){

	}
	//とりあえずまずは共有メモリにぶち込んでいく方法でやる?
	//絶対時間やばい
	//ros_Image構造体のポインタを渡す？
	int size = 0;
//>>>>>>> mori_ws
	//Header
	//sprintf(&data[size],"%u",img->header.seq);
	char *data;

	int i = find_sub(node_lst,node_lst[find_id(node_lst,this->ID)].topic_name);
	bool p = node_lst[find_id(node_lst,this->ID)].pub_stat;
	if(i != -1){
		data = &mem[PUB_ADDR2];
	}
	if(p){
		data = &mem[PUB_ADDR];
	}
	data[size] = img.header.seq;
	data[size+1] = img.header.seq/256;
	data[size+2] = img.header.seq/65536;
	data[size+3] = 0;
	size += 4;
	//sprintf(&data[size],"%c",img.header.sec);
	data[size] = img.header.sec;
	data[size+1] = img.header.sec/256;
	data[size+2] = img.header.sec/65536;
	data[size+3] = 0;
	size += 4;
	//sprintf(&data[size],"%c",img.header.nsec);
	data[size] = img.header.nsec;
	data[size+1] = img.header.nsec/256;
	data[size+2] = img.header.nsec/65536;
	data[size+3] = 0;
	size += 4;
	//frame_id
	data[size] = strlen(img.header.frame_id.c_str());
	data[size+1] = strlen(img.header.frame_id.c_str())/256;
	data[size+2] = strlen(img.header.frame_id.c_str())/65536;
	data[size+3] = 0;
	size += 4;
	sprintf(&data[size],"%s",img.header.frame_id.c_str());
	size += strlen(img.header.frame_id.c_str());
	//rosImage
	//height
	//sprintf(&data[size],"%x",img.height);
	data[size] = img.height;
	data[size+1] = img.height/256;
	data[size+2] = img.height/65536;
	data[size+3] = 0;
	size += 4;
	//width
	//sprintf(&data[size],"%u",img.width);
	data[size] = img.width;
	data[size+1] = img.width/256;
	data[size+2] = img.width/65536;
	data[size+3] = 0;
	size += 4;
	//encoding
	data[size] = strlen(img.encoding.c_str());
	data[size+1] = strlen(img.encoding.c_str())/256;
	data[size+2] = strlen(img.encoding.c_str())/65536;
	data[size+3] = 0;
	size += 4;
	sprintf(&data[size],"%s",img.encoding.c_str());
	size += strlen(img.encoding.c_str());
	//endian
	sprintf(&data[size],"%c",img.is_bigendian);
	size += sizeof(img.is_bigendian);
	//step
	//sprintf(&data[size],"%u",img.step);
	data[size] = img.step;
	data[size+1] = img.step/256;
	data[size+2] = img.step/65536;
	data[size+3] = 0;
	size += 4;
	//data
	data[size] = img.step*img.height;
	data[size+1] = (img.step*img.height)/256;
	data[size+2] = (img.step*img.height)/65536;
	data[size+3] = 0;
	size += 4; 	//length space
	memcpy(&data[size],img.data,img.step*img.height);
	size += img.step*img.height;
	//ROS_INFO("size [%d]",size);
	if(i != -1){
		//ROS_INFO("internal publishing");
		char sbuf[4];
		intptr_t *pdq;
		sbuf[0] = node_lst[i].ID;
		sbuf[1] = size;
		sbuf[2] = size/256;
		sbuf[3] = size/65536;
		pdq = (intptr_t) &sbuf;
		snd_dtq(SUB_DTQ,*pdq);
	}

#if 0
<<<<<<< HEAD
//開発用のダミー関数
void ros::Publisher::publish_dummy(){
	if(ros_sem != 0){
		slp_tsk();
=======
#endif
	if(p){
		//ROS_INFO("PUBLISH ID[%d]",this->ID);
		char pbuf[4];
		intptr_t *pdq;
		pbuf[0] = this->ID;
		pbuf[1] = size;
		pbuf[2] = size/256;
		pbuf[3] = size/65536;
		pdq = (intptr_t) &pbuf;
		//ROS_INFO("SEND DTQ PUB");
		//dly_tsk(10);
		snd_dtq(PUB_DTQ,*pdq);
//>>>>>>> mori_ws
	}

}*/

void ros::Rate::sleep(){
	wait_ms(1000/this->rate);
}

void ros::spin(){
	slp_tsk();
}
