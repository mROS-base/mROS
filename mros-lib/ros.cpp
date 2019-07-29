
#include "ros.h"

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


template <class T>
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
	//pstr += "<topic_type>std_msgs/String</topic_type>\n";
	pstr += "<topic_type>";
	pstr += message_traits::DataType<T*>().value();
	pstr += "</topic_type>\n";
	pstr += "<caller_id>/";
	pstr +=	node_nv[ID_find(IDv,id)].c_str();
	pstr += "</caller_id>\n";
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

template<class T>
void ros::Publisher::publish(T& data){
	while(ros_sem != 0){

	}
	int size = data.dataSize();
	int i = find_sub(node_lst,node_lst[find_id(node_lst,this->ID)].topic_name);

	if(i != -1){
		char sbuf[4];
		//memcpy(&mem[PUB_ADDR2],&data.data,size); //for int
		//memcpy(&mem[PUB_ADDR2],data.data.c_str(),size); //for string
		char *memPtr = &mem[PUB_ADDR2];
		data.memCopy(memPtr);
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
	char *memPtr = &mem[PUB_ADDR];
	data.memCopy(memPtr);
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

#include "message_class_specialization.h"