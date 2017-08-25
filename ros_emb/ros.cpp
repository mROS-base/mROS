#include "ros_emb.h"

void ros::init(int argc,char *argv,const char *node_name){

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
	Subscriber sub;
	char tmp[8];
	sprintf(tmp,"%d",fp);
	std::string sstr;
	sstr = "<methodCall>registerSubscriber</methodCall>\n";
		sstr += "<topic_name>";
		sstr += topic;
		sstr += "</topic_name>\n";
		sstr += "<topic_type>std_msgs/String</topic_type>\n";
		sstr += "<caller_id>/mros_node</caller_id>\n";
		sstr += "<message_definition>std_msgs/String</message_definition>\n";
		sstr += "<fptr>";
		sstr += tmp;
		sstr += "</fptr>\n";
		intptr_t *sdq;
		//syslog(LOG_NOTICE,"subscriber: fptr [%s]",tmp);
		memcpy(mem,sstr.c_str(),strlen(sstr.c_str()));
		//syslog(LOG_NOTICE,"USR_TASK:length [%d]",strlen(sstr.c_str()));
		char sbuf[3];
		sbuf[0] = 0;		//IDここでは考えなくてよさそう
		sbuf[1] = 0;		//共有メモリオフセット
		int ssize = strlen(sstr.c_str());
		sbuf[2] = ssize;
		sbuf[3] = ssize/256;
		//for(int i=0;i<4;i++){
		//syslog(LOG_NOTICE,"USR_TASK: data [%x]",sbuf[i]);
		//}
		sdq = (intptr_t) &sbuf;
		//syslog(LOG_NOTICE,"USR_TASK:data [%8x]",*dq);
		snd_dtq(XML_DTQ,*sdq); //sndはデータ本体を渡す？big-little?なエンディアン 20b1->1b02で渡される
	return sub;
}

ros::Publisher ros::NodeHandle::advertise(std::string topic,int queue_size){
	Publisher pub;
	string pstr;
			pstr = "<methodCall>registerPublisher</methodCall>\n";
			pstr += "<topic_name>/mros_msg</topic_name>\n";
			pstr += "<topic_type>std_msgs/String</topic_type>\n";
			pstr += "<caller_id>/mros_node2</caller_id>\n";
			pstr += "<message_definition>std_msgs/String</message_definition>\n";
			pstr += "<fptr>12345671</fptr>\n";
			//syslog(LOG_NOTICE,"USR_TASK: data [%s]",pstr.c_str());
			intptr_t *pdq;
			int offset = 2;
			memcpy(&mem[256*offset],pstr.c_str(),strlen(pstr.c_str()));
			//syslog(LOG_NOTICE,"USR_TASK:length [%d]",strlen(pstr.c_str()));
			char pbuf[3];
			pbuf[0] = 0;		//IDここでは考えなくてよさそう
			pbuf[1] = offset;		//共有メモリオフセット
			int psize = strlen(pstr.c_str());
			pbuf[2] = psize;
			pbuf[3] = psize/256;
			//for(int i=0;i<4;i++){
			//syslog(LOG_NOTICE,"USR_TASK: pdata [%x]",pbuf[i]);
			//}
			pdq = (intptr_t) &pbuf;
			//syslog(LOG_NOTICE,"USR_TASK:data [%8x]",*dq);
			snd_dtq(XML_DTQ,*pdq); //sndはデータ本体を渡す？big-little?なエンディアン 20b1->1b02で渡される
	return pub;
}

void ros::Publisher::publish(char* data){
			int size = strlen(data);
			char pbuf[4];
			int offset = 4;
			memcpy(&mem[256*offset],data,size);
			intptr_t *pdq;
			pbuf[0] = 1;
			pbuf[1] = offset;
			pbuf[2] = size;
			pbuf[3] = size/256;
			pdq = (intptr_t) &pbuf;
			snd_dtq(PUB_DTQ,*pdq);
			slp_tsk();
}

void ros::Rate::sleep(){
	wait_ms(this->rate);
}

void ros::spine(){
	slp_tsk();
}

/*
bool ros::ok(){
	return true;
}


void ros::spineOnce(){

}
*/

void ros_info(const char c,char cc){
	printf("%s,%s",c,cc);
}
