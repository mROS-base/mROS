#include "ros.h"


void ros::init(int argc,char *argv,const char *node_name){

}

template<class M,class T> ros::Subscriber ros::NodeHandle::subscriber(std::string topic,int queue_size,void (T::*fp)(M)){
	Subscriber sub;
	return sub;
}
template<class M> ros::Subscriber ros::NodeHandle::subscriber(std::string topic,int queue_size,void (*fp)(M)){
	Subscriber sub;
	return sub;
}
template<class M> ros::Publisher ros::NodeHandle::advertise(char topic,int queue_size){
	Publisher pub;
	return pub;
}

void ros_info(const char c,char cc){
	printf("%s,%s",c,cc);
}
