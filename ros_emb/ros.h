#ifndef _ROS_H_
#define _ROS_H_

#include <string>
#include <stdio.h>


//ROSの関数をmROSにマッピングする

namespace ros{

typedef class Publisher{
public:
	Publisher(){};
	char topic;
	char node;

}Publisher;

typedef class Subscriber{
public:
	Subscriber(){};
	char topic;
	char node;

}Subscriber;

void init(int argc,char *argv,const char *node_name);


class NodeHandle{
	Subscriber sub;
	Publisher pub;
public:
	template <class M, class T> Subscriber subscriber(std::string topic,int queue_size,void(T::*fp)(M));
	template <class M> Subscriber subscriber(std::string topic,int queue_size,void(*fp)(M));
	template <class M > Publisher advertise(char topic,int queue_size);
};


//Rate;
//spineOnce();

}

class std_msgs{
public:
	class String{
	public:
		typedef class ConstPtr{
		public:
		std::string data;
		}ConstPtr;
	};
};

void ros_info(const char c,char cc);


#endif
