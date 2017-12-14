#ifndef _ROS_HEADER_
#define _ROS_HEADER_

#include <string>
#include "mros.h"

class Header{
public:
	unsigned int  seq;
	unsigned int sec;
	unsigned int nsec;
	std::string frame_id;
};

class ros_Image{
public:
	Header header;
	unsigned int height;
	unsigned int width;
	std::string encoding;
	unsigned char is_bigendian;
	unsigned int step;
	unsigned char data[320*4*240];
};

namespace ros{

typedef class Publisher{
public:
	char topic;
	char node;
	void publish(char *data);
	void imgpublish(ros_Image *data);
	void publish_dummy();
	char ID;
}Publisher;

typedef class Subscriber{
public:
	char topic;
	char node;
	char ID;

}Subscriber;

void init(int argc,char *argv,const char *node_name);

//現状キューサイズは機能していない
class NodeHandle{
	Subscriber sub;
	Publisher pub;
public:
	Subscriber subscriber(std::string topic,std::string type,int queue_size,void(*fp)(std::string));
	Publisher advertise(std::string topic,std::string type,int queue_size);
};

class Rate{
	int rate;
public:
	Rate(int rate){this->rate = rate;};
	void sleep();
};

void spin();
//bool ok();
//void spinOnce();

}
/*
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
*/


#define ROS_INFO(...) syslog(LOG_NOTICE,__VA_ARGS__)

#endif
