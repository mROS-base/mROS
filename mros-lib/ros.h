//ros.h below

#ifndef _ROS_HEADER_
#define _ROS_HEADER_

#include <string>
#include <vector>
#include "mros.h"



//==========================node_name==============================================================================================================================
//========================================================================================================================================================
//========================================================================================================================================================
/******************************************************
 * Node information structure for XML-MAS TASK        *
 * ****************************************************/
typedef struct node{
		std::string node_name;				//Node_Name
		bool node_type;						//true->subscriber false->publisher
		bool pub_stat;				//publishing state
		char ID;							//for mROS ID>1
		std::string topic_name;				//ROS
		std::string topic_type;				//ROS
		std::string callerid;				//ROS
		std::string message_definition;		//ROS
		std::string uri;					//ROS	自ノードのURI
		int port;							//for sub　通信相手となるノードのXML-RPC受付ポート
		std::string fptr;					//for sub コールバック関数のポインタ
		std::string ip;						//for sub 通信相手となるノードのIP
public:
		node(){ this->pub_stat = false;};
		void set_node_type(bool type){this->node_type=type;};
		void set_pub(){this->pub_stat = true;};
		void set_ID(char c){this->ID = c;};
		void set_topic_name(std::string t){this->topic_name=t;};
		void set_topic_type(std::string t){this->topic_type=t;};
		void set_callerid(std::string t){this->callerid=t;};
		void set_message_definition(std::string t){this->message_definition=t;};
		void set_uri(std::string t){this->uri=t;};
		void set_port(int t){this->port=t;};
		void set_fptr(std::string t){this->fptr=t;};
		void set_ip(std::string t){this->ip = t;};
}node;

/******mROS node list***********/
extern int find_node(std::vector<node> list,std::string topic);
extern int find_id(std::vector<node> list,char ID);
//同一デバイス通信用
extern int find_sub(std::vector<node> list,std::string topic);

//========================================================================================================================================================
//========================================================================================================================================================
//========================================================================================================================================================
//========================================================================================================================================================


class Header{
public:
	unsigned int  seq;
	unsigned int sec;
	unsigned int nsec;
	std::string frame_id;
};



namespace sensor_msgs{
class Image{
public:
	Header header;
	unsigned int height;
	unsigned int width;
	std::string encoding;
	unsigned char is_bigendian;
	unsigned int step;
	unsigned char *data;
};
}

namespace message_traits
{
	template <class T>
	struct MD5Sum{static const char* value();};

	template <class T>
	struct DataType{static const char* value();};

	template <class T>
	struct Definition{static const char* value();};
}

//========================================================================================================================================================
//========================================================================================================================================================
//========================================================================================================================================================


namespace ros{

class Publisher{
public:
	char topic;
	char node;
	template <class T>
	void publish(T& data);
	void publish(sensor_msgs::Image& img);
	char ID;
};

typedef class Subscriber{
public:
	char topic;
	char node;
	char ID;

}Subscriber;

void init(int argc,char *argv,std::string node_name);

//現状キューサイズは機能していない
class NodeHandle{
	//Subscriber sub;
	//Publisher pub;
public:
	template<class T>
	Subscriber subscribe(std::string topic,int queue_size,void(*fp)(T));
	template <class T>
	Publisher advertise(std::string topic,int queue_size);

	Publisher advertise(std::string topic,int queue_size);
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


#define ROS_INFO(...) syslog(LOG_NOTICE,__VA_ARGS__)

#endif

