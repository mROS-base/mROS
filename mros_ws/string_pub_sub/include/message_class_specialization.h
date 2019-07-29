#include "ros.h"

#include "std_msgs/String.h"



template ros::Subscriber ros::NodeHandle::subscribe(std::string,int,void (*fp)(std_msgs::String*));
template ros::Publisher ros::NodeHandle::advertise<std_msgs::String>(string, int);
template void ros::Publisher::publish(std_msgs::String&);
