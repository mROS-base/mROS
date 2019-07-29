#include "ros.h"

#include "sensor_msgs/Image.h"



template ros::Subscriber ros::NodeHandle::subscribe(std::string,int,void (*fp)(sensor_msgs::Image*));
template ros::Publisher ros::NodeHandle::advertise<sensor_msgs::Image>(string, int);
template void ros::Publisher::publish(sensor_msgs::Image&);
