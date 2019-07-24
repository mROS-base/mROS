#include "ros.h"

#include "std_msgs/String.h"

#include "mros_test/PersonalData.h"


template ros::Subscriber ros::NodeHandle::subscribe(std::string,int,void (*fp)(std_msgs::String*));
template ros::Publisher ros::NodeHandle::advertise<std_msgs::String>(string, int);
template void ros::Publisher::publish(std_msgs::String&);

template ros::Subscriber ros::NodeHandle::subscribe(std::string,int,void (*fp)(mros_test::PersonalData*));
template ros::Publisher ros::NodeHandle::advertise<mros_test::PersonalData>(string, int);
template void ros::Publisher::publish(mros_test::PersonalData&);