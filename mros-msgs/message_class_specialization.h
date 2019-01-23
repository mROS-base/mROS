#include "ros.h"

#include "std_msgs/UInt32MultiArray.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/String.h"

#include "mros_test/LightSensorValues.h"
#include "mros_test/PersonalData.h"


template ros::Subscriber ros::NodeHandle::subscribe(std::string,int,void (*fp)(std_msgs::UInt32MultiArray*));
template ros::Publisher ros::NodeHandle::advertise<std_msgs::UInt32MultiArray>(string, int);
template void ros::Publisher::publish(std_msgs::UInt32MultiArray&);
template ros::Subscriber ros::NodeHandle::subscribe(std::string,int,void (*fp)(std_msgs::UInt32*));
template ros::Publisher ros::NodeHandle::advertise<std_msgs::UInt32>(string, int);
template void ros::Publisher::publish(std_msgs::UInt32&);
template ros::Subscriber ros::NodeHandle::subscribe(std::string,int,void (*fp)(std_msgs::String*));
template ros::Publisher ros::NodeHandle::advertise<std_msgs::String>(string, int);
template void ros::Publisher::publish(std_msgs::String&);

template ros::Subscriber ros::NodeHandle::subscribe(std::string,int,void (*fp)(mros_test::LightSensorValues*));
template ros::Publisher ros::NodeHandle::advertise<mros_test::LightSensorValues>(string, int);
template void ros::Publisher::publish(mros_test::LightSensorValues&);
template ros::Subscriber ros::NodeHandle::subscribe(std::string,int,void (*fp)(mros_test::PersonalData*));
template ros::Publisher ros::NodeHandle::advertise<mros_test::PersonalData>(string, int);
template void ros::Publisher::publish(mros_test::PersonalData&);