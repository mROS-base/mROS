#include "ros.h"


#include "custom_pub_sub/UserTypeTest.h"



template ros::Subscriber ros::NodeHandle::subscribe(std::string,int,void (*fp)(custom_pub_sub::UserTypeTest*));
template ros::Publisher ros::NodeHandle::advertise<custom_pub_sub::UserTypeTest>(string, int);
template void ros::Publisher::publish(custom_pub_sub::UserTypeTest&);