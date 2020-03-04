#include "ros.h"


#include "custom_pubsub/UserTypeTest.h"



template ros::Subscriber ros::NodeHandle::subscribe(std::string,int,void (*fp)(custom_pubsub::UserTypeTest*));
template ros::Publisher ros::NodeHandle::advertise<custom_pubsub::UserTypeTest>(std::string, int);
template void ros::Publisher::publish(custom_pubsub::UserTypeTest&);