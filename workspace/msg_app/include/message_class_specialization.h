#include "ros.h"


#include "mros_test/UserTypeTest.h"



template ros::Subscriber ros::NodeHandle::subscribe(std::string,int,void (*fp)(mros_test::UserTypeTest*));
template ros::Publisher ros::NodeHandle::advertise<mros_test::UserTypeTest>(string, int);
template void ros::Publisher::publish(mros_test::UserTypeTest&);