#include "ros.h"


#include "mros_test/PersonalData.h"



template ros::Subscriber ros::NodeHandle::subscribe(std::string,int,void (*fp)(mros_test::PersonalData*));
template ros::Publisher ros::NodeHandle::advertise<mros_test::PersonalData>(string, int);
template void ros::Publisher::publish(mros_test::PersonalData&);