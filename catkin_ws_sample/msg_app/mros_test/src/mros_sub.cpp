#include "ros/ros.h"
#include "mros_test/UserTypeTest.h"
#include <sstream>

using namespace ros; 


void mrosCallback(mros_test::UserTypeTest msg){
  ROS_INFO("I heard from [%s %s]",msg.nameVal.firstName.c_str(),msg.nameVal.lastName.c_str());
  ROS_INFO("red value [%f]",msg.ledVal.red);
  ROS_INFO("green value [%f]",msg.ledVal.green);
  ROS_INFO("blue value [%f]",msg.ledVal.blue);
}

int main(int argc,char **argv){
  init(argc,argv,"mros_listener");

  NodeHandle n;

  Subscriber sub = n.subscribe("mros_msg",1000,mrosCallback);

  spin();

  return 0;
}
