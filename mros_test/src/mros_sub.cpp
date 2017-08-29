#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

using namespace ros; 


void mrosCallback(const std_msgs::String::ConstPtr& msg){
      ROS_INFO("I heard [%s]",msg->data.c_str());
}

int main(int argc,char **argv){
    init(argc,argv,"mros_listener");
    
    NodeHandle n;

    Subscriber sub = n.subscribe("mros_msg",1000,mrosCallback);

    spin();

    return 0;
}
