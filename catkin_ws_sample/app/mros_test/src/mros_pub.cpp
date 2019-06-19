#include "ros/ros.h"
#include "std_msgs/String.h"

#include <string>
#include <sstream>

using namespace ros; 


int main(int argc,char **argv){
  init(argc,argv,"mros_talker");

  NodeHandle n;

  Publisher pub = n.advertise<std_msgs::String>("test_string",1000);

  Rate loop_rate(10);
  int count =0;
  bool set_msg=false; 
  while(ok()){
    std_msgs::String msg;
    std::stringstream ss;
    std::string str,nav;
    //ss << "Hello World!" << count;
    //ss << std::cin;
    //nav = "\n\n[Type LED color] \n [red] or [green] or [blue] or (reset)";
    nav = "Type what you send\n";
    ROS_INFO("%s",nav.c_str());
    std::cin >> str;
    msg.data = str;
    ROS_INFO("%s",msg.data.c_str());

    //if(set_msg){
    pub.publish(msg);
    set_msg = false;
    //	}
    spinOnce();
    loop_rate.sleep();
    ++count;    
  }
  return 0;
}
