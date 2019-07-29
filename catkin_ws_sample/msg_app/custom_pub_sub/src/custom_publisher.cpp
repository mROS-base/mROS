#include "ros/ros.h"
#include "std_msgs/String.h"
#include "custom_pub_sub/UserTypeTest.h"

#include <string>
#include <sstream>

using namespace ros; 


int main(int argc,char **argv){
  init(argc,argv,"mros_talker");
  
  NodeHandle n;
  Publisher pub = n.advertise<custom_pub_sub::UserTypeTest>("test_msg",1000);
  Rate loop_rate(1);
  int count =0;
  bool set_msg=false;
  custom_pub_sub::UserTypeTest msg;
  std::stringstream ss;
  std::string nav;
  std::string str;
  int hoge = 0;
  int val;
  while(ok()){
      nav = "type first name";
      ROS_INFO(nav.c_str());
      std::cin >> str;
      msg.nameVal.firstName = str;
      nav = "type last name";
      ROS_INFO(nav.c_str());
      std::cin >> str;
      msg.nameVal.lastName = str;
      int ledVal;
      nav = "type red value (0-128)";
      ROS_INFO(nav.c_str());
      while (1) {
        if (scanf("%d", &ledVal) == 1 && ledVal >= 0 && ledVal <= 128){
          break; 
        }
        nav = "wrong input: please try again";
        ROS_INFO(nav.c_str());
        scanf("%*[^\n]");
      }
      msg.ledVal.red = (float)ledVal/128.0;
      while (1) {
        nav = "type green value (0-128)";
        ROS_INFO(nav.c_str());
        if (scanf("%d", &ledVal) == 1 && ledVal >= 0 && ledVal <= 128){
          break; 
        }
        nav = "wrong input: please try again";
        ROS_INFO(nav.c_str());
        scanf("%*[^\n]");
      }
      msg.ledVal.green = (float)ledVal/128.0;
      nav = "type blue value (0-128)";
      ROS_INFO(nav.c_str());
      while (1) {
        if (scanf("%d", &ledVal) == 1 && ledVal >= 0 && ledVal <= 128){
          break; 
        }
        nav = "wrong input: please try again";
        ROS_INFO(nav.c_str());
        scanf("%*[^\n]");
      }
      msg.ledVal.blue = (float)ledVal/128.0;
      pub.publish(msg);
  }
  return 0;
}
