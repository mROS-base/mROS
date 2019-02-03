#include "app.h"
//TODO: catkin統合時におそらく名前衝突するのでなにかしら考える
//#include "../mros-lib/ros.h"
#include "ros.h"

//mbed library 
#include "mbed.h"
#include "EthernetInterface.h"
#include "SoftPWM.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/UInt32MultiArray.h"
#include "mros_test/PersonalData.h"

//pin assign
static DigitalOut ledu(P6_12);                                  // LED-User
static SoftPWM ledr(P6_13);                                     // LED-Red
static SoftPWM ledg(P6_14);                                     // LED-Green
static SoftPWM ledb(P6_15);                                     // LED-Blue

//超音波センサHC-SR04の関数
//参考（https://developer.mbed.org/users/haru36rr/notebook/hcsr04_operation_check/）
static DigitalOut USSTriger (P2_14);         //P11 :超音波センサ トリガ出力
Timer ActiveTime;
/* 割り込み処理宣言 */
Ticker TrigerTiming;                //Trigerピン :インターバルタイマ
static InterruptIn USSEcho (P2_15);          //p12 :超音波センサ  エコー入力
unsigned short USSDistance;         //USSDistance:超音波センサ測定距離
static DigitalIn Button (P6_0);		//ユーザボタン

void Triger (){
  USSTriger = 1;
  wait_us(10);
  USSTriger = 0;
}

void RiseEcho(){
  ActiveTime.start();
}

void FallEcho(){
  unsigned long ActiveWidth;
  ActiveTime.stop();
  ActiveWidth = ActiveTime.read_us();
  USSDistance = ActiveWidth * 0.0170;
  ActiveTime.reset();
}

void init(void){
  TrigerTiming.attach( Triger , 0.060 );      //USSTriger周期 60ms
  USSEcho.rise( RiseEcho );                   //USSEcho立ち上がり時割り込み
  USSEcho.fall( FallEcho );                   //USSEcho立ち下がり時割り込み
}

/*****mROS user task code*******/

void usr_task1(){
#ifndef _USR_TASK_1_
#define _USR_TASK_1_

  syslog(LOG_NOTICE,"========Activate user task1========");
  int argc = 0;
  char *argv = NULL;
  ros::init(argc,argv,"mros_node");
  ros::NodeHandle n;
  //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("mros_msg",1);
  ros::Publisher chatter_pub = n.advertise<mros_test::PersonalData>("mros_msg",1);
  ros::Rate loop_rate(5);
  mros_test::PersonalData msg;
  msg.floatVal = 1.919;
  msg.doubleVal = 11.4514;
  msg.boolVal = true;
  /*
  mros_test::PersonalData msg;
  msg.first_name = "Charlie";
  msg.last_name = "Parker";
  msg.age = 32;
  msg.lsValue.person1 = 0;
  msg.lsValue.person2.push_back(10);
  msg.lsValue.name = "hoge";

  for (int i = 1 ; i <= 10 ; i ++)
  {
      msg.score.push_back(i);
  }
  */
  //std_msgs::String str;
  //std_msgs::UInt32 str;
  //str.data = 4294967200;

  //char msg[100];
  /*
  mros_test::StrMsg msg;
  msg.baker = "unchi";
  msg.parker = "kongutte";
  msg.coltrane = "shitteru????";
  */
  //mros_test::PersonalData msg;
  //msg.first_name = "Phil";
  //msg.last_name = "Woods";
  //msg.age = 83;
  //msg.score = 100000;
  int count=0;
  init();
  bool b = false;
  bool bb = true;
  syslog(LOG_NOTICE,"Data Publish Start");
  while(1){
    /*
    if(Button.read() == 0 && bb){
      b = !b;
      bb = false;
    }else if(Button.read() == 1){
      bb = true;
    }
    if(b){
      wait_ms(1000);
      //sprintf(msg,"Distance[%d]cm\0",USSDistance);
      std::ostringstream s;
      s << "Distance[" << USSDistance << "]cm\0" << std::flush;
      msg.data = s.str();
      syslog(LOG_NOTICE, "%s", msg.data.c_str());
      chatter_pub.publish(msg);
      loop_rate.sleep();
    }*/
    wait_ms(1000);
    //ROS_INFO("USER TASK1: publishing string,%s",msg.parker);
    chatter_pub.publish(msg);
    msg.floatVal += 0.1;
    msg.doubleVal += 0.01;
    msg.boolVal = !msg.boolVal;
  //  msg.lsValue.person1 += 2;
  //  msg.age ++;
  ////  count ++;
  }
#endif
}


/******* LED for mbed library　*******/
void led_init(){
  ledu = 0;
  ledr.period_ms(10);
  ledr = 0.0f;
  ledg.period_ms(10);
  ledg = 0.0f;
  ledb.period_ms(10);
  ledb = 0.0f;
}

void LED_switch(string *msg){
  if(msg->find("red") != -1){
    if(ledr == 0){
      ledr = (float)100/128;//LED RED
    }else{
      ledr = 0;
    }
  }else if(msg->find("blue") != -1){
    if(ledb == 0){
      ledb = (float)100/128;		//LED BLUE
    }else{
      ledb = 0;
    }
  }else if(msg->find("green") != -1){
    if(ledg == 0){
      ledg = (float)100/128;		//LED GREEN
    }else{
      ledg = 0;
    }
  }else if(msg->find("reset") != -1){
    ledr = 0;
    ledg = 0;
    ledb = 0;
  }else if(msg->find("end") != -1){
    syslog(LOG_NOTICE,"GOOD BYE !!");
    exit(1);
  }
}

/*******  callback **********/
void Callback(mros_test::PersonalData *msg){	
  //LED_switch(msg);
  ROS_INFO("I hear msgs from ros host");
  ROS_INFO("age: %1.4f", msg->floatVal);
  ROS_INFO("age: %2.5lf", msg->doubleVal);
  if(msg->boolVal){
    ROS_INFO("bool val is true");
  } else {
    ROS_INFO("bool val is false");
  }
  /*
  ROS_INFO("name: %s", (msg->first_name + " " + msg->last_name).c_str() );
  ROS_INFO("age: %u", msg->age);
  ROS_INFO("lsValue.person1: %u", msg->lsValue.person1);
  ROS_INFO("lsValue.person2:");
  for(std::vector<uint32_t>::const_iterator it = msg->lsValue.person2.begin(); it != msg->lsValue.person2.end(); ++it)
  {
      ROS_INFO("%d", *it);
  }
  ROS_INFO("lsValue.name: %s", msg->lsValue.name.c_str());
  ROS_INFO("score:");
  for(std::vector<uint32_t>::const_iterator it = msg->score.begin(); it != msg->score.end(); ++it)
  {
      ROS_INFO("%d", *it);
  }
  */
}

/*****mROS user task code*******/
void usr_task2(){

#ifndef _USR_TASK_2_
#define _USR_TASK_2_

  syslog(LOG_NOTICE,"========Activate user task2========");
  led_init();
  ledg = (float)100/128;
  int argc = 0;
  char *argv = NULL;
  ros::init(argc,argv,"mros_node2");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("test_msg",1,Callback);
  ros::spin();
#endif
}
