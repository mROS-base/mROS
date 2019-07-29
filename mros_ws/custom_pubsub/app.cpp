#include "app.h"
//TODO: catkin統合時におそらく名前衝突するのでなにかしら考える
//#include "../mros-lib/ros.h"
#include "ros.h"

//mbed library 
#include "mbed.h"
#include "EthernetInterface.h"
#include "SoftPWM.h"
#include "custom_pubsub/UserTypeTest.h"

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
static custom_pubsub::UserTypeTest pubMsg;
void usr_task1(){
#ifndef _USR_TASK_1_
#define _USR_TASK_1_

  syslog(LOG_NOTICE,"========Activate user task1========");
  int argc = 0;
  char *argv = NULL;
  ros::init(argc,argv,"mros_node");
  ros::NodeHandle n;
  //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("mros_msg",1);
  ros::Publisher chatter_pub = n.advertise<custom_pubsub::UserTypeTest>("mros_msg",1);
  ros::Rate loop_rate(5);
  pubMsg.nameVal.firstName="Charlie";
  pubMsg.nameVal.lastName="Parker";
  pubMsg.ledVal.red = 0.0;
  pubMsg.ledVal.green = 0.0;
  pubMsg.ledVal.blue = 0.0;
  syslog(LOG_NOTICE,"Data Publish Start");
  while(1){
    wait_ms(1000);
    chatter_pub.publish(pubMsg);
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
/*
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
*/
/*******  callback **********/
void Callback(custom_pubsub::UserTypeTest *msg){	
  //LED_switch(msg);
  ROS_INFO("I hear msgs from ros host");
  string name = msg->nameVal.firstName + " " + msg->nameVal.lastName;
  ROS_INFO("recieved name: %s", name.c_str());
  pubMsg.nameVal.firstName = msg->nameVal.firstName;
  pubMsg.nameVal.lastName = msg->nameVal.lastName;
  pubMsg.ledVal.red = msg->ledVal.red;
  pubMsg.ledVal.green = msg->ledVal.green;
  pubMsg.ledVal.blue = msg->ledVal.blue;
  ledr = msg->ledVal.red;
  ledg = msg->ledVal.green;
  ledb = msg->ledVal.blue;
 
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
