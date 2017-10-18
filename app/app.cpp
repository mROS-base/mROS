#include "app.h"
#include "../mros-lib/ros.h"

//mbed library 
#include "mbed.h"
#include "EthernetInterface.h"
#include "SoftPWM.h"

static DigitalOut ledu(P6_12);                                  // LED-User
static SoftPWM ledr(P6_13);                                     // LED-Red
static SoftPWM ledg(P6_14);                                     // LED-Green
static SoftPWM ledb(P6_15);                                     // LED-Blue

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



//コールバック関数
void Callback(string *msg){	//ConstPtr& msgの意味が分からん
	LED_switch(msg);
	ros_info("I heard [%s]",msg->c_str());
}

/*
void usr_task1(){

#ifndef _USR_TASK_
#define _USR_TASK_

	syslog(LOG_NOTICE,"========Activate user task1========");
	//テストコード_USR_TASK_
	led_init();
	int argc = 0;
	char *argv = NULL;
	ros::init(argc,argv,"/mros_node");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscriber("test_string",1,Callback);
	ros::spin();
#endif
}
*/
void usr_task2(){
#ifndef _USR_TASK_2_
#define _USR_TASK_2_

	syslog(LOG_NOTICE,"========Activate user task2========");
	int argc = 0;
	char *argv = NULL;
	ros::init(argc,argv,"mros_node2");
	ros::NodeHandle n;			//ノードハンドラ
	ros::Publisher chatter_pub = n.advertise("mros_msg", 1);		//パブリッシャとして登録XML-RPCをする
	ros::Rate loop_rate(200);
	//publish loop
#endif

	char* str;
	int count=0;
	while(1){
		sprintf(str,"mROS Hello!! [%d]",count);
		chatter_pub.publish(str);
		count++;
		loop_rate.sleep();
	}

}
