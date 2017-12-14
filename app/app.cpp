#include "app.h"
#include "../mros-lib/ros.h"
//評価用プログラム


//mbed library 
/*
#include "mbed.h"
#include "EthernetInterface.h"
*/
#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include "syssvc/serial.h"
#include "syssvc/syslog.h"
#include "kernel_cfg.h"
#include "mbed.h"
#include "EthernetInterface.h"
#include "syssvc/logtask.h"


#define LOOP (1000)


/*****mROS user task code*******/
void usr_task1(){
#ifndef _USR_TASK_1_
#define _USR_TASK_1_
	SYSUTM time1[1000];
	SYSUTM time2[1000];
	unsigned long time[1000];
	syslog(LOG_NOTICE,"========Activate user task1========");
	int argc = 0;
	char *argv = NULL;
	ros::init(argc,argv,"mros_node");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise("mros_msg", 1);
	ros::Rate loop_rate(100);
#endif
/*
	string msg = "mROS MESSAGE";

	char buf[64];
	int count=0;
	while(1){
			sprintf(buf,"internal msg [%d]",count);
			chatter_pub.publish(buf);
			loop_rate.sleep();
			count++;
		}
*/
	string msg = "1";
	int count=0;
	sus_tsk(LOGTASK);
	for(unsigned int i=0;i<18;i++){
		msg += msg;
	}
	while(count < LOOP){
		get_utm(&time1[count]);
		chatter_pub.publish(msg.c_str());
		get_utm(&time2[count]);
		//time[count] = time2 - time1;
		loop_rate.sleep();
		count++;
	}
	rsm_tsk(LOGTASK);
	for(unsigned int i=0;i<LOOP;i++){
		ROS_INFO("%ld",time1[i]);
		dly_tsk(10);
	}
	ROS_INFO("===============================");
	for(unsigned int i=0;i<LOOP;i++){
		ROS_INFO("%ld",time2[i]);
		dly_tsk(10);
	}
}

/*******  callback **********/
void Callback(string *msg){	
	syslog(LOG_NOTICE,"I heard [%s]",msg->c_str());
}

/*****mROS user task code*******/
void usr_task2(){

#ifndef _USR_TASK_2_
#define _USR_TASK_2_

	syslog(LOG_NOTICE,"========Activate user task2========");
	int argc = 0;
	char *argv = NULL;
	ros::init(argc,argv,"mros_node2");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscriber("mros_msg",1,Callback);
	ros::spin();
#endif
}
