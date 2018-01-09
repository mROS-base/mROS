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


#define LOOP (200)
//-->TCPSocketConnection.cppp
extern SYSUTM time3[1000];
extern SYSUTM time4[1000];
SYSUTM stime1[1000];
SYSUTM stime2[1000];
int evl_count;

/*****mROS user task code*******/
void usr_task1(){
#ifndef _USR_TASK_1_
#define _USR_TASK_1_
	SYSUTM time1[1000];
	SYSUTM time2[1000];
	//SYSUTM time[1000];
	syslog(LOG_NOTICE,"========Activate user task1========");

	int argc = 0;
	char *argv = NULL;
	ros::init(argc,argv,"mros_node");
	ros::NodeHandle n;
	evl_count = 0;
	ros::Publisher chatter_pub = n.advertise("mros_msg", 1);
	ros::Rate loop_rate(10);
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


	for(unsigned int i=0;i<18;i++){
		msg += msg;
		//ROS_INFO("data size[%d]",msg.size());
		evl_count = 1;
		wait_ms(1000);
		sus_tsk(LOGTASK);
	while(evl_count <= LOOP){
		get_utm(&time1[evl_count-1]);
		chatter_pub.publish(msg.c_str());
		get_utm(&time2[evl_count-1]);
		//time[i] += (time2 - time1);
		loop_rate.sleep();
		evl_count++;
	}

	rsm_tsk(LOGTASK);
	ROS_INFO("========================================");
	ROS_INFO("data size[%d]",msg.size());
	for(unsigned int j=0;j<LOOP;j++){
		ROS_INFO("%ld",time2[j]-time1[j]);
		dly_tsk(10);
	}
	}
	/*
	ROS_INFO("===============================");
	for(unsigned int j=0;j<LOOP;j++){
		ROS_INFO("%ld",stime2[j] - stime1[j]);
		dly_tsk(10);
	}
*/
	/*
	rsm_tsk(LOGTASK);
	for(unsigned int i=0;i<20;i++){
		ROS_INFO("%ld",time[i]/200);
		dly_tsk(10);
	}
	ROS_INFO("===============================");

	for(unsigned int i=0;i<LOOP;i++){
		ROS_INFO("%ld",time2[i]);
		dly_tsk(10);
	}*/
}
void Callback(string *msg){

}

/*****mROS user task code*******/
void usr_task2(){

#ifndef _USR_TASK_2_
#define _USR_TASK_2_
	syslog(LOG_NOTICE,"========Activate user task2========");
	int argc = 0;
	char *argv = NULL;
	ros::init(argc,argv,"mros_node");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscriber("mros_msg", 1,Callback);
	/*
	SYSUTM time1[1000];
	SYSUTM time2[1000];
	ros::Rate loop_rate(100);
	*/
#endif
	/*
	TCPSocketConnection tcp;
	tcp.connect("192.168.11.4",11411);
	ROS_INFO("connected");
	string msg = "1";
		int count=0;
		sus_tsk(LOGTASK);
		for(unsigned int i=0;i<10;i++){
			msg += msg;
		}
		while(count < LOOP){
			get_utm(&time1[count]);
			tcp.send(msg.c_str(),1024);
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
		*/
}
