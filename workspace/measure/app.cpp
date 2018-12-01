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


#define LOOP (250)
//-->TCPSocketConnection.cppp
/*
extern SYSUTM time3[1000];
extern SYSUTM time4[1000];
extern SYSUTM time5[1000];
extern SYSUTM time6[1000];
extern SYSUTM stime1[2000];
SYSUTM stime2[2000];
int evl_count;
int rcv_count;
*/
/*****mROS user task code*******/
int call_count=0;
void usr_task1(){
#ifndef _USR_TASK_1_
#define _USR_TASK_1_
	SYSUTM time1[1000];
	SYSUTM time2[1000];
	syslog(LOG_NOTICE,"========Activate user task1========");
	int argc = 0;
	char *argv = NULL;
	ros::init(argc,argv,"mros_node");
	ros::NodeHandle n;
	int evl_count;
	evl_count = 0;
	ros::Publisher chatter_pub = n.advertise("mros_msg", 1);
	ros::Rate loop_rate(10);
#endif

/*
	char buf[64];
	int count=0;
	while(1){
			sprintf(buf,"internal msg [%d]",count);
			chatter_pub.publish(buf);
			loop_rate.sleep();
			count++;
		}
*/

	std_msgs::String msg;
	msg.data = "1";


	for(unsigned int i=0;i<19;i++){
		msg.data += msg.data;
	}
		ROS_INFO("data size[%d]",msg.data.size());
		evl_count = 1;
		dly_tsk(1000);
		sus_tsk(LOGTASK);

	while(evl_count <= LOOP){
		get_utm(&time1[evl_count-1]);
		chatter_pub.publish(msg);
		get_utm(&time2[evl_count-1]);
		loop_rate.sleep();
		evl_count++;
	}

	rsm_tsk(LOGTASK);
	ROS_INFO("========================================\ncall_count[%d]",call_count);
	ROS_INFO("data size[%d]",msg.data.size());
	for(unsigned int j=50;j<LOOP;j++){
		ROS_INFO("%ld",time2[j]-time1[j]);
		dly_tsk(10);
	}
	/*
	ROS_INFO("========================================");
			ROS_INFO("data size[%d]",msg.size());
			for(unsigned int j=50;j<LOOP;j++){
				ROS_INFO("%ld",time6[j]-time5[j]);
				dly_tsk(10);
	}
	//sus_tsk(LOGTASK);
	//evl_count = 1;
		ROS_INFO("========================================");
		ROS_INFO("data size[%d]",msg.size());
		for(unsigned int j=50;j<LOOP;j++){
			ROS_INFO("%ld",time4[j]-time3[j]);
			dly_tsk(10);
		}
*/

}
void Callback(string *msg){
call_count++;
/*
	get_utm(&stime2[rcv_count-1]);
	ROS_INFO("Call back: rcv_count = [%d]",rcv_count);
	rcv_count++;
	if(rcv_count == LOOP+1){
		rsm_tsk(LOGTASK);
		ROS_INFO("============================================");
		for(int i=50;i<LOOP;i++){
			ROS_INFO("%ld",stime2[i]-stime1[i]);
			dly_tsk(10);
		}
		sus_tsk(LOGTASK);
		rcv_count = 1;
	}

	/*
	if(*msg == "end"){
		ROS_INFO("[%s]",msg->c_str());
		/*
		for(int i=0;i<200*10;i++){
			ROS_INFO("%d",stime2-stime1);
			dly_tsk(10);
		}
		}
		*/
	if(msg->size() != 1024){
	ROS_INFO("error size");
	}

}

/*****mROS user task code*******/
void usr_task2(){

#ifndef _USR_TASK_2_
#define _USR_TASK_2_
	syslog(LOG_NOTICE,"========Activate user task2========");
	int argc = 0;
	char *argv = NULL;
	//rcv_count = 0;
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
