#ifndef _TCP_ROS_H_
#define _TCP_ROS_H_

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "mbed.h"
#include "EthernetInterface.h"


void add_len(char *buf,int len);
int pub_gen_header(char *buf,string id,string msg_def,string topic,string type,string md5);
int sub_gen_header(char *buf,string id,string nodelay,string topic,string type,string md5);
int pub_gen_msg(char *buf,char *msg);
int pub_gen_dummy(char *buf);
void pibot(char *data,int n);
int pub_gen_img_msg(char *buf,char *msgint,int size);
bool check_head(char *buf);



#endif
