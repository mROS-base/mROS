
#include "sensor_msgs/Image.h"



static void callCallback(int id, void (*fp)(void *), char *rbuf){
	switch(id){
		case 20:
			subtask_methods::CallCallbackFuncs<20>().call(fp,rbuf);
			break;
	
	
	}
}

static const char* getMD5Sum(int id){
	switch(id){
		case 20:
			return message_traits::MD5Sum<20>().value();
	
	
	}
	return NULL;
}