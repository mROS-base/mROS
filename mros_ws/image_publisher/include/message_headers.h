
#include "sensor_msgs/Image.h"



void callCallback(int id, void (*fp)(intptr_t), char *rbuf){
	switch(id){
		case 20:
			subtask_methods::CallCallbackFuncs<20>().call(fp,rbuf);
			break;
	
	
	}
}

std::string getMD5Sum(int id){
	switch(id){
		case 20:
			return message_traits::MD5Sum<20>().value();
			break;
	
	
	}
}