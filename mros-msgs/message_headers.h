
#include "std_msgs/UInt32MultiArray.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/String.h"



void callCallback(int id, void (*fp)(intptr_t), char *rbuf){
	switch(id){
		case 4:
			subtask_methods::CallCallbackFuncs<4>().call(fp,rbuf);
			break;
		case 6:
			subtask_methods::CallCallbackFuncs<6>().call(fp,rbuf);
			break;
		case 1:
			subtask_methods::CallCallbackFuncs<1>().call(fp,rbuf);
			break;
	
	
	}
}

std::string getMD5Sum(int id){
	switch(id){
		case 4:
			return message_traits::MD5Sum<4>().value();
			break;
		case 6:
			return message_traits::MD5Sum<6>().value();
			break;
		case 1:
			return message_traits::MD5Sum<1>().value();
			break;
	
	
	}
}