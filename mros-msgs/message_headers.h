
#include "std_msgs/UInt32MultiArray.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/String.h"

#include "mros_test/LightSensorValues.h"


void callCallback(int id, void (*fp)(intptr_t), char *rbuf){
	switch(id){
		case 17:
			subtask_methods::CallCallbackFuncs<17>().call(fp,rbuf);
			break;
		case 7:
			subtask_methods::CallCallbackFuncs<7>().call(fp,rbuf);
			break;
		case 9:
			subtask_methods::CallCallbackFuncs<9>().call(fp,rbuf);
			break;
	
		case 100:
			subtask_methods::CallCallbackFuncs<100>().call(fp,rbuf);
			break;
	
	}
}

std::string getMD5Sum(int id){
	switch(id){
		case 17:
			return message_traits::MD5Sum<17>().value();
			break;
		case 7:
			return message_traits::MD5Sum<7>().value();
			break;
		case 9:
			return message_traits::MD5Sum<9>().value();
			break;
	
		case 100:
			return message_traits::MD5Sum<100>().value();
			break;
	
	}
}