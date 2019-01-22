
#include "std_msgs/UInt32MultiArray.h"

#include "mros_test/StrMsg.h"
#include "mros_test/LightSensorValues.h"
#include "mros_test/PersonalData.h"


void callCallback(int id, void (*fp)(intptr_t), char *rbuf){
	switch(id){
		case 4:
			subtask_methods::CallCallbackFuncs<4>().call(fp,rbuf);
			break;
	
		case 100:
			subtask_methods::CallCallbackFuncs<100>().call(fp,rbuf);
			break;
		case 101:
			subtask_methods::CallCallbackFuncs<101>().call(fp,rbuf);
			break;
		case 102:
			subtask_methods::CallCallbackFuncs<102>().call(fp,rbuf);
			break;
	
	}
}

std::string getMD5Sum(int id){
	switch(id){
		case 4:
			return message_traits::MD5Sum<4>().value();
			break;
	
		case 100:
			return message_traits::MD5Sum<100>().value();
			break;
		case 101:
			return message_traits::MD5Sum<101>().value();
			break;
		case 102:
			return message_traits::MD5Sum<102>().value();
			break;
	
	}
}