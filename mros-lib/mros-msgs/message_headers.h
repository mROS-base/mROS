
#include "std_msgs/UInt32.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/String.h"

#include "mros_test/StrMsg.h"


void callCallback(int id, void (*fp)(intptr_t), char *rbuf){
	switch(id){
		case 6:
			subtask_methods::CallCallbackFuncs<6>().call(fp,rbuf);
			break;
		case 10:
			subtask_methods::CallCallbackFuncs<10>().call(fp,rbuf);
			break;
		case 3:
			subtask_methods::CallCallbackFuncs<3>().call(fp,rbuf);
			break;
		case 1:
			subtask_methods::CallCallbackFuncs<1>().call(fp,rbuf);
			break;
	
		case 100:
			subtask_methods::CallCallbackFuncs<100>().call(fp,rbuf);
			break;
	
	}
}

std::string getMD5Sum(int id){
	switch(id){
		case 6:
			return message_traits::MD5Sum<6>().value();
			break;
		case 10:
			return message_traits::MD5Sum<10>().value();
			break;
		case 3:
			return message_traits::MD5Sum<3>().value();
			break;
		case 1:
			return message_traits::MD5Sum<1>().value();
			break;
	
		case 100:
			return message_traits::MD5Sum<100>().value();
			break;
	
	}
}