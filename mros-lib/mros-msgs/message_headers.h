#include "std_msgs/String.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt32.h"

std::string getMD5Sum(int id){
	switch(id){
		case 1:
			return message_traits::MD5Sum<1>().value();
			break;
		case 3:
			return message_traits::MD5Sum<3>().value();
			break;
		case 6:
			return message_traits::MD5Sum<6>().value();
			break;
		case 10:
			return message_traits::MD5Sum<10>().value();
			break;
	}
}

void callCallback(int id, void (*fp)(intptr_t), char *rbuf){
	switch(id){
		case 1:
			subtask_methods::CallCallbackFuncs<1>().call(fp,rbuf);
			break;
		case 3:
			subtask_methods::CallCallbackFuncs<3>().call(fp,rbuf);
			break;
		case 6:
			subtask_methods::CallCallbackFuncs<6>().call(fp,rbuf);
			break;
		case 10:
			subtask_methods::CallCallbackFuncs<10>().call(fp,rbuf);
			break;
	}
}