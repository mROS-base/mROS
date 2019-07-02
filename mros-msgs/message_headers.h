
#include "std_msgs/String.h"

#include "mros_test/PersonalData.h"


void callCallback(int id, void (*fp)(intptr_t), char *rbuf){
	switch(id){
		case 9:
			subtask_methods::CallCallbackFuncs<9>().call(fp,rbuf);
			break;
	
		case 103:
			subtask_methods::CallCallbackFuncs<103>().call(fp,rbuf);
			break;
	
	}
}

std::string getMD5Sum(int id){
	switch(id){
		case 9:
			return message_traits::MD5Sum<9>().value();
			break;
	
		case 103:
			return message_traits::MD5Sum<103>().value();
			break;
	
	}
}