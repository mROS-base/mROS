
#include "std_msgs/String.h"

#include "mros_test/PersonalData.h"


void callCallback(int id, void (*fp)(intptr_t), char *rbuf){
	switch(id){
		case 9:
			subtask_methods::CallCallbackFuncs<9>().call(fp,rbuf);
			break;
	
		case 102:
			subtask_methods::CallCallbackFuncs<102>().call(fp,rbuf);
			break;
	
	}
}

std::string getMD5Sum(int id){
	switch(id){
		case 9:
			return message_traits::MD5Sum<9>().value();
			break;
	
		case 102:
			return message_traits::MD5Sum<102>().value();
			break;
	
	}
}