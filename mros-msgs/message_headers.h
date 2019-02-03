

#include "mros_test/PersonalData.h"


void callCallback(int id, void (*fp)(intptr_t), char *rbuf){
	switch(id){
	
		case 102:
			subtask_methods::CallCallbackFuncs<102>().call(fp,rbuf);
			break;
	
	}
}

std::string getMD5Sum(int id){
	switch(id){
	
		case 102:
			return message_traits::MD5Sum<102>().value();
			break;
	
	}
}