
#include "mros_test/StrMsg.h"

void callCallback(int id, void (*fp)(intptr_t), char *rbuf){
	switch(id){
		case 100:
			subtask_methods::CallCallbackFuncs<100>().call(fp,rbuf);
			break;
	
	}
}

std::string getMD5Sum(int id){
	switch(id){
		case 100:
			return message_traits::MD5Sum<100>().value();
			break;
	
	}
}