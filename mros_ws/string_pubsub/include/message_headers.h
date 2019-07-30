
#include "std_msgs/String.h"



static void callCallback(int id, void (*fp)(void *), char *rbuf){
	switch(id){
		case 9:
			subtask_methods::CallCallbackFuncs<9>().call(fp,rbuf);
			break;
	
	
	}
}

static const char* getMD5Sum(int id){
	switch(id){
		case 9:
			return message_traits::MD5Sum<9>().value();
	
	
	}
	return NULL;
}