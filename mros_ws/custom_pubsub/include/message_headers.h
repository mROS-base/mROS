

#include "custom_pubsub/UserTypeTest.h"


static void callCallback(int id, void (*fp)(void *), char *rbuf){
	switch(id){
	
		case 103:
			subtask_methods::CallCallbackFuncs<103>().call(fp,rbuf);
			break;
	
	}
}

static const char* getMD5Sum(int id){
	switch(id){
	
		case 103:
			return message_traits::MD5Sum<103>().value();
	
	}
	return NULL;
}