#ifndef _CUSTOM_PUBSUB_USERTYPETEST_H
#define _CUSTOM_PUBSUB_USERTYPETEST_H

#include "custom_pubsub/LEDValues.h"
#include "custom_pubsub/PersonName.h"


#include <string.h>

using namespace std;


static const int USERTYPETEST_MSG_ID = 103;

namespace custom_pubsub{
class UserTypeTest{
public:
  custom_pubsub::LEDValues ledVal;
  custom_pubsub::PersonName nameVal;

  int dataSize(){
    return  ledVal.dataSize() +  nameVal.dataSize() +  4*2;
  }

  void memCopy(char*& addrPtr){
    int size; 
    
    ledVal.memCopy(addrPtr);
    
    nameVal.memCopy(addrPtr);
    
  }

  void deserialize(char*& rbuf){
    uint32_t size;
    
    ledVal.deserialize(rbuf);
    
    
    nameVal.deserialize(rbuf);
    
    
  }
};

}

namespace message_traits
{
template<>
struct MD5Sum<USERTYPETEST_MSG_ID>
{
  static const char* value()
  {
    return "4c13c13aa2f193d25835cfad6215cb75";
  }

};

template<>
struct DataType<custom_pubsub::UserTypeTest*>
{
  static const char* value()
  {
    return "custom_pubsub/UserTypeTest";
  }

};

template<>
struct DataTypeId<custom_pubsub::UserTypeTest*>
{
  static int value()
  {
    return USERTYPETEST_MSG_ID;
  }

};

template<>
struct Definition<custom_pubsub::UserTypeTest*>
{
	static const char* value()
	{
		return "LEDValues ledVal\n\
PersonName nameVal\n\
";
	}
};
}

namespace subtask_methods
{
  template<>
  struct CallCallbackFuncs<USERTYPETEST_MSG_ID>{
    static void call(void (*fp)(void *), char *rbuf)
    {
      custom_pubsub::UserTypeTest msg;
      rbuf += 4;
      msg.deserialize(rbuf);
      fp(&msg);
    }
  };
}

#endif