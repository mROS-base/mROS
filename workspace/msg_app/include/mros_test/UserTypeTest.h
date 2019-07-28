#ifndef _MROS_TEST_USERTYPETEST_H
#define _MROS_TEST_USERTYPETEST_H

#include "mros_test/LEDValues.h"
#include "mros_test/PersonName.h"



static const int USERTYPETEST_MSG_ID = 103;

namespace mros_test{
class UserTypeTest{
public:
  mros_test::LEDValues ledVal;
  mros_test::PersonName nameVal;

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
struct DataType<mros_test::UserTypeTest*>
{
  static const char* value()
  {
    return "mros_test/UserTypeTest";
  }

};

template<>
struct DataTypeId<mros_test::UserTypeTest*>
{
  static const int value()
  {
    return USERTYPETEST_MSG_ID;
  }

};

template<>
struct Definition<mros_test::UserTypeTest*>
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
    static void call(void (*fp)(intptr_t), char *rbuf)
    {
      mros_test::UserTypeTest msg;
      rbuf += 4;
      msg.deserialize(rbuf);
      fp(&msg);
    }
  };
}

#endif