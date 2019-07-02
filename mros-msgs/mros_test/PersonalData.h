#ifndef _MROS_TEST_PERSONALDATA_H
#define _MROS_TEST_PERSONALDATA_H

#include "mros_test/PersonName.h"
#include "mros_test/LEDValues.h"



static const int PERSONALDATA_MSG_ID = 103;

namespace mros_test{
class PersonalData{
public:
  int32_t intVal;
  uint8_t boolVal;
  mros_test::PersonName nameVal;
  mros_test::LEDValues ledVal;

  int dataSize(){
    return  4 +  1 +  nameVal.dataSize() +  ledVal.dataSize() +  4*2;
  }

  void memCopy(char*& addrPtr){
    int size; 
    
    memcpy(addrPtr,&intVal,4);
    addrPtr += 4;
    
    memcpy(addrPtr,&boolVal,1);
    addrPtr += 1;
    
    nameVal.memCopy(addrPtr);
    
    ledVal.memCopy(addrPtr);
    
  }

  void deserialize(char*& rbuf){
    uint32_t size;
    memcpy(&intVal,rbuf,4);
    rbuf += 4;
    
    memcpy(&boolVal,rbuf,1);
    rbuf += 1;
    
    
    nameVal.deserialize(rbuf);
    
    
    ledVal.deserialize(rbuf);
    
    
  }
};

}

namespace message_traits
{
template<>
struct MD5Sum<PERSONALDATA_MSG_ID>
{
  static const char* value()
  {
    return "0107a86535f313508e11b1f77d855c02";
  }

};

template<>
struct DataType<mros_test::PersonalData*>
{
  static const char* value()
  {
    return "mros_test/PersonalData";
  }

};

template<>
struct DataTypeId<mros_test::PersonalData*>
{
  static const int value()
  {
    return PERSONALDATA_MSG_ID;
  }

};

template<>
struct Definition<mros_test::PersonalData*>
{
	static const char* value()
	{
		return "int32 intVal\n\
bool boolVal\n\
PersonName nameVal\n\
LEDValues ledVal\n\
";
	}
};
}

namespace subtask_methods
{
  template<>
  struct CallCallbackFuncs<PERSONALDATA_MSG_ID>{
    static void call(void (*fp)(intptr_t), char *rbuf)
    {
      mros_test::PersonalData msg;
      rbuf += 4;
      msg.deserialize(rbuf);
      fp(&msg);
    }
  };
}

#endif