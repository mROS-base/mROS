#ifndef _MROS_TEST_PERSONALDATA_H
#define _MROS_TEST_PERSONALDATA_H

#include <mros_test/LightSensorValues.h>


static const int PERSONALDATA_MSG_ID = 101;

namespace mros_test{
class PersonalData{
public:
  string first_name;
  string last_name;
  uint8_t age;
  mros_test::LightSensorValues score;

  int dataSize(){
    return  first_name.size() +  last_name.size() +  1 +  4*2;
  }

  void memCopy(char *addrPtr){
    int size; 
    
    size = first_name.size();
    memcpy(addrPtr,&size,4);
    addrPtr += 4;
    memcpy(addrPtr, first_name.c_str(),size);
    addrPtr += size;
    
    size = last_name.size();
    memcpy(addrPtr,&size,4);
    addrPtr += 4;
    memcpy(addrPtr, last_name.c_str(),size);
    addrPtr += size;
    
    memcpy(addrPtr,&age,1);
    addrPtr += 1;
    
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
    return "e14b6aceb189db5b36c3622ba0793936";
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
		return "string first_name\n\
string last_name\n\
uint8 age\n\
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
      int size;
      rbuf += 4;
      {
        memcpy(&size,rbuf,4);
        rbuf += 4;
        char buf_char[size+1];
        memcpy(&buf_char,rbuf,size);
        buf_char[size] = '\0';
        msg.first_name = buf_char;
        rbuf += size;
      }
      {
        memcpy(&size,rbuf,4);
        rbuf += 4;
        char buf_char[size+1];
        memcpy(&buf_char,rbuf,size);
        buf_char[size] = '\0';
        msg.last_name = buf_char;
        rbuf += size;
      }
      memcpy(&msg.age,rbuf,1);
      rbuf += 1;
      msg.score.deserialize(rbuf);
      fp(&msg);
    }
  };
}

#endif