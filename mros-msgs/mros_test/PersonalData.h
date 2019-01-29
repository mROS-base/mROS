#ifndef _MROS_TEST_PERSONALDATA_H
#define _MROS_TEST_PERSONALDATA_H

#include "mros_test/LightSensorValues.h"


static const int PERSONALDATA_MSG_ID = 102;

namespace mros_test{
class PersonalData{
public:
  string first_name;
  string last_name;
  uint8_t age;
  mros_test::LightSensorValues lsValue;
  std::vector<uint32_t> score;

  int dataSize(){
    return  first_name.size() +  last_name.size() +  1 +  lsValue.dataSize() +  score.size()*4 + 4  +  4*3;
  }

  void memCopy(char*& addrPtr){
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
    
    lsValue.memCopy(addrPtr);
    {
      size = score.size();
      memcpy(addrPtr,&size,4);
      addrPtr += 4;
      const uint_t* ptr = score.data();
      for(int i=0; i<size ; i++){
        memcpy(addrPtr, &(ptr[i]),4);
        addrPtr += 4;
      }
    }
    
  }

  void deserialize(char*& rbuf){
    uint32_t size;
    {
      memcpy(&size,rbuf,4);
      rbuf += 4;
      char buf_char[size+1];
      memcpy(&buf_char,rbuf,size);
      buf_char[size] = '\0';
      first_name = buf_char;
      rbuf += size;
    }
    
    {
      memcpy(&size,rbuf,4);
      rbuf += 4;
      char buf_char[size+1];
      memcpy(&buf_char,rbuf,size);
      buf_char[size] = '\0';
      last_name = buf_char;
      rbuf += size;
    }
    
    memcpy(&age,rbuf,1);
    rbuf += 1;
    
    
    lsValue.deserialize(rbuf);
    
    {
      uint32_t size;
      memcpy(&size,rbuf,4);
      rbuf += 4;
      score.reserve(size);
      for(int i=0;i<size;i++){
        uint32_t buf;
        memcpy(&buf,rbuf,4);
        score.push_back(buf);
        rbuf += 4;
      }
    }
    
    
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
    return "e4ff7d235751e125e4834bb3cf7fcead";
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
LightSensorValues lsValue\n\
uint32[] score\n\
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