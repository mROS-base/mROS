#ifndef _MROS_TEST_LIGHTSENSORVALUES_H
#define _MROS_TEST_LIGHTSENSORVALUES_H


static const int LIGHTSENSORVALUES_MSG_ID = 100;

namespace mros_test{
class LightSensorValues{
public:
  uint32_t person1;
  std::vector<uint32_t> person2;

  int dataSize(){
    return  4 +  person2.size()*4 + 4  +  4*0;
  }

  void memCopy(char *addrPtr){
    int size; 
    
    memcpy(addrPtr,&person1,4);
    addrPtr += 4;
    {
      size = person2.size();
      memcpy(addrPtr,&size,4);
      addrPtr += 4;
      const uint32_t* ptr = person2.data();
      for(int i=0; i<size ; i++){
        memcpy(addrPtr, ptr[i],4);
        addrPtr += 4;
      }
    }
    
  }
};

}

namespace message_traits
{
template<>
struct MD5Sum<LIGHTSENSORVALUES_MSG_ID>
{
  static const char* value()
  {
    return "72eab09f844ca0e296c9e9f0f0d1dddf";
  }

};

template<>
struct DataType<mros_test::LightSensorValues*>
{
  static const char* value()
  {
    return "mros_test/LightSensorValues";
  }

};

template<>
struct DataTypeId<mros_test::LightSensorValues*>
{
  static const int value()
  {
    return LIGHTSENSORVALUES_MSG_ID;
  }

};

template<>
struct Definition<mros_test::LightSensorValues*>
{
	static const char* value()
	{
		return "uint32 person1\n\
uint32[] person2\n\
";
	}
};
}

namespace subtask_methods
{
  template<>
  struct CallCallbackFuncs<LIGHTSENSORVALUES_MSG_ID>{
    static void call(void (*fp)(intptr_t), char *rbuf)
    {
      mros_test::LightSensorValues msg;
      int size;
      rbuf += 4;
      memcpy(&msg.person1,rbuf,4);
      rbuf += 4;
      {
        uint32_t size;
        memcpy(&size,rbuf,4);
        rbuf += 4;
        msg.person2.reserve(size);
        for(int i=0;i<size;i++){
          int buf;
          memcpy(&buf,rbuf,4);
          msg.person2.push_back(buf);
          rbuf += 4;
        }

      }
      
      fp(&msg);
    }
  };
}

#endif