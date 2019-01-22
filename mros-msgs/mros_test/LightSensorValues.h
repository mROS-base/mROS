#ifndef _MROS_TEST_LIGHTSENSORVALUES_H
#define _MROS_TEST_LIGHTSENSORVALUES_H


static const int LIGHTSENSORVALUES_MSG_ID = 101;

namespace mros_test{
class LightSensorValues{
public:
  uint16_t person1;
  uint16_t person2;

  int dataSize(){
    return  2 +  2 +  4*0;
  }

  void memCopy(char *addrPtr){
    int size; 
    
    memcpy(addrPtr,&person1,2);
    addrPtr += 2;
    
    memcpy(addrPtr,&person2,2);
    addrPtr += 2;
    
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
    return "33ab5201cf78cba520b2a7b66f884ee2";
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
		return "uint16 person1\n\
uint16 person2\n\
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
      memcpy(&msg.person1,rbuf,2);
      rbuf += 2;
      memcpy(&msg.person2,rbuf,2);
      rbuf += 2;
      
      fp(&msg);
    }
  };
}

#endif