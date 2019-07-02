#ifndef _MROS_TEST_LIGHTSENSORVALUES_H
#define _MROS_TEST_LIGHTSENSORVALUES_H




static const int LIGHTSENSORVALUES_MSG_ID = 101;

namespace mros_test{
class LightSensorValues{
public:
  uint32_t person1;
  uint64_t person2;

  int dataSize(){
    return  4 +  8 +  4*0;
  }

  void memCopy(char*& addrPtr){
    int size; 
    
    memcpy(addrPtr,&person1,4);
    addrPtr += 4;
    
    memcpy(addrPtr,&person2,8);
    addrPtr += 8;
    
  }

  void deserialize(char*& rbuf){
    uint32_t size;
    memcpy(&person1,rbuf,4);
    rbuf += 4;
    
    memcpy(&person2,rbuf,8);
    rbuf += 8;
    
    
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
    return "802a3122c4dd54ea81ad83498f147724";
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
uint64 person2\n\
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
      rbuf += 4;
      msg.deserialize(rbuf);
      fp(&msg);
    }
  };
}

#endif