#ifndef _MROS_TEST_LIGHTSENSORVALUES_H
#define _MROS_TEST_LIGHTSENSORVALUES_H


static const int LIGHTSENSORVALUES_MSG_ID = 101;

namespace mros_test{
class LightSensorValues{
public:
  int16_t forward;
  int16_t side;

  int dataSize(){
    return  2 +  2 +  4*0;
  }

  void memCopy(char *addrPtr){
    int size; 
    
    memcpy(addrPtr,&forward,2);
    addrPtr += 2;
    
    memcpy(addrPtr,&side,2);
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
    return "b16183d05d874e9be40998cad708bb18";
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
		return "int16 forward\n\
int16 side\n\
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
      memcpy(&msg.forward,rbuf,2);
      rbuf += 2;
      memcpy(&msg.side,rbuf,2);
      rbuf += 2;
      
      fp(&msg);
    }
  };
}

#endif