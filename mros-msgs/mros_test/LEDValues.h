#ifndef _MROS_TEST_LEDVALUES_H
#define _MROS_TEST_LEDVALUES_H




static const int LEDVALUES_MSG_ID = 102;

namespace mros_test{
class LEDValues{
public:
  float red;
  float green;
  float blue;

  int dataSize(){
    return  4 +  4 +  4 +  4*2;
  }

  void memCopy(char*& addrPtr){
    int size; 
    
    memcpy(addrPtr,&red,4);
    addrPtr += 4;
    
    memcpy(addrPtr,&green,4);
    addrPtr += 4;
    
    memcpy(addrPtr,&blue,4);
    addrPtr += 4;
    
  }

  void deserialize(char*& rbuf){
    uint32_t size;
    memcpy(&red,rbuf,4);
    rbuf += 4;
    
    memcpy(&green,rbuf,4);
    rbuf += 4;
    
    memcpy(&blue,rbuf,4);
    rbuf += 4;
    
    
  }
};

}

namespace message_traits
{
template<>
struct MD5Sum<LEDVALUES_MSG_ID>
{
  static const char* value()
  {
    return "fc84fca2ee69069d6d5c4147f9b2e33a";
  }

};

template<>
struct DataType<mros_test::LEDValues*>
{
  static const char* value()
  {
    return "mros_test/LEDValues";
  }

};

template<>
struct DataTypeId<mros_test::LEDValues*>
{
  static const int value()
  {
    return LEDVALUES_MSG_ID;
  }

};

template<>
struct Definition<mros_test::LEDValues*>
{
	static const char* value()
	{
		return "float32 red\n\
float32 green\n\
float32 blue\n\
";
	}
};
}

namespace subtask_methods
{
  template<>
  struct CallCallbackFuncs<LEDVALUES_MSG_ID>{
    static void call(void (*fp)(intptr_t), char *rbuf)
    {
      mros_test::LEDValues msg;
      rbuf += 4;
      msg.deserialize(rbuf);
      fp(&msg);
    }
  };
}

#endif