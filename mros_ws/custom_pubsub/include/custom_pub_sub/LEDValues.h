#ifndef _CUSTOM_PUB_SUB_LEDVALUES_H
#define _CUSTOM_PUB_SUB_LEDVALUES_H




static const int LEDVALUES_MSG_ID = 101;

namespace custom_pub_sub{
class LEDValues{
public:
  float red;
  float green;
  float blue;

  int dataSize(){
    return  4 +  4 +  4 +  4*0;
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
struct DataType<custom_pub_sub::LEDValues*>
{
  static const char* value()
  {
    return "custom_pub_sub/LEDValues";
  }

};

template<>
struct DataTypeId<custom_pub_sub::LEDValues*>
{
  static const int value()
  {
    return LEDVALUES_MSG_ID;
  }

};

template<>
struct Definition<custom_pub_sub::LEDValues*>
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
      custom_pub_sub::LEDValues msg;
      rbuf += 4;
      msg.deserialize(rbuf);
      fp(&msg);
    }
  };
}

#endif