#ifndef _MROS_TEST_LIGHTSENSORVALUES_H
#define _MROS_TEST_LIGHTSENSORVALUES_H




static const int LIGHTSENSORVALUES_MSG_ID = 100;

namespace mros_test{
class LightSensorValues{
public:
  uint32_t person1;
  std::vector<uint32_t> person2;
  string name;

  int dataSize(){
    return  4 +  person2.size()*4 + 4  +  name.size() +  4*1;
  }

  void memCopy(char*& addrPtr){
    int size; 
    
    memcpy(addrPtr,&person1,4);
    addrPtr += 4;
    {
      size = person2.size();
      memcpy(addrPtr,&size,4);
      addrPtr += 4;
      const uint_t* ptr = person2.data();
      for(int i=0; i<size ; i++){
        memcpy(addrPtr, &(ptr[i]),4);
        addrPtr += 4;
      }
    }
    
    size = name.size();
    memcpy(addrPtr,&size,4);
    addrPtr += 4;
    memcpy(addrPtr, name.c_str(),size);
    addrPtr += size;
    
  }

  void deserialize(char*& rbuf){
    uint32_t size;
    memcpy(&person1,rbuf,4);
    rbuf += 4;
    
    {
      uint32_t size;
      memcpy(&size,rbuf,4);
      rbuf += 4;
      person2.reserve(size);
      for(int i=0;i<size;i++){
        uint32_t buf;
        memcpy(&buf,rbuf,4);
        person2.push_back(buf);
        rbuf += 4;
      }
    }
    
    {
      memcpy(&size,rbuf,4);
      rbuf += 4;
      char buf_char[size+1];
      memcpy(&buf_char,rbuf,size);
      buf_char[size] = '\0';
      name = buf_char;
      rbuf += size;
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
    return "e2d6cfcf50f98a5a24d82739e108711f";
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
string name\n\
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