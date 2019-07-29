#ifndef _MROS_TEST_PERSONNAME_H
#define _MROS_TEST_PERSONNAME_H




static const int PERSONNAME_MSG_ID = 102;

namespace mros_test{
class PersonName{
public:
  string firstName;
  string lastName;

  int dataSize(){
    return  firstName.size() +  lastName.size() +  4*2;
  }

  void memCopy(char*& addrPtr){
    int size; 
    
    size = firstName.size();
    memcpy(addrPtr,&size,4);
    addrPtr += 4;
    memcpy(addrPtr, firstName.c_str(),size);
    addrPtr += size;
    
    size = lastName.size();
    memcpy(addrPtr,&size,4);
    addrPtr += 4;
    memcpy(addrPtr, lastName.c_str(),size);
    addrPtr += size;
    
  }

  void deserialize(char*& rbuf){
    uint32_t size;
    {
      memcpy(&size,rbuf,4);
      rbuf += 4;
      char buf_char[size+1];
      memcpy(&buf_char,rbuf,size);
      buf_char[size] = '\0';
      firstName = buf_char;
      rbuf += size;
    }
    
    {
      memcpy(&size,rbuf,4);
      rbuf += 4;
      char buf_char[size+1];
      memcpy(&buf_char,rbuf,size);
      buf_char[size] = '\0';
      lastName = buf_char;
      rbuf += size;
    }
    
    
  }
};

}

namespace message_traits
{
template<>
struct MD5Sum<PERSONNAME_MSG_ID>
{
  static const char* value()
  {
    return "2603aab1ad50ac2b8ea55f4de7eec3a0";
  }

};

template<>
struct DataType<mros_test::PersonName*>
{
  static const char* value()
  {
    return "mros_test/PersonName";
  }

};

template<>
struct DataTypeId<mros_test::PersonName*>
{
  static const int value()
  {
    return PERSONNAME_MSG_ID;
  }

};

template<>
struct Definition<mros_test::PersonName*>
{
	static const char* value()
	{
		return "string firstName\n\
string lastName\n\
";
	}
};
}

namespace subtask_methods
{
  template<>
  struct CallCallbackFuncs<PERSONNAME_MSG_ID>{
    static void call(void (*fp)(intptr_t), char *rbuf)
    {
      mros_test::PersonName msg;
      rbuf += 4;
      msg.deserialize(rbuf);
      fp(&msg);
    }
  };
}

#endif