#ifndef _MROS_TEST_STRMSG_H
#define _MROS_TEST_STRMSG_H


static const int STRMSG_MSG_ID = 100;

namespace mros_test{
class StrMsg{
public:
  string baker;
  string parker;
  string coltrane;

  int dataSize(){
    return  baker.size() +  parker.size() +  coltrane.size() +  4*3;
  }

  void memCopy(char *addrPtr){
    int size; 
    
    size = baker.size();
    memcpy(addrPtr,&size,4);
    addrPtr += 4;
    memcpy(addrPtr, baker.c_str(),size);
    addrPtr += size;
    
    size = parker.size();
    memcpy(addrPtr,&size,4);
    addrPtr += 4;
    memcpy(addrPtr, parker.c_str(),size);
    addrPtr += size;
    
    size = coltrane.size();
    memcpy(addrPtr,&size,4);
    addrPtr += 4;
    memcpy(addrPtr, coltrane.c_str(),size);
    addrPtr += size;
    
  }
};

}

namespace message_traits
{
template<>
struct MD5Sum<STRMSG_MSG_ID>
{
  static const char* value()
  {
    return "28050cf6f81bfbf1fd18c5ec5c694ad6";
  }

};

template<>
struct DataType<mros_test::StrMsg*>
{
  static const char* value()
  {
    return "mros_test/StrMsg";
  }

};

template<>
struct DataTypeId<mros_test::StrMsg*>
{
  static const int value()
  {
    return STRMSG_MSG_ID;
  }

};

template<>
struct Definition<mros_test::StrMsg*>
{
	static const char* value()
	{
		return "string baker\n\
string parker\n\
string coltrane\n\
";
	}
};
}

namespace subtask_methods
{
  template<>
  struct CallCallbackFuncs<STRMSG_MSG_ID>{
    static void call(void (*fp)(intptr_t), char *rbuf)
    {
      mros_test::StrMsg msg;
      int size;
      rbuf += 4;
      {
        memcpy(&size,rbuf,4);
        rbuf += 4;
        char buf_char[size+1];
        memcpy(&buf_char,rbuf,size);
        buf_char[size] = '\0';
        msg.baker = buf_char;
        rbuf += size;
      }
      {
        memcpy(&size,rbuf,4);
        rbuf += 4;
        char buf_char[size+1];
        memcpy(&buf_char,rbuf,size);
        buf_char[size] = '\0';
        msg.parker = buf_char;
        rbuf += size;
      }
      {
        memcpy(&size,rbuf,4);
        rbuf += 4;
        char buf_char[size+1];
        memcpy(&buf_char,rbuf,size);
        buf_char[size] = '\0';
        msg.coltrane = buf_char;
        rbuf += size;
      }
      
      fp(&msg);
    }
  };
}

#endif