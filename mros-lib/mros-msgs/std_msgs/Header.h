#ifndef _STD_MSGS_HEADER_H
#define _STD_MSGS_HEADER_H

#include "std_msgs/Time.h"



static const int HEADER_MSG_ID = 104;

namespace std_msgs{
class Header{
public:
  uint32_t seq;
  std_msgs::Time stamp;
  string frame_id;

  int dataSize(){
    return  4 +  stamp.dataSize() +  frame_id.size() +  4*3;
  }

  void memCopy(char*& addrPtr){
    int size; 
    
    memcpy(addrPtr,&seq,4);
    addrPtr += 4;
    
    stamp.memCopy(addrPtr);
    
    size = frame_id.size();
    memcpy(addrPtr,&size,4);
    addrPtr += 4;
    memcpy(addrPtr, frame_id.c_str(),size);
    addrPtr += size;
    
  }

  void deserialize(char*& rbuf){
    uint32_t size;
    memcpy(&seq,rbuf,4);
    rbuf += 4;
    
    
    stamp.deserialize(rbuf);
    
    {
      memcpy(&size,rbuf,4);
      rbuf += 4;
      char buf_char[size+1];
      memcpy(&buf_char,rbuf,size);
      buf_char[size] = '\0';
      frame_id = buf_char;
      rbuf += size;
    }
    
    
  }
};

}

namespace message_traits
{
template<>
struct MD5Sum<HEADER_MSG_ID>
{
  static const char* value()
  {
    return "3695c7678a2b8f86015eccf2f844688c";
  }

};

template<>
struct DataType<std_msgs::Header*>
{
  static const char* value()
  {
    return "std_msgs/Header";
  }

};

template<>
struct DataTypeId<std_msgs::Header*>
{
  static int value()
  {
    return (int)HEADER_MSG_ID;
  }

};

template<>
struct Definition<std_msgs::Header*>
{
	static const char* value()
	{
		return "uint32 seq\n\
time stamp\n\
string frame_id\n\
";
	}
};
}

namespace subtask_methods
{
  template<>
  struct CallCallbackFuncs<HEADER_MSG_ID>{
    static void call(void (*fp)(void *), char *rbuf)
    {
      std_msgs::Header msg;
      rbuf += 4;
      msg.deserialize(rbuf);
      fp(&msg);
    }
  };
}

#endif
