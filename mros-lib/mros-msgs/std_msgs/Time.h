#ifndef _STD_MSGS_TIME_H
#define _STD_MSGS_TIME_H




static const int TIME_MSG_ID = 103;

namespace std_msgs{
class Time{
public:
  uint32_t sec;
  uint32_t nsec;

  int dataSize(){
    return  4 +  4 +  4*2;
  }

  void memCopy(char*& addrPtr){
    int size; 
    
    memcpy(addrPtr,&sec,4);
    addrPtr += 4;
    
    memcpy(addrPtr,&nsec,4);
    addrPtr += 4;
    
  }

  void deserialize(char*& rbuf){
    uint32_t size;
    memcpy(&sec,rbuf,4);
    rbuf += 4;
    
    memcpy(&nsec,rbuf,4);
    rbuf += 4;
    
    
  }
};

}

namespace message_traits
{
template<>
struct MD5Sum<TIME_MSG_ID>
{
  static const char* value()
  {
    return "4771ad66fef816d2e4bead2f45a1cde6";
  }

};

template<>
struct DataType<std_msgs::Time*>
{
  static const char* value()
  {
    return "std_msgs/Time";
  }

};

template<>
struct DataTypeId<std_msgs::Time*>
{
  static const int value()
  {
    return TIME_MSG_ID;
  }

};

template<>
struct Definition<std_msgs::Time*>
{
	static const char* value()
	{
		return "uint32 sec\n\
uint32 nsec\n\
";
	}
};
}

namespace subtask_methods
{
  template<>
  struct CallCallbackFuncs<TIME_MSG_ID>{
    static void call(void (*fp)(intptr_t), char *rbuf)
    {
      std_msgs::Time msg;
      rbuf += 4;
      msg.deserialize(rbuf);
      fp(&msg);
    }
  };
}

#endif