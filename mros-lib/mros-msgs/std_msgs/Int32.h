#ifndef _STD_MSGS_INT32_H
#define _STD_MSGS_INT32_H

#include <string.h>

static const int INT32_MSG_ID = 7;

namespace std_msgs{
class Int32{
public:
	int32_t data;
  int dataSize(){return 4;}
  void memCopy(char *addrPtr){
    memcpy(addrPtr,&data,4);
  }
};
}

namespace message_traits
{
template<>
struct MD5Sum<INT32_MSG_ID>
{
  static const char* value()
  {
    return "da5909fbe378aeaf85e547e830cc1bb7";
  }

};

template<>
struct DataType<std_msgs::Int32*>
{
  static const char* value()
  {
    return "std_msgs/Int32";
  }

};

template<>
struct DataTypeId<std_msgs::Int32*>
{
  static int value()
  {
    return (int)INT32_MSG_ID;
  }

};

template<>
struct Definition<std_msgs::Int32*>
{
	static const char* value()
	{
		return "int32 data\n\\n\
";
	}
};
}

namespace subtask_methods
{
  template<>
  struct CallCallbackFuncs<INT32_MSG_ID>{
    static void call(void (*fp)(void *), char *rbuf)
    {
      std_msgs::Int32 msg;
      rbuf += 4;
      memcpy(&msg.data,rbuf,4);
      //msg.data = (unsigned int)rbuf[4] + (unsigned int)rbuf[5]*256 + (unsigned int)rbuf[6]*65536 + (int)rbuf[7]*16777216;
      fp(&msg);
    }
  };
}

#endif
