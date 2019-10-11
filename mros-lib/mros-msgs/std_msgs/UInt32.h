#ifndef _STD_MSGS_UINT32_H
#define _STD_MSGS_UINT32_H

#include <string.h>

static const int UINT32_MSG_ID = 7;

namespace std_msgs{
class UInt32{
public:
	uint32_t data;
  int dataSize(){return 4;}
  void memCopy(char *addrPtr){
    memcpy(addrPtr,&data,4);
  }
};
}

namespace message_traits
{
template<>
struct MD5Sum<UINT32_MSG_ID>
{
  static const char* value()
  {
    return "304a39449588c7f8ce2df6e8001c5fce";
  }

};

template<>
struct DataType<std_msgs::UInt32*>
{
  static const char* value()
  {
    return "std_msgs/UInt32";
  }

};

template<>
struct DataTypeId<std_msgs::UInt32*>
{
  static int value()
  {
    return (int)UINT32_MSG_ID;
  }

};

template<>
struct Definition<std_msgs::UInt32*>
{
	static const char* value()
	{
		return "uint32 data\n\\n\
";
	}
};
}

namespace subtask_methods
{
  template<>
  struct CallCallbackFuncs<UINT32_MSG_ID>{
    static void call(void (*fp)(void *), char *rbuf)
    {
      std_msgs::UInt32 msg;
      rbuf += 4;
      memcpy(&msg.data,rbuf,4);
      //msg.data = (unsigned int)rbuf[4] + (unsigned int)rbuf[5]*256 + (unsigned int)rbuf[6]*65536 + (int)rbuf[7]*16777216;
      fp(&msg);
    }
  };
}

#endif
