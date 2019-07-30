#ifndef _STD_MSGS_INT8_H
#define _STD_MSGS_INT8_H

#include <string.h>

static const int INT8_MSG_ID = 1;

namespace std_msgs{
class Int8{
public:
	int8_t data;
  int dataSize(){return 1;}
  void memCopy(char *addrPtr){
    memcpy(addrPtr,&data,1);
  }
};
}

namespace message_traits
{
template<>
struct MD5Sum<INT8_MSG_ID>
{
  static const char* value()
  {
    return "27ffa0c9c4b8fb8492252bcad9e5c57b";
  }

};

template<>
struct DataType<std_msgs::Int8*>
{
  static const char* value()
  {
    return "std_msgs/UInt8";
  }

};

template<>
struct DataTypeId<std_msgs::Int8*>
{
  static const int value()
  {
    return INT8_MSG_ID;
  }

};

template<>
struct Definition<std_msgs::Int8*>
{
	static const char* value()
	{
		return "int8 data\n\\n\
";
	}
};
}

namespace subtask_methods
{
  template<>
  struct CallCallbackFuncs<INT8_MSG_ID>{
    static void call(void (*fp)(intptr_t), char *rbuf)
    {
      std_msgs::Int8 msg;
      rbuf += 4;
      memcpy(&msg.data,rbuf,1);
      //msg.data = (int)rbuf[4];
      fp(&msg);
    }
  };
}

#endif
