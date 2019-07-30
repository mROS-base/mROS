#ifndef _STD_MSGS_INT16_
#define _STD_MSGS_INT16_

#include <string.h>

static const int INT16_MSG_ID = 6;

namespace std_msgs{
class Int16{
public:
	int16_t data;
  static const int id = INT16_MSG_ID;
  int dataSize(){return 2;}

  void memCopy(char *addrPtr){
    memcpy(addrPtr,&data,2);
  }
};
}

namespace message_traits
{

template<>
struct MD5Sum<INT16_MSG_ID>
{
  static const char* value()
  {
    return "8524586e34fbd7cb1c08c5f5f1ca0e57";
  }

};

template<>
struct DataType<std_msgs::UInt16*>
{
  static const char* value()
  {
    return "std_msgs/UInt16";
  }

};

template<>
struct DataTypeId<std_msgs::Int16*>
{
  static const int value()
  {
    return INT16_MSG_ID;
  }

};

template<>
struct Definition<std_msgs::Int16*>
{
	static const char* value()
	{
		return "int16 data\n\
";
	}
};
}
/*
namespace subtask_methods
{
  template<>
  struct CallCallbackFuncs<std_msgs::UInt16&>{
    static void call(void (*fp)(), char *rbuf)
    {
      std_msgs::UInt16 msg;
      msg.data = (int)rbuf[4] + (int)rbuf[5]*256;
    }
  };
}
*/

namespace subtask_methods
{
  template<>
  struct CallCallbackFuncs<INT16_MSG_ID>{
    static void call(void (*fp)(intptr_t), char *rbuf)
    {
      std_msgs::Int16 msg;
      rbuf += 4;
      memcpy(&msg.data,rbuf,2);
      //msg.data = (int)rbuf[4] + (int)rbuf[5]*256;
      fp(&msg);
    }
  };
}
#endif
