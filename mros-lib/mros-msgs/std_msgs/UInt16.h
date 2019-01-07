#ifndef _STD_MSGS_UINT16_
#define _STD_MSGS_UINT16_

static const int UINT16_MSG_ID = 10;

namespace std_msgs{
class UInt16{
public:
	int data;
  static const int id = UINT16_MSG_ID;
  int dataSize(){return 2;}
};
}

namespace message_traits
{

template<>
struct MD5Sum<UINT16_MSG_ID>
{
  static const char* value()
  {
    return "1df79edf208b629fe6b81923a544552d";
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
struct DataTypeId<std_msgs::UInt16*>
{
  static const int value()
  {
    return UINT16_MSG_ID;
  }

};

template<>
struct Definition<std_msgs::UInt16*>
{
	static const char* value()
	{
		return "uint16 data\n\
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
  struct CallCallbackFuncs<UINT16_MSG_ID>{
    static void call(void (*fp)(intptr_t), char *rbuf)
    {
      std_msgs::UInt16 msg;
      msg.data = (int)rbuf[4] + (int)rbuf[5]*256;
      fp(&msg);
    }
  };
}
#endif