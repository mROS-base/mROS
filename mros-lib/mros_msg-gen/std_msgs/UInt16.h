#ifndef _STD_MSGS_UINT16_H
#define _STD_MSGS_UINT16_H


static const int UINT16_MSG_ID = 13;

namespace std_msgs{
class UInt16{
public:
	std::string data;
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
		return "int16 data\n\\n\
";
	}
};
}

namespace subtask_methods
{
  template<>
  struct CallCallbackFuncs<UINT16_MSG_ID>{
    static void call(void (*fp)(intptr_t), char *rbuf)
    {
      std_msgs::String msg;
      msg.data = &rbuf[8];
      fp(&msg);
    }
  };
}

#endif