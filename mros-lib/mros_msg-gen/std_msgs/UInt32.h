#ifndef _STD_MSGS_UINT32_H
#define _STD_MSGS_UINT32_H


static const int UINT32_MSG_ID = 6;

namespace std_msgs{
class UInt32{
public:
	std::string data;
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
  static const int value()
  {
    return UINT32_MSG_ID;
  }

};

template<>
struct Definition<std_msgs::UInt32*>
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
  struct CallCallbackFuncs<UINT32_MSG_ID>{
    static void call(void (*fp)(intptr_t), char *rbuf)
    {
      std_msgs::String msg;
      msg.data = &rbuf[8];
      fp(&msg);
    }
  };
}

#endif