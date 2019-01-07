#ifndef _STD_MSGS_INT32_H
#define _STD_MSGS_INT32_H


static const int INT32_MSG_ID = 5;

namespace std_msgs{
class Int32{
public:
	std::string data;
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
  static const int value()
  {
    return INT32_MSG_ID;
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
    static void call(void (*fp)(intptr_t), char *rbuf)
    {
      std_msgs::String msg;
      msg.data = &rbuf[8];
      fp(&msg);
    }
  };
}

#endif