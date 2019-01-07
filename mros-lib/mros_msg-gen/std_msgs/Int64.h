#ifndef _STD_MSGS_INT64_H
#define _STD_MSGS_INT64_H


static const int INT64_MSG_ID = 7;

namespace std_msgs{
class Int64{
public:
	std::string data;
};
}

namespace message_traits
{
template<>
struct MD5Sum<INT64_MSG_ID>
{
  static const char* value()
  {
    return "34add168574510e6e17f5d23ecc077ef";
  }

};

template<>
struct DataType<std_msgs::Int64*>
{
  static const char* value()
  {
    return "std_msgs/Int64";
  }

};

template<>
struct DataTypeId<std_msgs::Int64*>
{
  static const int value()
  {
    return INT64_MSG_ID;
  }

};

template<>
struct Definition<std_msgs::Int64*>
{
	static const char* value()
	{
		return "int64 data\n\\n\
";
	}
};
}

namespace subtask_methods
{
  template<>
  struct CallCallbackFuncs<INT64_MSG_ID>{
    static void call(void (*fp)(intptr_t), char *rbuf)
    {
      std_msgs::String msg;
      msg.data = &rbuf[8];
      fp(&msg);
    }
  };
}

#endif