#ifndef _STD_MSGS_INT16_H
#define _STD_MSGS_INT16_H


static const int INT16_MSG_ID = 4;

namespace std_msgs{
class Int16{
public:
	std::string data;
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
struct DataType<std_msgs::Int16*>
{
  static const char* value()
  {
    return "std_msgs/Int16";
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
		return "int16 data\n\\n\
";
	}
};
}

namespace subtask_methods
{
  template<>
  struct CallCallbackFuncs<INT16_MSG_ID>{
    static void call(void (*fp)(intptr_t), char *rbuf)
    {
      std_msgs::String msg;
      msg.data = &rbuf[8];
      fp(&msg);
    }
  };
}

#endif