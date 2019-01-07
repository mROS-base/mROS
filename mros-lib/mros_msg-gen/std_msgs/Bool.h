#ifndef _STD_MSGS_BOOL_H
#define _STD_MSGS_BOOL_H


static const int BOOL_MSG_ID = 1;

namespace std_msgs{
class Bool{
public:
	std::string data;
};
}

namespace message_traits
{
template<>
struct MD5Sum<BOOL_MSG_ID>
{
  static const char* value()
  {
    return "8b94c1b53db61fb6aed406028ad6332a";
  }

};

template<>
struct DataType<std_msgs::Bool*>
{
  static const char* value()
  {
    return "std_msgs/Bool";
  }

};

template<>
struct DataTypeId<std_msgs::Bool*>
{
  static const int value()
  {
    return BOOL_MSG_ID;
  }

};

template<>
struct Definition<std_msgs::Bool*>
{
	static const char* value()
	{
		return "bool data\n\\n\
";
	}
};
}

namespace subtask_methods
{
  template<>
  struct CallCallbackFuncs<BOOL_MSG_ID>{
    static void call(void (*fp)(intptr_t), char *rbuf)
    {
      std_msgs::String msg;
      msg.data = &rbuf[8];
      fp(&msg);
    }
  };
}

#endif