#ifndef _STD_MSGS_TIME_H
#define _STD_MSGS_TIME_H


static const int TIME_MSG_ID = 11;

namespace std_msgs{
class Time{
public:
	std::string data;
};
}

namespace message_traits
{
template<>
struct MD5Sum<TIME_MSG_ID>
{
  static const char* value()
  {
    return "cd7166c74c552c311fbcc2fe5a7bc289";
  }

};

template<>
struct DataType<std_msgs::Time*>
{
  static const char* value()
  {
    return "std_msgs/Time";
  }

};

template<>
struct DataTypeId<std_msgs::Time*>
{
  static const int value()
  {
    return TIME_MSG_ID;
  }

};

template<>
struct Definition<std_msgs::Time*>
{
	static const char* value()
	{
		return "ime data\n\\n\
";
	}
};
}

namespace subtask_methods
{
  template<>
  struct CallCallbackFuncs<TIME_MSG_ID>{
    static void call(void (*fp)(intptr_t), char *rbuf)
    {
      std_msgs::String msg;
      msg.data = &rbuf[8];
      fp(&msg);
    }
  };
}

#endif