#ifndef _STD_MSGS_FLOAT32_H
#define _STD_MSGS_FLOAT32_H


static const int FLOAT32_MSG_ID = 9;

namespace std_msgs{
class Float32{
public:
	std::string data;
};
}

namespace message_traits
{
template<>
struct MD5Sum<FLOAT32_MSG_ID>
{
  static const char* value()
  {
    return "73fcbf46b49191e672908e50842a83d4";
  }

};

template<>
struct DataType<std_msgs::Float32*>
{
  static const char* value()
  {
    return "std_msgs/Float32";
  }

};

template<>
struct DataTypeId<std_msgs::Float32*>
{
  static const int value()
  {
    return FLOAT32_MSG_ID;
  }

};

template<>
struct Definition<std_msgs::Float32*>
{
	static const char* value()
	{
		return "float32 data\n\\n\
";
	}
};
}

namespace subtask_methods
{
  template<>
  struct CallCallbackFuncs<FLOAT32_MSG_ID>{
    static void call(void (*fp)(intptr_t), char *rbuf)
    {
      std_msgs::String msg;
      msg.data = &rbuf[8];
      fp(&msg);
    }
  };
}

#endif