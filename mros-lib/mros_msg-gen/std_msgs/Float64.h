#ifndef _STD_MSGS_FLOAT64_H
#define _STD_MSGS_FLOAT64_H


static const int FLOAT64_MSG_ID = 10;

namespace std_msgs{
class Float64{
public:
	std::string data;
};
}

namespace message_traits
{
template<>
struct MD5Sum<FLOAT64_MSG_ID>
{
  static const char* value()
  {
    return "fdb28210bfa9d7c91146260178d9a584";
  }

};

template<>
struct DataType<std_msgs::Float64*>
{
  static const char* value()
  {
    return "std_msgs/Float64";
  }

};

template<>
struct DataTypeId<std_msgs::Float64*>
{
  static const int value()
  {
    return FLOAT64_MSG_ID;
  }

};

template<>
struct Definition<std_msgs::Float64*>
{
	static const char* value()
	{
		return "float64 data\n\\n\
";
	}
};
}

namespace subtask_methods
{
  template<>
  struct CallCallbackFuncs<FLOAT64_MSG_ID>{
    static void call(void (*fp)(intptr_t), char *rbuf)
    {
      std_msgs::String msg;
      msg.data = &rbuf[8];
      fp(&msg);
    }
  };
}

#endif