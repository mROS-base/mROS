#include "ros.h"

namespace std_msgs{
class UInt16{
public:
	int data;
};
}

namespace message_traits
{
template<>
struct MD5Sum<std_msgs::UInt16*>
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
struct Definition<std_msgs::UInt16*>
{
	static const char* value()
	{
		return "uint16 data\n\
";
	}
};
}

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
/*
namespace subtask_methods
{
  template<>
  struct CallCallbackFuncs<"std_msgs/UInt16">{
    static void call(void (*fp)(), char *rbuf)
    {
      std_msgs::UInt16 msg;
      msg.data = (int)rbuf[4] + (int)rbuf[5]*256;
    }
  };
}*/