#ifndef _STD_MSGS_UINT64_H
#define _STD_MSGS_UINT64_H


static const int UINT64_MSG_ID = 8;

namespace std_msgs{
class UInt64{
public:
  uint64_t data;
  int dataSize(){return 8;}
  void memCopy(char *addrPtr){
    memcpy(addrPtr,&data,8);
  }
};
}

namespace message_traits
{
template<>
struct MD5Sum<UINT64_MSG_ID>
{
  static const char* value()
  {
    return "1b2a79973e8bf53d7b53acb71299cb57";
  }

};

template<>
struct DataType<std_msgs::UInt64*>
{
  static const char* value()
  {
    return "std_msgs/UInt64";
  }

};

template<>
struct DataTypeId<std_msgs::UInt64*>
{
  static const int value()
  {
    return UINT64_MSG_ID;
  }

};

template<>
struct Definition<std_msgs::UInt64*>
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
  struct CallCallbackFuncs<UINT64_MSG_ID>{
    static void call(void (*fp)(intptr_t), char *rbuf)
    {
      std_msgs::UInt64 msg;
      rbuf += 4;
      memcpy(&msg.data,rbuf,8);
      //msg.data = &rbuf[8];
      fp(&msg);
    }
  };
}

#endif