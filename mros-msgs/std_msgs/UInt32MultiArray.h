

static const int UINT32MULTIARRAY_MSG_ID = 4;

namespace std_msgs{
class UInt32MultiArray{
public:
	uint32_t data;
  int dataSize(){return 4;}
  void memCopy(char *addrPtr){
    memcpy(addrPtr,&data,1);
  }
};
}

namespace message_traits
{
template<>
struct MD5Sum<UINT32MULTIARRAY_MSG_ID>
{
  static const char* value()
  {
    return "4d6a180abc9be191b96a7eda6c8a233d";
  }

};

template<>
struct DataType<std_msgs::UInt32MultiArray*>
{
  static const char* value()
  {
    return "std_msgs/UInt32";
  }

};

template<>
struct DataTypeId<std_msgs::UInt32MultiArray*>
{
  static const int value()
  {
    return UINT32MULTIARRAY_MSG_ID;
  }

};

template<>
struct Definition<std_msgs::UInt32MultiArray*>
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
  struct CallCallbackFuncs<UINT32MULTIARRAY_MSG_ID>{
    static void call(void (*fp)(intptr_t), char *rbuf)
    {
    }
  };
}
