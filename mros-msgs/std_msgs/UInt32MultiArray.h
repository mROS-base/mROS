#include <vector>

static const int UINT32MULTIARRAY_MSG_ID = 17;

namespace std_msgs{
class UInt32MultiArray{
public:
	std::vector<uint32_t> data;
  int dataSize(){return data.size() * 4 + 12;}
  void memCopy(char *addrPtr){
    addrPtr += 8;
    uint32_t size = data.size();
    memcpy(addrPtr,&size,4);
    addrPtr += 4;
    const uint32_t* ptr = data.data();
    for(int i=0; i<size ; i++){
      memcpy(addrPtr, &(ptr[i]),4);
      addrPtr += 4;
    }
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
    return "std_msgs/UInt32MultiArray";
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
      std_msgs::UInt32MultiArray msg;
      rbuf += 4;
      rbuf += 8;
      uint32_t buf_int;
      uint32_t arr_size;
      memcpy(&arr_size,rbuf,4 );
      rbuf += 4;
      msg.data.reserve(arr_size);
      for(int i=0 ; i < arr_size ; i++){
        memcpy(&buf_int,rbuf, 4);
        msg.data.push_back(buf_int);
        rbuf += 4;
      }
      fp(&msg);
    }
  };
}
