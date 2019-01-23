#include <vector>

static const int UINT64MULTIARRAY_MSG_ID = 18;

namespace std_msgs{
class UInt64MultiArray{
public:
	std::vector<uint64_t> data;
  int dataSize(){return data.size() * 8+ 12;}
  void memCopy(char *addrPtr){
    addrPtr += 8;
    uint32_t size = data.size();
    memcpy(addrPtr,&size,4);
    addrPtr += 4;
    const uint64_t* ptr = data.data();
    for(int i=0; i<size ; i++){
      memcpy(addrPtr, &(ptr[i]),8);
      addrPtr += 8;
    }
  }
};
}

namespace message_traits
{
template<>
struct MD5Sum<UINT64MULTIARRAY_MSG_ID>
{
  static const char* value()
  {
    return "6088f127afb1d6c72927aa1247e945af";
  }

};

template<>
struct DataType<std_msgs::UInt64MultiArray*>
{
  static const char* value()
  {
    return "std_msgs/UInt64MultiArray";
  }

};

template<>
struct DataTypeId<std_msgs::UInt64MultiArray*>
{
  static const int value()
  {
    return UINT64MULTIARRAY_MSG_ID;
  }

};

template<>
struct Definition<std_msgs::UInt64MultiArray*>
{
	static const char* value()
	{
		return "uint64[] data\n\\n\
";
	}
};
}

namespace subtask_methods
{
  template<>
  struct CallCallbackFuncs<UINT64MULTIARRAY_MSG_ID>{
    static void call(void (*fp)(intptr_t), char *rbuf)
    {
      std_msgs::UInt64MultiArray msg;
      rbuf += 4;
      rbuf += 8;
      uint64_t buf_int;
      uint32_t arr_size;
      memcpy(&arr_size,rbuf,4 );
      rbuf += 4;
      msg.data.reserve(arr_size);
      for(int i=0 ; i < arr_size ; i++){
        memcpy(&buf_int,rbuf, 8);
        msg.data.push_back(buf_int);
        rbuf += 8;
      }
      fp(&msg);
    }
  };
}
