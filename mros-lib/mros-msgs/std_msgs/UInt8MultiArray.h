#include <vector>
#include <string.h>

static const int UINT8MULTIARRAY_MSG_ID = 15;

namespace std_msgs{
class UInt8MultiArray{
public:
	std::vector<uint8_t> data;
  int dataSize(){return data.size() * 1 + 12;}
  void memCopy(char *addrPtr){
    addrPtr += 8;
    uint32_t size = data.size();
    memcpy(addrPtr,&size,4);
    addrPtr += 4;
    const uint8_t* ptr = data.data();
    for(int i=0; i<size ; i++){
      memcpy(addrPtr, &(ptr[i]),1);
      addrPtr += 1;
    }
  }
};
}

namespace message_traits
{
template<>
struct MD5Sum<UINT8MULTIARRAY_MSG_ID>
{
  static const char* value()
  {
    return "82373f1612381bb6ee473b5cd6f5d89c";
  }

};

template<>
struct DataType<std_msgs::UInt8MultiArray*>
{
  static const char* value()
  {
    return "std_msgs/UInt8MultiArray";
  }

};

template<>
struct DataTypeId<std_msgs::UInt8MultiArray*>
{
  static const int value()
  {
    return UINT8MULTIARRAY_MSG_ID;
  }

};

template<>
struct Definition<std_msgs::UInt8MultiArray*>
{
	static const char* value()
	{
		return "uint8[] data\n\\n\
";
	}
};
}

namespace subtask_methods
{
  template<>
  struct CallCallbackFuncs<UINT8MULTIARRAY_MSG_ID>{
    static void call(void (*fp)(intptr_t), char *rbuf)
    {
      std_msgs::UInt8MultiArray msg;
      rbuf += 4;
      rbuf += 8;
      uint8_t buf_int;
      uint32_t arr_size;
      memcpy(&arr_size,rbuf,4 );
      rbuf += 4;
      msg.data.reserve(arr_size);
      for(int i=0 ; i < arr_size ; i++){
        memcpy(&buf_int,rbuf, 1);
        msg.data.push_back(buf_int);
        rbuf += 1;
      }
      fp(&msg);
    }
  };
}
