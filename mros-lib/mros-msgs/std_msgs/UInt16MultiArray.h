#include <vector>
#include <string.h>

static const int UINT16MULTIARRAY_MSG_ID = 16;

namespace std_msgs{
class UInt16MultiArray{
public:
	std::vector<uint16_t> data;
  int dataSize(){return data.size() * 2+ 12;}
  void memCopy(char *addrPtr){
    addrPtr += 8;
    uint32_t size = data.size();
    memcpy(addrPtr,&size,4);
    addrPtr += 4;
    const uint16_t* ptr = data.data();
    for(int i=0; i<size ; i++){
      memcpy(addrPtr, &(ptr[i]),2);
      addrPtr += 2;
    }
  }
};
}

namespace message_traits
{
template<>
struct MD5Sum<UINT16MULTIARRAY_MSG_ID>
{
  static const char* value()
  {
    return "52f264f1c973c4b73790d384c6cb4484";
  }

};

template<>
struct DataType<std_msgs::UInt16MultiArray*>
{
  static const char* value()
  {
    return "std_msgs/UInt16MultiArray";
  }

};

template<>
struct DataTypeId<std_msgs::UInt16MultiArray*>
{
  static int value()
  {
    return (int)UINT16MULTIARRAY_MSG_ID;
  }

};

template<>
struct Definition<std_msgs::UInt16MultiArray*>
{
	static const char* value()
	{
		return "uint16[] data\n\\n\
";
	}
};
}

namespace subtask_methods
{
  template<>
  struct CallCallbackFuncs<UINT16MULTIARRAY_MSG_ID>{
    static void call(void (*fp)(void *), char *rbuf)
    {
      std_msgs::UInt16MultiArray msg;
      rbuf += 4;
      rbuf += 8;
      uint16_t buf_int;
      uint32_t arr_size;
      memcpy(&arr_size,rbuf,4 );
      rbuf += 4;
      msg.data.reserve(arr_size);
      for(int i=0 ; i < arr_size ; i++){
        memcpy(&buf_int,rbuf, 2);
        msg.data.push_back(buf_int);
        rbuf += 2;
      }
      fp(&msg);
    }
  };
}
