#include <vector>
#include <string.h>

static const int INT64MULTIARRAY_MSG_ID = 14;

namespace std_msgs{
class Int64MultiArray{
public:
	std::vector<int64_t> data;
  int dataSize(){return data.size() * 8+ 12;}
  void memCopy(char *addrPtr){
    addrPtr += 8;
    uint32_t size = data.size();
    memcpy(addrPtr,&size,4);
    addrPtr += 4;
    const int64_t* ptr = data.data();
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
struct MD5Sum<INT64MULTIARRAY_MSG_ID>
{
  static const char* value()
  {
    return "54865aa6c65be0448113a2afc6a49270";
  }

};

template<>
struct DataType<std_msgs::Int64MultiArray*>
{
  static const char* value()
  {
    return "std_msgs/Int64MultiArray";
  }

};

template<>
struct DataTypeId<std_msgs::Int64MultiArray*>
{
  static int value()
  {
    return (int)INT64MULTIARRAY_MSG_ID;
  }

};

template<>
struct Definition<std_msgs::Int64MultiArray*>
{
	static const char* value()
	{
		return "int64[] data\n\\n\
";
	}
};
}

namespace subtask_methods
{
  template<>
  struct CallCallbackFuncs<INT64MULTIARRAY_MSG_ID>{
    static void call(void (*fp)(void *), char *rbuf)
    {
      std_msgs::Int64MultiArray msg;
      rbuf += 4;
      rbuf += 8;
      int64_t buf_int;
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
