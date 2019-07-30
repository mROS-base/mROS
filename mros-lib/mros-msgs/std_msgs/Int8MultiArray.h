#include <vector>
#include <string.h>

static const int INT8MULTIARRAY_MSG_ID = 11;

namespace std_msgs{
class Int8MultiArray{
public:
	std::vector<int8_t> data;
  int dataSize(){return data.size() * 1 + 12;}
  void memCopy(char *addrPtr){
    addrPtr += 8;
    uint32_t size = data.size();
    memcpy(addrPtr,&size,4);
    addrPtr += 4;
    const int8_t* ptr = data.data();
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
struct MD5Sum<INT8MULTIARRAY_MSG_ID>
{
  static const char* value()
  {
    return "d7c1af35a1b4781bbe79e03dd94b7c13";
  }

};

template<>
struct DataType<std_msgs::Int8MultiArray*>
{
  static const char* value()
  {
    return "std_msgs/Int8MultiArray";
  }

};

template<>
struct DataTypeId<std_msgs::Int8MultiArray*>
{
  static int value()
  {
    return (int)INT8MULTIARRAY_MSG_ID;
  }

};

template<>
struct Definition<std_msgs::Int8MultiArray*>
{
	static const char* value()
	{
		return "int8[] data\n\\n\
";
	}
};
}

namespace subtask_methods
{
  template<>
  struct CallCallbackFuncs<INT8MULTIARRAY_MSG_ID>{
    static void call(void (*fp)(void *), char *rbuf)
    {
      std_msgs::Int8MultiArray msg;
      rbuf += 4;
      rbuf += 8;
      int8_t buf_int;
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
