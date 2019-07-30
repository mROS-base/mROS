#include <vector>

#include <string.h>

static const int INT16MULTIARRAY_MSG_ID = 12;

namespace std_msgs{
class Int16MultiArray{
public:
	std::vector<int16_t> data;
  int dataSize(){return data.size() * 2+ 12;}
  void memCopy(char *addrPtr){
    addrPtr += 8;
    uint32_t size = data.size();
    memcpy(addrPtr,&size,4);
    addrPtr += 4;
    const int16_t* ptr = data.data();
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
struct MD5Sum<INT16MULTIARRAY_MSG_ID>
{
  static const char* value()
  {
    return "d9338d7f523fcb692fae9d0a0e9f067c";
  }

};

template<>
struct DataType<std_msgs::Int16MultiArray*>
{
  static const char* value()
  {
    return "std_msgs/Int16MultiArray";
  }

};

template<>
struct DataTypeId<std_msgs::Int16MultiArray*>
{
  static const int value()
  {
    return INT16MULTIARRAY_MSG_ID;
  }

};

template<>
struct Definition<std_msgs::Int16MultiArray*>
{
	static const char* value()
	{
		return "int16[] data\n\\n\
";
	}
};
}

namespace subtask_methods
{
  template<>
  struct CallCallbackFuncs<INT16MULTIARRAY_MSG_ID>{
    static void call(void (*fp)(intptr_t), char *rbuf)
    {
      std_msgs::Int16MultiArray msg;
      rbuf += 4;
      rbuf += 8;
      int16_t buf_int;
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
