#ifndef _SENSOR_MSGS_TEST_IMAGE_H
#define _SENSOR_MSGS_IMAGE_H

#include "std_msgs/Header.h"



static const int IMAGE_MSG_ID = 20;

namespace sensor_msgs{
class Image{
public:
  std_msgs::Header header;
  uint32_t height;
  uint32_t width;
  string encoding;
  uint8_t is_bigendian;
  uint32_t step;
  std::vector<uint8_t> data;

  int dataSize(){
    return  header.dataSize() +  4 +  4 +  encoding.size() +  1 +  4 +  data.size()*1 + 4  +  4*4;
  }

  void memCopy(char*& addrPtr){
    int size; 
    
    header.memCopy(addrPtr);
    
    memcpy(addrPtr,&height,4);
    addrPtr += 4;
    
    memcpy(addrPtr,&width,4);
    addrPtr += 4;
    
    size = encoding.size();
    memcpy(addrPtr,&size,4);
    addrPtr += 4;
    memcpy(addrPtr, encoding.c_str(),size);
    addrPtr += size;
    
    memcpy(addrPtr,&is_bigendian,1);
    addrPtr += 1;
    
    memcpy(addrPtr,&step,4);
    addrPtr += 4;
    {
      size = data.size();
      memcpy(addrPtr,&size,4);
      addrPtr += 4;
      const uint8_t* ptr = data.data();
      for(int i=0; i<size ; i++){
        memcpy(addrPtr, &(ptr[i]),1);
        addrPtr += 1;
      }
    }
    
  }

  void deserialize(char*& rbuf){
    uint32_t size;
    
    header.deserialize(rbuf);
    
    memcpy(&height,rbuf,4);
    rbuf += 4;
    
    memcpy(&width,rbuf,4);
    rbuf += 4;
    
    {
      memcpy(&size,rbuf,4);
      rbuf += 4;
      char buf_char[size+1];
      memcpy(&buf_char,rbuf,size);
      buf_char[size] = '\0';
      encoding = buf_char;
      rbuf += size;
    }
    
    memcpy(&is_bigendian,rbuf,1);
    rbuf += 1;
    
    memcpy(&step,rbuf,4);
    rbuf += 4;
    
    {
      uint32_t size;
      memcpy(&size,rbuf,4);
      rbuf += 4;
      data.reserve(size);
      for(int i=0;i<size;i++){
        uint8_t buf;
        memcpy(&buf,rbuf,1);
        data.push_back(buf);
        rbuf += 1;
      }
    }
    
    
  }
};

}

namespace message_traits
{
template<>
struct MD5Sum<IMAGE_MSG_ID>
{
  static const char* value()
  {
    return "060021388200f6f0f447d0fcd9c64743";
  }

};

template<>
struct DataType<sensor_msgs::Image*>
{
  static const char* value()
  {
    return "sensor_msgs/Image";
  }

};

template<>
struct DataTypeId<sensor_msgs::Image*>
{
  static int value()
  {
    return (int)IMAGE_MSG_ID;
  }

};

template<>
struct Definition<sensor_msgs::Image*>
{
	static const char* value()
	{
		return "std_msgs header\n\
uint32 height\n\
uint32 width\n\
string encoding\n\
uint8 is_bigendian\n\
uint32 step\n\
uint8[] data\n\
";
	}
};
}

namespace subtask_methods
{
  template<>
  struct CallCallbackFuncs<IMAGE_MSG_ID>{
    static void call(void (*fp)(void *), char *rbuf)
    {
      sensor_msgs::Image msg;
      rbuf += 4;
      msg.deserialize(rbuf);
      fp(&msg);
    }
  };
}

#endif
