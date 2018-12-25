namespace std_msgs{
class String{
public:
	std::string data;
};
}

namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum
{
  static const char* value()
  {
    return "992ce8a1687cec8c8bd883ec73ca41d1";
  }

};

template<class ContainerAllocator>
struct DataType
{
  static const char* value()
  {
    return "std_msgs/String";
  }

};
}