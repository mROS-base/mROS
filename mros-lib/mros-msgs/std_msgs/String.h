#include "ros.h"

namespace std_msgs{
class String{
public:
	std::string data;
};
}

namespace message_traits
{
template<>
struct MD5Sum<std_msgs::String*>
{
  static const char* value()
  {
    return "992ce8a1687cec8c8bd883ec73ca41d1";
  }

};

template<>
struct DataType<std_msgs::String*>
{
  static const char* value()
  {
    return "std_msgs/String";
  }

};

template<>
struct Definition<std_msgs::String*>
{
	static const char* value()
	{
		return "string data\n\
";
	}
};
}