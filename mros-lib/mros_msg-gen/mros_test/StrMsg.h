#ifndef _MROS_TEST_STRMSG_H
#define _MROS_TEST_STRMSG_H


static const int STRMSG_MSG_ID = 100;

namespace mros_test{
class StrMsg{
public:
  string str1;
  string str2;
  string str3;
};
}

namespace message_traits
{
template<>
struct MD5Sum<STRMSG_MSG_ID>
{
  static const char* value()
  {
    return "035386b5c67e99b44146d9acc5131771";
  }

};

template<>
struct DataType<mros_test::StrMsg*>
{
  static const char* value()
  {
    return "mros_test/StrMsg";
  }

};

template<>
struct DataTypeId<mros_test::StrMsg*>
{
  static const int value()
  {
    return STRMSG_MSG_ID;
  }

};

template<>
struct Definition<mros_test::StrMsg*>
{
	static const char* value()
	{
		return "string str1\n\
string str2\n\
string str3\n\
";
	}
};
}

namespace subtask_methods
{
  template<>
  struct CallCallbackFuncs<STRMSG_MSG_ID>{
    static void call(void (*fp)(intptr_t), char *rbuf)
    {
      mros_test::StrMsg msg;
      msg.data = &rbuf[8];
      fp(&msg);
    }
  };
}

#endif