#ifndef _{{msg.PKG}}_{{msg.NAME}}_H
#define _{{msg.PKG}}_{{msg.NAME}}_H

{% set id_name = msg.NAME + '_MSG_ID'%}
static const int {{id_name}} = {{msg.id}};

namespace {{msg.pkg}}{
class {{msg.name}}{
public:
{%for def_data in msg.def %}  {{def_data[0]}} {{def_data[1]}};
{% endfor %}};
}

namespace message_traits
{
template<>
struct MD5Sum<{{id_name}}>
{
  static const char* value()
  {
    return "{{msg.md5}}";
  }

};

template<>
struct DataType<{{msg.pkg}}::{{msg.name}}*>
{
  static const char* value()
  {
    return "{{msg.pkg}}/{{msg.name}}";
  }

};

template<>
struct DataTypeId<{{msg.pkg}}::{{msg.name}}*>
{
  static const int value()
  {
    return {{id_name}};
  }

};

template<>
struct Definition<{{msg.pkg}}::{{msg.name}}*>
{
	static const char* value()
	{
		return "{% for def_str in msg.def %}{{ def_str[0] }} {{def_str[1]}}\n\
{% endfor %}";
	}
};
}

namespace subtask_methods
{
  template<>
  struct CallCallbackFuncs<{{id_name}}>{
    static void call(void (*fp)(intptr_t), char *rbuf)
    {
      {{msg.pkg}}::{{msg.name}} msg;
      msg.data = &rbuf[8];
      fp(&msg);
    }
  };
}

#endif