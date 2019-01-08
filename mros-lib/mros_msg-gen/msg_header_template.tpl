#ifndef _{{msg.PKG}}_{{msg.NAME}}_H
#define _{{msg.PKG}}_{{msg.NAME}}_H

{% set id_name = msg.NAME + '_MSG_ID'%}
static const int {{id_name}} = {{msg.id}};

namespace {{msg.pkg}}{
class {{msg.name}}{
public:
{%for def_data in msg.def %}  {{def_data[0]}} {{def_data[1]}};
{% endfor %}
  int dataSize(){
    return {%for def_data in msg.def %} {{def_data[1]}}.size() + {%endfor%} 4*{{defSize}};
  }

  void memCopy(char *addrPtr){
    int size; {%for def_data in msg.def %}
    size = {{def_data[1]}}.size();
    memcpy(addrPtr,&size,4);
    addrPtr += 4;
    memcpy(addrPtr, {{def_data[1]}}.c_str(),size);
    addrPtr += size;
    {% endfor %}
  }
};

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
      int ptr = 8;
      {% for def_str in msg.def %}msg.{{ def_str[1]}} = &rbuf[ptr];
      ptr += msg.{{ def_str[1] }}.size() + 3;
      {% endfor %}
      fp(&msg);
    }
  };
}

#endif