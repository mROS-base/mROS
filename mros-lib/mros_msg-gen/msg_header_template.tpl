#ifndef _{{msg.PKG}}_{{msg.NAME}}_H
#define _{{msg.PKG}}_{{msg.NAME}}_H

{% set id_name = msg.NAME + '_MSG_ID'%}
static const int {{id_name}} = {{msg.id}};

namespace {{msg.pkg}}{
class {{msg.name}}{
public:
{%for def_data in msg.def %}  {{def_data.cppType}} {{def_data.typeName}};
{% endfor %}
  int dataSize(){
    return {%for def_data in msg.def %} {%if def_data.rosType == 'string'%}{{def_data.typeName}}.size(){% else %}{{def_data.size}}{%endif%} + {%endfor%} 4*{{msg.strNum}};
  }

  void memCopy(char *addrPtr){
    int size; 
    {%for def_data in msg.def %}{% if def_data.rosType == 'string' %}
    size = {{def_data.typeName}}.size();
    memcpy(addrPtr,&size,4);
    addrPtr += 4;
    memcpy(addrPtr, {{def_data.typeName}}.c_str(),size);
    addrPtr += size;
    {% else %}
    memcpy(addrPtr,&{{def_data.typeName}},{{def_data.size}});
    addrPtr += {{def_data.size}};
    {% endif %}{% endfor %}
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
		return "{% for def_str in msg.def %}{{ def_str.rosType }} {{def_str.typeName}}\n\
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
      int size;
      rbuf += 4;
      {% for def_str in msg.def %}{% if def_str.rosType == 'string' %}{
        memcpy(&size,rbuf,4);
        rbuf += 4;
        char buf_char[size+1];
        memcpy(&buf_char,rbuf,size);
        buf_char[size] = '\0';
        msg.{{def_str.typeName}} = buf_char;
        rbuf += size;
      }
      {% elif  def_str.rosType[-2:] == "[]"%}{
        {{def_str.rosType[:-2]}}_t size;
        memcpy(&size,rbuf,4);
        rbuf += 4;
        msg.person2.reserve(size);
        for(int i=0;i<size;i++){
          int buf;
          memcpy(&buf,rbuf,{{def_str.size}});
          msg.{{def_str.typeName}}.push_back(buf);
          rbuf += {{def_str.size}};
        }

      }
      {% else %}memcpy(&msg.{{def_str.typeName}},rbuf,{{def_str.size}});
      rbuf += {{def_str.size}};
      {% endif %}{% endfor %}
      fp(&msg);
    }
  };
}

#endif