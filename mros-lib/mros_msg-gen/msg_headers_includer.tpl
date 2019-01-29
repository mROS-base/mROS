{% for msg in std_msgs %}
#include "{{msg.pkg}}/{{msg.name}}.h"{% endfor %}
{% for msg in msgs %}
#include "{{msg.pkg}}/{{msg.name}}.h"{% endfor %}


void callCallback(int id, void (*fp)(intptr_t), char *rbuf){
	switch(id){
	{%for msg in std_msgs %}	case {{ msg.id }}:
			subtask_methods::CallCallbackFuncs<{{ msg.id }}>().call(fp,rbuf);
			break;
	{% endfor %}
	{%for msg in msgs %}	case {{ msg.id }}:
			subtask_methods::CallCallbackFuncs<{{ msg.id }}>().call(fp,rbuf);
			break;
	{% endfor %}
	}
}

std::string getMD5Sum(int id){
	switch(id){
	{%for msg in std_msgs %}	case {{ msg.id }}:
			return message_traits::MD5Sum<{{ msg.id }}>().value();
			break;
	{% endfor %}
	{%for msg in msgs %}	case {{ msg.id }}:
			return message_traits::MD5Sum<{{ msg.id }}>().value();
			break;
	{% endfor %}
	}
}