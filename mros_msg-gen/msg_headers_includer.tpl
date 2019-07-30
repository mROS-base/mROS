{% for msg in std_msgs %}
#include "{{msg.pkg}}/{{msg.name}}.h"{% endfor %}
{% for msg in msgs %}
#include "{{msg.pkg}}/{{msg.name}}.h"{% endfor %}


static void callCallback(int id, void (*fp)(void *), char *rbuf, int len){
	switch(id){
	{%for msg in std_msgs %}	case {{ msg.id }}:
			subtask_methods::CallCallbackFuncs<{{ msg.id }}>().call(fp,rbuf, len);
			break;
	{% endfor %}
	{%for msg in msgs %}	case {{ msg.id }}:
			subtask_methods::CallCallbackFuncs<{{ msg.id }}>().call(fp,rbuf);
			break;
	{% endfor %}
	}
}

static const char* getMD5Sum(int id){
	switch(id){
	{%for msg in std_msgs %}	case {{ msg.id }}:
			return message_traits::MD5Sum<{{ msg.id }}>().value();
	{% endfor %}
	{%for msg in msgs %}	case {{ msg.id }}:
			return message_traits::MD5Sum<{{ msg.id }}>().value();
	{% endfor %}
	}
	return NULL;
}