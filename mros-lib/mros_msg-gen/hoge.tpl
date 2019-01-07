{{% for msg in msgs %}}
{{msg.name}}
{{% endfor %}}

void callCallback(int id, void (*fp)(intptr_t), char *rbuf){
	switch(id){
	{%for msg in msgs %}
		case {{ msg.id }}:
			subtask_methods::CallCallbackFuncs<{{ msg.id }}>().call(fp,rbuf);
			break;
	{% endfor %}
	}
}