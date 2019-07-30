#include "ros.h"
{% for msg in std_msgs %}
#include "{{msg.pkg}}/{{msg.name}}.h"{% endfor %}
{% for msg in msgs %}
#include "{{msg.pkg}}/{{msg.name}}.h"{% endfor %}

{% for msg in std_msgs %}
template ros::Subscriber ros::NodeHandle::subscribe(std::string,int,void (*fp)({{msg.pkg}}::{{msg.name}}*));
template ros::Publisher ros::NodeHandle::advertise<{{msg.pkg}}::{{msg.name}}>(std::string, int);
template void ros::Publisher::publish({{msg.pkg}}::{{msg.name}}&);{% endfor %}
{% for msg in msgs %}
template ros::Subscriber ros::NodeHandle::subscribe(std::string,int,void (*fp)({{msg.pkg}}::{{msg.name}}*));
template ros::Publisher ros::NodeHandle::advertise<{{msg.pkg}}::{{msg.name}}>(std::string, int);
template void ros::Publisher::publish({{msg.pkg}}::{{msg.name}}&);{% endfor %}
