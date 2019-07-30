#ifndef _MROS_LOG_H_
#define _MROS_LOG_H_

#include <kernel.h>
#include <t_syslog.h>

#define ROS_DEBUG(...) 		syslog(LOG_DEBUG, __VA_ARGS__)
#define ROS_INFO(...) 		syslog(LOG_NOTICE, __VA_ARGS__)
#define ROS_WARN(...) 		syslog(LOG_WARNING, __VA_ARGS__)
#define ROS_ERROR(...) 		syslog(LOG_ERROR, __VA_ARGS__)
#define ROS_FATAL(...) 		syslog(LOG_EMERG, __VA_ARGS__)


#endif /* _MROS_LOG_H_ */
