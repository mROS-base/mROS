#ifndef _MROS_LOG_H_
#define _MROS_LOG_H_

#include <kernel.h>
#include <t_syslog.h>

#ifdef MROS_LOG_DISABLE_DEBUG
#define ROS_DEBUG(...)
#else
#define ROS_DEBUG(...) 		syslog(LOG_DEBUG, __VA_ARGS__)
#endif

#ifdef MROS_LOG_DISABLE_INFO
#define ROS_INFO(...)
#else
#define ROS_INFO(...)       syslog(LOG_NOTICE, __VA_ARGS__)
#endif

#ifdef MROS_LOG_DISABLE_WARN
#define ROS_WARN(...)
#else
#define ROS_WARN(...)       syslog(LOG_WARNING, __VA_ARGS__)
#endif

#ifdef MROS_LOG_DISABLE_ERROR
#define ROS_ERROR(...)
#else
#define ROS_ERROR(...)      syslog(LOG_ERROR, __VA_ARGS__)
#endif

#ifdef MROS_LOG_DISABLE_FATAL
#define ROS_FATAL(...)
#else
#define ROS_FATAL(...)      syslog(LOG_EMERG, __VA_ARGS__)
#endif

#endif /* _MROS_LOG_H_ */