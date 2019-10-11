#ifndef _MROS_OS_TARGET_H_
#define _MROS_OS_TARGET_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <kernel.h>
#include <t_syslog.h>

#include "mros_types.h"

typedef ID	mRosTaskIdType;
typedef PRI	mRosTaskPriorityType;
typedef mros_uint32 mRosTaskSleepIntervalType;

#define MROS_TASK_MAX_PRIORITY 	TMIN_TPRI
#define MROS_TASK_MIN_PRIORITY 	TMAX_TPRI

#define MROS_LOG_EMRG		LOG_EMERG
#define MROS_LOG_ALERT		LOG_ALERT
#define MROS_LOG_CRIT		LOG_CRIT
#define MROS_LOG_ERROR		LOG_ERROR
#define MROS_LOG_WARNING	LOG_WARNING
#define MROS_LOG_NOTICE		LOG_NOTICE
#define MROS_LOG_INFO		LOG_INFO
#define MROS_LOG_DEBUG		LOG_DEBUG

#include "mros_log.h"


#ifdef __cplusplus
}
#endif


#endif /* _MROS_OS_TARGET_H_ */
