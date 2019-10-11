#ifndef _MROS_OS_H_
#define _MROS_OS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mros_types.h"

extern mRosTaskIdType mros_get_taskid(void);
extern mRosTaskPriorityType mros_get_taskpri(void);
extern void mros_change_taskpri(mRosTaskPriorityType priority);
extern void mros_sleep_task(void);
extern void mros_sleep_task_msec(mRosTaskSleepIntervalType msec);
extern void mros_wakeup_task(mRosTaskIdType task_id);

#ifdef __cplusplus
}
#endif


#endif /* _MROS_OS_H_ */
