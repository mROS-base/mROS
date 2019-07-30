#ifndef _MROS_EXCLUSIVE_AREA_H_
#define _MROS_EXCLUSIVE_AREA_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "mros_exclusive_ops.h"
#include "mros_wait_queue.h"

extern mRosExclusiveObjectType mros_exclusive_area MROS_MATTR_BSS_NOCLR;
extern mRosWaitQueueType mros_master_wait_queue MROS_MATTR_BSS_NOCLR;
extern mRosWaitQueueType mros_subscribe_wait_queue MROS_MATTR_BSS_NOCLR;

extern void mros_exclusive_area_init(mRosTaskIdType mas_task_id, mRosTaskIdType sub_task_id);

#ifdef __cplusplus
}
#endif

#endif /* _MROS_EXCLUSIVE_AREA_H_ */
