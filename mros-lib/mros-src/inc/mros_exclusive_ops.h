#ifndef _MROS_EXCLUSIVE_OPS_H_
#define _MROS_EXCLUSIVE_OPS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mros_types.h"
#include "mros_os.h"


typedef struct {
	/*
	 * 排他エリアのプライオリティ
	 * 排他エリア内では，タスクのプライオリティは本プライオリティに引き上げられる
	 */
	mRosTaskPriorityType	priority;
} mRosExclusiveObjectType;

typedef struct {
	mRosTaskPriorityType	org_priority;
} mROsExclusiveUnlockObjType;

extern void mros_exclusive_init(mRosExclusiveObjectType *exobj, mRosTaskPriorityType priority);
extern void mros_exclusive_lock(mRosExclusiveObjectType *exobj, mROsExclusiveUnlockObjType *unlock_obj);
extern void mros_exclusive_unlock(mROsExclusiveUnlockObjType *unlock_objj);

#ifdef __cplusplus
}
#endif

#endif /* _MROS_EXCLUSIVE_OPS_H_ */
