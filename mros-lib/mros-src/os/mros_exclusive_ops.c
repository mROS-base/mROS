#include "mros_os.h"
#include "mros_exclusive_ops.h"

void mros_exclusive_init(mRosExclusiveObjectType *exobj, mRosTaskPriorityType priority)
{
	exobj->priority = priority;
	return;
}

void mros_exclusive_lock(mRosExclusiveObjectType *exobj, mROsExclusiveUnlockObjType *unlock_obj)
{
	unlock_obj->org_priority = mros_get_taskpri();
	mros_change_taskpri(exobj->priority);
	return;
}

void mros_exclusive_unlock(mROsExclusiveUnlockObjType *unlock_obj)
{
	mros_change_taskpri(unlock_obj->org_priority);
	return;
}
