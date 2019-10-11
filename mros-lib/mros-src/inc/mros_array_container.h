#ifndef _MROS_ARRAY_CONTAINER_H_
#define _MROS_ARRAY_CONTAINER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mros_types.h"

typedef struct {
	mros_uint32 			count;
	mros_uint32 			array_num;
	mRosContainerObjType	*array;
} mRosArrayContainerType;

#define MROS_ARRAY_CONTAINER_CONFIG_DECLARE_MANAGER(manager_name, array_num)	\
	static mRosContainerObjType manager_name##_array [(array_num)] MROS_MATTR_BSS_NOCLR;	\
	static mRosArrayContainerType manager_name = {	\
		(0),	\
		(array_num),	\
		manager_name##_array,	\
	};

static inline void mros_array_container_add(mRosArrayContainerType *mgrp, mRosContainerObjType obj)
{
	if (mgrp->count >= mgrp->array_num) {
		return;
	}
	mgrp->array[mgrp->count] = obj;
	mgrp->count++;
}
#ifdef __cplusplus
}
#endif

#endif /* _MROS_ARRAY_CONTAINER_H_ */
