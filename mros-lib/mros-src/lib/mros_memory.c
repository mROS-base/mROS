#include "mros_memory.h"


#define MEMORY_ID(index)		((index) + 1U)
#define MEMORY_INDEX(id)		((id) - 1U)

#define MEMORY_OBJ(mid, id)		memory_manager[(mid)].memory_entries[MEMORY_INDEX((id))]

mRosReturnType mros_mem_init(mRosSizeType config_num, mRosMemoryConfigType **config, mRosMemoryManagerType *mgrp)
{
	mros_uint32 i;
	mros_uint32 j;
	mgrp->header_num = config_num;
	for (i = 0; i < config_num; i++) {
		mgrp->header_array[i].max_memory_num = config[i]->max_memory_num;
		mgrp->header_array[i].memsize = config[i]->memsize;

		mgrp->header_array[i].memory_entries = config[i]->memory_entries;
		mgrp->header_array[i].memory = config[i]->memory;

		for (j = 0; j < mgrp->header_array[i].max_memory_num; j++) {
			mRosMemoryListEntryType *entry = &(mgrp->header_array[i].memory_entries[j]);
			entry->data.mgrp = mgrp;
			entry->data.header_id = (mRosMemorySizeIdType)i;
			entry->data.memory_id = MEMORY_ID(j);
			entry->data.memsize = mgrp->header_array[i].memsize;
			entry->data.size = 0;
			entry->data.memp = &(mgrp->header_array[i].memory[(mgrp->header_array[i].memsize * j)]);
		}
		List_Init(&mgrp->header_array[i].head, mRosMemoryListEntryType, mgrp->header_array[i].max_memory_num, mgrp->header_array[i].memory_entries);
	}
	return MROS_E_OK;
}

mRosReturnType mros_mem_alloc(mRosMemoryManagerType *mgrp, mRosSizeType size, mRosMemoryListEntryType **memory)
{
	mros_uint32 i;
	if (memory == MROS_NULL) {
		return MROS_E_INVAL;
	}
	for (i = 0; i < mgrp->header_num; i++) {
		if (size > mgrp->header_array[i].memsize) {
			continue;
		}
		if (mgrp->header_array[i].head.free_num <= 0) {
			continue;
		}
		ListEntry_Alloc(&mgrp->header_array[i].head, mRosMemoryListEntryType, memory);
		(*memory)->data.size = size;
		return MROS_E_OK;
	}
	ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, MROS_E_NOMEM);
	return MROS_E_NOMEM;
}

mRosReturnType mros_mem_free(mRosMemoryManagerType *mgrp, mRosMemoryListEntryType *memory)
{
	if (memory == MROS_NULL) {
		return MROS_E_OK;
	}
	if (memory->data.header_id >= mgrp->header_num) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, MROS_E_INVAL);
		return MROS_E_INVAL;
	}
	if (memory->data.memory_id > mgrp->header_array[memory->data.header_id].max_memory_num) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, MROS_E_RANGE);
		return MROS_E_RANGE;
	}
	memory->data.size = 0;
	ListEntry_Free(&mgrp->header_array[memory->data.header_id].head, memory);
	return MROS_E_OK;
}

