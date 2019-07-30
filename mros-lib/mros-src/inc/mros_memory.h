#ifndef _MROS_MEMORY_H_
#define _MROS_MEMORY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mros_types.h"
#include "mros_list.h"

typedef mros_uint32 mRosMemorySizeIdType;

struct mRosMemoryManagerType;
typedef struct {
	/*
	 * manager
	 */
	struct mRosMemoryManagerType  *mgrp;
	/*
	 * id of memory_entries
	 */
	mRosIdType				memory_id;
	/*
	 * id of parent header
	 */
	mRosMemorySizeIdType	header_id;
	/*
	 * preallocation unit size
	 */
	mRosSizeType			memsize;
	/*
	 * real using size
	 */
	mRosSizeType			size;
	char 					*memp;
} mRosMemoryEntryType;

typedef ListEntryType(mRosMemoryListEntryType, mRosMemoryEntryType) mRosMemoryListEntryType;
typedef ListHeadType(mRosMemoryListEntryType) mRosMemoryListHeadType;

typedef struct {
	/*
	 * memory list header
	 */
	mRosMemoryListHeadType 	head;
	/*
	 * num of preallocation memory
	 */
	mRosSizeType 			max_memory_num;
	/*
	 * preallocation unit size
	 */
	mRosSizeType			memsize;
	/*
	 * memory list entry
	 */
	mRosMemoryListEntryType *memory_entries;
	/*
	 * raw data
	 */
	char					*memory;
} mRosMemoryHeaderType;

typedef struct mRosMemoryManagerType {
	/*
	 * num of header
	 */
	mRosSizeType 			header_num;
	/*
	 * header array
	 */
	mRosMemoryHeaderType 	*header_array;
} mRosMemoryManagerType;

/*
 * config memory
 */
typedef struct {
	mRosSizeType 			max_memory_num;
	mRosSizeType			memsize;
	mRosMemoryListEntryType *memory_entries;
	char					*memory;
} mRosMemoryConfigType;

extern mRosReturnType mros_mem_init(mRosSizeType config_num, mRosMemoryConfigType **config, mRosMemoryManagerType *mgrp);
extern mRosReturnType mros_mem_alloc(mRosMemoryManagerType *mgrp, mRosSizeType size, mRosMemoryListEntryType **memory);
extern mRosReturnType mros_mem_free(mRosMemoryManagerType *mgrp, mRosMemoryListEntryType *memory);

/*
 * Memory Config APIs
 */
#define MROS_MEMORY_CONFIG_DECLARE_ENTRY(entry_name, max_memory_num, memsize)	\
	static mRosMemoryListEntryType entry_name##_entries [(max_memory_num)] MROS_MATTR_BSS_NOCLR;	\
	static char entry_name##_memory [(max_memory_num) * (memsize)] MROS_MATTR_BSS_NOCLR;	\
	static mRosMemoryConfigType entry_name##_config = {	\
			(max_memory_num), \
			(memsize), \
			entry_name##_entries ,	\
			entry_name##_memory ,	\
	};

#define MROS_MEMORY_CONFIG_DECLARE_MANAGER(mem_manager_name, config_num)	\
	static mRosMemoryHeaderType mem_manager_name##_head_array [(config_num)] MROS_MATTR_BSS_NOCLR;	\
	mRosMemoryManagerType mem_manager_name = {	\
		(config_num),	\
		mem_manager_name##_head_array,	\
	};


#ifdef __cplusplus
}
#endif

#endif /* _MROS_MEMORY_H_ */
