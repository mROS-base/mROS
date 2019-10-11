#ifndef _MROS_LIST_H_
#define _MROS_LIST_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mros_types.h"

/*
 * PRIVATE
 */
#define ListEntry_Init(entry)	\
do { \
	(entry)->next = (entry);	\
	(entry)->prev = (entry);	\
} while (0)

#define ListEntry_InsertHead(first, elm) \
do { \
	if ((first) != MROS_NULL) {	\
		(elm)->next = (first);	\
		(elm)->prev = (first)->prev;	\
		(first)->prev->next = (elm); \
		(first)->prev = (elm);	\
	}	\
	(first) = (elm);	\
} while (0)

#define List_IsEmpty(first)	((first)->next == (first))
#define ListEntry_First(first)	(first)

#define ListEntry_Remove(first, elm)	\
do { \
	if ((first) != MROS_NULL) { \
		if ((first) == (elm)) { \
			if (List_IsEmpty(first)) {	\
				(first) = MROS_NULL;	\
			}	\
			else { \
				(first) = (elm)->next; \
			} \
		} \
		(elm)->next->prev = (elm)->prev;	\
		(elm)->prev->next = (elm)->next;	\
		ListEntry_Init(elm); \
	} \
} while (0)


/*
 * PUBLIC
 */
#define ListEntryType(name, dataTypeName)	\
struct name {	\
	struct name *next;	\
	struct name *prev;	\
	dataTypeName data;	\
}

#define ListHeadType(entry_type)	\
struct {	\
	mros_uint32 entry_size; \
	mros_uint32 entry_num;	\
	struct entry_type *entries;	\
	struct entry_type *free;	\
	mros_uint32 free_num;	\
}

#define List_Init(headp, entry_type, prealloc_size, memp)	\
do {	\
	mros_uint32 _i;\
	(headp)->entry_size = sizeof(entry_type);	\
	(headp)->entry_num = 0;	\
	(headp)->entries = MROS_NULL;	\
	(headp)->free = MROS_NULL;	\
	(headp)->free_num = (prealloc_size);	\
	for (_i = 0; _i < (prealloc_size); _i++) { \
		entry_type *_tmp = &(((entry_type*)(memp))[_i]); \
		ListEntry_Init(_tmp);	\
		ListEntry_InsertHead((headp)->free, _tmp);	\
	} \
} while (0)

#define List_InitEmpty(headp, entry_type)	\
do { 	\
	List_Init(headp, entry_type, 0, MROS_NULL);	\
} while (0)

#define ListEntry_Alloc(headp, entry_type, new_entrypp) \
do { \
	entry_type *_tmp;	\
	if ((headp)->free_num > 0) {	\
		_tmp = ListEntry_First((headp)->free);	\
		ListEntry_Remove((headp)->free, _tmp); \
		(headp)->free_num--;	\
		*(new_entrypp) = _tmp;	\
	} \
	else { \
		*(new_entrypp) = MROS_NULL;	\
	} \
} while (0)

#define ListEntry_Free(headp, entryp) \
do { \
	ListEntry_Init(entryp);	\
	ListEntry_InsertHead((headp)->free, entryp);	\
	(headp)->free_num++;	\
} while (0)


#define ListEntry_AddEntry(headp, entryp)	\
do { \
	ListEntry_Init(entryp);	\
	ListEntry_InsertHead((headp)->entries, entryp);	\
	(headp)->entry_num++;	\
} while (0)

#define ListEntry_RemoveEntry(headp, entryp)	\
do { \
	ListEntry_Remove((headp)->entries, entryp); \
	(headp)->entry_num--;	\
} while (0)

#define ListEntry_GetFirst(headp, entrypp)	\
do { \
	*(entrypp) = (headp)->entries;	\
} while (0)


#define ListEntry_RemoveAll(headp, entry_type) \
do { \
	while ((headp)->entry_num > 0) {	\
		entry_type _tmp = ListEntry_First((headp)->entries);	\
		ListEntry_RemoveEntry(headp, _tmp);	\
	}	\
} while (0)

#define ListEntry_Foreach(headp, var)	\
	mros_uint32 _i;	\
	(var) = (headp)->entries;	\
	for (_i = 0; \
		_i < (headp)->entry_num; \
		(var) = (var)->next, _i++)

#ifdef __cplusplus
}
#endif

#endif /* _MROS_LIST_H_ */
