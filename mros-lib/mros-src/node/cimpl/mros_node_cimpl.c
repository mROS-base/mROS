#include "mros_node_cimpl.h"
#include "mros_usr_config.h"
#include "mros_name.h"
#include <string.h>

typedef struct {
	mRosNodeIdType				node_id;
	mRosTaskIdType				task_id;
	char						node_name[MROS_NODE_NAME_MAXLEN];
	mros_uint32					namelen;
} mRosNodeEntryType;

#define MROS_TOPIC_NODE_ENTRY_INIT(entryp)	\
do {	\
	(entryp)->data.node_id = MROS_ID_NONE;	\
	(entryp)->data.namelen = 0;	\
} while (0)

typedef ListEntryType(mRosNodeListEntryType, mRosNodeEntryType) mRosNodeListEntryType;
typedef ListHeadType(mRosNodeListEntryType) mRosNodeEntryHeadType;

typedef struct {
	mRosNodeEntryHeadType	 	head;
	mRosNodeListEntryType 		*node_entries;
	mRosSizeType				max_node;
} mRosNodeManagerType;

/*
 * Topic Node Config APIs
 */
#define MROS_TOPIC_NODE_CONFIG_DECLARE_MANAGER(tnode_instance_name, max_node)	\
	static mRosNodeListEntryType tnode_instance_name##_array [(node_num)];	\
	static mRosNodeManagerType tnode_instance_name##_mgr;	\
	static mRosNodeConfigType  tnode_instance_name##_config = {	\
		(max_node),	\
		tnode_config_name##_array,	\
	};


static mRosNodeManagerType node_manager[MROS_NODE_TYPE_NUM] MROS_MATTR_BSS_NOCLR;
#define NODE_OBJ(type, id)		( \
									( type == MROS_NODE_TYPE_INNER) ? \
											&node_manager[(type)].node_entries[MROS_INDEX((id))] :	\
											&node_manager[(type)].node_entries[MROS_INDEX((id - node_manager[MROS_NODE_TYPE_INNER].max_node))] \
								)

#define NODE_TYPE(id)	( (id <= node_manager[MROS_NODE_TYPE_INNER].max_node) ? MROS_NODE_TYPE_INNER : MROS_NODE_TYPE_OUTER )

static mRosNodeListEntryType node_entries[MROS_NODE_TYPE_NUM][MROS_NODE_MAX_NUM] MROS_MATTR_BSS_NOCLR;
static char node_name_buffer[MROS_NODE_NAME_MAXLEN + 1] MROS_MATTR_BSS_NOCLR;

#define NODE_MAX_ID(type)	( \
	(type == MROS_NODE_TYPE_INNER) ? \
			node_manager[MROS_NODE_TYPE_INNER].max_node :  \
			(node_manager[MROS_NODE_TYPE_INNER].max_node + node_manager[MROS_NODE_TYPE_OUTER].max_node) \
	)

mRosReturnType mros_node_init(void)
{
	mros_uint32 i;
	mros_uint32 j;
	for (i = 0; i < MROS_NODE_TYPE_NUM; i++) {
		node_manager[i].node_entries = node_entries[i];
		for (j = 0; j < MROS_NODE_MAX_NUM; j++) {
			mRosNodeListEntryType *entry = &(node_manager[i].node_entries[j]);
			MROS_TOPIC_NODE_ENTRY_INIT(entry);
			entry->data.node_id = MROS_ID(j) + (((mRosNodeIdType)i) * MROS_NODE_MAX_NUM);
		}
		List_Init(&node_manager[i].head, mRosNodeListEntryType, MROS_NODE_MAX_NUM, node_manager[i].node_entries);
		node_manager[i].max_node = MROS_NODE_MAX_NUM;
	}
	return MROS_E_OK;
}

static mRosReturnType mros_node_get_node(const char *node_name, mros_uint32 len, mRosNodeEnumType type, mRosNodeIdType *id)
{
	mRosNodeListEntryType *p;
	*id = MROS_ID_NONE;
	ListEntry_Foreach(&node_manager[type].head, p) {
		if (len != p->data.namelen) {
			continue;
		}
		if (!strcmp(p->data.node_name, node_name)) {
			*id = p->data.node_id;
			break;
		}
	}
	if (*id == MROS_ID_NONE) {
		return MROS_E_NOENT;
	}
	return MROS_E_OK;
}

mRosReturnType mros_node_get_byname(const char *node_name, mRosNodeIdType *id)
{
	mros_uint32 len = strlen(node_name);

	mRosReturnType ret = mros_node_get_node(node_name, len, MROS_NODE_TYPE_INNER, id);
	if (ret != MROS_E_OK) {
		ret = mros_node_get_node(node_name, len, MROS_NODE_TYPE_OUTER, id);
	}

	return ret;
}

mRosReturnType mros_node_get_bytid(mRosNodeIdType *id)
{
	mRosTaskIdType task_id;
	task_id = mros_get_taskid();
	mRosNodeListEntryType *p;

	*id = MROS_ID_NONE;
	ListEntry_Foreach(&node_manager[MROS_NODE_TYPE_INNER].head, p) {
		if (task_id == p->data.task_id) {
			*id = p->data.node_id;
			break;
		}
	}
	if (*id == MROS_ID_NONE) {
		return MROS_E_NOENT;
	}
	return MROS_E_OK;
}

mRosNodeEnumType mros_node_type(mRosNodeIdType id)
{
	mRosNodeEnumType type = NODE_TYPE(id);
	if (id > NODE_MAX_ID(type)) {
		return MROS_NODE_TYPE_NUM;
	}
	return type;
}
const char* mros_node_name(mRosNodeIdType id)
{
	mRosNodeEnumType type = NODE_TYPE(id);
	if (type != MROS_NODE_TYPE_INNER) {
		return MROS_NULL;
	}
	return NODE_OBJ(type, id)->data.node_name;
}
static mRosReturnType mros_node_create(const char *node_name, mRosTaskIdType task_id, mRosNodeEnumType type, mRosNodeIdType *id)
{
	mRosNodeListEntryType *p;
	mros_uint32 len = 0;
	mRosReturnType ret;

	if (type >= MROS_NODE_TYPE_NUM) {
		return MROS_E_RANGE;
	}

	if (node_name != MROS_NULL) {
		len = strlen(node_name);
		if (len >= (MROS_NODE_NAME_MAXLEN + 1)) { /* for add slash on top */
			ROS_ERROR("%s %u ret=%d", __FUNCTION__, __LINE__, MROS_E_NOMEM);
			return MROS_E_NOMEM;
		}
		mros_name_formalize(node_name, len, node_name_buffer, &len);

		ret = mros_node_get_byname(node_name_buffer, id);
		if (ret == MROS_E_OK) {
			return MROS_E_OK;
		}
	}
	else {
		/* outer node */
	}

	ListEntry_Alloc(&node_manager[type].head, mRosNodeListEntryType, &p);
	if (p == MROS_NULL) {
		ROS_ERROR("%s %u ret=%d", __FUNCTION__, __LINE__, MROS_E_NOMEM);
		return MROS_E_NOMEM;
	}
	*id = p->data.node_id;
	p->data.task_id = task_id;
	if (node_name != MROS_NULL) {
		p->data.namelen = len;
		memcpy(p->data.node_name, node_name_buffer, len);
	}
	else {
		p->data.namelen = 0;
	}
	ListEntry_AddEntry(&node_manager[type].head, p);
	return MROS_E_OK;
}

mRosReturnType mros_node_create_inner(const char *node_name, mRosNodeIdType *id)
{
	mRosTaskIdType task_id;
	task_id = mros_get_taskid();
	return mros_node_create(node_name, task_id, MROS_NODE_TYPE_INNER, id);
}

mRosReturnType mros_node_create_outer(mRosNodeIdType *id)
{
	return mros_node_create(MROS_NULL, MROS_TASKID_NONE, MROS_NODE_TYPE_OUTER, id);
}

mRosReturnType mros_node_remove(mRosNodeIdType id)
{
	mRosNodeListEntryType *nodep;
	mRosNodeEnumType type = NODE_TYPE(id);
	if (id > NODE_MAX_ID(type)) {
		ROS_ERROR("%s %u ret=%d", __FUNCTION__, __LINE__, MROS_E_RANGE);
		return MROS_E_RANGE;
	}
	nodep = NODE_OBJ(type, id);
	ListEntry_RemoveEntry(&node_manager[type].head, nodep);
	ListEntry_Free(&node_manager[type].head, nodep);
	return MROS_E_OK;
}
