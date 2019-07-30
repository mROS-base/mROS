#include "mros_topic_connector_cimpl.h"
#include "mros_topic_callback.h"
#include "mros_node_cimpl.h"
#include "mros_topic_cimpl.h"
#include "mros_usr_config.h"
#include <string.h>

#define MROS_TOPIC_CONNECTOR_ENTRY_INIT(entryp)	\
do {	\
	(entryp)->data.queue_maxsize = 1U;		\
	(entryp)->data.value.topic_id = MROS_ID_NONE;		\
	(entryp)->data.value.node_id = MROS_ID_NONE;		\
	(entryp)->data.value.func_id = MROS_ID_NONE;	\
} while (0)

mRosReturnType mros_topic_connector_init(mRosTopicConnectorConfigType *config, mRosTopicConnectorManagerType *mgrp)
{
	mros_uint32 i;
	mgrp->is_error = MROS_FALSE;
	mgrp->conn_entries = config->conn_entries;
	mgrp->topic_entries = config->topic_entries;
	mgrp->max_connector = config->max_connector;
	for (i = 0; i < mgrp->max_connector; i++) {
		mRosTopicConnectorListEntryType *entry = &(mgrp->conn_entries[i]);
		MROS_TOPIC_CONNECTOR_ENTRY_INIT(entry);
		List_InitEmpty(&entry->data.queue_head, mRosMemoryListEntryType);
	}

	for (i = 0; i < MROS_TOPIC_MAX_NUM; i++) {
		mRosTopicConnectorListEntryRootType *topic_entry = &(mgrp->topic_entries[i]);
		topic_entry->data.topic_id = MROS_ID_NONE;
		List_Init(&topic_entry->data.head[MROS_NODE_TYPE_INNER], mRosTopicConnectorListEntryType, 0, MROS_NULL);
		List_Init(&topic_entry->data.head[MROS_NODE_TYPE_OUTER], mRosTopicConnectorListEntryType, 0, MROS_NULL);
	}
	List_Init(&mgrp->topic_head, mRosTopicConnectorListEntryRootType, MROS_TOPIC_MAX_NUM, mgrp->topic_entries);
	List_Init(&mgrp->conn_head, mRosTopicConnectorListEntryType, mgrp->max_connector, mgrp->conn_entries);
	return MROS_E_OK;
}

mRosContainerObjType mros_topic_connector_get_topic_first(mRosTopicConnectorManagerType *mgrp)
{
	mRosTopicConnectorListEntryRootType *p;

	if (mgrp->topic_head.entry_num == 0) {
		return MROS_COBJ_NULL;
	}
	ListEntry_GetFirst(&mgrp->topic_head, &p);
	return (mRosContainerObjType)p;
}
mRosContainerObjType mros_topic_connector_get_topic_next(mRosTopicConnectorManagerType *mgrp, mRosContainerObjType obj)
{
	mRosTopicConnectorListEntryRootType *first;
	mRosTopicConnectorListEntryRootType *entry = (mRosTopicConnectorListEntryRootType*)obj;
	ListEntry_GetFirst(&mgrp->topic_head, &first);
	if (first == MROS_NULL) {
		return MROS_COBJ_NULL;
	}
	if (entry->next == first) {
		return MROS_COBJ_NULL;
	}
	return (mRosContainerObjType)entry->next;
}

mRosContainerObjType mros_topic_connector_get_first(mRosTopicConnectorManagerType *mgrp, mRosNodeEnumType type, mRosContainerObjType topic_obj)
{
	mRosTopicConnectorListEntryRootType *topic_entry = (mRosTopicConnectorListEntryRootType*)topic_obj;
	mRosTopicConnectorListEntryType *p;

	if (topic_entry->data.head[type].entry_num == 0) {
		return MROS_COBJ_NULL;
	}
	ListEntry_GetFirst(&topic_entry->data.head[type], &p);

	return (mRosContainerObjType)p;
}


mRosContainerObjType mros_topic_connector_get_next(mRosTopicConnectorManagerType *mgrp, mRosContainerObjType topic_obj, mRosContainerObjType obj)
{
	mRosTopicConnectorListEntryRootType *topic_entry = (mRosTopicConnectorListEntryRootType*)topic_obj;
	mRosTopicConnectorListEntryType *entry = (mRosTopicConnectorListEntryType*)obj;
	mRosTopicConnectorListEntryType *first;

	mRosNodeEnumType type = mros_node_type(entry->data.value.node_id);
	ListEntry_GetFirst(&topic_entry->data.head[type], &first);
	if (first == MROS_NULL) {
		return MROS_COBJ_NULL;
	}
	if (entry->next == first) {
		return MROS_COBJ_NULL;
	}
	return (mRosContainerObjType)entry->next;
}

static mRosTopicConnectorListEntryRootType *mros_topic_connector_get_topic_head(mRosTopicConnectorManagerType *mgrp, mRosTopicIdType topic_id)
{
	mRosTopicConnectorListEntryRootType *topic_p;

	ListEntry_Foreach(&mgrp->topic_head, topic_p) {
		if (topic_p->data.topic_id == topic_id) {
			return topic_p;
		}
	}
	return MROS_NULL;
}
mRosContainerObjType mros_topic_connector_get_topic_obj(mRosTopicConnectorManagerType *mgrp, mRosTopicIdType topic_id)
{
	mRosTopicConnectorListEntryRootType *root = mros_topic_connector_get_topic_head(mgrp, topic_id);
	if (root == MROS_NULL) {
		return MROS_COBJ_NULL;
	}
	return (mRosContainerObjType)root;
}

static mRosTopicConnectorListEntryRootType *mros_topic_connector_create_topic_head(mRosTopicConnectorManagerType *mgrp, mRosTopicIdType topic_id)
{
	mRosTopicConnectorListEntryRootType *topic_p;

	ListEntry_Alloc(&mgrp->topic_head, mRosTopicConnectorListEntryRootType, &topic_p);
	if (topic_p == MROS_NULL) {
		return MROS_NULL;
	}
	topic_p->data.topic_id = topic_id;
	List_InitEmpty(&topic_p->data.head[MROS_NODE_TYPE_INNER], mRosTopicConnectorListEntryType);
	List_InitEmpty(&topic_p->data.head[MROS_NODE_TYPE_OUTER], mRosTopicConnectorListEntryType);
	ListEntry_AddEntry(&mgrp->topic_head, topic_p);
	return topic_p;
}

static mRosTopicConnectorListEntryType *mros_topic_connector_get_node(mRosTopicConnectorListEntryRootType *topic_p, mRosNodeIdType node_id)
{
	mRosTopicConnectorListEntryType *entry;
	mRosNodeEnumType type = mros_node_type(node_id);

	ListEntry_Foreach(&topic_p->data.head[type], entry) {
		if ((entry->data.value.node_id == node_id)) {
			return entry;
		}
	}
	return MROS_NULL;
}

mRosReturnType mros_topic_connector_add(mRosTopicConnectorManagerType *mgrp, mRosTopicConnectorType *connector, mRosSizeType queue_length, mRosMemoryManagerType *mempool)
{
	mRosTopicConnectorListEntryRootType *topic_p;
	mRosTopicConnectorListEntryRootType *org_topic_p;
	mRosTopicConnectorListEntryType *entry;

	org_topic_p = mros_topic_connector_get_topic_head(mgrp, connector->topic_id);
	if (org_topic_p == MROS_NULL) {
		topic_p = mros_topic_connector_create_topic_head(mgrp, connector->topic_id);
	}
	else {
		topic_p = org_topic_p;
	}
	if (topic_p == MROS_NULL) {
		(void)mros_topic_connector_remove(mgrp, connector);
		return MROS_E_NOMEM;
	}

	entry = mros_topic_connector_get_node(topic_p, connector->node_id);
	mRosNodeEnumType type = mros_node_type(connector->node_id);
	if (entry == MROS_NULL) {
		ListEntry_Alloc(&mgrp->conn_head, mRosTopicConnectorListEntryType, &entry);
		if (entry != MROS_NULL) {
			entry->data.value.topic_id = connector->topic_id;
			entry->data.value.node_id = connector->node_id;
			entry->data.value.func_id = connector->func_id;
			entry->data.queue_maxsize = queue_length;
			entry->data.mempool = mempool;
			entry->data.commp = MROS_NULL;
			ListEntry_AddEntry(&topic_p->data.head[type], entry);
			List_InitEmpty(&entry->data.queue_head, mRosMemoryListEntryType);
		}
		else {
			(void)mros_topic_connector_remove(mgrp, connector);
			return MROS_E_NOMEM;
		}
	}
	return MROS_E_OK;
}

mRosReturnType mros_topic_connector_remove(mRosTopicConnectorManagerType *mgrp, mRosTopicConnectorType *connector)
{
	mRosTopicConnectorListEntryRootType *topic_entryp;
	mRosTopicConnectorListEntryType *entryp;

	topic_entryp = mros_topic_connector_get_topic_head(mgrp, connector->topic_id);
	if (topic_entryp == MROS_NULL) {
		return MROS_E_NOENT;
	}
	entryp = mros_topic_connector_get_node(topic_entryp, connector->node_id);
	if (entryp == MROS_NULL) {
		return MROS_E_NOENT;
	}
	mRosNodeEnumType type = mros_node_type(connector->node_id);
	{
		ListEntry_RemoveEntry(&topic_entryp->data.head[type], entryp);
		if (topic_entryp->data.head[type].entry_num == 0) {
			ListEntry_RemoveEntry(&mgrp->topic_head, topic_entryp);
		}
	}
	if (entryp->data.commp != MROS_NULL) {
		entryp->data.commp->data.op.free(entryp->data.commp);
		entryp->data.commp = MROS_NULL;
	}
	ListEntry_Free(&mgrp->conn_head, entryp);
	return MROS_E_OK;
}

mRosReturnType mros_topic_connector_get(mRosContainerObjType obj, mRosTopicConnectorType *connector)
{
	mRosTopicConnectorListEntryType *entry = (mRosTopicConnectorListEntryType*)obj;
	*connector = entry->data.value;
	return MROS_E_OK;
}

mRosReturnType mros_topic_connector_get_connection(mRosContainerObjType obj, mRosCommTcpClientListReqEntryType **connection)
{
	mRosTopicConnectorListEntryType *entry = (mRosTopicConnectorListEntryType*)obj;
	*connection = entry->data.commp;
	return MROS_E_OK;
}

mRosReturnType mros_topic_connector_set_connection(mRosContainerObjType obj, mRosCommTcpClientListReqEntryType *connection)
{
	mRosTopicConnectorListEntryType *entry = (mRosTopicConnectorListEntryType*)obj;
	entry->data.commp = connection;
	return MROS_E_OK;
}


mRosContainerObjType mros_topic_connector_get_obj(mRosTopicConnectorManagerType *mgrp, mRosTopicConnectorType *connector)
{
	mRosTopicConnectorListEntryRootType *tp;
	mRosTopicConnectorListEntryRootType *topic_p = MROS_NULL;
	mRosTopicConnectorListEntryType *p;
	mRosTopicConnectorListEntryType *entry = MROS_NULL;

	{
		ListEntry_Foreach(&mgrp->topic_head, tp) {
			if (tp->data.topic_id == connector->topic_id) {
				topic_p = tp;
				break;
			}
		}
		if (tp == MROS_NULL) {
			return MROS_COBJ_NULL;
		}

	}
	mRosNodeEnumType type = mros_node_type(connector->node_id);
	{
		ListEntry_Foreach(&topic_p->data.head[type], p) {
			if ((p->data.value.node_id == connector->node_id)) {
				entry = p;
				break;
			}
		}
	}
	if (entry == MROS_NULL) {
		return MROS_COBJ_NULL;
	}
	return (mRosContainerObjType)entry;
}

mRosReturnType mros_topic_connector_get_topic(mRosContainerObjType topic_obj, mRosTopicIdType *topic_id)
{
	mRosTopicConnectorListEntryRootType *topic_p = (mRosTopicConnectorListEntryRootType*)topic_obj;
	*topic_id = topic_p->data.topic_id;
	return MROS_E_OK;
}


mRosReturnType mros_topic_connector_put_data(mRosContainerObjType obj, const char* data, mRosSizeType len)
{
	mRosReturnType ret;
	mRosMemoryListEntryType *mem_entryp;
	mRosTopicConnectorListEntryType *entry = (mRosTopicConnectorListEntryType*)obj;
	mRosNodeEnumType type = mros_node_type(entry->data.value.node_id);
	if (type != MROS_NODE_TYPE_INNER) {
		return MROS_E_INVAL;
	}
	if (entry->data.queue_head.entry_num >= entry->data.queue_maxsize) {
		ROS_WARN("%s %s() %u :WARNING: Removed topic data for queufull(%u).", __FILE__, __FUNCTION__, __LINE__, entry->data.queue_maxsize);
		ListEntry_GetFirst(&entry->data.queue_head, &mem_entryp);
		ListEntry_RemoveEntry(&entry->data.queue_head, mem_entryp);
		(void)mros_mem_free(entry->data.mempool, mem_entryp);
	}
	ret = mros_mem_alloc(entry->data.mempool, len, &mem_entryp);
	if (ret != MROS_E_OK) {
		return ret;
	}
	mem_entryp->data.size = len;
	memcpy(mem_entryp->data.memp, data, mem_entryp->data.size);
	ListEntry_AddEntry(&entry->data.queue_head, mem_entryp);
	return MROS_E_OK;
}
mRosReturnType mros_topic_connector_alloc_data(mRosContainerObjType obj, char **data, mRosSizeType len)
{
	mRosReturnType ret;
	mRosMemoryListEntryType *mem_entryp;
	mRosTopicConnectorListEntryType *entry = (mRosTopicConnectorListEntryType*)obj;
	mRosNodeEnumType type = mros_node_type(entry->data.value.node_id);
	if (type != MROS_NODE_TYPE_INNER) {
		return MROS_E_INVAL;
	}
	if (entry->data.queue_head.entry_num >= entry->data.queue_maxsize) {
		ROS_WARN("%s %s() %u :WARNING: Removed topic data for queufull(%u).", __FILE__, __FUNCTION__, __LINE__, entry->data.queue_maxsize);
		ListEntry_GetFirst(&entry->data.queue_head, &mem_entryp);
		ListEntry_RemoveEntry(&entry->data.queue_head, mem_entryp);
		(void)mros_mem_free(entry->data.mempool, mem_entryp);
	}
	ret = mros_mem_alloc(entry->data.mempool, len, &mem_entryp);
	if (ret != MROS_E_OK) {
		return ret;
	}
	mem_entryp->data.size = len;
	*data = mem_entryp->data.memp;
	ListEntry_AddEntry(&entry->data.queue_head, mem_entryp);
	return MROS_E_OK;
}


mRosReturnType mros_topic_connector_send_data(mRosTopicConnectorManagerType *mgrp, mRosContainerObjType obj, const char* data, mRosSizeType len)
{
	mRosReturnType ret;
	mRosTopicConnectorListEntryType *entry = (mRosTopicConnectorListEntryType*)obj;

	mRosNodeEnumType type = mros_node_type(entry->data.value.node_id);
	if (type == MROS_NODE_TYPE_INNER) {
		//TOPIC ==> inner node callback
		mros_uint32 type_id;
		(void)mros_topic_get_typeid(entry->data.value.topic_id, &type_id);
		mros_topic_callback(type_id, entry->data.value.func_id, data, len);
		return MROS_E_OK;
	}
	//TOPIC ==> outer node
	if (entry->data.commp == MROS_NULL) {
		return MROS_E_NOTCONN;
	}
	ret = entry->data.commp->data.op.topic_data_send(&entry->data.commp->data.client, data, len);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		mgrp->is_error = MROS_TRUE;
		mros_comm_tcp_client_close(&entry->data.commp->data.client);
	}
	return ret;
}

mRosReturnType mros_topic_connector_receive_data(mRosTopicConnectorManagerType *mgrp, mRosContainerObjType obj, mRosMemoryListEntryType **memp)
{
	mRosReturnType ret;
	mRosMemoryListEntryType *data;
	mRosTopicConnectorListEntryType *entry = (mRosTopicConnectorListEntryType*)obj;

	*memp = MROS_NULL;
	mRosNodeEnumType type = mros_node_type(entry->data.value.node_id);
	if (type == MROS_NODE_TYPE_INNER) {
		if (entry->data.queue_head.entry_num == 0) {
			return MROS_E_NOENT;
		}
		ListEntry_GetFirst(&entry->data.queue_head, &data);
		ListEntry_RemoveEntry(&entry->data.queue_head, data);
		*memp = data;
		return MROS_E_OK;
	}
	//outer node
	if (entry->data.commp == MROS_NULL) {
		return MROS_NULL;
	}
	ret = entry->data.commp->data.op.topic_data_receive(&entry->data.commp->data.client, entry->data.mempool, memp);
	if ((ret != MROS_E_OK) && (ret != MROS_E_NOENT)) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		mgrp->is_error = MROS_TRUE;
		mros_comm_tcp_client_close(&entry->data.commp->data.client);
	}

	return ret;
}

static mros_boolean mros_topic_connector_purge_one(mRosTopicConnectorManagerType *mgrp, mRosTopicConnectorListEntryRootType *topic_root)
{
	mRosTopicConnectorListEntryType *entryp;

	ListEntry_Foreach(&topic_root->data.head[MROS_NODE_TYPE_OUTER], entryp) {
		if (entryp->data.commp != MROS_NULL) {
			if (mros_comm_tcp_client_is_connected(&entryp->data.commp->data.client) == MROS_FALSE) {
				entryp->data.commp->data.op.free(entryp->data.commp);
				entryp->data.commp = MROS_NULL;
				mros_node_remove(entryp->data.value.node_id);
				ListEntry_RemoveEntry(&topic_root->data.head[MROS_NODE_TYPE_OUTER], entryp);
				ListEntry_Free(&mgrp->conn_head, entryp);
				ROS_ERROR("removed connector topic_id=%d node_id=%d", entryp->data.value.topic_id, entryp->data.value.node_id);
				return MROS_TRUE;
			}
		}
	}
	return MROS_FALSE;
}

void mros_topic_connector_purge(mRosTopicConnectorManagerType *mgrp)
{
	mros_boolean is_removed = MROS_FALSE;
	mRosTopicConnectorListEntryRootType *topic_first = MROS_NULL;
	mRosTopicConnectorListEntryRootType *topic_root = MROS_NULL;

	if (mgrp->is_error == MROS_FALSE) {
		return;
	}
	if (mgrp->topic_head.entry_num == 0) {
		return;
	}

	ListEntry_GetFirst(&mgrp->topic_head, &topic_first);
	topic_root = topic_first;
	do {
		do {
			is_removed = mros_topic_connector_purge_one(mgrp, topic_root);
		} while (is_removed == MROS_TRUE);
		topic_root = topic_root->next;
	} while (topic_root != topic_first);

	mgrp->is_error = MROS_FALSE;
	return;
}
