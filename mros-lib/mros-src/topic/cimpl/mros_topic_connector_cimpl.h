#ifndef _MROS_TOPIC_CONNECTOR_CIMPL_H_
#define _MROS_TOPIC_CONNECTOR_CIMPL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mros_node_cimpl.h"
#include "mros_memory.h"
#include "mros_comm_tcp_client_factory_cimpl.h"

typedef struct {
	mRosTopicIdType				topic_id;
	mRosNodeIdType				node_id;
	mRosFuncIdType				func_id;
} mRosTopicConnectorType;

/******************************************************
 * START: do not use these data types
 ******************************************************/
typedef struct {
	mRosTopicConnectorType				value;
	mRosSizeType						queue_maxsize;
	mRosMemoryListHeadType 				queue_head;
	mRosMemoryManagerType				*mempool;
	mRosCommTcpClientListReqEntryType	*commp;
} mRosTopicConnectorEntryType;
typedef ListEntryType(mRosTopicConnectorListEntryType, mRosTopicConnectorEntryType) mRosTopicConnectorListEntryType;
typedef ListHeadType(mRosTopicConnectorListEntryType) mRosTopicConnectorListHeadType;



typedef struct {
	mRosTopicIdType						topic_id;
	mRosTopicConnectorListHeadType		head[MROS_NODE_TYPE_NUM];
} mRosTopicConnectorEntryRootType;
typedef ListEntryType(mRosTopicConnectorListEntryRootType, mRosTopicConnectorEntryRootType) mRosTopicConnectorListEntryRootType;
typedef ListHeadType(mRosTopicConnectorListEntryRootType) mRosTopicConnectorListEntryRootHeadType;

typedef struct {
	mros_boolean								is_error;
	mRosSizeType								max_connector;

	mRosTopicConnectorListHeadType			 	conn_head;
	mRosTopicConnectorListEntryType				*conn_entries;

	mRosTopicConnectorListEntryRootHeadType		topic_head;
	mRosTopicConnectorListEntryRootType			*topic_entries;
} mRosTopicConnectorManagerType;

typedef struct {
	mRosSizeType								max_connector;
	mRosTopicConnectorListEntryType				*conn_entries;
	mRosTopicConnectorListEntryRootType			*topic_entries;
} mRosTopicConnectorConfigType;
/******************************************************
 * END: do not use these data types
 ******************************************************/

extern mRosReturnType mros_topic_connector_init(mRosTopicConnectorConfigType *config, mRosTopicConnectorManagerType *mgrp);

extern mRosContainerObjType mros_topic_connector_get_topic_first(mRosTopicConnectorManagerType *mgrp);
extern mRosContainerObjType mros_topic_connector_get_topic_next(mRosTopicConnectorManagerType *mgrp, mRosContainerObjType obj);
extern mRosContainerObjType mros_topic_connector_get_topic_obj(mRosTopicConnectorManagerType *mgrp, mRosTopicIdType topic_id);

extern mRosContainerObjType mros_topic_connector_get_first(mRosTopicConnectorManagerType *mgrp, mRosNodeEnumType type, mRosContainerObjType topic_obj);
extern mRosContainerObjType mros_topic_connector_get_next(mRosTopicConnectorManagerType *mgrp, mRosContainerObjType topic_obj, mRosContainerObjType obj);


extern mRosReturnType mros_topic_connector_get_topic(mRosContainerObjType topic_obj, mRosTopicIdType *topic_id);
extern mRosReturnType mros_topic_connector_get(mRosContainerObjType obj, mRosTopicConnectorType *connector);
extern mRosContainerObjType mros_topic_connector_get_obj(mRosTopicConnectorManagerType *mgrp, mRosTopicConnectorType *connector);

extern mRosReturnType mros_topic_connector_get_connection(mRosContainerObjType obj, mRosCommTcpClientListReqEntryType **connection);
extern mRosReturnType mros_topic_connector_set_connection(mRosContainerObjType obj, mRosCommTcpClientListReqEntryType *connection);

extern mRosReturnType mros_topic_connector_add(mRosTopicConnectorManagerType *mgrp, mRosTopicConnectorType *connector, mRosSizeType queue_length, mRosMemoryManagerType *mempool);
extern mRosReturnType mros_topic_connector_remove(mRosTopicConnectorManagerType *mgrp, mRosTopicConnectorType *connector);


extern mRosReturnType mros_topic_connector_put_data(mRosContainerObjType obj, const char *data, mRosSizeType len);
extern mRosReturnType mros_topic_connector_send_data(mRosTopicConnectorManagerType *mgrp, mRosContainerObjType obj, const char *data, mRosSizeType len);
extern mRosReturnType mros_topic_connector_receive_data(mRosTopicConnectorManagerType *mgrp, mRosContainerObjType obj, mRosMemoryListEntryType **memp);

extern void mros_topic_connector_purge(mRosTopicConnectorManagerType *mgrp);

extern mRosReturnType mros_topic_connector_alloc_data(mRosContainerObjType obj, char **data, mRosSizeType len);


#ifdef __cplusplus
}
#endif
#endif /* _MROS_TOPIC_CONNECTOR_CIMPL_H_ */
