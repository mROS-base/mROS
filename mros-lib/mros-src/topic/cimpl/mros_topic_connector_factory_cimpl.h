#ifndef _MROS_TOPIC_CONNECTOR_FACTORY_CIMPL_H_
#define _MROS_TOPIC_CONNECTOR_FACTORY_CIMPL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mros_topic_connector_cimpl.h"

typedef enum {
	MROS_TOPIC_CONNECTOR_PUB = 0,
	MROS_TOPIC_CONNECTOR_SUB,
	MROS_TOPIC_CONNECTOR_NUM,
} mRosTopicConnectorEnumType;

extern mRosTopicConnectorManagerType *mros_topic_connector_factory_create(mRosTopicConnectorEnumType type);
extern mRosTopicConnectorManagerType *mros_topic_connector_factory_get(mRosTopicConnectorEnumType type);

#ifdef __cplusplus
}
#endif
#endif /* _MROS_TOPIC_CONNECTOR_FACTORY_CIMPL_H_ */
