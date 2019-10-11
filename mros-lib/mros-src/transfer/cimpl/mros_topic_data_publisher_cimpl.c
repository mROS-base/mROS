#include "mros_topic_data_publisher_cimpl.h"
#include "mros_topic_connector_factory_cimpl.h"
#include "mros_topic_cimpl.h"
#include "mros_array_container.h"
#include "mros_usr_config.h"

MROS_ARRAY_CONTAINER_CONFIG_DECLARE_MANAGER(mros_topic_pub_mgr, MROS_TOPIC_MAX_NUM);

static void mros_topic_publish(mRosTopicConnectorManagerType *mgrp, mRosNodeEnumType type, mRosContainerObjType topic_obj)
{
	mRosReturnType ret;
	mRosContainerObjType obj;
	mRosMemoryListEntryType *topic_data;
	mRosTopicIdType topic_id;

	ret = mros_topic_connector_get_topic(topic_obj, &topic_id);
	if (ret != MROS_E_OK) {
		return;
	}

	obj = mros_topic_connector_get_first(mgrp, type, topic_obj);
	if (obj == MROS_COBJ_NULL) {
		return;
	}
	while (obj != MROS_COBJ_NULL) {
		ret = mros_topic_connector_receive_data(mgrp, obj, &topic_data);
		if (ret != MROS_E_OK) {
			obj = mros_topic_connector_get_next(mgrp, topic_obj, obj);
			continue;
		}
		ret = mros_topic_add_data(topic_id, topic_data);
		if (ret != MROS_E_OK) {
			ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
			(void)mros_mem_free(topic_data->data.mgrp, topic_data);
		}
		obj = mros_topic_connector_get_next(mgrp, topic_obj, obj);
	}

	return;
}

mRosReturnType mros_topic_data_publisher_init(void)
{
	return MROS_E_OK;
}

void mros_topic_data_publisher_run(void)
{
	mRosContainerObjType topic_obj;
	mRosTopicConnectorManagerType *mgrp;

	mgrp = mros_topic_connector_factory_get(MROS_TOPIC_CONNECTOR_PUB);
	if (mgrp == MROS_NULL) {
		return;
	}
	mros_topic_pub_mgr.count = 0;

	/**************************
	 * INNER NODE
	 **************************/
	topic_obj = mros_topic_connector_get_topic_first(mgrp);
	while (topic_obj != MROS_COBJ_NULL) {
		mros_array_container_add(&mros_topic_pub_mgr, topic_obj);

		mros_topic_publish(mgrp, MROS_NODE_TYPE_INNER, topic_obj);
		topic_obj = mros_topic_connector_get_topic_next(mgrp, topic_obj);
	}

	/**************************
	 * OUTER NODE
	 **************************/
	mros_uint32 i;
	for (i = 0; i < mros_topic_pub_mgr.count; i++) {
		mros_topic_publish(mgrp, MROS_NODE_TYPE_OUTER, mros_topic_pub_mgr.array[i]);
	}

	mros_topic_connector_purge(mgrp);
	return;
}
