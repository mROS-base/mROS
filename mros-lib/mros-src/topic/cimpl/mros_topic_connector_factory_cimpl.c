#include "mros_topic_connector_factory_cimpl.h"
#include "mros_sys_config.h"

static mros_boolean mros_topic_connector_is_inialized[MROS_TOPIC_CONNECTOR_NUM] = {
	MROS_FALSE,
	MROS_FALSE,
};
/*
 * topic connector Config APIs
 */
#define MROS_TOPIC_CONNECTOR_CONFIG_DECLARE_MANAGER(manager_name, conn_entry_num)	\
	static mRosTopicConnectorListEntryType manager_name##_conn_array [(conn_entry_num)] MROS_MATTR_BSS_NOCLR;	\
	static mRosTopicConnectorListEntryRootType manager_name##_topic_array [MROS_TOPIC_MAX_NUM] MROS_MATTR_BSS_NOCLR;	\
	static mRosTopicConnectorManagerType manager_name MROS_MATTR_BSS_NOCLR;	\
	static mRosTopicConnectorConfigType manager_name##_config = {	\
		(conn_entry_num),	\
		manager_name##_conn_array,	\
		manager_name##_topic_array,	\
	};

MROS_TOPIC_CONNECTOR_CONFIG_DECLARE_MANAGER(pub_connector_mgr, (MROS_PUB_TOPIC_CONNECTOR_MAX_NUM) );
MROS_TOPIC_CONNECTOR_CONFIG_DECLARE_MANAGER(sub_connector_mgr, (MROS_SUB_TOPIC_CONNECTOR_MAX_NUM) );

mRosTopicConnectorManagerType *mros_topic_connector_factory_create(mRosTopicConnectorEnumType type)
{
	mRosTopicConnectorManagerType *mgrp = MROS_NULL;
	mRosTopicConnectorConfigType *cfgp = MROS_NULL;
	mRosReturnType ret;

	switch (type) {
	case MROS_TOPIC_CONNECTOR_PUB:
		mgrp = &pub_connector_mgr;
		cfgp = &pub_connector_mgr_config;
		break;
	case MROS_TOPIC_CONNECTOR_SUB:
		mgrp = &sub_connector_mgr;
		cfgp = &sub_connector_mgr_config;
		break;
	default:
		break;
	}
	if (mgrp == MROS_NULL) {
		return MROS_NULL;
	}
	if (mros_topic_connector_is_inialized[type] == MROS_TRUE) {
		return mgrp;
	}
	ret = mros_topic_connector_init(cfgp, mgrp);
	if (ret != MROS_E_OK) {
		return MROS_NULL;
	}
	mros_topic_connector_is_inialized[type] = MROS_TRUE;
	return mgrp;
}

mRosTopicConnectorManagerType *mros_topic_connector_factory_get(mRosTopicConnectorEnumType type)
{
	mRosTopicConnectorManagerType *mgrp = MROS_NULL;

	switch (type) {
	case MROS_TOPIC_CONNECTOR_PUB:
		mgrp = &pub_connector_mgr;
		break;
	case MROS_TOPIC_CONNECTOR_SUB:
		mgrp = &sub_connector_mgr;
		break;
	default:
		break;
	}
	if (mgrp == MROS_NULL) {
		return MROS_NULL;
	}
	if (mros_topic_connector_is_inialized[type] == MROS_FALSE) {
		return MROS_NULL;
	}
	return mgrp;
}
