#include "ros.h"
#include "message_headers.h"
#include "mros_node_cimpl.h"
#include "mros_topic_cimpl.h"
#include "mros_topic_connector_factory_cimpl.h"
#include "mros_exclusive_area.h"
#include "mros_wait_queue.h"
#include "mros_protocol_master_cimpl.h"
#include "mros_protocol_operation_cimpl.h"
#include "mros_usr_config.h"
#include "mros_topic_callback.h"
#include <string.h>


void ros::init(int argc, char *argv, std::string node_name)
{
	mRosNodeIdType id;
	mRosReturnType ret;
	mROsExclusiveUnlockObjType unlck_obj;

	mros_exclusive_lock(&mros_exclusive_area, &unlck_obj);
	ret = mros_node_create_inner(node_name.c_str(), &id);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
	}
	mros_exclusive_unlock(&unlck_obj);
	return;
}

template <class T>
ros::Subscriber ros::NodeHandle::subscribe(std::string topic, int queue_size, void (*fp) (T))
{
	Subscriber sub;
	mRosReturnType ret;
	mRosTopicConnectorType connector;
	mRosTopicConnectorManagerType *mgrp;
	mRosProtocolMasterRequestType req;
	mRosWaitListEntryType client_wait;
	mROsExclusiveUnlockObjType unlck_obj;
	mros_uint32 type_id;

	mros_client_wait_entry_init(&client_wait, &req);

	mros_exclusive_lock(&mros_exclusive_area, &unlck_obj);

	sub.set(MROS_COBJ_NULL);

	ret = mros_node_get_bytid(&connector.node_id);
	if (ret != MROS_E_OK) {
		mros_exclusive_unlock(&unlck_obj);
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return sub;
	}

	ret = mros_topic_create(topic.c_str(), message_traits::DataType<T>().value(), &connector.topic_id);
	if (ret != MROS_E_OK) {
		mros_exclusive_unlock(&unlck_obj);
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return sub;
	}
	(void)mros_topic_set_typeid(connector.topic_id, message_traits::DataTypeId<T>().value());
	(void)mros_topic_set_definition(connector.topic_id, message_traits::Definition<T>().value());
	(void)mros_topic_get_typeid(connector.topic_id, &type_id);
	(void)mros_topic_set_md5sum(connector.topic_id, getMD5Sum((int)type_id));

	mgrp = mros_topic_connector_factory_get(MROS_TOPIC_CONNECTOR_SUB);
	if (mgrp == MROS_NULL) {
		mros_exclusive_unlock(&unlck_obj);
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return sub;
	}
	connector.func_id = (mRosFuncIdType)fp;

	ret = mros_topic_connector_add(mgrp, &connector, queue_size, MROS_NULL);
	if (ret != MROS_E_OK) {
		mros_exclusive_unlock(&unlck_obj);
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return sub;
	}
	mRosContainerObjType obj = mros_topic_connector_get_obj(mgrp, &connector);
	if (obj == MROS_COBJ_NULL) {
		mros_exclusive_unlock(&unlck_obj);
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return sub;
	}
	sub.set(obj);

	//ROSマスタへ登録する
	req.req_type = MROS_PROTOCOL_MASTER_REQ_REGISTER_SUBSCRIBER;
	req.connector_obj = obj;

	mros_client_wait_for_request_done(&mros_master_wait_queue, &client_wait);
	mros_exclusive_unlock(&unlck_obj);
	return sub;
}

template <class T>
ros::Publisher ros::NodeHandle::advertise(std::string topic, int queue_size)
{
	Publisher pub;
	mRosReturnType ret;
	mRosTopicConnectorType connector;
	mRosTopicConnectorManagerType *mgrp;
	mRosProtocolMasterRequestType req;
	mRosWaitListEntryType client_wait;
	mROsExclusiveUnlockObjType unlck_obj;
	mros_uint32 type_id;

	mros_client_wait_entry_init(&client_wait, &req);

	mros_exclusive_lock(&mros_exclusive_area, &unlck_obj);
	pub.set(MROS_COBJ_NULL);

	ret = mros_node_get_bytid(&connector.node_id);
	if (ret != MROS_E_OK) {
		mros_exclusive_unlock(&unlck_obj);
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return pub;
	}

	ret = mros_topic_create(topic.c_str(), message_traits::DataType<T*>().value(), &connector.topic_id);
	if (ret != MROS_E_OK) {
		mros_exclusive_unlock(&unlck_obj);
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return pub;
	}
	(void)mros_topic_set_typeid(connector.topic_id, message_traits::DataTypeId<T*>().value());
	(void)mros_topic_set_definition(connector.topic_id, message_traits::Definition<T*>().value());
	(void)mros_topic_get_typeid(connector.topic_id, &type_id);
	(void)mros_topic_set_md5sum(connector.topic_id, getMD5Sum((int)type_id));

	mgrp = mros_topic_connector_factory_get(MROS_TOPIC_CONNECTOR_PUB);
	if (mgrp == MROS_NULL) {
		mros_exclusive_unlock(&unlck_obj);
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return pub;
	}
	connector.func_id = (mRosFuncIdType)MROS_ID_NONE;

	ret = mros_topic_connector_add(mgrp, &connector, queue_size, &ros_inner_topic_publisher_mempool);
	if (ret != MROS_E_OK) {
		mros_exclusive_unlock(&unlck_obj);
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return pub;
	}
	mRosContainerObjType obj = mros_topic_connector_get_obj(mgrp, &connector);
	if (obj == MROS_COBJ_NULL) {
		mros_exclusive_unlock(&unlck_obj);
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return pub;
	}

	pub.set(obj);

	//ROSマスタへ登録する
	req.req_type = MROS_PROTOCOL_MASTER_REQ_REGISTER_PUBLISHER;
	req.connector_obj = obj;

	mros_client_wait_for_request_done(&mros_master_wait_queue, &client_wait);
	mros_exclusive_unlock(&unlck_obj);
	return pub;
}


template <class T>
void ros::Publisher::publish(T& data)
{
	mRosReturnType ret;
	char *snd_data;
	char *bodyp;
	mRosSizeType size;
	mROsExclusiveUnlockObjType unlck_obj;

	size = mros_protocol_get_buffersize(data.dataSize());

	mros_exclusive_lock(&mros_exclusive_area, &unlck_obj);
	ret = mros_topic_connector_alloc_data((mRosContainerObjType)this->get(), &snd_data, size);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
	}
	bodyp = mros_protocol_get_body(snd_data);
	data.memCopy(bodyp);
	mros_exclusive_unlock(&unlck_obj);
	return;
}


void ros::Rate::sleep(void)
{
	mros_sleep_task_msec(ROS_RATE_RATE_SEC_UNIT/this->rate);
	return;
}

void ros::spin(void){
	slp_tsk();
	return;
}


void mros_topic_callback(mros_uint32 topic_id, mros_uint32 type_id, mRosFuncIdType func_id, const char *data, mros_uint32 len)
{
	void (*fp)(void *ptr);
	fp = (void (*)(void *))func_id;

	callCallback((int)type_id, fp, (char*)data);
	return;
}

#include "message_class_specialization.h"
