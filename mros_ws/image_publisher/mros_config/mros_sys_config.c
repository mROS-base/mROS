#include "mros_types.h"
#include "mros_memory.h"
#include "mros_sys_config.h"
#include "kernel.h"

void mros_sys_config_init(void)
{
	mRosReturnType ret;

	ret = mros_mem_init(ROS_INNER_TOPIC_PUBLISHER_CONFIG_NUM, ros_inner_topic_publisher_config, &ros_inner_topic_publisher_mempool);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return;
	}
	ret = mros_mem_init(ROS_OUTER_TOPIC_PUBLISHER_CONFIG_NUM, ros_outer_topic_publisher_config, &ros_outer_topic_publisher_mempool);
	if (ret != MROS_E_OK) {
		ROS_ERROR("%s %s() %u ret=%d", __FILE__, __FUNCTION__, __LINE__, ret);
		return;
	}
	return;
}


void usr_task_activation(void)
{
	mros_uint32 i;
	for (i = 0; i < MROS_USR_TASK_NUM; i++) {
		act_tsk(mros_usr_task_table[i]);
	}
}

