#include "mros_types.h"
#include "mros_memory.h"
#include "mros_usr_config.h"
#include "kernel_cfg.h"

/*******************************************************
 * START: Inner Publish topic data memory Config
 *******************************************************/
MROS_MEMORY_CONFIG_DECLARE_ENTRY(ros_inner_topic_publisher_mempool1, ROS_INNER_TOPIC_PUBLISHER_MEMPOOL1_QUELEN, ROS_INNER_TOPIC_PUBLISHER_MEMPOOL1_SIZE);
MROS_MEMORY_CONFIG_DECLARE_ENTRY(ros_inner_topic_publisher_mempool2, ROS_INNER_TOPIC_PUBLISHER_MEMPOOL2_QUELEN, ROS_INNER_TOPIC_PUBLISHER_MEMPOOL2_SIZE);
MROS_MEMORY_CONFIG_DECLARE_ENTRY(ros_inner_topic_publisher_mempool3, ROS_INNER_TOPIC_PUBLISHER_MEMPOOL3_QUELEN, ROS_INNER_TOPIC_PUBLISHER_MEMPOOL3_SIZE);
mRosMemoryConfigType *ros_inner_topic_publisher_config[ROS_INNER_TOPIC_PUBLISHER_CONFIG_NUM] = {
		&ros_inner_topic_publisher_mempool1_config,
		&ros_inner_topic_publisher_mempool2_config,
		&ros_inner_topic_publisher_mempool3_config,
};
MROS_MEMORY_CONFIG_DECLARE_MANAGER(ros_inner_topic_publisher_mempool, ROS_INNER_TOPIC_PUBLISHER_CONFIG_NUM);
/*******************************************************
 * END
 *******************************************************/


/*******************************************************
 * START: Outer Publish topic data memory Config
 *******************************************************/
MROS_MEMORY_CONFIG_DECLARE_ENTRY(ros_outer_topic_publisher_mempool1, ROS_OUTER_TOPIC_PUBLISHER_MEMPOOL1_QUELEN, ROS_OUTER_TOPIC_PUBLISHER_MEMPOOL1_SIZE);
MROS_MEMORY_CONFIG_DECLARE_ENTRY(ros_outer_topic_publisher_mempool2, ROS_OUTER_TOPIC_PUBLISHER_MEMPOOL2_QUELEN, ROS_OUTER_TOPIC_PUBLISHER_MEMPOOL2_SIZE);
MROS_MEMORY_CONFIG_DECLARE_ENTRY(ros_outer_topic_publisher_mempool3, ROS_OUTER_TOPIC_PUBLISHER_MEMPOOL3_QUELEN, ROS_OUTER_TOPIC_PUBLISHER_MEMPOOL3_SIZE);
mRosMemoryConfigType *ros_outer_topic_publisher_config[ROS_OUTER_TOPIC_PUBLISHER_CONFIG_NUM] = {
		&ros_outer_topic_publisher_mempool1_config,
		&ros_outer_topic_publisher_mempool2_config,
		&ros_outer_topic_publisher_mempool3_config,
};
MROS_MEMORY_CONFIG_DECLARE_MANAGER(ros_outer_topic_publisher_mempool, ROS_OUTER_TOPIC_PUBLISHER_CONFIG_NUM);
/*******************************************************
 * END
 *******************************************************/


/****************************************
 * USR OS TASK
 ****************************************/

mRosTaskIdType mros_usr_task_table[MROS_USR_TASK_NUM] = {
	USR_TASK1,
	USR_TASK2,
};
