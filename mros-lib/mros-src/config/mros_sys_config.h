#ifndef _MROS_SYS_CONFIG_H_
#define _MROS_SYS_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mros_usr_config.h"
#include "mros_os_config.h"
#include "mros_packet_config.h"



/**************************************
 * PROTOCOL
 **************************************/
/*
 * portno of master
 */
#define MROS_MASTER_PORT_NO						11311
/*
 * dhcp option of self node(0:not use, 1:use)
 */
#define MROS_NODE_USE_DHCP 						0
/*
 * ipaddr of master
 */
#define MROS_MASTER_IPADDR						"0.0.0.0"

/*
 * ipaddr of self node
 */
#define MROS_NODE_IPADDR						"127.0.0.1"

/*
 * subnet mask of self node
 */
#define MROS_NODE_SUBNET_MASK					"255.255.255.0"

/*
 * portno of slave
 */
#define MROS_SLAVE_PORT_NO						11411

/*
 * portno of pub
 */
#define MROS_PUBLISHER_PORT_NO					11511

/*
 * do not change this parameter
 */
#define MROS_TOPIC_TCP_CLIENT_MAX_NUM			( MROS_PUB_TOPIC_CONNECTOR_MAX_NUM + MROS_SUB_TOPIC_CONNECTOR_MAX_NUM )


/*****************************************
 * EXCLUSIVE AREA
 *****************************************/

/*
 * do not change this parameter
 */
#define MROS_GIANT_EXCLUSIVE_AREA_PRIORITY		( \
		( MROS_USR_TASK_PRI < MROS_TASK_PRI) ? \
				MROS_USR_TASK_PRI : \
				MROS_TASK_PRI \
	)


extern void mros_sys_config_init(void);
extern void usr_task_activation(void);

#ifdef __cplusplus
}
#endif

#endif /* _MROS_SYS_CONFIG_H_ */

