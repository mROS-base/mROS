#ifndef _ROS_EMB_H_
#define _ROS_EMB_H_


#include "target_test.h"
#ifdef __cplusplus
#include "xmlcall.h"
#include "xmlparser.h"
#include "node_server.h"
#endif

#ifndef ROS_MAIN_TASK_PRI
#define ROS_MAIN_TASK_PRI  4
#endif /* ROS_MAIN_TASK_PRI */

#ifndef TASK_PORTID
#define	TASK_PORTID		1			/* 文字入力するシリアルポートID */
#endif /* TASK_PORTID */

#ifndef ROS_MAIN_TASK_STACK_SIZE
#define ROS_MAIN_TASK_STACK_SIZE 1024 * 5
#endif  /* ROS_MAIN_TASK_STACK_SIZE */


#ifndef KMM_SIZE
#define	KMM_SIZE	(ROS_MAIN_TASK_STACK_SIZE * 16)	/* カーネルが割り付ける */
#endif /* KMM_SIZE */						/* メモリ領域のサイズ */

#ifndef LOOP_REF
#define LOOP_REF		ULONG_C(1000000)	/* 速度計測用のループ回数 */
#endif /* LOOP_REF */


#ifdef __cplusplus
extern "C" {
#endif

void network_init();
void connect_master();

//extern void	cyclic_handler(intptr_t exinf);
//extern void task(intptr_t exinf);
extern void main_task();

#ifdef __cplusplus
}
#endif

#endif /* _HTTPSAMPLE_H_ */
