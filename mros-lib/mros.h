#ifndef _mros_H_
#define _mros_H_

#include "target_test.h"


#ifdef __cplusplus
#include "ros.h"
#include "xmlcall.h"
#include "xmlparser.h"
#include "tcp_ros.h"
#endif

#ifndef MROS_USR_TASK_PRI
#define MAIN_TASK_PRI 6
#define MROS_USR_TASK_PRI  7
#define MROS_TASK_PRI  5
#endif /* ROS_USR_TASK_PRI */

#ifndef TASK_PORTID
#define	TASK_PORTID		1			/* serial port ID for something typing */
#endif /* TASK_PORTID */

#ifndef MROS_TASK_STACK_SIZE
#define MROS_SUB_STACK_SIZE 1024 * 16		//for subscribe/user task
#define MROS_PUB_STACK_SIZE 1024 * 1024	//for publish/user task
#define MROS_TASK_STACK_SIZE 1024 * 5	//for mros task
#endif	/*MROS_TASK_STACK_SIZE*/

#ifndef KMM_SIZE
#define	KMM_SIZE	(MROS_SUB_STACK_SIZE * 16)	/* カーネルが割り付ける */
#endif /* KMM_SIZE */						/* メモリ領域のサイズ */

#ifndef LOOP_REF
#define LOOP_REF		ULONG_C(1000000)	/* 速度計測用のループ回数 */
#endif /* LOOP_REF */

#ifndef MROS_DTQ			/* data queue ID */
#define MROS_DTQ
#define PUB_DTQ 1
#define SUB_DTQ 2
#define XML_DTQ 3
#endif	/*MROS_DTQ*/

#ifndef CYC
#define CYC_HDR 1
#define CYC
#endif	/*CYC*/

#ifndef MEM_ADD				/* base address of shared memory in mROS */
#define MEM_ADD
#define PUB_ADDR 0
#define SUB_ADDR 1024*1024
#define XML_ADDR 1024*1024 + 1024*2
#endif	/*MEM_ADD*/

#ifdef __cplusplus
extern "C" {
#endif

extern char mem[1024*1024*2];
extern int ros_sem;
extern int count;

extern void main_task();
extern void sub_task();
extern void pub_task();
extern void xml_slv_task();
extern void xml_mas_task();
extern void usr_task1();
extern void usr_task2();


extern void sus_all();
extern void rsm_all();

extern void cyclic_handler(intptr_t exinf);
#ifdef __cplusplus
}
#endif

#endif /* _mros_H_ */
