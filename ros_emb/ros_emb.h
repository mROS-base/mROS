#ifndef _ROS_EMB_H_
#define _ROS_EMB_H_


#include "target_test.h"
#ifdef __cplusplus
#include "ros.h"
#include "xmlcall.h"
#include "xmlparser.h"
#include "tcp_ros.h"
#endif

#ifndef MROS_USR_TASK_PRI
#define MAIN_TASK_PRI 7
#define MROS_USR_TASK_PRI  5
#define MROS_TASK_PRI  3
#endif /* ROS_USR_TASK_PRI */

#ifndef TASK_PORTID
#define	TASK_PORTID		1			/* 文字入力するシリアルポートID */
#endif /* TASK_PORTID */

#ifndef MROS_USR_TASK_STACK_SIZE
#define MROS_USR_TASK_STACK_SIZE 1024 * 5
#endif  /* ROS_MAIN_TASK_STACK_SIZE */


#ifndef KMM_SIZE
#define	KMM_SIZE	(MROS_USR_TASK_STACK_SIZE * 16)	/* カーネルが割り付ける */
#endif /* KMM_SIZE */						/* メモリ領域のサイズ */

#ifndef LOOP_REF
#define LOOP_REF		ULONG_C(1000000)	/* 速度計測用のループ回数 */
#endif /* LOOP_REF */

#ifndef MROS_CYC
#define MROS_CYC	1
#endif

#ifndef MROS_DTQ			/*PUB/SUBタスクのためのデータキュー*/
#define MROS_DTQ
#define PUB_DTQ 1
#define SUB_DTQ 2
#define XML_DTQ 3
#endif

#ifndef CYC
#define CYC_HDR 1
#define CYC
#endif

#ifdef __cplusplus
extern "C" {
#endif

extern char *mem;

//extern void cyclic_handler(intptr_t exinf);
//extern void task(intptr_t exinf);
extern void main_task();
extern void sub_task();
extern void pub_task();
extern void xml_slv_task();
extern void xml_mas_task();
extern void usr_task1();
extern void usr_task2();

extern void cyclic_handler(intptr_t exinf);
#ifdef __cplusplus
}
#endif

#endif /* _HTTPSAMPLE_H_ */
