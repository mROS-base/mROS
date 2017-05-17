#ifndef _ARDUINO_APP_H_
#define _ARDUINO_APP_H_

#define ADDITIONAL_LOOP_NUM 2 /* number of additional loops */

#ifndef KMM_SIZE
#define	KMM_SIZE	(INIT_MAIN_TASK_STACK_SIZE * 16) /* カーネルが割り付ける */
#endif /* KMM_SIZE */						         /* メモリ領域のサイズ */

#ifdef __cplusplus
extern "C" {
#endif

extern void	cyclic_handler(intptr_t exinf);

#ifdef __cplusplus
}
#endif

#endif /* _ARDUINO_APP_H_ */
