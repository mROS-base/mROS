#ifndef _BLINLY_H_
#define _BLINLY_H_

#ifndef BLINKY_MAIN_TASK_PRI
#define  BLINKY_MAIN_TASK_PRI  2
#endif /*  BLINKY_MAIN_TASK_PRI */

#ifndef BLINKY_MAIN_TASK_STACK_SIZE
#define BLINKY_MAIN_TASK_STACK_SIZE 1024
#endif  /* BLINKY_MAIN_TASK_STACK_SIZE */

#ifdef __cplusplus
extern "C" {
#endif

extern void blinky_main_task(intptr_t exinf);

#ifdef __cplusplus
}
#endif

#endif /* _BLINLY_H_ */
