#ifndef _ARDUINO_MAIN_H_
#define _ARDUINO_MAIN_H_

#include "arduino_app.h"

/*
 *  Max number of task
 */
#define RCA_NUM_TASK_MAX  5

/*
 *  Check number of task
 */
#if ADDITIONAL_LOOP_NUM > RCA_NUM_TASK_MAX
#error The number of loop is over!
#endif /* ADDITIONAL_LOOP_NUM > RCA_NUM_TASK_MAX */

/*
 * Default Macro  
 */

/*
 *  Priotiry
 */
#ifndef ARDUINO_MAIN_TASK_PRI
#define  ARDUINO_MAIN_TASK_PRI  2
#endif /*  ARDUINO_MAIN_TASK_PRI */

#ifndef LOOP_PRI
#define  LOOP_PRI  5
#endif /*  LOOP_PRI */

#ifndef RCA_TASK1_LOOP_PRI
#define  RCA_TASK1_LOOP_PRI  LOOP_PRI
#endif /*  RCA_TASK1_SETUP_PRI */

#ifndef RCA_TASK2_LOOP_PRI
#define  RCA_TASK2_LOOP_PRI  LOOP_PRI
#endif /*  RCA_TASK2_SETUP_PRI */

#ifndef RCA_TASK3_LOOP_PRI
#define  RCA_TASK3_LOOP_PRI  LOOP_PRI
#endif /*  RCA_TASK3_SETUP_PRI */

#ifndef RCA_TASK4_LOOP_PRI
#define  RCA_TASK4_LOOP_PRI  LOOP_PRI
#endif /*  RCA_TASK4_SETUP_PRI */

#ifndef RCA_TASK5_LOOP_PRI
#define  RCA_TASK5_LOOP_PRI  LOOP_PRI
#endif /*  RCA_TASK3_SETUP_PRI */

/*
 *  Stack Size
 */

#ifndef ARDUINO_MAIN_TASK_STACK_SIZE
#define ARDUINO_MAIN_TASK_STACK_SIZE 1024
#endif  /* ARDUINO_MAIN_TASK_STACK_SIZE */

#ifndef RCA_TASK1_STACK_SIZE
#define RCA_TASK1_STACK_SIZE 1024
#endif  /* RCA_TASK1_STACK_SIZE */

#ifndef RCA_TASK2_STACK_SIZE
#define RCA_TASK2_STACK_SIZE 1024
#endif  /* RCA_TASK2_STACK_SIZE */

#ifndef RCA_TASK3_STACK_SIZE
#define RCA_TASK3_STACK_SIZE 1024
#endif  /* RCA_TASK3_STACK_SIZE */

#ifndef RCA_TASK4_STACK_SIZE
#define RCA_TASK4_STACK_SIZE 1024
#endif  /* RCA_TASK4_STACK_SIZE */

#ifndef RCA_TASK5_STACK_SIZE
#define RCA_TASK5_STACK_SIZE 1024
#endif  /* RCA_TASK5_STACK_SIZE */

#ifdef __cplusplus
extern "C" {
#endif

extern void arduino_main_task(intptr_t exinf);
extern void rca_task1(intptr_t exinf);
extern void rca_task2(intptr_t exinf);
extern void rca_task3(intptr_t exinf);
extern void rca_task4(intptr_t exinf);
extern void rca_task5(intptr_t exinf);

#ifdef __cplusplus
}
#endif 

#endif /* _ARDUINO_MAIN_H_ */
