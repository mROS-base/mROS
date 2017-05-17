/*
 *		cfg1_out.cをリンクするために必要なスタブの定義
 */

STK_T *const	_kernel_istkpt = 0x00;
void _kernel__start(void){}

/*
 *  コア依存のスタブの定義 
 */
#include <core_cfg1_out.h>
