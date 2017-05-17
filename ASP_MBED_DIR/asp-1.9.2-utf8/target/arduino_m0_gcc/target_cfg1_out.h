/*
 *  @(#) $Id: target_cfg1_out.h 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		cfg1_out.cをリンクするために必要なスタブの定義
 */

STK_T *const	_kernel_istkpt = 0x00;

/*
 *  コア依存のスタブの定義 
 */
#include "core_cfg1_out.h"
