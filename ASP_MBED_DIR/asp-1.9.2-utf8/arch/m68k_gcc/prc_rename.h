/* This file is generated from prc_rename.def by genrename. */

#ifndef TOPPERS_PRC_RENAME_H
#define TOPPERS_PRC_RENAME_H

/*
 *  prc_config.c
 */
#define prc_initialize				_kernel_prc_initialize
#define prc_terminate				_kernel_prc_terminate

/*
 *  prc_support.S
 */
#define dispatch					_kernel_dispatch
#define start_dispatch				_kernel_start_dispatch
#define exit_and_dispatch			_kernel_exit_and_dispatch
#define call_exit_kernel			_kernel_call_exit_kernel
#define start_r						_kernel_start_r
#define ret_int						_kernel_ret_int
#define exchdr_entry				_kernel_exchdr_entry
#define lock_flag					_kernel_lock_flag
#define saved_iipm					_kernel_saved_iipm

#ifdef TOPPERS_LABEL_ASM

/*
 *  prc_config.c
 */
#define _prc_initialize				__kernel_prc_initialize
#define _prc_terminate				__kernel_prc_terminate

/*
 *  prc_support.S
 */
#define _dispatch					__kernel_dispatch
#define _start_dispatch				__kernel_start_dispatch
#define _exit_and_dispatch			__kernel_exit_and_dispatch
#define _call_exit_kernel			__kernel_call_exit_kernel
#define _start_r					__kernel_start_r
#define _ret_int					__kernel_ret_int
#define _exchdr_entry				__kernel_exchdr_entry
#define _lock_flag					__kernel_lock_flag
#define _saved_iipm					__kernel_saved_iipm

#endif /* TOPPERS_LABEL_ASM */


#endif /* TOPPERS_PRC_RENAME_H */
