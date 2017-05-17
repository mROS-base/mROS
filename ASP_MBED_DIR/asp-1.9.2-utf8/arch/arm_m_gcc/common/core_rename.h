/* This file is generated from core_rename.def by genrename. */

#ifndef TOPPERS_CORE_RENAME_H
#define TOPPERS_CORE_RENAME_H

/*
 *  core_config.c
 */
#define exc_tbl						_kernel_exc_tbl
#define vector_table				_kernel_vector_table
#define lock_flag					_kernel_lock_flag
#define saved_iipm					_kernel_saved_iipm
#define default_exc_handler			_kernel_default_exc_handler
#define default_int_handler			_kernel_default_int_handler
#define x_config_int				_kernel_x_config_int
#define core_initialize				_kernel_core_initialize
#define core_terminate				_kernel_core_terminate
#define bitpat_cfgint				_kernel_bitpat_cfgint
#define set_exc_int_priority		_kernel_set_exc_int_priority
#define enable_exc					_kernel_enable_exc
#define disable_exc					_kernel_disable_exc
#define iipm_enable_irq_tbl			_kernel_iipm_enable_irq_tbl
#define iipm_enable_systic_tbl		_kernel_iipm_enable_systic_tbl
#define iipm						_kernel_iipm
#define ief							_kernel_ief
#define ief_systick					_kernel_ief_systick

/*
 *  core_support.S
 */
#define core_int_entry				_kernel_core_int_entry
#define core_exc_entry				_kernel_core_exc_entry
#define ret_int						_kernel_ret_int
#define ret_exc						_kernel_ret_exc
#define svc_handler					_kernel_svc_handler
#define start_r						_kernel_start_r
#define dispatch					_kernel_dispatch
#define start_dispatch				_kernel_start_dispatch
#define exit_and_dispatch			_kernel_exit_and_dispatch
#define call_exit_kernel			_kernel_call_exit_kernel

/*
 *  start.S
 */
#define _start						_kernel__start

#ifdef TOPPERS_LABEL_ASM

/*
 *  core_config.c
 */
#define _exc_tbl					__kernel_exc_tbl
#define _vector_table				__kernel_vector_table
#define _lock_flag					__kernel_lock_flag
#define _saved_iipm					__kernel_saved_iipm
#define _default_exc_handler		__kernel_default_exc_handler
#define _default_int_handler		__kernel_default_int_handler
#define _x_config_int				__kernel_x_config_int
#define _core_initialize			__kernel_core_initialize
#define _core_terminate				__kernel_core_terminate
#define _bitpat_cfgint				__kernel_bitpat_cfgint
#define _set_exc_int_priority		__kernel_set_exc_int_priority
#define _enable_exc					__kernel_enable_exc
#define _disable_exc				__kernel_disable_exc
#define _iipm_enable_irq_tbl		__kernel_iipm_enable_irq_tbl
#define _iipm_enable_systic_tbl		__kernel_iipm_enable_systic_tbl
#define _iipm						__kernel_iipm
#define _ief						__kernel_ief
#define _ief_systick				__kernel_ief_systick

/*
 *  core_support.S
 */
#define _core_int_entry				__kernel_core_int_entry
#define _core_exc_entry				__kernel_core_exc_entry
#define _ret_int					__kernel_ret_int
#define _ret_exc					__kernel_ret_exc
#define _svc_handler				__kernel_svc_handler
#define _start_r					__kernel_start_r
#define _dispatch					__kernel_dispatch
#define _start_dispatch				__kernel_start_dispatch
#define _exit_and_dispatch			__kernel_exit_and_dispatch
#define _call_exit_kernel			__kernel_call_exit_kernel

/*
 *  start.S
 */
#define __start						__kernel__start

#endif /* TOPPERS_LABEL_ASM */


#endif /* TOPPERS_CORE_RENAME_H */
