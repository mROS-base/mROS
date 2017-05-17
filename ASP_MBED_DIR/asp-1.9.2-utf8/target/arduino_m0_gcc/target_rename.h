/* This file is generated from target_rename.def by genrename. */

#ifndef TOPPERS_TARGET_RENAME_H
#define TOPPERS_TARGET_RENAME_H

/*
 *  kernel_cfg.c
 */
#define ipm_mask_tbl				_kernel_ipm_mask_tbl
#define inh_tbl						_kernel_inh_tbl

/*
 *  target_config.c
 */
#define idf							_kernel_idf
#define ipm							_kernel_ipm
#define inh_ipm_tbl					_kernel_inh_ipm_tbl
#define bitpat_cfgint				_kernel_bitpat_cfgint
#define x_config_int				_kernel_x_config_int
#define target_exit					_kernel_target_exit
#define target_initialize			_kernel_target_initialize
#define default_int_handler			_kernel_default_int_handler

/*
 *  target_support.S
 */
#define undef_handler				_kernel_undef_handler
#define swi_handler					_kernel_swi_handler
#define prefetch_handler			_kernel_prefetch_handler
#define data_abort_handler			_kernel_data_abort_handler
#define interrupt_handler			_kernel_interrupt_handler
#define fiq_handler					_kernel_fiq_handler

/*
 *  target_timer.c
 */
#define target_timer_get_current	_kernel_target_timer_get_current
#define target_timer_probe_int		_kernel_target_timer_probe_int

/*
 *  trace_config.c
 */
#define log_dsp_enter				_kernel_log_dsp_enter
#define log_dsp_leave				_kernel_log_dsp_leave
#define log_inh_enter				_kernel_log_inh_enter
#define log_inh_leave				_kernel_log_inh_leave
#define log_exc_enter				_kernel_log_exc_enter
#define log_exc_leave				_kernel_log_exc_leave


#ifdef TOPPERS_LABEL_ASM

/*
 *  kernel_cfg.c
 */
#define _ipm_mask_tbl				__kernel_ipm_mask_tbl
#define _inh_tbl					__kernel_inh_tbl

/*
 *  target_config.c
 */
#define _idf						__kernel_idf
#define _ipm						__kernel_ipm
#define _inh_ipm_tbl				__kernel_inh_ipm_tbl
#define _bitpat_cfgint				__kernel_bitpat_cfgint
#define _x_config_int				__kernel_x_config_int
#define _target_exit				__kernel_target_exit
#define _target_initialize			__kernel_target_initialize
#define _default_int_handler		__kernel_default_int_handler

/*
 *  target_support.S
 */
#define _undef_handler				__kernel_undef_handler
#define _swi_handler				__kernel_swi_handler
#define _prefetch_handler			__kernel_prefetch_handler
#define _data_abort_handler			__kernel_data_abort_handler
#define _interrupt_handler			__kernel_interrupt_handler
#define _fiq_handler				__kernel_fiq_handler

/*
 *  target_timer.c
 */
#define _target_timer_get_current	__kernel_target_timer_get_current
#define _target_timer_probe_int		__kernel_target_timer_probe_int

/*
 *  trace_config.c
 */
#define _log_dsp_enter				__kernel_log_dsp_enter
#define _log_dsp_leave				__kernel_log_dsp_leave
#define _log_inh_enter				__kernel_log_inh_enter
#define _log_inh_leave				__kernel_log_inh_leave
#define _log_exc_enter				__kernel_log_exc_enter
#define _log_exc_leave				__kernel_log_exc_leave


#endif /* TOPPERS_LABEL_ASM */

#include "arm_m_gcc/common/core_rename.h"

#endif /* TOPPERS_TARGET_RENAME_H */
