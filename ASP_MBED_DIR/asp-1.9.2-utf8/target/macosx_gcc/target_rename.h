/* This file is generated from target_rename.def by genrename. */

#ifndef TOPPERS_TARGET_RENAME_H
#define TOPPERS_TARGET_RENAME_H

/*
 *  kernel_cfg.c
 */
#define sigmask_table				_kernel_sigmask_table
#define sigmask_disint_init			_kernel_sigmask_disint_init

/*
 *  target_config.c
 */
#define sigmask_intlock				_kernel_sigmask_intlock
#define sigmask_cpulock				_kernel_sigmask_cpulock
#define lock_flag					_kernel_lock_flag
#define saved_sigmask				_kernel_saved_sigmask
#define ipm_value					_kernel_ipm_value
#define sigmask_disint				_kernel_sigmask_disint
#define dispatch					_kernel_dispatch
#define exit_and_dispatch			_kernel_exit_and_dispatch
#define ret_int						_kernel_ret_int
#define ret_exc						_kernel_ret_exc
#define call_exit_kernel			_kernel_call_exit_kernel
#define start_r						_kernel_start_r
#define target_initialize			_kernel_target_initialize
#define target_exit					_kernel_target_exit

/*
 *  target_timer.c
 */
#define target_timer_get_current	_kernel_target_timer_get_current
#define target_timer_probe_int		_kernel_target_timer_probe_int
#define target_ovrtimer_start		_kernel_target_ovrtimer_start
#define target_ovrtimer_stop		_kernel_target_ovrtimer_stop
#define target_ovrtimer_get_current	_kernel_target_ovrtimer_get_current

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
#define _sigmask_table				__kernel_sigmask_table
#define _sigmask_disint_init		__kernel_sigmask_disint_init

/*
 *  target_config.c
 */
#define _sigmask_intlock			__kernel_sigmask_intlock
#define _sigmask_cpulock			__kernel_sigmask_cpulock
#define _lock_flag					__kernel_lock_flag
#define _saved_sigmask				__kernel_saved_sigmask
#define _ipm_value					__kernel_ipm_value
#define _sigmask_disint				__kernel_sigmask_disint
#define _dispatch					__kernel_dispatch
#define _exit_and_dispatch			__kernel_exit_and_dispatch
#define _ret_int					__kernel_ret_int
#define _ret_exc					__kernel_ret_exc
#define _call_exit_kernel			__kernel_call_exit_kernel
#define _start_r					__kernel_start_r
#define _target_initialize			__kernel_target_initialize
#define _target_exit				__kernel_target_exit

/*
 *  target_timer.c
 */
#define _target_timer_get_current	__kernel_target_timer_get_current
#define _target_timer_probe_int		__kernel_target_timer_probe_int
#define _target_ovrtimer_start		__kernel_target_ovrtimer_start
#define _target_ovrtimer_stop		__kernel_target_ovrtimer_stop
#define _target_ovrtimer_get_current	__kernel_target_ovrtimer_get_current

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


#endif /* TOPPERS_TARGET_RENAME_H */
