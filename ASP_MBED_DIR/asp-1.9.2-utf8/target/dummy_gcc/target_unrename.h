/* This file is generated from target_rename.def by genrename. */

/* This file is included only when target_rename.h has been included. */
#ifdef TOPPERS_TARGET_RENAME_H
#undef TOPPERS_TARGET_RENAME_H

/*
 *  target_config.c
 */
#undef dispatch
#undef start_dispatch
#undef exit_and_dispatch
#undef ret_int
#undef ret_exc
#undef call_exit_kernel
#undef start_r
#undef target_initialize
#undef target_exit

/*
 *  target_timer.c
 */
#undef target_timer_get_current
#undef target_timer_probe_int
#undef target_ovrtimer_start
#undef target_ovrtimer_stop
#undef target_ovrtimer_get_current

/*
 *  trace_config.c
 */
#undef log_dsp_enter
#undef log_dsp_leave
#undef log_inh_enter
#undef log_inh_leave
#undef log_exc_enter
#undef log_exc_leave

#ifdef TOPPERS_LABEL_ASM

/*
 *  target_config.c
 */
#undef _dispatch
#undef _start_dispatch
#undef _exit_and_dispatch
#undef _ret_int
#undef _ret_exc
#undef _call_exit_kernel
#undef _start_r
#undef _target_initialize
#undef _target_exit

/*
 *  target_timer.c
 */
#undef _target_timer_get_current
#undef _target_timer_probe_int
#undef _target_ovrtimer_start
#undef _target_ovrtimer_stop
#undef _target_ovrtimer_get_current

/*
 *  trace_config.c
 */
#undef _log_dsp_enter
#undef _log_dsp_leave
#undef _log_inh_enter
#undef _log_inh_leave
#undef _log_exc_enter
#undef _log_exc_leave

#endif /* TOPPERS_LABEL_ASM */


#endif /* TOPPERS_TARGET_RENAME_H */
