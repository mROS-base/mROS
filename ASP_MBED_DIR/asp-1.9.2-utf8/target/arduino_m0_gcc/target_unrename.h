/* This file is generated from target_rename.def by genrename. */

/* This file is included only when target_rename.h has been included. */
#ifdef TOPPERS_TARGET_RENAME_H
#undef TOPPERS_TARGET_RENAME_H

/*
 *  kernel_cfg.c
 */
#undef ipm_mask_tbl
#undef inh_tbl

/*
 *  target_config.c
 */
#undef idf
#undef ipm
#undef inh_ipm_tbl
#undef bitpat_cfgint
#undef x_config_int
#undef target_exit
#undef target_initialize
#undef default_int_handler

/*
 *  target_support.S
 */
#undef undef_handler
#undef swi_handler
#undef prefetch_handler
#undef data_abort_handler
#undef interrupt_handler
#undef fiq_handler

/*
 *  target_timer.c
 */
#undef target_timer_get_current
#undef target_timer_probe_int

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
 *  kernel_cfg.c
 */
#undef _ipm_mask_tbl
#undef _inh_tbl

/*
 *  target_config.c
 */
#undef _idf
#undef _ipm
#undef _inh_ipm_tbl
#undef _bitpat_cfgint
#undef _x_config_int
#undef _target_exit
#undef _target_initialize
#undef _default_int_handler

/*
 *  target_support.S
 */
#undef _undef_handler
#undef _swi_handler
#undef _prefetch_handler
#undef _data_abort_handler
#undef _interrupt_handler
#undef _fiq_handler

/*
 *  target_timer.c
 */
#undef _target_timer_get_current
#undef _target_timer_probe_int

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

#include "arm_m_gcc/common/core_unrename.h"

#endif /* TOPPERS_TARGET_RENAME_H */
