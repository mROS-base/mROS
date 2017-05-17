/* This file is generated from target_rename.def by genrename. */

/* This file is included only when target_rename.h has been included. */
#ifdef TOPPERS_TARGET_RENAME_H
#undef TOPPERS_TARGET_RENAME_H

/*
 *  kernel_cfg.c
 */
#undef bitpat_cfgint

/*
 *  target_config.c
 */
#undef board_id
#undef board_addr
#undef target_initialize
#undef target_exit
#undef x_config_int

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
#undef _bitpat_cfgint

/*
 *  target_config.c
 */
#undef _board_id
#undef _board_addr
#undef _target_initialize
#undef _target_exit
#undef _x_config_int

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

#include "m68k_gcc/prc_unrename.h"

#endif /* TOPPERS_TARGET_RENAME_H */
