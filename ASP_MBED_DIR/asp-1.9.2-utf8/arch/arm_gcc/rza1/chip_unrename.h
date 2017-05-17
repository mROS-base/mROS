/* This file is generated from chip_rename.def by genrename. */

/* This file is included only when chip_rename.h has been included. */
#ifdef TOPPERS_CHIP_RENAME_H
#undef TOPPERS_CHIP_RENAME_H

/*
 *  chip_config.c
 */
#undef default_int_handler
#undef x_config_int
#undef chip_exit
#undef chip_initialize


/*
 *  gic.c
 */
#undef gicc_init
#undef gicc_stop
#undef gicd_clear_pending
#undef gicd_config
#undef gicd_disable_int
#undef gicd_enable_int
#undef gicd_init
#undef gicd_probe_int
#undef gicd_set_pending
#undef gicd_set_priority
#undef gicd_set_target
#undef gicd_stop

/*
 *  kernel_cfg.c 
 */
#undef cfgint_tbl
#undef inh_tbl


#ifdef TOPPERS_LABEL_ASM

/*
 *  chip_config.c
 */
#undef _default_int_handler
#undef _x_config_int
#undef _chip_exit
#undef _chip_initialize


/*
 *  gic.c
 */
#undef _gicc_init
#undef _gicc_stop
#undef _gicd_clear_pending
#undef _gicd_config
#undef _gicd_disable_int
#undef _gicd_enable_int
#undef _gicd_init
#undef _gicd_probe_int
#undef _gicd_set_pending
#undef _gicd_set_priority
#undef _gicd_set_target
#undef _gicd_stop

/*
 *  kernel_cfg.c 
 */
#undef _cfgint_tbl
#undef _inh_tbl


#endif /* TOPPERS_LABEL_ASM */

#include "arm_gcc/common/core_unrename.h"

#endif /* TOPPERS_CHIP_RENAME_H */
