/* This file is generated from chip_rename.def by genrename. */

/* This file is included only when chip_rename.h has been included. */
#ifdef TOPPERS_CHIP_RENAME_H
#undef TOPPERS_CHIP_RENAME_H

/*
 *  chip_config.c
 */
#undef inh_tbl
#undef default_int_handler
#undef x_config_int
#undef chip_exit
#undef chip_initialize

/*
 *  chip_support.S
 */

/*
 *  kernel_cfg.c 
 */
#undef cfgint_tbl


#ifdef TOPPERS_LABEL_ASM

/*
 *  chip_config.c
 */
#undef _inh_tbl
#undef _default_int_handler
#undef _x_config_int
#undef _chip_exit
#undef _chip_initialize

/*
 *  chip_support.S
 */

/*
 *  kernel_cfg.c 
 */
#undef _cfgint_tbl


#endif /* TOPPERS_LABEL_ASM */

#include "arm_gcc/common/core_unrename.h"

#endif /* TOPPERS_CHIP_RENAME_H */
