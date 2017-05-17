/* This file is generated from chip_rename.def by genrename. */

/* This file is included only when chip_rename.h has been included. */
#ifdef TOPPERS_CHIP_RENAME_H
#undef TOPPERS_CHIP_RENAME_H

/*
 *  chip_config.c, chip_support.S
 */
#undef idf
#undef ipm
#undef inh_tbl
#undef ipm_mask_tbl
#undef inh_ipm_tbl
#undef bitpat_cfgint
#undef x_config_int
#undef irq_handler
#undef target_exc_handler

#undef undef_handler
#undef swi_handler
#undef prefetch_handler
#undef data_abort_handler
#undef interrupt_handler
#undef fiq_handler

/*
 *  core_config.c, core_support.S
 */
#undef core_initialize
#undef core_terminate
#undef default_int_handler


#ifdef TOPPERS_LABEL_ASM

/*
 *  chip_config.c, chip_support.S
 */
#undef _idf
#undef _ipm
#undef _inh_tbl
#undef _ipm_mask_tbl
#undef _inh_ipm_tbl
#undef _bitpat_cfgint
#undef _x_config_int
#undef _irq_handler
#undef _target_exc_handler

#undef _undef_handler
#undef _swi_handler
#undef _prefetch_handler
#undef _data_abort_handler
#undef _interrupt_handler
#undef _fiq_handler

/*
 *  core_config.c, core_support.S
 */
#undef _core_initialize
#undef _core_terminate
#undef _default_int_handler


#endif /* TOPPERS_LABEL_ASM */

#include "arm_gcc/common/core_unrename.h"

#endif /* TOPPERS_CHIP_RENAME_H */
