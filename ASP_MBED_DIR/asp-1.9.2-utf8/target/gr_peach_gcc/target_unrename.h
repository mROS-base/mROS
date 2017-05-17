/* This file is generated from target_rename.def by genrename. */

/* This file is included only when target_rename.h has been included. */
#ifdef TOPPERS_TARGET_RENAME_H
#undef TOPPERS_TARGET_RENAME_H


/*
 * target_config.c
 */
#undef target_initialize
#undef target_exit
#undef target_mmu_init

#ifdef TOPPERS_LABEL_ASM


/*
 * target_config.c
 */
#undef _target_initialize
#undef _target_exit
#undef _target_mmu_init

#endif /* TOPPERS_LABEL_ASM */

#include "arm_gcc/rza1/chip_unrename.h"

#endif /* TOPPERS_TARGET_RENAME_H */
