/* This file is generated from target_rename.def by genrename. */

#ifndef TOPPERS_TARGET_RENAME_H
#define TOPPERS_TARGET_RENAME_H


/*
 * target_config.c
 */
#define target_initialize			_kernel_target_initialize
#define target_exit					_kernel_target_exit
#define target_mmu_init				_kernel_target_mmu_init

#ifdef TOPPERS_LABEL_ASM


/*
 * target_config.c
 */
#define _target_initialize			__kernel_target_initialize
#define _target_exit				__kernel_target_exit
#define _target_mmu_init			__kernel_target_mmu_init

#endif /* TOPPERS_LABEL_ASM */

#include "arm_gcc/rza1/chip_rename.h"

#endif /* TOPPERS_TARGET_RENAME_H */
