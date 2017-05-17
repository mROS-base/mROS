/* This file is generated from target_rename.def by genrename. */

#ifndef TOPPERS_TARGET_RENAME_H
#define TOPPERS_TARGET_RENAME_H


/*
 * target_config.c
 */
#define target_initialize			_kernel_target_initialize
#define target_exit					_kernel_target_exit

#ifdef TOPPERS_LABEL_ASM


/*
 * target_config.c
 */
#define _target_initialize			__kernel_target_initialize
#define _target_exit				__kernel_target_exit

#endif /* TOPPERS_LABEL_ASM */

#include "arm_m_gcc/stm32f/chip_rename.h"

#endif /* TOPPERS_TARGET_RENAME_H */
