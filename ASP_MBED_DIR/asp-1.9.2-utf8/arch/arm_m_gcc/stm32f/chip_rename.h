/* This file is generated from chip_rename.def by genrename. */

#ifndef TOPPERS_CHIP_RENAME_H
#define TOPPERS_CHIP_RENAME_H


/*
 * chip_config.c
 */
#define chip_initialize				_kernel_chip_initialize
#define chip_exit					_kernel_chip_exit

#ifdef TOPPERS_LABEL_ASM


/*
 * chip_config.c
 */
#define _chip_initialize			__kernel_chip_initialize
#define _chip_exit					__kernel_chip_exit

#endif /* TOPPERS_LABEL_ASM */

#include "arm_m_gcc/common/core_rename.h"

#endif /* TOPPERS_CHIP_RENAME_H */
