/* This file is generated from chip_rename.def by genrename. */

#ifndef TOPPERS_CHIP_RENAME_H
#define TOPPERS_CHIP_RENAME_H

/*
 *  chip_config.c
 */
#define inh_tbl						_kernel_inh_tbl
#define default_int_handler			_kernel_default_int_handler
#define x_config_int				_kernel_x_config_int
#define chip_exit					_kernel_chip_exit
#define chip_initialize				_kernel_chip_initialize

/*
 *  chip_support.S
 */

/*
 *  kernel_cfg.c 
 */
#define cfgint_tbl					_kernel_cfgint_tbl


#ifdef TOPPERS_LABEL_ASM

/*
 *  chip_config.c
 */
#define _inh_tbl					__kernel_inh_tbl
#define _default_int_handler		__kernel_default_int_handler
#define _x_config_int				__kernel_x_config_int
#define _chip_exit					__kernel_chip_exit
#define _chip_initialize			__kernel_chip_initialize

/*
 *  chip_support.S
 */

/*
 *  kernel_cfg.c 
 */
#define _cfgint_tbl					__kernel_cfgint_tbl


#endif /* TOPPERS_LABEL_ASM */

#include "arm_gcc/common/core_rename.h"

#endif /* TOPPERS_CHIP_RENAME_H */
