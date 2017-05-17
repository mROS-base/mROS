/* This file is generated from chip_rename.def by genrename. */

#ifndef TOPPERS_CHIP_RENAME_H
#define TOPPERS_CHIP_RENAME_H

/*
 *  chip_config.c
 */
#define default_int_handler			_kernel_default_int_handler
#define x_config_int				_kernel_x_config_int
#define chip_exit					_kernel_chip_exit
#define chip_initialize				_kernel_chip_initialize


/*
 *  gic.c
 */
#define gicc_init					_kernel_gicc_init
#define gicc_stop					_kernel_gicc_stop
#define gicd_clear_pending			_kernel_gicd_clear_pending
#define gicd_config					_kernel_gicd_config
#define gicd_disable_int			_kernel_gicd_disable_int
#define gicd_enable_int				_kernel_gicd_enable_int
#define gicd_init					_kernel_gicd_init
#define gicd_probe_int				_kernel_gicd_probe_int
#define gicd_set_pending			_kernel_gicd_set_pending
#define gicd_set_priority			_kernel_gicd_set_priority
#define gicd_set_target				_kernel_gicd_set_target
#define gicd_stop					_kernel_gicd_stop

/*
 *  kernel_cfg.c 
 */
#define cfgint_tbl					_kernel_cfgint_tbl
#define inh_tbl						_kernel_inh_tbl


#ifdef TOPPERS_LABEL_ASM

/*
 *  chip_config.c
 */
#define _default_int_handler		__kernel_default_int_handler
#define _x_config_int				__kernel_x_config_int
#define _chip_exit					__kernel_chip_exit
#define _chip_initialize			__kernel_chip_initialize


/*
 *  gic.c
 */
#define _gicc_init					__kernel_gicc_init
#define _gicc_stop					__kernel_gicc_stop
#define _gicd_clear_pending			__kernel_gicd_clear_pending
#define _gicd_config				__kernel_gicd_config
#define _gicd_disable_int			__kernel_gicd_disable_int
#define _gicd_enable_int			__kernel_gicd_enable_int
#define _gicd_init					__kernel_gicd_init
#define _gicd_probe_int				__kernel_gicd_probe_int
#define _gicd_set_pending			__kernel_gicd_set_pending
#define _gicd_set_priority			__kernel_gicd_set_priority
#define _gicd_set_target			__kernel_gicd_set_target
#define _gicd_stop					__kernel_gicd_stop

/*
 *  kernel_cfg.c 
 */
#define _cfgint_tbl					__kernel_cfgint_tbl
#define _inh_tbl					__kernel_inh_tbl


#endif /* TOPPERS_LABEL_ASM */

#include "arm_gcc/common/core_rename.h"

#endif /* TOPPERS_CHIP_RENAME_H */
