/* This file is generated from core_rename.def by genrename. */

#ifndef TOPPERS_CORE_RENAME_H
#define TOPPERS_CORE_RENAME_H

/*
 *  kernel_cfg.c
 */
#define exch_tbl					_kernel_exch_tbl

/*
 *  arm.c
 */
#define dcache_enable				_kernel_dcache_enable
#define dcache_disable				_kernel_dcache_disable
#define icache_enable				_kernel_icache_enable
#define icache_disable				_kernel_icache_disable
#define cache_enable				_kernel_cache_enable
#define cache_disable				_kernel_cache_disable
#define mmu_map_memory				_kernel_mmu_map_memory
#define mmu_init					_kernel_mmu_init

/*
 *  core_support.S
 */
#define start_r						_kernel_start_r
#define ret_int						_kernel_ret_int
#define ret_int_1					_kernel_ret_int_1
#define ret_exc						_kernel_ret_exc
#define exch_tbl					_kernel_exch_tbl
#define dispatch					_kernel_dispatch
#define start_dispatch				_kernel_start_dispatch
#define exit_and_dispatch			_kernel_exit_and_dispatch
#define call_exit_kernel			_kernel_call_exit_kernel
#define default_exc_handler			_kernel_default_exc_handler

/*
 *  core_config.c
 */
#define excpt_nest_count			_kernel_excpt_nest_count
#define core_initialize				_kernel_core_initialize
#define core_terminate				_kernel_core_terminate
#define x_install_exc				_kernel_x_install_exc

/*
 *  gic.c
 */
#define gic_cpuif_init				_kernel_gic_cpuif_init
#define gic_cpuif_stop				_kernel_gic_cpuif_stop
#define gic_dist_disable_int		_kernel_gic_dist_disable_int
#define gic_dist_enable_int			_kernel_gic_dist_enable_int
#define gic_dist_clear_pending		_kernel_gic_dist_clear_pending
#define gic_dist_set_pending		_kernel_gic_dist_set_pending
#define gic_dist_probe_int			_kernel_gic_dist_probe_int
#define gic_dist_config				_kernel_gic_dist_config
#define gic_dist_set_priority		_kernel_gic_dist_set_priority
#define gic_dist_set_target			_kernel_gic_dist_set_target
#define gic_dist_init				_kernel_gic_dist_init
#define gic_dist_stop				_kernel_gic_dist_stop

/*
 *  gic_support.S
 */
#define irq_handler					_kernel_irq_handler
#define target_exc_handler			_kernel_target_exc_handler

#ifdef TOPPERS_LABEL_ASM

/*
 *  kernel_cfg.c
 */
#define _exch_tbl					__kernel_exch_tbl

/*
 *  arm.c
 */
#define _dcache_enable				__kernel_dcache_enable
#define _dcache_disable				__kernel_dcache_disable
#define _icache_enable				__kernel_icache_enable
#define _icache_disable				__kernel_icache_disable
#define _cache_enable				__kernel_cache_enable
#define _cache_disable				__kernel_cache_disable
#define _mmu_map_memory				__kernel_mmu_map_memory
#define _mmu_init					__kernel_mmu_init

/*
 *  core_support.S
 */
#define _start_r					__kernel_start_r
#define _ret_int					__kernel_ret_int
#define _ret_int_1					__kernel_ret_int_1
#define _ret_exc					__kernel_ret_exc
#define _exch_tbl					__kernel_exch_tbl
#define _dispatch					__kernel_dispatch
#define _start_dispatch				__kernel_start_dispatch
#define _exit_and_dispatch			__kernel_exit_and_dispatch
#define _call_exit_kernel			__kernel_call_exit_kernel
#define _default_exc_handler		__kernel_default_exc_handler

/*
 *  core_config.c
 */
#define _excpt_nest_count			__kernel_excpt_nest_count
#define _core_initialize			__kernel_core_initialize
#define _core_terminate				__kernel_core_terminate
#define _x_install_exc				__kernel_x_install_exc

/*
 *  gic.c
 */
#define _gic_cpuif_init				__kernel_gic_cpuif_init
#define _gic_cpuif_stop				__kernel_gic_cpuif_stop
#define _gic_dist_disable_int		__kernel_gic_dist_disable_int
#define _gic_dist_enable_int		__kernel_gic_dist_enable_int
#define _gic_dist_clear_pending		__kernel_gic_dist_clear_pending
#define _gic_dist_set_pending		__kernel_gic_dist_set_pending
#define _gic_dist_probe_int			__kernel_gic_dist_probe_int
#define _gic_dist_config			__kernel_gic_dist_config
#define _gic_dist_set_priority		__kernel_gic_dist_set_priority
#define _gic_dist_set_target		__kernel_gic_dist_set_target
#define _gic_dist_init				__kernel_gic_dist_init
#define _gic_dist_stop				__kernel_gic_dist_stop

/*
 *  gic_support.S
 */
#define _irq_handler				__kernel_irq_handler
#define _target_exc_handler			__kernel_target_exc_handler

#endif /* TOPPERS_LABEL_ASM */


#endif /* TOPPERS_CORE_RENAME_H */
