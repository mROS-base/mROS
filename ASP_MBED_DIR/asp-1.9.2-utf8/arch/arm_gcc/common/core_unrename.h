/* This file is generated from core_rename.def by genrename. */

/* This file is included only when core_rename.h has been included. */
#ifdef TOPPERS_CORE_RENAME_H
#undef TOPPERS_CORE_RENAME_H

/*
 *  kernel_cfg.c
 */
#undef exch_tbl

/*
 *  arm.c
 */
#undef dcache_enable
#undef dcache_disable
#undef icache_enable
#undef icache_disable
#undef cache_enable
#undef cache_disable
#undef mmu_map_memory
#undef mmu_init

/*
 *  core_support.S
 */
#undef start_r
#undef ret_int
#undef ret_int_1
#undef ret_exc
#undef exch_tbl
#undef dispatch
#undef start_dispatch
#undef exit_and_dispatch
#undef call_exit_kernel
#undef default_exc_handler

/*
 *  core_config.c
 */
#undef excpt_nest_count
#undef core_initialize
#undef core_terminate
#undef x_install_exc

/*
 *  gic.c
 */
#undef gic_cpuif_init
#undef gic_cpuif_stop
#undef gic_dist_disable_int
#undef gic_dist_enable_int
#undef gic_dist_clear_pending
#undef gic_dist_set_pending
#undef gic_dist_probe_int
#undef gic_dist_config
#undef gic_dist_set_priority
#undef gic_dist_set_target
#undef gic_dist_init
#undef gic_dist_stop

/*
 *  gic_support.S
 */
#undef irq_handler
#undef target_exc_handler

#ifdef TOPPERS_LABEL_ASM

/*
 *  kernel_cfg.c
 */
#undef _exch_tbl

/*
 *  arm.c
 */
#undef _dcache_enable
#undef _dcache_disable
#undef _icache_enable
#undef _icache_disable
#undef _cache_enable
#undef _cache_disable
#undef _mmu_map_memory
#undef _mmu_init

/*
 *  core_support.S
 */
#undef _start_r
#undef _ret_int
#undef _ret_int_1
#undef _ret_exc
#undef _exch_tbl
#undef _dispatch
#undef _start_dispatch
#undef _exit_and_dispatch
#undef _call_exit_kernel
#undef _default_exc_handler

/*
 *  core_config.c
 */
#undef _excpt_nest_count
#undef _core_initialize
#undef _core_terminate
#undef _x_install_exc

/*
 *  gic.c
 */
#undef _gic_cpuif_init
#undef _gic_cpuif_stop
#undef _gic_dist_disable_int
#undef _gic_dist_enable_int
#undef _gic_dist_clear_pending
#undef _gic_dist_set_pending
#undef _gic_dist_probe_int
#undef _gic_dist_config
#undef _gic_dist_set_priority
#undef _gic_dist_set_target
#undef _gic_dist_init
#undef _gic_dist_stop

/*
 *  gic_support.S
 */
#undef _irq_handler
#undef _target_exc_handler

#endif /* TOPPERS_LABEL_ASM */


#endif /* TOPPERS_CORE_RENAME_H */
