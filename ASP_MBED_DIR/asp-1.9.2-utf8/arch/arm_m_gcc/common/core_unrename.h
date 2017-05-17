/* This file is generated from core_rename.def by genrename. */

/* This file is included only when core_rename.h has been included. */
#ifdef TOPPERS_CORE_RENAME_H
#undef TOPPERS_CORE_RENAME_H

/*
 *  core_config.c
 */
#undef exc_tbl
#undef vector_table
#undef lock_flag
#undef saved_iipm
#undef default_exc_handler
#undef default_int_handler
#undef x_config_int
#undef core_initialize
#undef core_terminate
#undef bitpat_cfgint
#undef set_exc_int_priority
#undef enable_exc
#undef disable_exc
#undef iipm_enable_irq_tbl
#undef iipm_enable_systic_tbl
#undef iipm
#undef ief
#undef ief_systick

/*
 *  core_support.S
 */
#undef core_int_entry
#undef core_exc_entry
#undef ret_int
#undef ret_exc
#undef svc_handler
#undef start_r
#undef dispatch
#undef start_dispatch
#undef exit_and_dispatch
#undef call_exit_kernel

/*
 *  start.S
 */
#undef _start

#ifdef TOPPERS_LABEL_ASM

/*
 *  core_config.c
 */
#undef _exc_tbl
#undef _vector_table
#undef _lock_flag
#undef _saved_iipm
#undef _default_exc_handler
#undef _default_int_handler
#undef _x_config_int
#undef _core_initialize
#undef _core_terminate
#undef _bitpat_cfgint
#undef _set_exc_int_priority
#undef _enable_exc
#undef _disable_exc
#undef _iipm_enable_irq_tbl
#undef _iipm_enable_systic_tbl
#undef _iipm
#undef _ief
#undef _ief_systick

/*
 *  core_support.S
 */
#undef _core_int_entry
#undef _core_exc_entry
#undef _ret_int
#undef _ret_exc
#undef _svc_handler
#undef _start_r
#undef _dispatch
#undef _start_dispatch
#undef _exit_and_dispatch
#undef _call_exit_kernel

/*
 *  start.S
 */
#undef __start

#endif /* TOPPERS_LABEL_ASM */


#endif /* TOPPERS_CORE_RENAME_H */
