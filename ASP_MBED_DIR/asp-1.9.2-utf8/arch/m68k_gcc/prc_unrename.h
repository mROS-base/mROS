/* This file is generated from prc_rename.def by genrename. */

/* This file is included only when prc_rename.h has been included. */
#ifdef TOPPERS_PRC_RENAME_H
#undef TOPPERS_PRC_RENAME_H

/*
 *  prc_config.c
 */
#undef prc_initialize
#undef prc_terminate

/*
 *  prc_support.S
 */
#undef dispatch
#undef start_dispatch
#undef exit_and_dispatch
#undef call_exit_kernel
#undef start_r
#undef ret_int
#undef exchdr_entry
#undef lock_flag
#undef saved_iipm

#ifdef TOPPERS_LABEL_ASM

/*
 *  prc_config.c
 */
#undef _prc_initialize
#undef _prc_terminate

/*
 *  prc_support.S
 */
#undef _dispatch
#undef _start_dispatch
#undef _exit_and_dispatch
#undef _call_exit_kernel
#undef _start_r
#undef _ret_int
#undef _exchdr_entry
#undef _lock_flag
#undef _saved_iipm

#endif /* TOPPERS_LABEL_ASM */


#endif /* TOPPERS_PRC_RENAME_H */
