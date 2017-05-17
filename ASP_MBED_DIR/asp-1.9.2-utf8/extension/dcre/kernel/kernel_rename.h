/* This file is generated from kernel_rename.def by genrename. */

#ifndef TOPPERS_KERNEL_RENAME_H
#define TOPPERS_KERNEL_RENAME_H

/*
 *  startup.c
 */
#define kerflg						_kernel_kerflg
#define exit_kernel					_kernel_exit_kernel
#define initialize_kmm				_kernel_initialize_kmm
#define kernel_malloc				_kernel_kernel_malloc
#define kernel_free					_kernel_kernel_free

/*
 *  task.c
 */
#define p_runtsk					_kernel_p_runtsk
#define p_schedtsk					_kernel_p_schedtsk
#define reqflg						_kernel_reqflg
#define ipmflg						_kernel_ipmflg
#define disdsp						_kernel_disdsp
#define dspflg						_kernel_dspflg
#define ready_queue					_kernel_ready_queue
#define ready_primap				_kernel_ready_primap
#define free_tcb					_kernel_free_tcb
#define initialize_task				_kernel_initialize_task
#define search_schedtsk				_kernel_search_schedtsk
#define make_runnable				_kernel_make_runnable
#define make_non_runnable			_kernel_make_non_runnable
#define make_dormant				_kernel_make_dormant
#define make_active					_kernel_make_active
#define change_priority				_kernel_change_priority
#define rotate_ready_queue			_kernel_rotate_ready_queue
#define call_texrtn					_kernel_call_texrtn
#define calltex						_kernel_calltex

/*
 *  wait.c
 */
#define make_wait_tmout				_kernel_make_wait_tmout
#define wait_complete				_kernel_wait_complete
#define wait_tmout					_kernel_wait_tmout
#define wait_tmout_ok				_kernel_wait_tmout_ok
#define wait_release				_kernel_wait_release
#define wobj_make_wait				_kernel_wobj_make_wait
#define wobj_make_wait_tmout		_kernel_wobj_make_wait_tmout
#define init_wait_queue				_kernel_init_wait_queue

/*
 *  time_event.c
 */
#define current_time				_kernel_current_time
#define min_time					_kernel_min_time
#define next_time					_kernel_next_time
#define next_subtime				_kernel_next_subtime
#define last_index					_kernel_last_index
#define initialize_tmevt			_kernel_initialize_tmevt
#define tmevt_up					_kernel_tmevt_up
#define tmevt_down					_kernel_tmevt_down
#define tmevtb_insert				_kernel_tmevtb_insert
#define tmevtb_delete				_kernel_tmevtb_delete
#define tmevt_lefttim				_kernel_tmevt_lefttim
#define signal_time					_kernel_signal_time

/*
 *  semaphore.c
 */
#define free_semcb					_kernel_free_semcb
#define initialize_semaphore		_kernel_initialize_semaphore

/*
 *  eventflag.c
 */
#define free_flgcb					_kernel_free_flgcb
#define initialize_eventflag		_kernel_initialize_eventflag
#define check_flg_cond				_kernel_check_flg_cond

/*
 *  dataqueue.c
 */
#define free_dtqcb					_kernel_free_dtqcb
#define initialize_dataqueue		_kernel_initialize_dataqueue
#define enqueue_data				_kernel_enqueue_data
#define force_enqueue_data			_kernel_force_enqueue_data
#define dequeue_data				_kernel_dequeue_data
#define send_data					_kernel_send_data
#define force_send_data				_kernel_force_send_data
#define receive_data				_kernel_receive_data

/*
 *  pridataq.c
 */
#define free_pdqcb					_kernel_free_pdqcb
#define initialize_pridataq			_kernel_initialize_pridataq
#define enqueue_pridata				_kernel_enqueue_pridata
#define dequeue_pridata				_kernel_dequeue_pridata
#define send_pridata				_kernel_send_pridata
#define receive_pridata				_kernel_receive_pridata

/*
 *  mailbox.c
 */
#define free_mbxcb					_kernel_free_mbxcb
#define initialize_mailbox			_kernel_initialize_mailbox

/*
 *  mempfix.c
 */
#define free_mpfcb					_kernel_free_mpfcb
#define initialize_mempfix			_kernel_initialize_mempfix
#define get_mpf_block				_kernel_get_mpf_block

/*
 *  cyclic.c
 */
#define free_cyccb					_kernel_free_cyccb
#define initialize_cyclic			_kernel_initialize_cyclic
#define call_cychdr					_kernel_call_cychdr

/*
 *  alarm.c
 */
#define free_almcb					_kernel_free_almcb
#define initialize_alarm			_kernel_initialize_alarm
#define call_almhdr					_kernel_call_almhdr

/*
 *  interrupt.c
 */
#define free_isrcb					_kernel_free_isrcb
#define initialize_isr				_kernel_initialize_isr
#define call_isr					_kernel_call_isr
#define initialize_interrupt		_kernel_initialize_interrupt

/*
 *  exception.c
 */
#define initialize_exception		_kernel_initialize_exception

/*
 *  kernel_cfg.c
 */
#define initialize_object			_kernel_initialize_object
#define call_inirtn					_kernel_call_inirtn
#define call_terrtn					_kernel_call_terrtn
#define tmax_tskid					_kernel_tmax_tskid
#define tmax_stskid					_kernel_tmax_stskid
#define tinib_table					_kernel_tinib_table
#define atinib_table				_kernel_atinib_table
#define tcb_table					_kernel_tcb_table
#define torder_table				_kernel_torder_table
#define tmax_semid					_kernel_tmax_semid
#define tmax_ssemid					_kernel_tmax_ssemid
#define seminib_table				_kernel_seminib_table
#define aseminib_table				_kernel_aseminib_table
#define semcb_table					_kernel_semcb_table
#define tmax_flgid					_kernel_tmax_flgid
#define tmax_sflgid					_kernel_tmax_sflgid
#define flginib_table				_kernel_flginib_table
#define aflginib_table				_kernel_aflginib_table
#define flgcb_table					_kernel_flgcb_table
#define tmax_dtqid					_kernel_tmax_dtqid
#define tmax_sdtqid					_kernel_tmax_sdtqid
#define dtqinib_table				_kernel_dtqinib_table
#define adtqinib_table				_kernel_adtqinib_table
#define dtqcb_table					_kernel_dtqcb_table
#define tmax_pdqid					_kernel_tmax_pdqid
#define tmax_spdqid					_kernel_tmax_spdqid
#define pdqinib_table				_kernel_pdqinib_table
#define apdqinib_table				_kernel_apdqinib_table
#define pdqcb_table					_kernel_pdqcb_table
#define tmax_mbxid					_kernel_tmax_mbxid
#define tmax_smbxid					_kernel_tmax_smbxid
#define mbxinib_table				_kernel_mbxinib_table
#define ambxinib_table				_kernel_ambxinib_table
#define mbxcb_table					_kernel_mbxcb_table
#define tmax_mpfid					_kernel_tmax_mpfid
#define tmax_smpfid					_kernel_tmax_smpfid
#define mpfinib_table				_kernel_mpfinib_table
#define ampfinib_table				_kernel_ampfinib_table
#define mpfcb_table					_kernel_mpfcb_table
#define tmax_cycid					_kernel_tmax_cycid
#define tmax_scycid					_kernel_tmax_scycid
#define cycinib_table				_kernel_cycinib_table
#define acycinib_table				_kernel_acycinib_table
#define cyccb_table					_kernel_cyccb_table
#define tmax_almid					_kernel_tmax_almid
#define tmax_salmid					_kernel_tmax_salmid
#define alminib_table				_kernel_alminib_table
#define aalminib_table				_kernel_aalminib_table
#define almcb_table					_kernel_almcb_table
#define tnum_isr_queue				_kernel_tnum_isr_queue
#define isr_queue_table				_kernel_isr_queue_table
#define isr_queue_list				_kernel_isr_queue_list
#define tmax_isrid					_kernel_tmax_isrid
#define tnum_sisr					_kernel_tnum_sisr
#define sisrinib_table				_kernel_sisrinib_table
#define aisrinib_table				_kernel_aisrinib_table
#define isrcb_table					_kernel_isrcb_table
#define tnum_inhno					_kernel_tnum_inhno
#define inhinib_table				_kernel_inhinib_table
#define tnum_intno					_kernel_tnum_intno
#define intinib_table				_kernel_intinib_table
#define tnum_excno					_kernel_tnum_excno
#define excinib_table				_kernel_excinib_table
#define tmevt_heap					_kernel_tmevt_heap
#define istksz						_kernel_istksz
#define istk						_kernel_istk
#define istkpt						_kernel_istkpt
#define kmmsz						_kernel_kmmsz
#define kmm							_kernel_kmm


#ifdef TOPPERS_LABEL_ASM

/*
 *  startup.c
 */
#define _kerflg						__kernel_kerflg
#define _exit_kernel				__kernel_exit_kernel
#define _initialize_kmm				__kernel_initialize_kmm
#define _kernel_malloc				__kernel_kernel_malloc
#define _kernel_free				__kernel_kernel_free

/*
 *  task.c
 */
#define _p_runtsk					__kernel_p_runtsk
#define _p_schedtsk					__kernel_p_schedtsk
#define _reqflg						__kernel_reqflg
#define _ipmflg						__kernel_ipmflg
#define _disdsp						__kernel_disdsp
#define _dspflg						__kernel_dspflg
#define _ready_queue				__kernel_ready_queue
#define _ready_primap				__kernel_ready_primap
#define _free_tcb					__kernel_free_tcb
#define _initialize_task			__kernel_initialize_task
#define _search_schedtsk			__kernel_search_schedtsk
#define _make_runnable				__kernel_make_runnable
#define _make_non_runnable			__kernel_make_non_runnable
#define _make_dormant				__kernel_make_dormant
#define _make_active				__kernel_make_active
#define _change_priority			__kernel_change_priority
#define _rotate_ready_queue			__kernel_rotate_ready_queue
#define _call_texrtn				__kernel_call_texrtn
#define _calltex					__kernel_calltex

/*
 *  wait.c
 */
#define _make_wait_tmout			__kernel_make_wait_tmout
#define _wait_complete				__kernel_wait_complete
#define _wait_tmout					__kernel_wait_tmout
#define _wait_tmout_ok				__kernel_wait_tmout_ok
#define _wait_release				__kernel_wait_release
#define _wobj_make_wait				__kernel_wobj_make_wait
#define _wobj_make_wait_tmout		__kernel_wobj_make_wait_tmout
#define _init_wait_queue			__kernel_init_wait_queue

/*
 *  time_event.c
 */
#define _current_time				__kernel_current_time
#define _min_time					__kernel_min_time
#define _next_time					__kernel_next_time
#define _next_subtime				__kernel_next_subtime
#define _last_index					__kernel_last_index
#define _initialize_tmevt			__kernel_initialize_tmevt
#define _tmevt_up					__kernel_tmevt_up
#define _tmevt_down					__kernel_tmevt_down
#define _tmevtb_insert				__kernel_tmevtb_insert
#define _tmevtb_delete				__kernel_tmevtb_delete
#define _tmevt_lefttim				__kernel_tmevt_lefttim
#define _signal_time				__kernel_signal_time

/*
 *  semaphore.c
 */
#define _free_semcb					__kernel_free_semcb
#define _initialize_semaphore		__kernel_initialize_semaphore

/*
 *  eventflag.c
 */
#define _free_flgcb					__kernel_free_flgcb
#define _initialize_eventflag		__kernel_initialize_eventflag
#define _check_flg_cond				__kernel_check_flg_cond

/*
 *  dataqueue.c
 */
#define _free_dtqcb					__kernel_free_dtqcb
#define _initialize_dataqueue		__kernel_initialize_dataqueue
#define _enqueue_data				__kernel_enqueue_data
#define _force_enqueue_data			__kernel_force_enqueue_data
#define _dequeue_data				__kernel_dequeue_data
#define _send_data					__kernel_send_data
#define _force_send_data			__kernel_force_send_data
#define _receive_data				__kernel_receive_data

/*
 *  pridataq.c
 */
#define _free_pdqcb					__kernel_free_pdqcb
#define _initialize_pridataq		__kernel_initialize_pridataq
#define _enqueue_pridata			__kernel_enqueue_pridata
#define _dequeue_pridata			__kernel_dequeue_pridata
#define _send_pridata				__kernel_send_pridata
#define _receive_pridata			__kernel_receive_pridata

/*
 *  mailbox.c
 */
#define _free_mbxcb					__kernel_free_mbxcb
#define _initialize_mailbox			__kernel_initialize_mailbox

/*
 *  mempfix.c
 */
#define _free_mpfcb					__kernel_free_mpfcb
#define _initialize_mempfix			__kernel_initialize_mempfix
#define _get_mpf_block				__kernel_get_mpf_block

/*
 *  cyclic.c
 */
#define _free_cyccb					__kernel_free_cyccb
#define _initialize_cyclic			__kernel_initialize_cyclic
#define _call_cychdr				__kernel_call_cychdr

/*
 *  alarm.c
 */
#define _free_almcb					__kernel_free_almcb
#define _initialize_alarm			__kernel_initialize_alarm
#define _call_almhdr				__kernel_call_almhdr

/*
 *  interrupt.c
 */
#define _free_isrcb					__kernel_free_isrcb
#define _initialize_isr				__kernel_initialize_isr
#define _call_isr					__kernel_call_isr
#define _initialize_interrupt		__kernel_initialize_interrupt

/*
 *  exception.c
 */
#define _initialize_exception		__kernel_initialize_exception

/*
 *  kernel_cfg.c
 */
#define _initialize_object			__kernel_initialize_object
#define _call_inirtn				__kernel_call_inirtn
#define _call_terrtn				__kernel_call_terrtn
#define _tmax_tskid					__kernel_tmax_tskid
#define _tmax_stskid				__kernel_tmax_stskid
#define _tinib_table				__kernel_tinib_table
#define _atinib_table				__kernel_atinib_table
#define _tcb_table					__kernel_tcb_table
#define _torder_table				__kernel_torder_table
#define _tmax_semid					__kernel_tmax_semid
#define _tmax_ssemid				__kernel_tmax_ssemid
#define _seminib_table				__kernel_seminib_table
#define _aseminib_table				__kernel_aseminib_table
#define _semcb_table				__kernel_semcb_table
#define _tmax_flgid					__kernel_tmax_flgid
#define _tmax_sflgid				__kernel_tmax_sflgid
#define _flginib_table				__kernel_flginib_table
#define _aflginib_table				__kernel_aflginib_table
#define _flgcb_table				__kernel_flgcb_table
#define _tmax_dtqid					__kernel_tmax_dtqid
#define _tmax_sdtqid				__kernel_tmax_sdtqid
#define _dtqinib_table				__kernel_dtqinib_table
#define _adtqinib_table				__kernel_adtqinib_table
#define _dtqcb_table				__kernel_dtqcb_table
#define _tmax_pdqid					__kernel_tmax_pdqid
#define _tmax_spdqid				__kernel_tmax_spdqid
#define _pdqinib_table				__kernel_pdqinib_table
#define _apdqinib_table				__kernel_apdqinib_table
#define _pdqcb_table				__kernel_pdqcb_table
#define _tmax_mbxid					__kernel_tmax_mbxid
#define _tmax_smbxid				__kernel_tmax_smbxid
#define _mbxinib_table				__kernel_mbxinib_table
#define _ambxinib_table				__kernel_ambxinib_table
#define _mbxcb_table				__kernel_mbxcb_table
#define _tmax_mpfid					__kernel_tmax_mpfid
#define _tmax_smpfid				__kernel_tmax_smpfid
#define _mpfinib_table				__kernel_mpfinib_table
#define _ampfinib_table				__kernel_ampfinib_table
#define _mpfcb_table				__kernel_mpfcb_table
#define _tmax_cycid					__kernel_tmax_cycid
#define _tmax_scycid				__kernel_tmax_scycid
#define _cycinib_table				__kernel_cycinib_table
#define _acycinib_table				__kernel_acycinib_table
#define _cyccb_table				__kernel_cyccb_table
#define _tmax_almid					__kernel_tmax_almid
#define _tmax_salmid				__kernel_tmax_salmid
#define _alminib_table				__kernel_alminib_table
#define _aalminib_table				__kernel_aalminib_table
#define _almcb_table				__kernel_almcb_table
#define _tnum_isr_queue				__kernel_tnum_isr_queue
#define _isr_queue_table			__kernel_isr_queue_table
#define _isr_queue_list				__kernel_isr_queue_list
#define _tmax_isrid					__kernel_tmax_isrid
#define _tnum_sisr					__kernel_tnum_sisr
#define _sisrinib_table				__kernel_sisrinib_table
#define _aisrinib_table				__kernel_aisrinib_table
#define _isrcb_table				__kernel_isrcb_table
#define _tnum_inhno					__kernel_tnum_inhno
#define _inhinib_table				__kernel_inhinib_table
#define _tnum_intno					__kernel_tnum_intno
#define _intinib_table				__kernel_intinib_table
#define _tnum_excno					__kernel_tnum_excno
#define _excinib_table				__kernel_excinib_table
#define _tmevt_heap					__kernel_tmevt_heap
#define _istksz						__kernel_istksz
#define _istk						__kernel_istk
#define _istkpt						__kernel_istkpt
#define _kmmsz						__kernel_kmmsz
#define _kmm						__kernel_kmm


#endif /* TOPPERS_LABEL_ASM */

#include "target_rename.h"

#endif /* TOPPERS_KERNEL_RENAME_H */
