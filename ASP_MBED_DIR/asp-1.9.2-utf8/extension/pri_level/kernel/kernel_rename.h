/* This file is generated from kernel_rename.def by genrename. */

#ifndef TOPPERS_KERNEL_RENAME_H
#define TOPPERS_KERNEL_RENAME_H

/*
 *  startup.c
 */
#define kerflg						_kernel_kerflg
#define exit_kernel					_kernel_exit_kernel

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
#define ready_primap1				_kernel_ready_primap1
#define ready_primap2				_kernel_ready_primap2
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
#define initialize_semaphore		_kernel_initialize_semaphore

/*
 *  eventflag.c
 */
#define initialize_eventflag		_kernel_initialize_eventflag
#define check_flg_cond				_kernel_check_flg_cond

/*
 *  dataqueue.c
 */
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
#define initialize_pridataq			_kernel_initialize_pridataq
#define enqueue_pridata				_kernel_enqueue_pridata
#define dequeue_pridata				_kernel_dequeue_pridata
#define send_pridata				_kernel_send_pridata
#define receive_pridata				_kernel_receive_pridata

/*
 *  mailbox.c
 */
#define initialize_mailbox			_kernel_initialize_mailbox

/*
 *  mempfix.c
 */
#define initialize_mempfix			_kernel_initialize_mempfix
#define get_mpf_block				_kernel_get_mpf_block

/*
 *  cyclic.c
 */
#define initialize_cyclic			_kernel_initialize_cyclic
#define call_cychdr					_kernel_call_cychdr

/*
 *  alarm.c
 */
#define initialize_alarm			_kernel_initialize_alarm
#define call_almhdr					_kernel_call_almhdr

/*
 *  interrupt.c
 */
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
#define tinib_table					_kernel_tinib_table
#define torder_table				_kernel_torder_table
#define tcb_table					_kernel_tcb_table
#define tmax_semid					_kernel_tmax_semid
#define seminib_table				_kernel_seminib_table
#define semcb_table					_kernel_semcb_table
#define tmax_flgid					_kernel_tmax_flgid
#define flginib_table				_kernel_flginib_table
#define flgcb_table					_kernel_flgcb_table
#define tmax_dtqid					_kernel_tmax_dtqid
#define dtqinib_table				_kernel_dtqinib_table
#define dtqcb_table					_kernel_dtqcb_table
#define tmax_pdqid					_kernel_tmax_pdqid
#define pdqinib_table				_kernel_pdqinib_table
#define pdqcb_table					_kernel_pdqcb_table
#define tmax_mbxid					_kernel_tmax_mbxid
#define mbxinib_table				_kernel_mbxinib_table
#define mbxcb_table					_kernel_mbxcb_table
#define tmax_mpfid					_kernel_tmax_mpfid
#define mpfinib_table				_kernel_mpfinib_table
#define mpfcb_table					_kernel_mpfcb_table
#define tmax_cycid					_kernel_tmax_cycid
#define cycinib_table				_kernel_cycinib_table
#define cyccb_table					_kernel_cyccb_table
#define tmax_almid					_kernel_tmax_almid
#define alminib_table				_kernel_alminib_table
#define almcb_table					_kernel_almcb_table
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


#ifdef TOPPERS_LABEL_ASM

/*
 *  startup.c
 */
#define _kerflg						__kernel_kerflg
#define _exit_kernel				__kernel_exit_kernel

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
#define _ready_primap1				__kernel_ready_primap1
#define _ready_primap2				__kernel_ready_primap2
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
#define _initialize_semaphore		__kernel_initialize_semaphore

/*
 *  eventflag.c
 */
#define _initialize_eventflag		__kernel_initialize_eventflag
#define _check_flg_cond				__kernel_check_flg_cond

/*
 *  dataqueue.c
 */
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
#define _initialize_pridataq		__kernel_initialize_pridataq
#define _enqueue_pridata			__kernel_enqueue_pridata
#define _dequeue_pridata			__kernel_dequeue_pridata
#define _send_pridata				__kernel_send_pridata
#define _receive_pridata			__kernel_receive_pridata

/*
 *  mailbox.c
 */
#define _initialize_mailbox			__kernel_initialize_mailbox

/*
 *  mempfix.c
 */
#define _initialize_mempfix			__kernel_initialize_mempfix
#define _get_mpf_block				__kernel_get_mpf_block

/*
 *  cyclic.c
 */
#define _initialize_cyclic			__kernel_initialize_cyclic
#define _call_cychdr				__kernel_call_cychdr

/*
 *  alarm.c
 */
#define _initialize_alarm			__kernel_initialize_alarm
#define _call_almhdr				__kernel_call_almhdr

/*
 *  interrupt.c
 */
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
#define _tinib_table				__kernel_tinib_table
#define _torder_table				__kernel_torder_table
#define _tcb_table					__kernel_tcb_table
#define _tmax_semid					__kernel_tmax_semid
#define _seminib_table				__kernel_seminib_table
#define _semcb_table				__kernel_semcb_table
#define _tmax_flgid					__kernel_tmax_flgid
#define _flginib_table				__kernel_flginib_table
#define _flgcb_table				__kernel_flgcb_table
#define _tmax_dtqid					__kernel_tmax_dtqid
#define _dtqinib_table				__kernel_dtqinib_table
#define _dtqcb_table				__kernel_dtqcb_table
#define _tmax_pdqid					__kernel_tmax_pdqid
#define _pdqinib_table				__kernel_pdqinib_table
#define _pdqcb_table				__kernel_pdqcb_table
#define _tmax_mbxid					__kernel_tmax_mbxid
#define _mbxinib_table				__kernel_mbxinib_table
#define _mbxcb_table				__kernel_mbxcb_table
#define _tmax_mpfid					__kernel_tmax_mpfid
#define _mpfinib_table				__kernel_mpfinib_table
#define _mpfcb_table				__kernel_mpfcb_table
#define _tmax_cycid					__kernel_tmax_cycid
#define _cycinib_table				__kernel_cycinib_table
#define _cyccb_table				__kernel_cyccb_table
#define _tmax_almid					__kernel_tmax_almid
#define _alminib_table				__kernel_alminib_table
#define _almcb_table				__kernel_almcb_table
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


#endif /* TOPPERS_LABEL_ASM */

#include "target_rename.h"

#endif /* TOPPERS_KERNEL_RENAME_H */
