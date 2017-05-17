/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      High Reliable system Profile Kernel
 * 
 *  Copyright (C) 2005-2011 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 * 
 *  上記著作権者は，以下の(1)～(4)の条件を満たす場合に限り，本ソフトウェ
 *  ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
 *  変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
 *  (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *      権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *      スコード中に含まれていること．
 *  (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
 *      用できる形で再配布する場合には，再配布に伴うドキュメント（利用
 *      者マニュアルなど）に，上記の著作権表示，この利用条件および下記
 *      の無保証規定を掲載すること．
 *  (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
 *      用できない形で再配布する場合には，次のいずれかの条件を満たすこ
 *      と．
 *    (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
 *        作権表示，この利用条件および下記の無保証規定を掲載すること．
 *    (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
 *        報告すること．
 *  (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *      害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
 *      また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
 *      由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
 *      免責すること．
 * 
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *  の責任を負わない．
 * 
 *  @(#) $Id: bit_mutex.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		ミューテックス機能の整合性検査
 */

#include "kernel/kernel_impl.h"
#include "kernel/task.h"
#include "kernel/mutex.h"

/*
 *  ミューテックスIDからミューテックス管理ブロックを取り出すためのマク
 *  ロ（mutex.cより）
 */
#define INDEX_MTX(mtxid)	((uint_t)((mtxid) - TMIN_MTXID))
#define get_mtxcb(mtxid)	(&(mtxcb_table[INDEX_MTX(mtxid)]))

/*
 *  ミューテックス管理ブロック中のmutex_queueへのポインタから，ミューテッ
 *  クス管理ブロックへのポインタを取り出すためのマクロ（mutex.cより）
 */
#define MTXCB_QUEUE(p_queue) \
			((MTXCB *)(((char *) p_queue) - offsetof(MTXCB, mutex_queue)))

/*
 *  ミューテックスのプロトコルを判断するマクロ（mutex.cより）
 */
#define MTXPROTO_MASK			0x03U
#define MTXPROTO(p_mtxcb)		((p_mtxcb)->p_mtxinib->mtxatr & MTXPROTO_MASK)
#define MTX_CEILING(p_mtxcb)	(MTXPROTO(p_mtxcb) == TA_CEILING)

/*
 *   エラーコードの定義
 */
#define E_SYS_LINENO	ERCD(E_SYS, -(__LINE__))

/*
 *  管理ブロックのアドレスの正当性のチェック
 */
#define VALID_TCB(p_tcb) \
		((((char *) p_tcb) - ((char *) tcb_table)) % sizeof(TCB) == 0 \
			&& TMIN_TSKID <= TSKID(p_tcb) && TSKID(p_tcb) <= tmax_tskid)

#define VALID_MTXCB(p_mtxcb) \
		((((char *) p_mtxcb) - ((char *) mtxcb_table)) % sizeof(MTXCB) == 0 \
			&& TMIN_MTXID <= MTXID(p_mtxcb) && MTXID(p_mtxcb) <= tmax_mtxid)
				
/*
 *  キューのチェックのための関数
 *
 *  p_queueにp_entryが含まれているかを調べる．含まれていればtrue，含ま
 *  れていない場合にはfalseを返す．ダブルリンクの不整合の場合にも，
 *  falseを返す．
 */
static bool_t
in_queue(QUEUE *p_queue, QUEUE *p_entry)
{
	QUEUE	*p_current, *p_next;

	p_current = p_queue->p_next;
	if (p_current->p_prev != p_queue) {
		return(false);					/* ダブルリンクの不整合 */
	}
	while (p_current != p_queue) {
		if (p_current == p_entry) {
			return(true);				/* p_entryが含まれていた */
		}

		/*
		 *  キューの次の要素に進む
		 */
		p_next = p_current->p_next;
		if (p_next->p_prev != p_current) {
			return(false);				 /* ダブルリンクの不整合 */
		}
		p_current = p_next;
	}
	return(false);
}

/*
 *  タスク毎の検査
 */
static ER
bit_mutex_task(ID tskid)
{
	TCB			*p_tcb;
	MTXCB		*p_mtxcb;
	QUEUE		*p_queue, *p_next;
	uint_t		pri;

	if (!(TMIN_TSKID <= (tskid) && (tskid) <= tmax_tskid)) {
		return(E_ID);
	}
	p_tcb = get_tcb(tskid);
	pri = p_tcb->bpriority;

	/*
	 *  タスクがロックしているミューテックスのキューの検査
	 */
	p_queue = p_tcb->mutex_queue.p_next;
	if (p_queue->p_prev != &(p_tcb->mutex_queue)) {
		return(E_SYS_LINENO);
	}
	while (p_queue != &(p_tcb->mutex_queue)) {
		p_mtxcb = MTXCB_QUEUE(p_queue);
		if (!VALID_MTXCB(p_mtxcb)) {
			return(E_SYS_LINENO);
		}

		/*
		 *  ミューテックスをロックしているタスクのチェック
		 */
		if (p_mtxcb->p_loctsk != p_tcb) {
			return(E_SYS_LINENO);
		}

		/*
		 *  現在優先度の計算
		 */
		if (MTXPROTO(p_mtxcb)) {
			if (p_mtxcb->p_mtxinib->ceilpri < pri) {
				pri = p_mtxcb->p_mtxinib->ceilpri;
			}
		}

		/*
		 *  キューの次の要素に進む
		 */
		p_next = p_queue->p_next;
		if (p_next->p_prev != p_queue) {
			return(E_SYS_LINENO);
		}
		p_queue = p_next;
	}

	/*
	 *  現在優先度の検査
	 */
	if (p_tcb->priority != pri) {
		return(E_SYS_LINENO);
	}

	/*
	 *  タスクが待っているミューテックスに関する検査
	 */
	if (TSTAT_WAIT_MTX(p_tcb->tstat)) {
		p_mtxcb = ((WINFO_MTX *)(p_tcb->p_winfo))->p_mtxcb;
		if (!VALID_MTXCB(p_mtxcb)) {
			return(E_SYS_LINENO);
		}
		if (!in_queue(&(p_mtxcb->wait_queue), &(p_tcb->task_queue))) {
			return(E_SYS_LINENO);
		}
	}
	return(E_OK);
}

/*
 *  ミューテックス毎の検査
 */
static ER
bit_mutex_mutex(ID mtxid)
{
	MTXCB		*p_mtxcb;
	TCB			*p_tcb;
	QUEUE		*p_queue, *p_next;
	uint_t		pri;

	if (!(TMIN_MTXID <= (mtxid) && (mtxid) <= tmax_mtxid)) {
		return(E_ID);
	}
	p_mtxcb = get_mtxcb(mtxid);

	/*
	 *  初期化ブロックへのポインタの検査
	 */
	if (p_mtxcb->p_mtxinib != &(mtxinib_table[INDEX_MTX(mtxid)])) {
		return(E_SYS_LINENO);
	}

	/*
	 *  ミューテックス待ちキューの検査
	 */
	p_queue = p_mtxcb->wait_queue.p_next;
	if (p_queue->p_prev != &(p_mtxcb->wait_queue)) {
		return(E_SYS_LINENO);
	}
	pri = TMIN_TPRI;
	while (p_queue != &(p_mtxcb->wait_queue)) {
		p_tcb = (TCB *) p_queue;
		if (!VALID_TCB(p_tcb)) {
			return(E_SYS_LINENO);
		}

		/*
		 *  キューがタスク優先度順になっているかの検査
		 */
		if (MTXPROTO(p_mtxcb) != TA_NULL) {
			if (p_tcb->priority < pri) {
				return(E_SYS_LINENO);
			}
		}
		pri = p_tcb->priority;

		/*
		 *  タスク状態の検査
		 *
		 *  ミューテックス待ち状態のタスクの検査は，タスク毎の検査で行っ
		 *  ているため，ここでは行わない．
		 */
		if (!TSTAT_WAIT_MTX(p_tcb->tstat)) {
			return(E_SYS_LINENO);
		}

		/*
		 *  優先度上限の検査
		 */
		if (MTXPROTO(p_mtxcb) == TA_CEILING) {
			if (p_tcb->bpriority < p_mtxcb->p_mtxinib->ceilpri) {
				return(E_SYS_LINENO);
			}
		}

		/*
		 *  キューの次の要素に進む
		 */
		p_next = p_queue->p_next;
		if (p_next->p_prev != p_queue) {
			return(E_SYS_LINENO);
		}
		p_queue = p_next;
	}

	/*
	 *  ミューテックスをロックしているタスクの検査
	 */
	p_tcb = p_mtxcb->p_loctsk;
	if (p_tcb == NULL) {
		/*
		 *  ミューテックスがロックされていない時
		 */
		if (!queue_empty(&(p_mtxcb->wait_queue))) {
			return(E_SYS_LINENO);
		}
	}
	else {
		/*
		 *  ミューテックスがロックされている時
		 *
		 *  ミューテックスをロックしているタスクの検査は，タスク毎の検
		 *  査で行っているため，ここでは行わない．
		 */
		if (!VALID_TCB(p_tcb)) {
			return(E_SYS_LINENO);
		}
		if (!in_queue(&(p_tcb->mutex_queue), &(p_mtxcb->mutex_queue))) {
			return(E_SYS_LINENO);
		}

		/*
		 *  優先度上限の検査
		 */
		if (MTXPROTO(p_mtxcb) == TA_CEILING) {
			if (p_tcb->bpriority < p_mtxcb->p_mtxinib->ceilpri) {
				return(E_SYS_LINENO);
			}
		}
	}
	return(E_OK);
}

/*
 *  整合性検査ルーチン本体
 */
ER
bit_mutex(void)
{
	ID		tskid, mtxid;
	ER		ercd;

	/*
	 *  タスク毎の検査
	 */
	for (tskid = TMIN_TSKID; tskid <= tmax_tskid; tskid++) {
		ercd = bit_mutex_task(tskid);
		if (ercd != E_OK) {
			return(ercd);
		}
	}

	/*
	 *  ミューテックス毎の検査
	 */
	for (mtxid = TMIN_MTXID; mtxid <= tmax_mtxid; mtxid++) {
		ercd = bit_mutex_mutex(mtxid);
		if (ercd != E_OK) {
			return(ercd);
		}
	}
	return(E_OK);
}
