/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
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
 *  $Id: task.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		タスク管理モジュール
 */

#include "kernel_impl.h"
#include "wait.h"
#include "task.h"

/*
 *  トレースログマクロのデフォルト定義
 */
#ifndef LOG_TEX_ENTER
#define LOG_TEX_ENTER(p_tcb, texptn)
#endif /* LOG_TEX_ENTER */

#ifndef LOG_TEX_LEAVE
#define LOG_TEX_LEAVE(p_tcb)
#endif /* LOG_TEX_LEAVE */

#ifdef TOPPERS_tskini

/*
 *  実行状態のタスク
 */
TCB		*p_runtsk;

/*
 *  最高優先順位のタスク
 */
TCB		*p_schedtsk;

/*
 *  タスクディスパッチ／タスク例外処理ルーチン起動要求フラグ
 */
bool_t	reqflg;

/*
 *  割込み優先度マスク全解除状態
 */
bool_t	ipmflg;

/*
 *  ディスパッチ禁止状態
 */
bool_t	disdsp;

/*
 *  タスクディスパッチ可能状態
 */
bool_t	dspflg;

/*
 *  レディキュー
 */
QUEUE	ready_queue[TNUM_TPRI];

/*
 *  レディキューサーチのためのビットマップ
 */
uint16_t	ready_primap;

/*
 *  使用していないTCBのリスト
 */
QUEUE	free_tcb;

/*
 *  タスク管理モジュールの初期化
 */
void
initialize_task(void)
{
	uint_t	i, j;
	TCB		*p_tcb;
	TINIB	*p_tinib;

	p_runtsk = NULL;
	p_schedtsk = NULL;
	reqflg = false;
	ipmflg = true;
	disdsp = false;
	dspflg = true;

	for (i = 0; i < TNUM_TPRI; i++) {
		queue_initialize(&(ready_queue[i]));
	}
	ready_primap = 0U;

	for (i = 0; i < tnum_stsk; i++) {
		j = INDEX_TSK(torder_table[i]);
		p_tcb = &(tcb_table[j]);
		p_tcb->p_tinib = &(tinib_table[j]);
		p_tcb->actque = false;
		make_dormant(p_tcb);
		queue_initialize(&(p_tcb->mutex_queue));
		if ((p_tcb->p_tinib->tskatr & TA_ACT) != 0U) {
			(void) make_active(p_tcb);
		}
	}
	queue_initialize(&free_tcb);
	for (j = 0; i < tnum_tsk; i++, j++) {
		p_tcb = &(tcb_table[i]);
		p_tinib = &(atinib_table[j]);
		p_tinib->tskatr = TA_NOEXS;
		p_tcb->p_tinib = ((const TINIB *) p_tinib);
		queue_insert_prev(&free_tcb, &(p_tcb->task_queue));
	}
}

#endif /* TOPPERS_tskini */

/*
 *  ビットマップサーチ関数
 *
 *  bitmap内の1のビットの内，最も下位（右）のものをサーチし，そのビッ
 *  ト番号を返す．ビット番号は，最下位ビットを0とする．bitmapに0を指定
 *  してはならない．この関数では，bitmapが16ビットであることを仮定し，
 *  uint16_t型としている．
 *
 *  ビットサーチ命令を持つプロセッサでは，ビットサーチ命令を使うように
 *  書き直した方が効率が良い場合がある．このような場合には，ターゲット
 *  依存部でビットサーチ命令を使ったbitmap_searchを定義し，
 *  OMIT_BITMAP_SEARCHをマクロ定義すればよい．また，ビットサーチ命令の
 *  サーチ方向が逆などの理由で優先度とビットとの対応を変更したい場合に
 *  は，PRIMAP_BITをマクロ定義すればよい．
 *
 *  また，ライブラリにffsがあるなら，次のように定義してライブラリ関数を
 *  使った方が効率が良い可能性もある．
 *		#define	bitmap_search(bitmap) (ffs(bitmap) - 1)
 */
#ifndef PRIMAP_BIT
#define	PRIMAP_BIT(pri)		(1U << (pri))
#endif /* PRIMAP_BIT */

#ifndef OMIT_BITMAP_SEARCH

static const unsigned char bitmap_search_table[] = { 0, 1, 0, 2, 0, 1, 0,
												3, 0, 1, 0, 2, 0, 1, 0 };

Inline uint_t
bitmap_search(uint16_t bitmap)
{
	uint_t	n = 0U;

	assert(bitmap != 0U);
	if ((bitmap & 0x00ffU) == 0U) {
		bitmap >>= 8;
		n += 8;
	}
	if ((bitmap & 0x0fU) == 0U) {
		bitmap >>= 4;
		n += 4;
	}
	return(n + bitmap_search_table[(bitmap & 0x0fU) - 1]);
}

#endif /* OMIT_BITMAP_SEARCH */

/*
 *  優先度ビットマップが空かのチェック
 */
Inline bool_t
primap_empty(void)
{
	return(ready_primap == 0U);
}

/*
 *  優先度ビットマップのサーチ
 */
Inline uint_t
primap_search(void)
{
	return(bitmap_search(ready_primap));
}

/*
 *  優先度ビットマップのセット
 */
Inline void
primap_set(uint_t pri)
{
	ready_primap |= PRIMAP_BIT(pri);
}

/*
 *  優先度ビットマップのクリア
 */
Inline void
primap_clear(uint_t pri)
{
	ready_primap &= ~PRIMAP_BIT(pri);
}

/*
 *  最高優先順位タスクのサーチ
 */
#ifdef TOPPERS_tsksched

TCB *
search_schedtsk(void)
{
	uint_t	schedpri;

	schedpri = primap_search();
	return((TCB *)(ready_queue[schedpri].p_next));
}

#endif /* TOPPERS_tsksched */

/*
 *  実行できる状態への遷移
 *
 *  最高優先順位のタスクを更新するのは，実行できるタスクがなかった場合
 *  と，p_tcbの優先度が最高優先順位のタスクの優先度よりも高い場合であ
 *  る．
 */
#ifdef TOPPERS_tskrun

bool_t
make_runnable(TCB *p_tcb)
{
	uint_t	pri = p_tcb->priority;

	queue_insert_prev(&(ready_queue[pri]), &(p_tcb->task_queue));
	primap_set(pri);

	if (p_schedtsk == (TCB *) NULL || pri < p_schedtsk->priority) {
		p_schedtsk = p_tcb;
		return(dspflg);
	}
	return(false);
}

#endif /* TOPPERS_tskrun */

/*
 *  実行できる状態から他の状態への遷移
 *
 *  最高優先順位のタスクを更新するのは，p_tcbが最高優先順位のタスクで
 *  あった場合である．p_tcbと同じ優先度のタスクが他にある場合は，p_tcb
 *  の次のタスクが最高優先順位になる．そうでない場合は，レディキューを
 *  サーチする必要がある．
 */
#ifdef TOPPERS_tsknrun

bool_t
make_non_runnable(TCB *p_tcb)
{
	uint_t	pri = p_tcb->priority;
	QUEUE	*p_queue = &(ready_queue[pri]);

	queue_delete(&(p_tcb->task_queue));
	if (queue_empty(p_queue)) {
		primap_clear(pri);
		if (p_schedtsk == p_tcb) {
			p_schedtsk = primap_empty() ? (TCB *) NULL : search_schedtsk();
			return(dspflg);
		}
	}
	else {
		if (p_schedtsk == p_tcb) {
			p_schedtsk = (TCB *)(p_queue->p_next);
			return(dspflg);
		}
	}
	return(false);
}

#endif /* TOPPERS_tsknrun */

/*
 *  休止状態への遷移
 */
#ifdef TOPPERS_tskdmt

void
make_dormant(TCB *p_tcb)
{
	p_tcb->tstat = TS_DORMANT;
	p_tcb->bpriority = p_tcb->p_tinib->ipriority;	
	p_tcb->priority = p_tcb->p_tinib->ipriority;
	p_tcb->wupque = false;
	p_tcb->enatex = false;
	p_tcb->texptn = 0U;
	LOG_TSKSTAT(p_tcb);
}

#endif /* TOPPERS_tskdmt */

/*
 *  休止状態から実行できる状態への遷移
 */
#ifdef TOPPERS_tskact

bool_t
make_active(TCB *p_tcb)
{
	activate_context(p_tcb);
	p_tcb->tstat = TS_RUNNABLE;
	LOG_TSKSTAT(p_tcb);
	return(make_runnable(p_tcb));
}

#endif /* TOPPERS_tskact */

/*
 *  タスクの優先度の変更
 *
 *  タスクが実行できる状態の場合には，レディキューの中での位置を変更す
 *  る．オブジェクトの待ちキューの中で待ち状態になっている場合には，待
 *  ちキューの中での位置を変更する．
 *
 *  最高優先順位のタスクを更新するのは，(1) p_tcbが最高優先順位のタス
 *  クであって，その優先度を下げた場合，(2) p_tcbが最高優先順位のタス
 *  クではなく，変更後の優先度が最高優先順位のタスクの優先度よりも高い
 *  場合である．(1)の場合には，レディキューをサーチする必要がある．
 */
#ifdef TOPPERS_tskpri

bool_t
change_priority(TCB *p_tcb, uint_t newpri, bool_t mtxmode)
{
	uint_t	oldpri;

	oldpri = p_tcb->priority;
	p_tcb->priority = newpri;

	if (TSTAT_RUNNABLE(p_tcb->tstat)) {
		/*
		 *  タスクが実行できる状態の場合
		 */
		queue_delete(&(p_tcb->task_queue));
		if (queue_empty(&(ready_queue[oldpri]))) {
			primap_clear(oldpri);
		}
		if (mtxmode) {
			queue_insert_next(&(ready_queue[newpri]), &(p_tcb->task_queue));
		}
		else {
			queue_insert_prev(&(ready_queue[newpri]), &(p_tcb->task_queue));
		}
		primap_set(newpri);

		if (p_schedtsk == p_tcb) {
			if (newpri >= oldpri) {
				p_schedtsk = search_schedtsk();
				return(p_schedtsk != p_tcb && dspflg);
			}
		}
		else {
			if (mtxmode ? newpri <= p_schedtsk->priority
						: newpri < p_schedtsk->priority) {
				p_schedtsk = p_tcb;
				return(dspflg);
			}
		}
	}
	else {
		if (TSTAT_WAIT_WOBJCB(p_tcb->tstat)) {
			/*
			 *  タスクが，同期・通信オブジェクトの管理ブロックの共通部
			 *  分（WOBJCB）の待ちキューにつながれている場合
			 */
			wobj_change_priority(((WINFO_WOBJ *)(p_tcb->p_winfo))->p_wobjcb,
																	p_tcb);
		}
	}
	return(false);
}

#endif /* TOPPERS_tskpri */

/*
 *  レディキューの回転
 *
 *  最高優先順位のタスクを更新するのは，最高優先順位のタスクがタスクキ
 *  ューの末尾に移動した場合である．
 */
#ifdef TOPPERS_tskrot

bool_t
rotate_ready_queue(uint_t pri)
{
	QUEUE	*p_queue = &(ready_queue[pri]);
	QUEUE	*p_entry;

	if (!queue_empty(p_queue) && p_queue->p_next->p_next != p_queue) {
		p_entry = queue_delete_next(p_queue);
		queue_insert_prev(p_queue, p_entry);
		if (p_schedtsk == (TCB *) p_entry) {
			p_schedtsk = (TCB *)(p_queue->p_next);
			return(dspflg);
		}
	}
	return(false);
}

#endif /* TOPPERS_tskrot */

/*
 *  タスク例外処理ルーチンの呼出し
 *
 *  ASPカーネルでは，タスク例外処理ルーチン内でCPUロック状態に遷移し，
 *  元の状態に戻さずにリターンした場合，カーネルが元の状態に戻す．
 */
#ifdef TOPPERS_tsktex

void
call_texrtn(void)
{
	TEXPTN	texptn;
	bool_t	saved_disdsp;

	saved_disdsp = disdsp;
	p_runtsk->enatex = false;
	do {
		texptn = p_runtsk->texptn;
		p_runtsk->texptn = 0U;

		t_unlock_cpu();
		LOG_TEX_ENTER(p_runtsk, texptn);
		(*((TEXRTN)(p_runtsk->p_tinib->texrtn)))(texptn,
												p_runtsk->p_tinib->exinf);
		LOG_TEX_LEAVE(p_runtsk);
		if (!t_sense_lock()) {
			t_lock_cpu();
		}
		if (!ipmflg) {
			t_set_ipm(TIPM_ENAALL);
			ipmflg = true;
		}
		disdsp = saved_disdsp;
		dspflg = !disdsp;
		p_runtsk->enatex = false;
		if (p_runtsk != p_schedtsk && dspflg) {
			/*
			 *  ここでdispatchを呼び出す処理は，相互再帰呼出しになって
			 *  いるが，dispatchを呼ぶ前にp_runtsk->enatexをfalseにして
			 *  おけば支障がない．その理由については，「TOPPERS/ASP カー
			 *  ネル 設計メモ」を参照のこと．
			 */
			dispatch();
		}
	} while (p_runtsk->texptn != 0U);
	p_runtsk->enatex = true;
}

/*
 *  タスク例外処理ルーチンの起動
 */
#ifndef OMIT_CALLTEX

void
calltex(void)
{
	if (p_runtsk->enatex && p_runtsk->texptn != 0U && ipmflg) {
		call_texrtn();
	}
}

#endif /* OMIT_CALLTEX */
#endif /* TOPPERS_tsktex */
