/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2005-2010 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: task_sync.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		タスク付属同期機能
 */

#include "kernel_impl.h"
#include "check.h"
#include "task.h"
#include "wait.h"

/*
 *  トレースログマクロのデフォルト定義
 */
#ifndef LOG_SLP_TSK_ENTER
#define LOG_SLP_TSK_ENTER()
#endif /* LOG_SLP_TSK_ENTER */

#ifndef LOG_SLP_TSK_LEAVE
#define LOG_SLP_TSK_LEAVE(ercd)
#endif /* LOG_SLP_TSK_LEAVE */

#ifndef LOG_TSLP_TSK_ENTER
#define LOG_TSLP_TSK_ENTER(tmout)
#endif /* LOG_TSLP_TSK_ENTER */

#ifndef LOG_TSLP_TSK_LEAVE
#define LOG_TSLP_TSK_LEAVE(ercd)
#endif /* LOG_TSLP_TSK_LEAVE */

#ifndef LOG_WUP_TSK_ENTER
#define LOG_WUP_TSK_ENTER(tskid)
#endif /* LOG_WUP_TSK_ENTER */

#ifndef LOG_WUP_TSK_LEAVE
#define LOG_WUP_TSK_LEAVE(ercd)
#endif /* LOG_WUP_TSK_LEAVE */

#ifndef LOG_IWUP_TSK_ENTER
#define LOG_IWUP_TSK_ENTER(tskid)
#endif /* LOG_IWUP_TSK_ENTER */

#ifndef LOG_IWUP_TSK_LEAVE
#define LOG_IWUP_TSK_LEAVE(ercd)
#endif /* LOG_IWUP_TSK_LEAVE */

#ifndef LOG_CAN_WUP_ENTER
#define LOG_CAN_WUP_ENTER(tskid)
#endif /* LOG_CAN_WUP_ENTER */

#ifndef LOG_CAN_WUP_LEAVE
#define LOG_CAN_WUP_LEAVE(ercd)
#endif /* LOG_CAN_WUP_LEAVE */

#ifndef LOG_REL_WAI_ENTER
#define LOG_REL_WAI_ENTER(tskid)
#endif /* LOG_REL_WAI_ENTER */

#ifndef LOG_REL_WAI_LEAVE
#define LOG_REL_WAI_LEAVE(ercd)
#endif /* LOG_REL_WAI_LEAVE */

#ifndef LOG_IREL_WAI_ENTER
#define LOG_IREL_WAI_ENTER(tskid)
#endif /* LOG_IREL_WAI_ENTER */

#ifndef LOG_IREL_WAI_LEAVE
#define LOG_IREL_WAI_LEAVE(ercd)
#endif /* LOG_IREL_WAI_LEAVE */

#ifndef LOG_SUS_TSK_ENTER
#define LOG_SUS_TSK_ENTER(tskid)
#endif /* LOG_SUS_TSK_ENTER */

#ifndef LOG_SUS_TSK_LEAVE
#define LOG_SUS_TSK_LEAVE(ercd)
#endif /* LOG_SUS_TSK_LEAVE */

#ifndef LOG_RSM_TSK_ENTER
#define LOG_RSM_TSK_ENTER(tskid)
#endif /* LOG_RSM_TSK_ENTER */

#ifndef LOG_RSM_TSK_LEAVE
#define LOG_RSM_TSK_LEAVE(ercd)
#endif /* LOG_RSM_TSK_LEAVE */

#ifndef LOG_DLY_TSK_ENTER
#define LOG_DLY_TSK_ENTER(dlytim)
#endif /* LOG_DLY_TSK_ENTER */

#ifndef LOG_DLY_TSK_LEAVE
#define LOG_DLY_TSK_LEAVE(ercd)
#endif /* LOG_DLY_TSK_LEAVE */

/*
 *  起床待ち
 */
#ifdef TOPPERS_slp_tsk

ER
slp_tsk(void)
{
	WINFO	winfo;
	ER		ercd;

	LOG_SLP_TSK_ENTER();
	CHECK_DISPATCH();

	t_lock_cpu();
	if (p_runtsk->wupque) {
		p_runtsk->wupque = false;
		ercd = E_OK;
	}
	else {
		p_runtsk->tstat = (TS_WAITING | TS_WAIT_SLP);
		make_wait(&winfo);
		LOG_TSKSTAT(p_runtsk);
		dispatch();
		ercd = winfo.wercd;
	}
	t_unlock_cpu();

  error_exit:
	LOG_SLP_TSK_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_slp_tsk */

/*
 *  起床待ち（タイムアウトあり）
 */
#ifdef TOPPERS_tslp_tsk

ER
tslp_tsk(TMO tmout)
{
	WINFO	winfo;
	TMEVTB	tmevtb;
	ER		ercd;

	LOG_TSLP_TSK_ENTER(tmout);
	CHECK_DISPATCH();
	CHECK_TMOUT(tmout);

	t_lock_cpu();
	if (p_runtsk->wupque) {
		p_runtsk->wupque = false;
		ercd = E_OK;
	}
	else if (tmout == TMO_POL) {
		ercd = E_TMOUT;
	}
	else {
		p_runtsk->tstat = (TS_WAITING | TS_WAIT_SLP);
		make_wait_tmout(&winfo, &tmevtb, tmout);
		LOG_TSKSTAT(p_runtsk);
		dispatch();
		ercd = winfo.wercd;
	}
	t_unlock_cpu();

  error_exit:
	LOG_TSLP_TSK_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_tslp_tsk */

/*
 *  タスクの起床
 */
#ifdef TOPPERS_wup_tsk

ER
wup_tsk(ID tskid)
{
	TCB		*p_tcb;
	ER		ercd;

	LOG_WUP_TSK_ENTER(tskid);
	CHECK_TSKCTX_UNL();
	CHECK_TSKID_SELF(tskid);
	p_tcb = get_tcb_self(tskid);

	t_lock_cpu();
	if (p_tcb->p_tinib->tskatr == TA_NOEXS) {
		ercd = E_NOEXS;
	}
	else if (TSTAT_DORMANT(p_tcb->tstat)) {
		ercd = E_OBJ;
	}
	else if (TSTAT_WAIT_SLP(p_tcb->tstat)) {
		if (wait_complete(p_tcb)) {
			dispatch();
		}
		ercd = E_OK;
	}
	else if (!(p_tcb->wupque)) {
		p_tcb->wupque = true;
		ercd = E_OK;
	}
	else {
		ercd = E_QOVR;
	}
	t_unlock_cpu();

  error_exit:
	LOG_WUP_TSK_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_wup_tsk */

/*
 *  タスクの起床（非タスクコンテキスト用）
 */
#ifdef TOPPERS_iwup_tsk

ER
iwup_tsk(ID tskid)
{
	TCB		*p_tcb;
	ER		ercd;

	LOG_IWUP_TSK_ENTER(tskid);
	CHECK_INTCTX_UNL();
	CHECK_TSKID(tskid);
	p_tcb = get_tcb(tskid);

	i_lock_cpu();
	if (p_tcb->p_tinib->tskatr == TA_NOEXS) {
		ercd = E_NOEXS;
	}
	else if (TSTAT_DORMANT(p_tcb->tstat)) {
		ercd = E_OBJ;
	}
	else if (TSTAT_WAIT_SLP(p_tcb->tstat)) {
		if (wait_complete(p_tcb)) {
			reqflg = true;
		}
		ercd = E_OK;
	}
	else if (!(p_tcb->wupque)) {
		p_tcb->wupque = true;
		ercd = E_OK;
	}
	else {
		ercd = E_QOVR;
	}
	i_unlock_cpu();

  error_exit:
	LOG_IWUP_TSK_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_iwup_tsk */

/*
 *  タスク起床要求のキャンセル
 */
#ifdef TOPPERS_can_wup

ER_UINT
can_wup(ID tskid)
{
	TCB		*p_tcb;
	ER_UINT	ercd;

	LOG_CAN_WUP_ENTER(tskid);
	CHECK_TSKCTX_UNL();
	CHECK_TSKID_SELF(tskid);
	p_tcb = get_tcb_self(tskid);

	t_lock_cpu();
	if (p_tcb->p_tinib->tskatr == TA_NOEXS) {
		ercd = E_NOEXS;
	}
	else if (TSTAT_DORMANT(p_tcb->tstat)) {
		ercd = E_OBJ;
	}
	else {
		ercd = p_tcb->wupque ? 1 : 0;
		p_tcb->wupque = false;
	}
	t_unlock_cpu();

  error_exit:
	LOG_CAN_WUP_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_can_wup */

/*
 *  待ち状態の強制解除
 */
#ifdef TOPPERS_rel_wai

ER
rel_wai(ID tskid)
{
	TCB		*p_tcb;
	ER		ercd;

	LOG_REL_WAI_ENTER(tskid);
	CHECK_TSKCTX_UNL();
	CHECK_TSKID(tskid);
	p_tcb = get_tcb(tskid);

	t_lock_cpu();
	if (p_tcb->p_tinib->tskatr == TA_NOEXS) {
		ercd = E_NOEXS;
	}
	else if (!TSTAT_WAITING(p_tcb->tstat)) {
		ercd = E_OBJ;
	}
	else {
		if (wait_release(p_tcb)) {
			dispatch();
		}
		ercd = E_OK;
	}
	t_unlock_cpu();

  error_exit:
	LOG_REL_WAI_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_rel_wai */

/*
 *  待ち状態の強制解除（非タスクコンテキスト用）
 */
#ifdef TOPPERS_irel_wai

ER
irel_wai(ID tskid)
{
	TCB		*p_tcb;
	ER		ercd;

	LOG_IREL_WAI_ENTER(tskid);
	CHECK_INTCTX_UNL();
	CHECK_TSKID(tskid);
	p_tcb = get_tcb(tskid);

	i_lock_cpu();
	if (p_tcb->p_tinib->tskatr == TA_NOEXS) {
		ercd = E_NOEXS;
	}
	else if (!TSTAT_WAITING(p_tcb->tstat)) {
		ercd = E_OBJ;
	}
	else {
		if (wait_release(p_tcb)) {
			reqflg = true;
		}
		ercd = E_OK;
	}
	i_unlock_cpu();

  error_exit:
	LOG_IREL_WAI_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_irel_wai */

/*
 *  強制待ち状態への移行
 */
#ifdef TOPPERS_sus_tsk

ER
sus_tsk(ID tskid)
{
	TCB		*p_tcb;
	ER		ercd;

	LOG_SUS_TSK_ENTER(tskid);
	CHECK_TSKCTX_UNL();
	CHECK_TSKID_SELF(tskid);
	p_tcb = get_tcb_self(tskid);

	t_lock_cpu();
	if (p_tcb == p_runtsk && !dspflg) {
		ercd = E_CTX;
	}
	else if (p_tcb->p_tinib->tskatr == TA_NOEXS) {
		ercd = E_NOEXS;
	}
	else if (TSTAT_DORMANT(p_tcb->tstat)) {
		ercd = E_OBJ;
	}
	else if (TSTAT_RUNNABLE(p_tcb->tstat)) {
		/*
		 *  実行できる状態から強制待ち状態への遷移
		 */
		p_tcb->tstat = TS_SUSPENDED;
		LOG_TSKSTAT(p_tcb);
		if (make_non_runnable(p_tcb)) {
			dispatch();
		}
		ercd = E_OK;
	}
	else if (TSTAT_SUSPENDED(p_tcb->tstat)) {
		ercd = E_QOVR;
	}
	else {
		/*
		 *  待ち状態から二重待ち状態への遷移
		 */
		p_tcb->tstat |= TS_SUSPENDED;
		LOG_TSKSTAT(p_tcb);
		ercd = E_OK;
	}
	t_unlock_cpu();

  error_exit:
	LOG_SUS_TSK_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_sus_tsk */

/*
 *  強制待ち状態からの再開
 */
#ifdef TOPPERS_rsm_tsk

ER
rsm_tsk(ID tskid)
{
	TCB		*p_tcb;
	ER		ercd;

	LOG_RSM_TSK_ENTER(tskid);
	CHECK_TSKCTX_UNL();
	CHECK_TSKID(tskid);
	p_tcb = get_tcb(tskid);

	t_lock_cpu();
	if (p_tcb->p_tinib->tskatr == TA_NOEXS) {
		ercd = E_NOEXS;
	}
	else if (!TSTAT_SUSPENDED(p_tcb->tstat)) {
		ercd = E_OBJ;
	}
	else if (!TSTAT_WAITING(p_tcb->tstat)) {
		/*
		 *  強制待ち状態から実行できる状態への遷移
		 */
		p_tcb->tstat = TS_RUNNABLE;
		LOG_TSKSTAT(p_tcb);
		if (make_runnable(p_tcb)) {
			dispatch();
		}
		ercd = E_OK;
	}
	else {
		/*
		 *  二重待ち状態から待ち状態への遷移
		 */
		p_tcb->tstat &= ~TS_SUSPENDED;
		LOG_TSKSTAT(p_tcb);
		ercd = E_OK;
	}
	t_unlock_cpu();

  error_exit:
	LOG_RSM_TSK_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_rsm_tsk */

/*
 *  自タスクの遅延
 */
#ifdef TOPPERS_dly_tsk

ER
dly_tsk(RELTIM dlytim)
{
	WINFO	winfo;
	TMEVTB	tmevtb;
	ER		ercd;

	LOG_DLY_TSK_ENTER(dlytim);
	CHECK_DISPATCH();
	CHECK_PAR(dlytim <= TMAX_RELTIM);

	t_lock_cpu();
	p_runtsk->tstat = (TS_WAITING | TS_WAIT_DLY);
	(void) make_non_runnable(p_runtsk);
	p_runtsk->p_winfo = &winfo;
	winfo.p_tmevtb = &tmevtb;
	tmevtb_enqueue(&tmevtb, dlytim, (CBACK) wait_tmout_ok, (void *) p_runtsk);
	LOG_TSKSTAT(p_runtsk);
	dispatch();
	ercd = winfo.wercd;
	t_unlock_cpu();

  error_exit:
	LOG_DLY_TSK_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_dly_tsk */
