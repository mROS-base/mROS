/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2005-2014 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: task_except.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		タスク例外処理機能
 */

#include "kernel_impl.h"
#include "check.h"
#include "task.h"

/*
 *  トレースログマクロのデフォルト定義
 */
#ifndef LOG_DEF_TEX_ENTER
#define LOG_DEF_TEX_ENTER(tskid, pk_dtex)
#endif /* LOG_DEF_TEX_ENTER */

#ifndef LOG_DEF_TEX_LEAVE
#define LOG_DEF_TEX_LEAVE(ercd)
#endif /* LOG_DEF_TEX_LEAVE */

#ifndef LOG_RAS_TEX_ENTER
#define LOG_RAS_TEX_ENTER(tskid, rasptn)
#endif /* LOG_RAS_TEX_ENTER */

#ifndef LOG_RAS_TEX_LEAVE
#define LOG_RAS_TEX_LEAVE(ercd)
#endif /* LOG_RAS_TEX_LEAVE */

#ifndef LOG_IRAS_TEX_ENTER
#define LOG_IRAS_TEX_ENTER(tskid, rasptn)
#endif /* LOG_IRAS_TEX_ENTER */

#ifndef LOG_IRAS_TEX_LEAVE
#define LOG_IRAS_TEX_LEAVE(ercd)
#endif /* LOG_IRAS_TEX_LEAVE */

#ifndef LOG_DIS_TEX_ENTER
#define LOG_DIS_TEX_ENTER()
#endif /* LOG_DIS_TEX_ENTER */

#ifndef LOG_DIS_TEX_LEAVE
#define LOG_DIS_TEX_LEAVE(ercd)
#endif /* LOG_DIS_TEX_LEAVE */

#ifndef LOG_ENA_TEX_ENTER
#define LOG_ENA_TEX_ENTER()
#endif /* LOG_ENA_TEX_ENTER */

#ifndef LOG_ENA_TEX_LEAVE
#define LOG_ENA_TEX_LEAVE(ercd)
#endif /* LOG_ENA_TEX_LEAVE */

#ifndef LOG_SNS_TEX_ENTER
#define LOG_SNS_TEX_ENTER()
#endif /* LOG_SNS_TEX_ENTER */

#ifndef LOG_SNS_TEX_LEAVE
#define LOG_SNS_TEX_LEAVE(state)
#endif /* LOG_SNS_TEX_LEAVE */

#ifndef LOG_REF_TEX_ENTER
#define LOG_REF_TEX_ENTER(tskid, pk_rtex)
#endif /* LOG_REF_TEX_ENTER */

#ifndef LOG_REF_TEX_LEAVE
#define LOG_REF_TEX_LEAVE(ercd, pk_rtex)
#endif /* LOG_REF_TEX_LEAVE */

/*
 *  タスク例外処理ルーチンの定義
 */
#ifdef TOPPERS_def_tex

ER_UINT
def_tex(ID tskid, const T_DTEX *pk_dtex)
{
	TCB		*p_tcb;
	TINIB	*p_tinib;
	ER		ercd;

	LOG_DEF_TEX_ENTER(tskid, pk_dtex);
	CHECK_TSKCTX_UNL();
	CHECK_TSKID_SELF(tskid);
	CHECK_RSATR(pk_dtex->texatr, TA_NULL);
	if (pk_dtex->texrtn != NULL) {
		CHECK_ALIGN_FUNC(pk_dtex->texrtn);
		CHECK_NONNULL_FUNC(pk_dtex->texrtn);
	}
	p_tcb = get_tcb_self(tskid);
	p_tinib = (TINIB *)(p_tcb->p_tinib);

	t_lock_cpu();
	if (p_tcb->p_tinib->tskatr == TA_NOEXS) {
		ercd = E_NOEXS;
	}
	else if (TSKID(p_tcb) > tmax_stskid) {
		if (pk_dtex->texrtn != NULL) {
			if (p_tinib->texrtn != NULL) {
				ercd = E_OBJ;
			}
			else {
				p_tinib->texatr = pk_dtex->texatr;
				p_tinib->texrtn = pk_dtex->texrtn;
				ercd = E_OK;
			}
		}
		else {
			if (p_tinib->texrtn != NULL) {
				p_tinib->texrtn = NULL;
				p_tcb->enatex = false;
				p_tcb->texptn = 0U;
				ercd = E_OK;
			}
			else {
				ercd = E_OBJ;
			}
		}
	}
	else {
		ercd = E_OBJ;
	}
	t_unlock_cpu();

  error_exit:
	LOG_DEF_TEX_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_def_tex */

/*
 *  タスク例外処理の要求
 */
#ifdef TOPPERS_ras_tex

ER
ras_tex(ID tskid, TEXPTN rasptn)
{
	TCB		*p_tcb;
	ER		ercd;

	LOG_RAS_TEX_ENTER(tskid, rasptn);
	CHECK_TSKCTX_UNL();
	CHECK_TSKID_SELF(tskid);
	CHECK_PAR(rasptn != 0U);
	p_tcb = get_tcb_self(tskid);

	t_lock_cpu();
	if (p_tcb->p_tinib->tskatr == TA_NOEXS) {
		ercd = E_NOEXS;
	}
	else if (TSTAT_DORMANT(p_tcb->tstat) || p_tcb->p_tinib->texrtn == NULL) {
		ercd = E_OBJ;
	}
	else {
		p_tcb->texptn |= rasptn;
		if (p_tcb == p_runtsk && p_runtsk->enatex && ipmflg) {
			call_texrtn();
		}
		ercd = E_OK;
	}
	t_unlock_cpu();

  error_exit:
	LOG_RAS_TEX_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_ras_tex */

/*
 *  タスク例外処理の要求（非タスクコンテキスト用）
 */
#ifdef TOPPERS_iras_tex

ER
iras_tex(ID tskid, TEXPTN rasptn)
{
	TCB		*p_tcb;
	ER		ercd;

	LOG_IRAS_TEX_ENTER(tskid, rasptn);
	CHECK_INTCTX_UNL();
	CHECK_TSKID(tskid);
	CHECK_PAR(rasptn != 0U);
	p_tcb = get_tcb(tskid);

	i_lock_cpu();
	if (p_tcb->p_tinib->tskatr == TA_NOEXS) {
		ercd = E_NOEXS;
	}
	else if (TSTAT_DORMANT(p_tcb->tstat) || p_tcb->p_tinib->texrtn == NULL) {
		ercd = E_OBJ;
	}
	else {
		p_tcb->texptn |= rasptn;
		if (p_tcb == p_runtsk && p_runtsk->enatex && ipmflg) {
			reqflg = true;
		}
		ercd = E_OK;
	}
	i_unlock_cpu();

  error_exit:
	LOG_IRAS_TEX_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_iras_tex */

/*
 *  タスク例外処理の禁止
 */
#ifdef TOPPERS_dis_tex

ER
dis_tex(void)
{
	ER		ercd;

	LOG_DIS_TEX_ENTER();
	CHECK_TSKCTX_UNL();

	t_lock_cpu();
	if (p_runtsk->p_tinib->texrtn == NULL) {
		ercd = E_OBJ;
	}
	else {
		p_runtsk->enatex = false;
		ercd = E_OK;
	}
	t_unlock_cpu();

  error_exit:
	LOG_DIS_TEX_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_dis_tex */

/*
 *  タスク例外処理の許可
 */
#ifdef TOPPERS_ena_tex

ER
ena_tex(void)
{
	ER		ercd;

	LOG_ENA_TEX_ENTER();
	CHECK_TSKCTX_UNL();

	t_lock_cpu();
	if (p_runtsk->p_tinib->texrtn == NULL) {
		ercd = E_OBJ;
	}
	else {
		p_runtsk->enatex = true;
		if (p_runtsk->texptn != 0U && ipmflg) {
			call_texrtn();
		}
		ercd = E_OK;
	}
	t_unlock_cpu();

  error_exit:
	LOG_ENA_TEX_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_ena_tex */

/*
 *  タスク例外処理禁止状態の参照
 */
#ifdef TOPPERS_sns_tex

bool_t
sns_tex(void)
{
	bool_t	state;

	LOG_SNS_TEX_ENTER();
	state = (p_runtsk != NULL && p_runtsk->enatex) ? false : true;
	LOG_SNS_TEX_LEAVE(state);
	return(state);
}

#endif /* TOPPERS_sns_tex */

/*
 *  タスク例外処理の状態参照
 */
#ifdef TOPPERS_ref_tex

ER
ref_tex(ID tskid, T_RTEX *pk_rtex)
{
	TCB		*p_tcb;
	ER		ercd;

	LOG_REF_TEX_ENTER(tskid, pk_rtex);
	CHECK_TSKCTX_UNL();
	CHECK_TSKID_SELF(tskid);
	p_tcb = get_tcb_self(tskid);

	t_lock_cpu();
	if (p_tcb->p_tinib->tskatr == TA_NOEXS) {
		ercd = E_NOEXS;
	}
	else if (TSTAT_DORMANT(p_tcb->tstat) || p_tcb->p_tinib->texrtn == NULL) {
		ercd = E_OBJ;
	}
	else {
		pk_rtex->texstat = (p_tcb->enatex) ? TTEX_ENA : TTEX_DIS;
		pk_rtex->pndptn = p_tcb->texptn;
		ercd = E_OK;
	}
	t_unlock_cpu();

  error_exit:
	LOG_REF_TEX_LEAVE(ercd, pk_rtex);
	return(ercd);
}

#endif /* TOPPERS_ref_tex */
