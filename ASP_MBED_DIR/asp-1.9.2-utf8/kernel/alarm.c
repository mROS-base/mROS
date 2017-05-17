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
 *  @(#) $Id: alarm.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		アラームハンドラ機能
 */

#include "kernel_impl.h"
#include "check.h"
#include "alarm.h"

/*
 *  トレースログマクロのデフォルト定義
 */
#ifndef LOG_ALM_ENTER
#define LOG_ALM_ENTER(p_almcb)
#endif /* LOG_ALM_ENTER */

#ifndef LOG_ALM_LEAVE
#define LOG_ALM_LEAVE(p_almcb)
#endif /* LOG_ALM_LEAVE */

#ifndef LOG_ACRE_ALM_ENTER
#define LOG_ACRE_ALM_ENTER(pk_calm)
#endif /* LOG_ACRE_ALM_ENTER */

#ifndef LOG_ACRE_ALM_LEAVE
#define LOG_ACRE_ALM_LEAVE(ercd)
#endif /* LOG_ACRE_ALM_LEAVE */

#ifndef LOG_DEL_ALM_ENTER
#define LOG_DEL_ALM_ENTER(almid)
#endif /* LOG_DEL_ALM_ENTER */

#ifndef LOG_DEL_ALM_LEAVE
#define LOG_DEL_ALM_LEAVE(ercd)
#endif /* LOG_DEL_ALM_LEAVE */

#ifndef LOG_STA_ALM_ENTER
#define LOG_STA_ALM_ENTER(almid, almtim)
#endif /* LOG_STA_ALM_ENTER */

#ifndef LOG_STA_ALM_LEAVE
#define LOG_STA_ALM_LEAVE(ercd)
#endif /* LOG_STA_ALM_LEAVE */

#ifndef LOG_ISTA_ALM_ENTER
#define LOG_ISTA_ALM_ENTER(almid, almtim)
#endif /* LOG_ISTA_ALM_ENTER */

#ifndef LOG_ISTA_ALM_LEAVE
#define LOG_ISTA_ALM_LEAVE(ercd)
#endif /* LOG_ISTA_ALM_LEAVE */

#ifndef LOG_STP_ALM_ENTER
#define LOG_STP_ALM_ENTER(almid)
#endif /* LOG_STP_ALM_ENTER */

#ifndef LOG_STP_ALM_LEAVE
#define LOG_STP_ALM_LEAVE(ercd)
#endif /* LOG_STP_ALM_LEAVE */

#ifndef LOG_ISTP_ALM_ENTER
#define LOG_ISTP_ALM_ENTER(almid)
#endif /* LOG_ISTP_ALM_ENTER */

#ifndef LOG_ISTP_ALM_LEAVE
#define LOG_ISTP_ALM_LEAVE(ercd)
#endif /* LOG_ISTP_ALM_LEAVE */

#ifndef LOG_REF_ALM_ENTER
#define LOG_REF_ALM_ENTER(almid, pk_ralm)
#endif /* LOG_REF_ALM_ENTER */

#ifndef LOG_REF_ALM_LEAVE
#define LOG_REF_ALM_LEAVE(ercd, pk_ralm)
#endif /* LOG_REF_ALM_LEAVE */

/*
 *  アラームハンドラの数
 */
#define tnum_alm	((uint_t)(tmax_almid - TMIN_ALMID + 1))
#define tnum_salm	((uint_t)(tmax_salmid - TMIN_ALMID + 1))

/*
 *  アラームハンドラIDからアラームハンドラ管理ブロックを取り出すためのマクロ
 */
#define INDEX_ALM(almid)	((uint_t)((almid) - TMIN_ALMID))
#define get_almcb(almid)	(&(almcb_table[INDEX_ALM(almid)]))

#ifdef TOPPERS_almini

/*
 *  使用していないアラームハンドラ管理ブロックのリスト
 *
 *  アラームハンドラ管理ブロックの先頭にはキューにつなぐための領域がな
 *  いため，タイムイベントブロック（tmevtb）の領域を用いる．
 */
QUEUE	free_almcb;

/*
 *  アラームハンドラ機能の初期化
 */
void
initialize_alarm(void)
{
	uint_t	i, j;
	ALMCB	*p_almcb;
	ALMINIB	*p_alminib;

	for (i = 0; i < tnum_salm; i++) {
		p_almcb = &(almcb_table[i]);
		p_almcb->p_alminib = &(alminib_table[i]);
		p_almcb->almsta = false;
	}
	queue_initialize(&free_almcb);
	for (j = 0; i < tnum_alm; i++, j++) {
		p_almcb = &(almcb_table[i]);
		p_alminib = &(aalminib_table[j]);
		p_alminib->almatr = TA_NOEXS;
		p_almcb->p_alminib = ((const ALMINIB *) p_alminib);
		queue_insert_prev(&free_almcb, ((QUEUE *) &(p_almcb->tmevtb)));
	}
}

#endif /* TOPPERS_almini */

/*
 *  アラームハンドラの生成
 */
#ifdef TOPPERS_acre_alm

ER_UINT
acre_alm(const T_CALM *pk_calm)
{
	ALMCB	*p_almcb;
	ALMINIB	*p_alminib;
	ER		ercd;

	LOG_ACRE_ALM_ENTER(pk_calm);
	CHECK_TSKCTX_UNL();
	CHECK_RSATR(pk_calm->almatr, TA_NULL);
	CHECK_ALIGN_FUNC(pk_calm->almhdr);
	CHECK_NONNULL_FUNC(pk_calm->almhdr);

	t_lock_cpu();
	if (tnum_alm == 0 || queue_empty(&free_almcb)) {
		ercd = E_NOID;
	}
	else {
		p_almcb = ((ALMCB *)(((char *) queue_delete_next(&free_almcb))
												- offsetof(ALMCB, tmevtb)));
		p_alminib = (ALMINIB *)(p_almcb->p_alminib);
		p_alminib->almatr = pk_calm->almatr;
		p_alminib->exinf = pk_calm->exinf;
		p_alminib->almhdr = pk_calm->almhdr;

		p_almcb->almsta = false;
		ercd = ALMID(p_almcb);
	}
	t_unlock_cpu();

  error_exit:
	LOG_ACRE_ALM_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_acre_alm */

/*
 *  アラームハンドラの削除
 */
#ifdef TOPPERS_del_alm

ER
del_alm(ID almid)
{
	ALMCB	*p_almcb;
	ALMINIB	*p_alminib;
	ER		ercd;

	LOG_DEL_ALM_ENTER(almid);
	CHECK_TSKCTX_UNL();
	CHECK_ALMID(almid);
	p_almcb = get_almcb(almid);

	t_lock_cpu();
	if (p_almcb->p_alminib->almatr == TA_NOEXS) {
		ercd = E_NOEXS;
	}
	else if (ALMID(p_almcb) > tmax_salmid) {
		if (p_almcb->almsta) {
			p_almcb->almsta = false;
			tmevtb_dequeue(&(p_almcb->tmevtb));
		}

		p_alminib = (ALMINIB *)(p_almcb->p_alminib);
		p_alminib->almatr = TA_NOEXS;
		queue_insert_prev(&free_almcb, ((QUEUE *) &(p_almcb->tmevtb)));
		ercd = E_OK;
	}
	else {
		ercd = E_OBJ;
	}
	t_unlock_cpu();

  error_exit:
	LOG_DEL_ALM_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_del_alm */

/*
 *  アラームハンドラの動作開始
 */
#ifdef TOPPERS_sta_alm

ER
sta_alm(ID almid, RELTIM almtim)
{
	ALMCB	*p_almcb;
	ER		ercd;

	LOG_STA_ALM_ENTER(almid, almtim);
	CHECK_TSKCTX_UNL();
	CHECK_ALMID(almid);
	CHECK_PAR(almtim <= TMAX_RELTIM);
	p_almcb = get_almcb(almid);

	t_lock_cpu();
	if (p_almcb->p_alminib->almatr == TA_NOEXS) {
		ercd = E_NOEXS;
	}
	else {
		if (p_almcb->almsta) {
			tmevtb_dequeue(&(p_almcb->tmevtb));
		}
		else {
			p_almcb->almsta = true;
		}
		tmevtb_enqueue(&(p_almcb->tmevtb), almtim,
									(CBACK) call_almhdr, (void *) p_almcb);
		ercd = E_OK;
	}
	t_unlock_cpu();

  error_exit:
	LOG_STA_ALM_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_sta_alm */

/*
 *  アラームハンドラの動作開始（非タスクコンテキスト用）
 */
#ifdef TOPPERS_ista_alm

ER
ista_alm(ID almid, RELTIM almtim)
{
	ALMCB	*p_almcb;
	ER		ercd;

	LOG_ISTA_ALM_ENTER(almid, almtim);
	CHECK_INTCTX_UNL();
	CHECK_ALMID(almid);
	CHECK_PAR(almtim <= TMAX_RELTIM);
	p_almcb = get_almcb(almid);

	i_lock_cpu();
	if (p_almcb->p_alminib->almatr == TA_NOEXS) {
		ercd = E_NOEXS;
	}
	else {
		if (p_almcb->almsta) {
			tmevtb_dequeue(&(p_almcb->tmevtb));
		}
		else {
			p_almcb->almsta = true;
		}
		tmevtb_enqueue(&(p_almcb->tmevtb), almtim,
									(CBACK) call_almhdr, (void *) p_almcb);
		ercd = E_OK;
	}
	i_unlock_cpu();

  error_exit:
	LOG_ISTA_ALM_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_ista_alm */

/*
 *  アラームハンドラの動作停止
 */
#ifdef TOPPERS_stp_alm

ER
stp_alm(ID almid)
{
	ALMCB	*p_almcb;
	ER		ercd;

	LOG_STP_ALM_ENTER(almid);
	CHECK_TSKCTX_UNL();
	CHECK_ALMID(almid);
	p_almcb = get_almcb(almid);

	t_lock_cpu();
	if (p_almcb->p_alminib->almatr == TA_NOEXS) {
		ercd = E_NOEXS;
	}
	else {
		if (p_almcb->almsta) {
			p_almcb->almsta = false;
			tmevtb_dequeue(&(p_almcb->tmevtb));
		}
		ercd = E_OK;
	}
	t_unlock_cpu();

  error_exit:
	LOG_STP_ALM_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_stp_alm */

/*
 *  アラームハンドラの動作停止（非タスクコンテキスト用）
 */
#ifdef TOPPERS_istp_alm

ER
istp_alm(ID almid)
{
	ALMCB	*p_almcb;
	ER		ercd;

	LOG_ISTP_ALM_ENTER(almid);
	CHECK_INTCTX_UNL();
	CHECK_ALMID(almid);
	p_almcb = get_almcb(almid);

	i_lock_cpu();
	if (p_almcb->p_alminib->almatr == TA_NOEXS) {
		ercd = E_NOEXS;
	}
	else {
		if (p_almcb->almsta) {
			p_almcb->almsta = false;
			tmevtb_dequeue(&(p_almcb->tmevtb));
		}
		ercd = E_OK;
	}
	i_unlock_cpu();

  error_exit:
	LOG_ISTP_ALM_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_istp_alm */

/*
 *  アラームハンドラの状態参照
 */
#ifdef TOPPERS_ref_alm

ER
ref_alm(ID almid, T_RALM *pk_ralm)
{
	ALMCB	*p_almcb;
	ER		ercd;
    
	LOG_REF_ALM_ENTER(almid, pk_ralm);
	CHECK_TSKCTX_UNL();
	CHECK_ALMID(almid);
	p_almcb = get_almcb(almid);

	t_lock_cpu();
	if (p_almcb->p_alminib->almatr == TA_NOEXS) {
		ercd = E_NOEXS;
	}
	else {
		if (p_almcb->almsta) {
			pk_ralm->almstat = TALM_STA;
			pk_ralm->lefttim = tmevt_lefttim(&(p_almcb->tmevtb));
		}
		else {
			pk_ralm->almstat = TALM_STP;
		}
		ercd = E_OK;
	}
	t_unlock_cpu();

  error_exit:
	LOG_REF_ALM_LEAVE(ercd, pk_ralm);
	return(ercd);
}

#endif /* TOPPERS_ref_alm */

/*
 *  アラームハンドラ起動ルーチン
 */
#ifdef TOPPERS_almcal

void
call_almhdr(ALMCB *p_almcb)
{
	PRI		saved_ipm;

	/*
	 *  アラームハンドラを停止状態にする．
	 */
	p_almcb->almsta = false;

	/*
	 *  アラームハンドラを，CPUロック解除状態で呼び出す．
	 */
	saved_ipm = i_get_ipm();
	i_unlock_cpu();

	LOG_ALM_ENTER(p_almcb);
	(*((ALMHDR)(p_almcb->p_alminib->almhdr)))(p_almcb->p_alminib->exinf);
	LOG_ALM_LEAVE(p_almcb);

	if (!i_sense_lock()) {
		i_lock_cpu();
	}
	i_set_ipm(saved_ipm);
}

#endif /* TOPPERS_almcal */
