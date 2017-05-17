/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
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
 *  @(#) $Id: overrun.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		オーバランハンドラ機能
 */

#include "kernel_impl.h"
#include "check.h"
#include "task.h"
#include "overrun.h"

#ifdef TOPPERS_SUPPORT_OVRHDR
#include "target_timer.h"

/*
 *  トレースログマクロのデフォルト定義
 */
#ifndef LOG_OVR_ENTER
#define LOG_OVR_ENTER(p_runtsk)
#endif /* LOG_OVR_ENTER */

#ifndef LOG_OVR_LEAVE
#define LOG_OVR_LEAVE(p_runtsk)
#endif /* LOG_OVR_LEAVE */

#ifndef LOG_STA_OVR_ENTER
#define LOG_STA_OVR_ENTER(tskid, ovrtim)
#endif /* LOG_STA_OVR_ENTER */

#ifndef LOG_STA_OVR_LEAVE
#define LOG_STA_OVR_LEAVE(ercd)
#endif /* LOG_STA_OVR_LEAVE */

#ifndef LOG_ISTA_OVR_ENTER
#define LOG_ISTA_OVR_ENTER(tskid, ovrtim)
#endif /* LOG_ISTA_OVR_ENTER */

#ifndef LOG_ISTA_OVR_LEAVE
#define LOG_ISTA_OVR_LEAVE(ercd)
#endif /* LOG_ISTA_OVR_LEAVE */

#ifndef LOG_ISTP_OVR_ENTER
#define LOG_ISTP_OVR_ENTER(tskid)
#endif /* LOG_ISTP_OVR_ENTER */

#ifndef LOG_ISTP_OVR_LEAVE
#define LOG_ISTP_OVR_LEAVE(ercd)
#endif /* LOG_ISTP_OVR_LEAVE */

#ifndef LOG_STP_OVR_ENTER
#define LOG_STP_OVR_ENTER(tskid)
#endif /* LOG_STP_OVR_ENTER */

#ifndef LOG_STP_OVR_LEAVE
#define LOG_STP_OVR_LEAVE(ercd)
#endif /* LOG_STP_OVR_LEAVE */

#ifndef LOG_REF_OVR_ENTER
#define LOG_REF_OVR_ENTER(tskid, pk_rovr)
#endif /* LOG_REF_OVR_ENTER */

#ifndef LOG_REF_OVR_LEAVE
#define LOG_REF_OVR_LEAVE(ercd, pk_rovr)
#endif /* LOG_REF_OVR_LEAVE */

#ifdef TOPPERS_ovrini

/*
 *  オーバランタイマが動作中かを示すフラグ
 */
bool_t	ovrtimer_flag;

/*
 *  オーバランハンドラ機能の初期化
 */
void
initialize_overrun(void)
{
	ovrtimer_flag = false;
}

#endif /* TOPPERS_ovrini */

/*
 *  オーバランハンドラ用タイマの動作開始
 */
#ifdef TOPPERS_ovrsta
#ifndef OMIT_OVRTIMER_START

void
ovrtimer_start(void)
{
	if (p_runtsk->leftotm > 0U) {
		target_ovrtimer_start(p_runtsk->leftotm);
		ovrtimer_flag = true;
	}
}

#endif /* OMIT_OVRTIMER_START */
#endif /* TOPPERS_ovrsta */

/*
 *  オーバランハンドラ用タイマの停止
 */
#ifdef TOPPERS_ovrstp
#ifndef OMIT_OVRTIMER_STOP

void
ovrtimer_stop(void)
{
	if (ovrtimer_flag) {
		assert(p_runtsk->leftotm > 0U);
		p_runtsk->leftotm = target_ovrtimer_stop();
		ovrtimer_flag = false;
	}
}

#endif /* OMIT_OVRTIMER_STOP */
#endif /* TOPPERS_ovrstp */

/*
 *  オーバランハンドラの動作開始
 */
#ifdef TOPPERS_sta_ovr

ER
sta_ovr(ID tskid, OVRTIM ovrtim)
{
	TCB		*p_tcb;
	ER		ercd;

	LOG_STA_OVR_ENTER(tskid, ovrtim);
	CHECK_TSKCTX_UNL();
	CHECK_OBJ(ovrinib.ovrhdr != NULL);
	CHECK_TSKID_SELF(tskid);
	CHECK_PAR(0U < ovrtim && ovrtim <= TMAX_OVRTIM);
	p_tcb = get_tcb_self(tskid);

	t_lock_cpu();
	if (p_tcb == p_runtsk) {
		if (p_tcb->leftotm > 0U) {
			(void) target_ovrtimer_stop();
		}
		target_ovrtimer_start(ovrtim);
		ovrtimer_flag = true;
	}
	p_tcb->leftotm = ovrtim;
	ercd = E_OK;
	t_unlock_cpu();

  error_exit:
	LOG_STA_OVR_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_sta_ovr */

/*
 *  オーバランハンドラの動作開始（非タスクコンテキスト用）
 */
#ifdef TOPPERS_ista_ovr

ER
ista_ovr(ID tskid, OVRTIM ovrtim)
{
	TCB		*p_tcb;
	ER		ercd;

	LOG_ISTA_OVR_ENTER(tskid, ovrtim);
	CHECK_INTCTX_UNL();
	CHECK_OBJ(ovrinib.ovrhdr != NULL);
	CHECK_TSKID(tskid);
	CHECK_PAR(0U < ovrtim && ovrtim <= TMAX_OVRTIM);
	p_tcb = get_tcb(tskid);

	i_lock_cpu();
	p_tcb->leftotm = ovrtim;
	ercd = E_OK;
	i_unlock_cpu();

  error_exit:
	LOG_ISTA_OVR_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_ista_ovr */

/*
 *  オーバランハンドラの動作停止
 */
#ifdef TOPPERS_stp_ovr

ER
stp_ovr(ID tskid)
{
	TCB		*p_tcb;
	ER		ercd;

	LOG_STP_OVR_ENTER(tskid);
	CHECK_TSKCTX_UNL();
	CHECK_OBJ(ovrinib.ovrhdr != NULL);
	CHECK_TSKID_SELF(tskid);
	p_tcb = get_tcb_self(tskid);

	t_lock_cpu();
	if (p_tcb->leftotm > 0U) {
		if (p_tcb == p_runtsk) {
			(void) target_ovrtimer_stop();
			ovrtimer_flag = false;
		}
		p_tcb->leftotm = 0U;
	}
	ercd = E_OK;
	t_unlock_cpu();

  error_exit:
	LOG_STP_OVR_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_stp_ovr */

/*
 *  オーバランハンドラの動作停止（非タスクコンテキスト用）
 */
#ifdef TOPPERS_istp_ovr

ER
istp_ovr(ID tskid)
{
	TCB		*p_tcb;
	ER		ercd;

	LOG_ISTP_OVR_ENTER(tskid);
	CHECK_INTCTX_UNL();
	CHECK_OBJ(ovrinib.ovrhdr != NULL);
	CHECK_TSKID(tskid);
	p_tcb = get_tcb(tskid);

	i_lock_cpu();
	p_tcb->leftotm = 0U;
	ercd = E_OK;
	i_unlock_cpu();

  error_exit:
	LOG_ISTP_OVR_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_istp_ovr */

/*
 *  オーバランハンドラの状態参照
 */
#ifdef TOPPERS_ref_ovr

ER
ref_ovr(ID tskid, T_ROVR *pk_rovr)
{
	TCB		*p_tcb;
	ER		ercd;
    
	LOG_REF_OVR_ENTER(tskid, pk_rovr);
	CHECK_TSKCTX_UNL();
	CHECK_OBJ(ovrinib.ovrhdr != NULL);
	CHECK_TSKID_SELF(tskid);
	p_tcb = get_tcb_self(tskid);

	t_lock_cpu();
	if (p_tcb->leftotm > 0U) {
		pk_rovr->ovrstat = TOVR_STA;
		if (p_tcb == p_runtsk) {
			pk_rovr->leftotm = target_ovrtimer_get_current();
		}
		else {
			pk_rovr->leftotm = p_tcb->leftotm;
		}
	}
	else {
		pk_rovr->ovrstat = TOVR_STP;
	}
	ercd = E_OK;
	t_unlock_cpu();

  error_exit:
	LOG_REF_OVR_LEAVE(ercd, pk_rovr);
	return(ercd);
}

#endif /* TOPPERS_ref_ovr */

/*
 *  オーバランハンドラ起動ルーチン
 *
 *  オーバランハンドラの呼出し後に，呼出し前の状態（CPUロックフラグ，割
 *  込み優先度マスク）に戻さないのは，このルーチンからのリターン後に，
 *  割込み出口処理で元の状態に戻すためである．
 */
#ifdef TOPPERS_ovrcal

void
call_ovrhdr(void)
{
	assert(sense_context());
	assert(!i_sense_lock());
	assert(ovrinib.ovrhdr != NULL);

	i_lock_cpu();
	if (p_runtsk!= NULL && p_runtsk->leftotm == 1U) {
		p_runtsk->leftotm = 0U;
		i_unlock_cpu();

		LOG_OVR_ENTER(p_runtsk);
		((OVRHDR)(ovrinib.ovrhdr))(TSKID(p_runtsk), p_runtsk->p_tinib->exinf);
		LOG_OVR_LEAVE(p_runtsk);
	}
	else {
		/*
		 *  このルーチンが呼び出される前に，オーバランハンドラの起動が
		 *  キャンセルされた場合
		 */
		i_unlock_cpu();
	}
}

#endif /* TOPPERS_ovrcal */
#endif /* TOPPERS_SUPPORT_OVRHDR */
