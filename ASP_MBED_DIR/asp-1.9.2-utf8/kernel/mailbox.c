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
 *  $Id: mailbox.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		メールボックス機能
 */

#include "kernel_impl.h"
#include "check.h"
#include "task.h"
#include "wait.h"
#include "mailbox.h"

/*
 *  トレースログマクロのデフォルト定義
 */
#ifndef LOG_ACRE_MBX_ENTER
#define LOG_ACRE_MBX_ENTER(pk_cmbx)
#endif /* LOG_ACRE_MBX_ENTER */

#ifndef LOG_ACRE_MBX_LEAVE
#define LOG_ACRE_MBX_LEAVE(ercd)
#endif /* LOG_ACRE_MBX_LEAVE */

#ifndef LOG_DEL_MBX_ENTER
#define LOG_DEL_MBX_ENTER(mbxid)
#endif /* LOG_DEL_MBX_ENTER */

#ifndef LOG_DEL_MBX_LEAVE
#define LOG_DEL_MBX_LEAVE(ercd)
#endif /* LOG_DEL_MBX_LEAVE */

#ifndef LOG_SND_MBX_ENTER
#define LOG_SND_MBX_ENTER(mbxid, pk_msg)
#endif /* LOG_SND_MBX_ENTER */

#ifndef LOG_SND_MBX_LEAVE
#define LOG_SND_MBX_LEAVE(ercd)
#endif /* LOG_SND_MBX_LEAVE */

#ifndef LOG_RCV_MBX_ENTER
#define LOG_RCV_MBX_ENTER(mbxid, ppk_msg)
#endif /* LOG_RCV_MBX_ENTER */

#ifndef LOG_RCV_MBX_LEAVE
#define LOG_RCV_MBX_LEAVE(ercd, pk_msg)
#endif /* LOG_RCV_MBX_LEAVE */

#ifndef LOG_PRCV_MBX_ENTER
#define LOG_PRCV_MBX_ENTER(mbxid, ppk_msg)
#endif /* LOG_PRCV_MBX_ENTER */

#ifndef LOG_PRCV_MBX_LEAVE
#define LOG_PRCV_MBX_LEAVE(ercd, pk_msg)
#endif /* LOG_PRCV_MBX_LEAVE */

#ifndef LOG_TRCV_MBX_ENTER
#define LOG_TRCV_MBX_ENTER(mbxid, ppk_msg, tmout)
#endif /* LOG_TRCV_MBX_ENTER */

#ifndef LOG_TRCV_MBX_LEAVE
#define LOG_TRCV_MBX_LEAVE(ercd, pk_msg)
#endif /* LOG_TRCV_MBX_LEAVE */

#ifndef LOG_INI_MBX_ENTER
#define LOG_INI_MBX_ENTER(mbxid)
#endif /* LOG_INI_MBX_ENTER */

#ifndef LOG_INI_MBX_LEAVE
#define LOG_INI_MBX_LEAVE(ercd)
#endif /* LOG_INI_MBX_LEAVE */

#ifndef LOG_REF_MBX_ENTER
#define LOG_REF_MBX_ENTER(mbxid, pk_rmbx)
#endif /* LOG_REF_MBX_ENTER */

#ifndef LOG_REF_MBX_LEAVE
#define LOG_REF_MBX_LEAVE(ercd, pk_rmbx)
#endif /* LOG_REF_MBX_LEAVE */

/*
 *  メールボックスの数
 */
#define tnum_mbx	((uint_t)(tmax_mbxid - TMIN_MBXID + 1))
#define tnum_smbx	((uint_t)(tmax_smbxid - TMIN_MBXID + 1))

/*
 *  メールボックスIDからメールボックス管理ブロックを取り出すためのマクロ
 */
#define INDEX_MBX(mbxid)	((uint_t)((mbxid) - TMIN_MBXID))
#define get_mbxcb(mbxid)	(&(mbxcb_table[INDEX_MBX(mbxid)]))

#ifdef TOPPERS_mbxini

/*
 *  使用していないメールボックス管理ブロックのリスト
 */
QUEUE	free_mbxcb;

/* 
 *  メールボックス機能の初期化
 */
void
initialize_mailbox(void)
{
	uint_t	i, j;
	MBXCB	*p_mbxcb;
	MBXINIB	*p_mbxinib;

	for (i = 0; i < tnum_smbx; i++) {
		p_mbxcb = &(mbxcb_table[i]);
		queue_initialize(&(p_mbxcb->wait_queue));
		p_mbxcb->p_mbxinib = &(mbxinib_table[i]);
		p_mbxcb->pk_head = NULL;
	}
	queue_initialize(&free_mbxcb);
	for (j = 0; i < tnum_mbx; i++, j++) {
		p_mbxcb = &(mbxcb_table[i]);
		p_mbxinib = &(ambxinib_table[j]);
		p_mbxinib->mbxatr = TA_NOEXS;
		p_mbxcb->p_mbxinib = ((const MBXINIB *) p_mbxinib);
		queue_insert_prev(&free_mbxcb, &(p_mbxcb->wait_queue));
	}
}

#endif /* TOPPERS_mbxini */

/*
 *  メッセージ優先度の取出し
 */
#define	MSGPRI(pk_msg)	(((T_MSG_PRI *)(pk_msg))->msgpri)

/*
 *  優先度順メッセージキューへの挿入
 */
Inline void
enqueue_msg_pri(T_MSG **ppk_prevmsg_next, T_MSG *pk_msg)
{
	T_MSG	*pk_nextmsg;

	while ((pk_nextmsg = *ppk_prevmsg_next) != NULL) {
		if (MSGPRI(pk_nextmsg) > MSGPRI(pk_msg)) {
			break;
		}
		ppk_prevmsg_next = &(pk_nextmsg->pk_next);
	}
	pk_msg->pk_next = pk_nextmsg;
	*ppk_prevmsg_next = pk_msg;
}

/*
 *  メールボックスの生成
 */
#ifdef TOPPERS_acre_mbx

ER_UINT
acre_mbx(const T_CMBX *pk_cmbx)
{
	MBXCB	*p_mbxcb;
	MBXINIB	*p_mbxinib;
	ER		ercd;

	LOG_ACRE_MBX_ENTER(pk_cmbx);
	CHECK_TSKCTX_UNL();
	CHECK_RSATR(pk_cmbx->mbxatr, TA_TPRI|TA_MPRI);
	CHECK_MPRI(pk_cmbx->maxmpri);
	CHECK_NOSPT(pk_cmbx->mprihd == NULL);

	t_lock_cpu();
	if (tnum_mbx == 0 || queue_empty(&free_mbxcb)) {
		ercd = E_NOID;
	}
	else {
		p_mbxcb = ((MBXCB *) queue_delete_next(&free_mbxcb));
		p_mbxinib = (MBXINIB *)(p_mbxcb->p_mbxinib);
		p_mbxinib->mbxatr = pk_cmbx->mbxatr;
		p_mbxinib->maxmpri = pk_cmbx->maxmpri;

		queue_initialize(&(p_mbxcb->wait_queue));
		p_mbxcb->pk_head = NULL;
		ercd = MBXID(p_mbxcb);
	}
	t_unlock_cpu();

  error_exit:
	LOG_ACRE_MBX_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_acre_mbx */

/*
 *  メールボックスの削除
 */
#ifdef TOPPERS_del_mbx

ER
del_mbx(ID mbxid)
{
	MBXCB	*p_mbxcb;
	MBXINIB	*p_mbxinib;
	bool_t	dspreq;
	ER		ercd;

	LOG_DEL_MBX_ENTER(mbxid);
	CHECK_TSKCTX_UNL();
	CHECK_MBXID(mbxid);
	p_mbxcb = get_mbxcb(mbxid);

	t_lock_cpu();
	if (p_mbxcb->p_mbxinib->mbxatr == TA_NOEXS) {
		ercd = E_NOEXS;
	}
	else if (MBXID(p_mbxcb) > tmax_smbxid) {
		dspreq = init_wait_queue(&(p_mbxcb->wait_queue));
		p_mbxinib = (MBXINIB *)(p_mbxcb->p_mbxinib);
		p_mbxinib->mbxatr = TA_NOEXS;
		queue_insert_prev(&free_mbxcb, &(p_mbxcb->wait_queue));
		if (dspreq) {
			dispatch();
		}
		ercd = E_OK;
	}
	else {
		ercd = E_OBJ;
	}
	t_unlock_cpu();

  error_exit:
	LOG_DEL_MBX_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_del_mbx */

/*
 *  メールボックスへの送信
 */
#ifdef TOPPERS_snd_mbx

ER
snd_mbx(ID mbxid, T_MSG *pk_msg)
{
	MBXCB	*p_mbxcb;
	TCB		*p_tcb;
	ER		ercd;
    
	LOG_SND_MBX_ENTER(mbxid, pk_msg);
	CHECK_TSKCTX_UNL();
	CHECK_MBXID(mbxid);
	p_mbxcb = get_mbxcb(mbxid);

	t_lock_cpu();
	if (p_mbxcb->p_mbxinib->mbxatr == TA_NOEXS) {
		ercd = E_NOEXS;
	}
	else if (!((p_mbxcb->p_mbxinib->mbxatr & TA_MPRI) == 0U
				|| (TMIN_MPRI <= MSGPRI(pk_msg)
					&& MSGPRI(pk_msg) <= p_mbxcb->p_mbxinib->maxmpri))) {
		ercd = E_PAR;
	}
	else if (!queue_empty(&(p_mbxcb->wait_queue))) {
		p_tcb = (TCB *) queue_delete_next(&(p_mbxcb->wait_queue));
		((WINFO_MBX *)(p_tcb->p_winfo))->pk_msg = pk_msg;
		if (wait_complete(p_tcb)) {
			dispatch();
		}
		ercd = E_OK;
	}
	else if ((p_mbxcb->p_mbxinib->mbxatr & TA_MPRI) != 0U) {
		enqueue_msg_pri(&(p_mbxcb->pk_head), pk_msg);
		ercd = E_OK;
	}
	else {
		pk_msg->pk_next = NULL;
		if (p_mbxcb->pk_head != NULL) {
			p_mbxcb->pk_last->pk_next = pk_msg;
		}
		else {
			p_mbxcb->pk_head = pk_msg;
		}
		p_mbxcb->pk_last = pk_msg;
		ercd = E_OK;
	}
	t_unlock_cpu();

  error_exit:
	LOG_SND_MBX_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_snd_mbx */

/*
 *  メールボックスからの受信
 */
#ifdef TOPPERS_rcv_mbx

ER
rcv_mbx(ID mbxid, T_MSG **ppk_msg)
{
	MBXCB	*p_mbxcb;
	WINFO_MBX winfo_mbx;
	ER		ercd;
    
	LOG_RCV_MBX_ENTER(mbxid, ppk_msg);
	CHECK_DISPATCH();
	CHECK_MBXID(mbxid);
	p_mbxcb = get_mbxcb(mbxid);
    
	t_lock_cpu();
	if (p_mbxcb->p_mbxinib->mbxatr == TA_NOEXS) {
		ercd = E_NOEXS;
	}
	else if (p_mbxcb->pk_head != NULL) {
		*ppk_msg = p_mbxcb->pk_head;
		p_mbxcb->pk_head = (*ppk_msg)->pk_next;
		ercd = E_OK;
	}
	else {
		p_runtsk->tstat = (TS_WAITING | TS_WAIT_MBX);
		wobj_make_wait((WOBJCB *) p_mbxcb, (WINFO_WOBJ *) &winfo_mbx);
		dispatch();
		ercd = winfo_mbx.winfo.wercd;
		if (ercd == E_OK) {
			*ppk_msg = winfo_mbx.pk_msg;
		}
	}
	t_unlock_cpu();

  error_exit:
	LOG_RCV_MBX_LEAVE(ercd, *ppk_msg);
	return(ercd);
}

#endif /* TOPPERS_rcv_mbx */

/*
 *  メールボックスからの受信（ポーリング）
 */
#ifdef TOPPERS_prcv_mbx

ER
prcv_mbx(ID mbxid, T_MSG **ppk_msg)
{
	MBXCB	*p_mbxcb;
	ER		ercd;
    
	LOG_PRCV_MBX_ENTER(mbxid, ppk_msg);
	CHECK_TSKCTX_UNL();
	CHECK_MBXID(mbxid);
	p_mbxcb = get_mbxcb(mbxid);
    
	t_lock_cpu();
	if (p_mbxcb->p_mbxinib->mbxatr == TA_NOEXS) {
		ercd = E_NOEXS;
	}
	else if (p_mbxcb->pk_head != NULL) {
		*ppk_msg = p_mbxcb->pk_head;
		p_mbxcb->pk_head = (*ppk_msg)->pk_next;
		ercd = E_OK;
	}
	else {
		ercd = E_TMOUT;
	}
	t_unlock_cpu();

  error_exit:
	LOG_PRCV_MBX_LEAVE(ercd, *ppk_msg);
	return(ercd);
}

#endif /* TOPPERS_prcv_mbx */

/*
 *  メールボックスからの受信（タイムアウトあり）
 */
#ifdef TOPPERS_trcv_mbx

ER
trcv_mbx(ID mbxid, T_MSG **ppk_msg, TMO tmout)
{
	MBXCB	*p_mbxcb;
	WINFO_MBX winfo_mbx;
	TMEVTB	tmevtb;
	ER		ercd;
    
	LOG_TRCV_MBX_ENTER(mbxid, ppk_msg, tmout);
	CHECK_DISPATCH();
	CHECK_MBXID(mbxid);
	CHECK_TMOUT(tmout);
	p_mbxcb = get_mbxcb(mbxid);
    
	t_lock_cpu();
	if (p_mbxcb->p_mbxinib->mbxatr == TA_NOEXS) {
		ercd = E_NOEXS;
	}
	else if (p_mbxcb->pk_head != NULL) {
		*ppk_msg = p_mbxcb->pk_head;
		p_mbxcb->pk_head = (*ppk_msg)->pk_next;
		ercd = E_OK;
	}
	else if (tmout == TMO_POL) {
		ercd = E_TMOUT;
	}
	else {
		p_runtsk->tstat = (TS_WAITING | TS_WAIT_MBX);
		wobj_make_wait_tmout((WOBJCB *) p_mbxcb, (WINFO_WOBJ *) &winfo_mbx,
														&tmevtb, tmout);
		dispatch();
		ercd = winfo_mbx.winfo.wercd;
		if (ercd == E_OK) {
			*ppk_msg = winfo_mbx.pk_msg;
		}
	}
	t_unlock_cpu();

  error_exit:
	LOG_TRCV_MBX_LEAVE(ercd, *ppk_msg);
	return(ercd);
}

#endif /* TOPPERS_trcv_mbx */

/*
 *  メールボックスの再初期化
 */
#ifdef TOPPERS_ini_mbx

ER
ini_mbx(ID mbxid)
{
	MBXCB	*p_mbxcb;
	bool_t	dspreq;
	ER		ercd;
    
	LOG_INI_MBX_ENTER(mbxid);
	CHECK_TSKCTX_UNL();
	CHECK_MBXID(mbxid);
	p_mbxcb = get_mbxcb(mbxid);

	t_lock_cpu();
	if (p_mbxcb->p_mbxinib->mbxatr == TA_NOEXS) {
		ercd = E_NOEXS;
	}
	else {
		dspreq = init_wait_queue(&(p_mbxcb->wait_queue));
		p_mbxcb->pk_head = NULL;
		if (dspreq) {
			dispatch();
		}
		ercd = E_OK;
	}
	t_unlock_cpu();

  error_exit:
	LOG_INI_MBX_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_ini_mbx */

/*
 *  メールボックスの状態参照
 */
#ifdef TOPPERS_ref_mbx

ER
ref_mbx(ID mbxid, T_RMBX *pk_rmbx)
{
	MBXCB	*p_mbxcb;
	ER		ercd;
    
	LOG_REF_MBX_ENTER(mbxid, pk_rmbx);
	CHECK_TSKCTX_UNL();
	CHECK_MBXID(mbxid);
	p_mbxcb = get_mbxcb(mbxid);

	t_lock_cpu();
	if (p_mbxcb->p_mbxinib->mbxatr == TA_NOEXS) {
		ercd = E_NOEXS;
	}
	else {
		pk_rmbx->wtskid = wait_tskid(&(p_mbxcb->wait_queue));
		pk_rmbx->pk_msg = p_mbxcb->pk_head;
		ercd = E_OK;
	}
	t_unlock_cpu();

  error_exit:
	LOG_REF_MBX_LEAVE(ercd, pk_rmbx);
	return(ercd);
}

#endif /* TOPPERS_ref_mbx */
