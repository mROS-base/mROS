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
 *  @(#) $Id: exception.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		CPU例外管理機能
 */

#include "kernel_impl.h"
#include "task.h"
#include "exception.h"

/*
 *  トレースログマクロのデフォルト定義
 */
#ifndef LOG_XSNS_DPN_ENTER
#define LOG_XSNS_DPN_ENTER(p_excinf)
#endif /* LOG_XSNS_DPN_ENTER */

#ifndef LOG_XSNS_DPN_LEAVE
#define LOG_XSNS_DPN_LEAVE(state)
#endif /* LOG_XSNS_DPN_LEAVE */

#ifndef LOG_XSNS_XPN_ENTER
#define LOG_XSNS_XPN_ENTER(p_excinf)
#endif /* LOG_XSNS_XPN_ENTER */

#ifndef LOG_XSNS_XPN_LEAVE
#define LOG_XSNS_XPN_LEAVE(state)
#endif /* LOG_XSNS_XPN_LEAVE */

/* 
 *  CPU例外ハンドラ管理機能の初期化
 */
#ifdef TOPPERS_excini
#ifndef OMIT_INITIALIZE_EXCEPTION

void
initialize_exception(void)
{
	uint_t			i;
	const EXCINIB	*p_excinib;

	for (i = 0; i < tnum_excno; i++) {
		p_excinib = &(excinib_table[i]);
		x_define_exc(p_excinib->excno, p_excinib->exc_entry);
	}
}

#endif /* OMIT_INITIALIZE_EXCEPTION */
#endif /* TOPPERS_excini */

/*
 *  CPU例外の発生したコンテキストの参照
 */

/*
 *  CPU例外発生時のディスパッチ保留状態の参照
 *
 *  CPU例外ハンドラ中でdisdspが変化することはないため，CPU例外が発生し
 *  た時のdisdspを保存しておく必要はない．
 */
#ifdef TOPPERS_xsns_dpn

bool_t
xsns_dpn(void *p_excinf)
{
	bool_t	state;

	LOG_XSNS_DPN_ENTER(p_excinf);
	state = (kerflg && exc_sense_intmask(p_excinf)
									&& !disdsp) ? false : true;
	LOG_XSNS_DPN_LEAVE(state);
	return(state);
}

#endif /* TOPPERS_xsns_dpn */

/*
 *  CPU例外発生時のタスク例外処理保留状態の参照
 *
 *  CPU例外ハンドラ中でp_runtskとp_runtsk->enatexが変化することはない
 *  ため，CPU例外が発生した時のp_runtsk->enatexを保存しておく必要はな
 *  い．
 */
#ifdef TOPPERS_xsns_xpn

bool_t
xsns_xpn(void *p_excinf)
{
	bool_t	state;

	LOG_XSNS_XPN_ENTER(p_excinf);
	state = (kerflg && exc_sense_intmask(p_excinf)
									&& p_runtsk->enatex) ? false : true;
	LOG_XSNS_XPN_LEAVE(state);
	return(state);
}

#endif /* TOPPERS_xsns_xpn */
