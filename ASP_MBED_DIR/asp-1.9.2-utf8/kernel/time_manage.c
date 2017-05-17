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
 *  $Id: time_manage.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		システム時刻管理機能
 */

#include "kernel_impl.h"
#include "check.h"
#include "time_event.h"

/*
 *  トレースログマクロのデフォルト定義
 */
#ifndef LOG_GET_TIM_ENTER
#define LOG_GET_TIM_ENTER(p_systim)
#endif /* LOG_GET_TIM_ENTER */

#ifndef LOG_GET_TIM_LEAVE
#define LOG_GET_TIM_LEAVE(ercd, systim)
#endif /* LOG_GET_TIM_LEAVE */

#ifndef LOG_GET_UTM_ENTER
#define LOG_GET_UTM_ENTER(p_sysutm)
#endif /* LOG_GET_UTM_ENTER */

#ifndef LOG_GET_UTM_LEAVE
#define LOG_GET_UTM_LEAVE(ercd, sysutm)
#endif /* LOG_GET_UTM_LEAVE */

/*
 *  システム時刻の参照
 */
#ifdef TOPPERS_get_tim

ER
get_tim(SYSTIM *p_systim)
{
	ER		ercd;

	LOG_GET_TIM_ENTER(p_systim);
	CHECK_TSKCTX_UNL();

	t_lock_cpu();
	*p_systim = current_time;
	ercd = E_OK;
	t_unlock_cpu();

  error_exit:
	LOG_GET_TIM_LEAVE(ercd, *p_systim);
	return(ercd);
}

#endif /* TOPPERS_get_tim */

/*
 *  性能評価用システム時刻の参照
 */
#ifdef TOPPERS_get_utm
#ifdef TOPPERS_SUPPORT_GET_UTM
#ifndef OMIT_GET_UTM
#include "target_timer.h"

ER
get_utm(SYSUTM *p_sysutm)
{
	SYSUTM	utime;
	SYSTIM	time;
#if TIC_DENO != 1
	uint_t	subtime;
#endif /* TIC_DENO != 1 */
	CLOCK	clock1, clock2;
	bool_t	ireq;
	SIL_PRE_LOC;

	LOG_GET_UTM_ENTER(p_sysutm);

	SIL_LOC_INT();
	time = next_time;
#if TIC_DENO != 1
	subtime = next_subtime;
#endif /* TIC_DENO != 1 */
	clock1 = target_timer_get_current();
	ireq = target_timer_probe_int();
	clock2 = target_timer_get_current();
	SIL_UNL_INT();

	utime = ((SYSUTM) time) * 1000U;
#if TIC_DENO != 1
	utime += subtime * 1000U / TIC_DENO;
#endif /* TIC_DENO != 1 */

	if (!ireq || clock1 > clock2) {
		utime -= TIC_NUME * 1000U / TIC_DENO;
	}
	utime += TO_USEC(clock1);
	*p_sysutm = utime;

	LOG_GET_UTM_LEAVE(E_OK, *p_sysutm);
	return(E_OK);
}

#endif /* OMIT_GET_UTM */
#endif /* TOPPERS_SUPPORT_GET_UTM */
#endif /* TOPPERS_get_utm */
