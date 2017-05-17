/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2008-2011 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: core_timer.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *  タイマドライバ（SYSTIC用）
 */
#include <sil.h>
#include "kernel_impl.h"
#include "time_event.h"
#include "target_timer.h"

/*
 *  タイマの起動処理
 */
void
target_timer_initialize(intptr_t exinf)
{
	CLOCK    cyc;
	uint32_t tmp;

#ifdef SYSTIC_USE_CALIBRATION
	/* CALIBRATIONレジスタの値を使用 */
	cyc = (sil_rew_mem((void *)SYSTIC_CALIBRATION) & SYSTIC_TENMS) / 10;
#else 
	cyc = TO_CLOCK(TIC_NUME, TIC_DENO) - 1;
#endif /* SYSTIC_USE_CALIBRATION */

	/* 停止 */
	tmp = sil_rew_mem((void *)SYSTIC_CONTROL_STATUS);
	tmp &= ~SYSTIC_ENABLE;
	sil_wrw_mem((void *)SYSTIC_CONTROL_STATUS, tmp);

	sil_wrw_mem((void *)SYSTIC_RELOAD_VALUE, cyc);
	sil_wrw_mem((void *)SYSTIC_CURRENT_VALUE, cyc);

	tmp = sil_rew_mem((void *)SYSTIC_CONTROL_STATUS);

#ifdef SYSTIC_USE_STCLK
	/* 外部クロックの使用 */
	tmp |= SYSTIC_ENABLE;
#else
	/* プロセッサクロックの使用 */
	tmp |= SYSTIC_ENABLE|SYSTIC_CLKSOURCE;
#endif /* SYSTIC_USE_STCLK */

	sil_wrw_mem((void *)SYSTIC_CONTROL_STATUS, tmp);
}

/*
 *  タイマの停止処理
 */
void
target_timer_terminate(intptr_t exinf)
{
	/* タイマを停止 */
	sil_wrw_mem((void*)SYSTIC_CONTROL_STATUS, 0x00);
}

/*
 *  タイマ割込みハンドラ
 */
void
target_timer_handler(void)
{
	/* probe_int のため，COUNTFLAGをクリア */
	sil_rew_mem((void *)SYSTIC_CONTROL_STATUS);

	i_begin_int(INTNO_TIMER);
	signal_time();                    /* タイムティックの供給 */
	i_end_int(INTNO_TIMER);
}
