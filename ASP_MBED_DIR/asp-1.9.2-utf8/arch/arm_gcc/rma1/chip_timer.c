/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2006-2011 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: chip_timer.c 2742 2016-01-09 04:25:18Z ertl-honda $
 */

/*
 *  タイマドライバ（ARM PrimeCell Timer Module用）
 */

#include "kernel_impl.h"
#include "time_event.h"
#include <sil.h>
#include "target_timer.h"

/*
 *  タイマのクロックを保持する変数
 *  target_initialize() で初期化される．
 */
uint32_t timer_clock;

/*
 *  タイマの起動処理
 */
void
target_timer_initialize(intptr_t exinf)
{
	CLOCK    cyc = TO_CLOCK(TIC_NUME, TIC_DENO);

#ifdef USE_CMT4
	/* 32-bit, free-running, enable interrupt, RCLK/1 */
	sil_wrh_mem((void*)CM4CSR, (CMxCSR_CMM|0x27));
	sil_wrw_mem((void*)CM4COR, cyc);
	sil_wrw_mem((void*)CM4CNT, 0);
	sil_wrh_mem((void*)CM4STR, CMxSTR_STR5);
#else /* !USE_CMT4 */
	/*
	 *  タイマ周期を設定し，タイマの動作を開始する．
	 */
	sil_wrb_mem((void*)TMU_TSTR,
				(sil_reb_mem((void*)TMU_TSTR) & ~TMU_STR));  /* タイマ停止 */
	assert(cyc <= MAX_CLOCK);             /* タイマ上限値のチェック */

	sil_wrh_mem((void*)TMU_TCR, (TCR_UNIE | TCR_TPSC_4)); /* 分周比設定、割り込み許可 */
	sil_wrw_mem((void*)TMU_TCOR, cyc); /* timer constantレジスタをセット */
	sil_wrw_mem((void*)TMU_TCNT, cyc); /* カウンターをセット */

	/* 割り込み要求をクリア */
	sil_wrh_mem((void*)TMU_TCR,
				(sil_reh_mem((void*)TMU_TCR) & ~TCR_UNF));

	/* タイマスタート */
	sil_wrb_mem((void*)TMU_TSTR,
				(sil_reb_mem((void*)TMU_TSTR) | TMU_STR));
#endif /* USE_CMT4 */
}

/*
 *  タイマの停止処理
 */
void
target_timer_terminate(intptr_t exinf)
{
#ifdef USE_CMT4
	/* Stop timer */
	sil_wrh_mem((void*)CM4STR, 0);
	/* Clear interrupt request */
	sil_wrh_mem((void*)CM4CSR,
				sil_reh_mem((void*)CM4CSR) & ~(CMxCSR_CMF|CMxCSR_OVF));
#else /* !USE_CMT4 */
	/* タイマを停止 */
	sil_wrb_mem((void*)TMU_TSTR,
				(sil_reb_mem((void*)TMU_TSTR) & ~TMU_STR));
	/* 割り込み要求をクリア */
	sil_wrh_mem((void*)TMU_TCR,0);
#endif /* USE_CMT4 */
}

/*
 *  タイマ割込みハンドラ
 */
void
target_timer_handler(void)
{
#ifdef USE_CMT4
	/* Clear interrupt request */
	sil_wrh_mem((void*)CM4CSR,
				sil_reh_mem((void*)CM4CSR) & ~(CMxCSR_CMF|CMxCSR_OVF));
#else /* !USE_CMT4 */
	/* 割り込み要求をクリア */
	sil_wrh_mem((void*)TMU_TCR,
				(sil_reh_mem((void*)TMU_TCR) & ~TCR_UNF));
#endif /* USE_CMT4 */

	i_begin_int(INTNO_TIMER);
	signal_time();                    /* タイムティックの供給 */
	i_end_int(INTNO_TIMER);
}
