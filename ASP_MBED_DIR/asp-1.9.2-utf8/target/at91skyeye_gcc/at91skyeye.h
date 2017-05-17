/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 *
 *  Copyright (C) 2006 by GJ Business Division RICOH COMPANY,LTD. JAPAN
 *  Copyright (C) 2007-2010 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: at91skyeye.h 2741 2016-01-09 04:25:00Z ertl-honda $
 */

#ifndef TOPPERS_AT91SKYEYE_H
#define TOPPERS_AT91SKYEYE_H

#include <sil.h>

/*
 * 割込みハンドラ番号から，IRC操作のためのビットパターンを求めるマクロ
 */
#define INTNO_BITPAT(intno) (1U << intno)

/*
 *  割込みコントローラ関係
 */
#define AIC_IVR      (0xFFFFF100)
#define AIC_ISR      (0xFFFFF108)
#define AIC_IMR      (0xFFFFF110)
#define AIC_IECR     (0xFFFFF120)
#define AIC_IDCR     (0xFFFFF124)
#define AIC_ICCR     (0xFFFFF128)
#define AIC_EOI      (0xFFFFF130)
#define AIC_IPR      (0xFFFFF10C)

#define IRQ_USART0      2
#define IRQ_TC1         5

/*
 *  タイマ関連
 */
#define TIMER_1_CCR      (0xFFFE0040)
#define TIMER_1_CMR      (0xFFFE0044)
#define TIMER_1_CV       (0xFFFE0050)
#define TIMER_1_SR       (0xFFFE0060)
#define TIMER_1_RC       (0xFFFE005C)
#define PIT_PIVR         (0xFFFFFD38)

/*
 * デバイスマネージャのマルチプロセッサ向け機能へのアクセスレジスタ
 * skyeyeのconfファイルの設定と整合させる 
 */
#define MUTEX_ID_REG     (0xFFFFFF00)
#define MUTEX_CNT_REG    (0xFFFFFF04)
#define IPI_REG          (0xffffff80)

/*
 *  UART関連
 */
#define USART0_THR       (0xFFFD001C)  // USART Transmitter Holding Register
#define USART0_RPR       (0xFFFD0030)  // USART Receive Pointer Register
#define USART0_RCR       (0xFFFD0034)  // USART Receive Counter Register
#define USART0_CSR       (0xFFFD0014)  // USART Channel Satus Register
#define USART0_IER       (0xFFFD0008)  // USART Interrupt Enable Register
#define USART0_IDR       (0xFFFD000C)  // USART Interrupt Disable Register

/* 
 *  サイクルカウンタ関連 
 */ 
#define CYCLE_COUNTER_REG (0xFFFFFFC0)

/*
 *  バージョンレジスタ
 */
#define SKYEYE_VER_REG  (0xFFFE1000)
#define DEVM_VER_REG    (0xFFFE1010)

#ifndef TOPPERS_MACRO_ONLY

/*
 * IRC操作関数
 */

/*
 * 割込み要求のマスク
 */
Inline void
at91skyeye_disable_int(uint32_t mask)
{
    sil_wrw_mem((void *)(AIC_IDCR), mask);
}

/*
 * 割込み要求のマスクの解除
 */
Inline void
at91skyeye_enable_int(uint32_t mask)
{
    sil_wrw_mem((void *)(AIC_IECR), mask);
}

/*
 * 割込み要求のクリア
 */
Inline void
at91skyeye_clear_int(uint32_t mask)
{
    sil_wrw_mem((void *)(AIC_ICCR), mask);	
}

/*
 * 割込み要求のチェック
 */
Inline bool_t
at91skyeye_probe_int(uint32_t mask)
{
    return((sil_rew_mem((void *)(AIC_IPR)) & mask) == mask);
}

/*
 * 終了処理
 */
Inline void
at91skyeye_exit(void)
{    
}

/* 
 *  サイクルカウンタの読み込み 
 */ 
Inline uint32_t 
cycle_counter_get_current(void)
{ 
        return sil_rew_mem((void*)(CYCLE_COUNTER_REG)); 
}

/*
 *  トレースログに関する定義
 *  サイクルカウンタを用いる．
 */
#define TRACE_GET_TIM()   cycle_counter_get_current()

#endif /* TOPPPERS_MACRO_ONLY */


#endif /* TOPPERS_AT91SAM7S_H */
