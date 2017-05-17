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
 *  @(#) $Id: chip_timer.h 2742 2016-01-09 04:25:18Z ertl-honda $
 */

/*
 *  タイマドライバ（CMT4 or TMU用）
 *  チャネル0,1,2のいずれかを使用
 */

#ifndef TOPPERS_CHIP_TIMER_H
#define TOPPERS_CHIP_TIMER_H

#include <sil.h>

/*
 * CM4レジスタ
 */
#define CM4STR  0xe6148000
#define CM4CSR  0xe6148040
#define CM4CNT  0xe6148044
#define CM4COR  0xe6148048

#define CMxSTR_STR5 0x0020
#define CMxCSR_CMF  0x8000
#define CMxCSR_OVF  0x4000
#define CMxCSR_CMM  0x0100

/*
 * TMUレジスタ
 */
#if USE_TMU_ID == 0
#define TMU_BASE  0xFFF80000
#define TMU_IRQ_BASE  (198 + 32)
#else /* USE_TMU_ID == 1 */
#define TMU_BASE  0xFFF90000
#define TMU_IRQ_BASE  (170 + 32)
#endif /* USE_TMU_ID == 0 */

#define TMU_TSTR  (TMU_BASE + 0x0004)  /* B */

#if USE_TMU_CH == 0
#define TMU_TCOR  (TMU_BASE + 0x0008)  /* W */
#define TMU_TCNT  (TMU_BASE + 0x000C)  /* W */
#define TMU_TCR   (TMU_BASE + 0x0010)  /* H */
#define TMU_STR   0x01
#define GIC_IRQNO_TMU (TMU_IRQ_BASE + 0)
#elif USE_TMU_CH == 1
#define TMU_TCOR  (TMU_BASE + 0x0014)  /* W */
#define TMU_TCNT  (TMU_BASE + 0x0018)  /* W */
#define TMU_TCR   (TMU_BASE + 0x001C)  /* H */
#define TMU_STR   0x02
#define GIC_IRQNO_TMU (TMU_IRQ_BASE + 1)
#elif USE_TMU_CH == 2
#define TMU_TCOR  (TMU_BASE + 0x0020)  /* W */
#define TMU_TCNT  (TMU_BASE + 0x0024)  /* W */
#define TMU_TCR   (TMU_BASE + 0x0028)  /* H */
#define TMU_STR   0x04
#define GIC_IRQNO_TMU (TMU_IRQ_BASE + 2)
#else /* USE_TMU_CH == 2 */
#error
#endif /* USE_TMU_CH == 0 */

/*
 *  TCRのビット定義
 */
#define TCR_UNF        0x0100
#define TCR_UNIE       0x0020
#define TCR_TPSC_4     0x0000
#define TCR_TPSC_16    0x0001
#define TCR_TPSC_64    0x0002
#define TCR_TPSC_256   0x0003
#define TCR_TPSC_1024  0x0004

#ifdef USE_CMT4

/*
 *  タイマ割込みハンドラ登録のための定数
 */
#define INHNO_TIMER     (169+32) /* 割込みハンドラ番号 */
#define INTNO_TIMER     (169+32) /* 割込み番号 */

#else /* !USE_CMT4 */

/*
 *  タイマ割込みハンドラ登録のための定数
 */
#define INHNO_TIMER     GIC_IRQNO_TMU /* 割込みハンドラ番号 */
#define INTNO_TIMER     GIC_IRQNO_TMU /* 割込み番号 */

#endif /* USE_CMT4 */

#ifndef INTPRI_TIMER
#define INTPRI_TIMER    -6            /* 割込み優先度 */
#endif /* INTPRI_TIMER */

#define INTATR_TIMER    0U            /* 割込み属性 */

#ifndef TOPPERS_MACRO_ONLY

/*
 *  タイマ値の内部表現の型
 */
typedef uint32_t    CLOCK;

/*
 *  タイマのクロックを保持する変数  
 */
extern uint32_t timer_clock;

/*
 *  タイマ値の内部表現とミリ秒・μ秒単位との変換
 */
#define TO_CLOCK(nume, deno) (timer_clock * (nume) / (deno))
#define TO_USEC(clock)       (((SYSUTM) clock) * 1000U / timer_clock)


/*
 *  設定できる最大のタイマ周期（単位は内部表現）
 */
#define MAX_CLOCK        ((CLOCK) 0xffffffffU)

/*
 *  タイマの現在値を割込み発生前の値とみなすかの判断
 */
#define GET_TOLERANCE    100U    /* 処理遅れの見積り値（単位は内部表現）*/
#define BEFORE_IREQ(clock) \
            ((clock) >= TO_CLOCK(TIC_NUME, TIC_DENO) - GET_TOLERANCE)

/*
 *  タイマの起動処理
 *
 *  タイマを初期化し，周期的なタイマ割込み要求を発生させる．
 */
extern void    target_timer_initialize(intptr_t exinf);

/*
 *  タイマの停止処理
 *
 *  タイマの動作を停止させる．
 */
extern void    target_timer_terminate(intptr_t exinf);

/*
 *  タイマの現在値の読出し
 */
Inline CLOCK
target_timer_get_current(void)
{
#ifdef USE_CMT4
	return(sil_rew_mem((void*)CM4CNT));
#else  /* !USE_CMT4 */
	return(TO_CLOCK(TIC_NUME, TIC_DENO) - sil_rew_mem((void*)TMU_TCNT));
#endif /* USE_CMT4 */
}

/*
 *  タイマ割込み要求のチェック
 */
Inline bool_t
target_timer_probe_int(void)
{
#ifdef USE_CMT4
	return(sil_reh_mem((void*)CM4CSR) & CMxCSR_CMF);
#else  /* !USE_CMT4 */
	return(sil_reh_mem((void*)TMU_TCR) & TCR_UNF);
#endif /* USE_CMT4 */
}

/*
 *  タイマ割込みハンドラ
 */
extern void    target_timer_handler(void);

#endif /* TOPPERS_MACRO_ONLY */
#endif /* TOPPERS_CHIP_TIMER_H */
