/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 *
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2005-2016 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: chip_timer.h 2758 2016-03-10 15:15:26Z ertl-honda $
 */

/*
 *  タイマドライバ（RZ/A1 OSTM用）
 *  チャネル０を使用
 */

#ifndef TOPPERS_CHIP_TIMER_H
#define TOPPERS_CHIP_TIMER_H

#include <sil.h>

/* Register address */
#define TOPPERS_REG_OSTM0_BASE  0xFCFEC000
#define TOPPERS_REG_OSTM0CMP    (TOPPERS_REG_OSTM0_BASE)            /* OSTM0CMP register (32bit) */
#define TOPPERS_REG_OSTM0CNT    (TOPPERS_REG_OSTM0_BASE + 0x04)     /* OSTM0CNT register (32bit) */
#define TOPPERS_REG_OSTM0TE     (TOPPERS_REG_OSTM0_BASE + 0x10)     /* OSTM0TE register (8bit) */
#define TOPPERS_REG_OSTM0TS     (TOPPERS_REG_OSTM0_BASE + 0x14)     /* OSTM0TS register (8bit) */
#define TOPPERS_REG_OSTM0TT     (TOPPERS_REG_OSTM0_BASE + 0x18)     /* OSTM0TT register (8bit) */
#define TOPPERS_REG_OSTM0CTL    (TOPPERS_REG_OSTM0_BASE + 0x20)     /* OSTM0CTL register (8bit) */

/* OSTMnTS register */
#define TOPPERS_BIT_OSTMTS      0x01    /* OSTMnTS bit - 1:start count */

/* OSTMnTT register */
#define TOPPERS_BIT_OSTMTT      0x01    /* OSTMnTT bit - 1:stop count */

/* OSTMnCTL register */
#define TOPPERS_BIT_OSTMMD0     0x01    /* OSTMnMD0 bit - 1:enable interrupt at start */
#define TOPPERS_BIT_OSTMMD1     0x02    /* OSTMnMD1 bit - 0:interval timer mode */


/*
 *  タイマ割込みハンドラ登録のための定数
 */
#define INHNO_TIMER     TOPPERS_INTID_OSTM0
#define INTNO_TIMER     TOPPERS_INTID_OSTM0

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
 *  タイマ値の内部表現とミリ秒・μ秒単位との変換
 */
#define TO_CLOCK(nume, deno) (OSTM_CLK / 1000U * (nume) / (deno))
#define TO_USEC(clock)       (((SYSUTM) clock) * 1000000U / OSTM_CLK)


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
extern void target_timer_initialize(intptr_t exinf);

/*
 *  タイマの停止処理
 *
 *  タイマの動作を停止させる．
 */
extern void target_timer_terminate(intptr_t exinf);

/*
 *  タイマの現在値の読出し
 */
Inline CLOCK
target_timer_get_current(void)
{
	return (sil_rew_mem((void *)TOPPERS_REG_OSTM0CMP) - sil_rew_mem((void *)TOPPERS_REG_OSTM0CNT));
}

/*
 *  タイマ割込み要求のチェック
 */
Inline bool_t
target_timer_probe_int(void)
{
	return (x_probe_int(INHNO_TIMER));
}

/*
 *  タイマ割込みハンドラ
 */
extern void    target_timer_handler(void);

#endif /* TOPPERS_MACRO_ONLY */
#endif /* TOPPERS_CHIP_TIMER_H */
