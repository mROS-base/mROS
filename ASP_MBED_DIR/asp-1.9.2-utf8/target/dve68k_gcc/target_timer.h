/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2005-2008 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: target_timer.h 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		タイマドライバ（DVE68K/40用）
 */

#ifndef TOPPERS_TARGET_TIMER_H
#define TOPPERS_TARGET_TIMER_H

#include <sil.h>
#include "dve68k.h"

/*
 *  タイマ割込みハンドラ登録のための定数
 */
#define INHNO_TIMER		TINHNO_TT0		/* 割込みハンドラ番号 */
#define INTNO_TIMER		TINTNO_TT0		/* 割込み番号 */
#define INTPRI_TIMER	TIRQ_LEVEL4		/* 割込み優先度 */
#define INTATR_TIMER	TA_EDGE			/* 割込み属性 */

#ifndef TOPPERS_MACRO_ONLY

/*
 *  タイマ値の内部表現の型
 */
typedef uint32_t	CLOCK;

/*
 *  タイマ値の内部表現とミリ秒・μ秒単位との変換
 *
 *  DVE68K/40では，タイマは1μ秒毎にカウントアップする．
 */
#define TIMER_CLOCK				1000U
#define TO_CLOCK(nume, deno)	((CLOCK)(TIMER_CLOCK * (nume) / (deno)))
#define TO_USEC(clock)			(((SYSUTM) clock) * 1000U / TIMER_CLOCK)

/*
 *  設定できる最大のタイマ周期（単位は内部表現）
 */
#define MAX_CLOCK		((CLOCK) 0xffffffU)

/*
 *  タイマ停止までの時間（nsec単位）
 *
 *  値に根拠はない．
 */
#define TIMER_STOP_DELAY	200U

/*
 *  レジスタの設定値
 */
#define CSR12_START		0x80000000U		/* タイマ動作 */
#define CSR12_STOP		0x00000000U		/* タイマ停止 */

/*
 *  タイマの起動処理
 *
 *  タイマを初期化し，周期的なタイマ割込み要求を発生させる．
 */
extern void	target_timer_initialize(intptr_t exinf);

/*
 *  タイマの停止処理
 *
 *  タイマの動作を停止させる．
 */
extern void	target_timer_terminate(intptr_t exinf);

/*
 *  タイマの現在値の読出し
 */
Inline CLOCK
target_timer_get_current(void)
{
	CLOCK		clk;
	uint32_t	saved_csr12;
	SIL_PRE_LOC;

	/*
	 *  タイマの動作を一時的に停止し，タイマ値を読み出す．タイマの動作
	 *  を一時的に停止させると，システム時刻がずれるために望ましくない
	 *  が，DVE68K/40のハードウェア的な制約であり，やむをえない．ずれを
	 *  最小に抑えるために，割込みロック状態とする．
	 */
	SIL_LOC_INT();
	saved_csr12 = dga_read((void *) TADR_DGA_CSR12);
	dga_write((void *) TADR_DGA_CSR12, CSR12_STOP);
	sil_dly_nse(TIMER_STOP_DELAY);
	clk = dga_read((void *) TADR_DGA_CSR13) & 0x00ffffffU;
	dga_write((void *) TADR_DGA_CSR12, saved_csr12);
	SIL_UNL_INT();
	return(clk);
}

/*
 *  タイマ割込み要求のチェック
 */
Inline bool_t
target_timer_probe_int(void)
{
	return(x_probe_int(INTNO_TIMER));
}

/*
 *  タイマ割込みハンドラ
 */
extern void	target_timer_handler(void);

#endif /* TOPPERS_MACRO_ONLY */
#endif /* TOPPERS_TARGET_TIMER_H */
