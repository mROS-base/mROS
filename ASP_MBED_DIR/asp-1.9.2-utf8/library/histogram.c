/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
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
 *  @(#) $Id: histogram.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		実行時間分布集計モジュール
 */

#include <kernel.h>
#include <t_syslog.h>
#include <test_lib.h>
#include <histogram.h>
#include "target_test.h"

/*
 *  実行時間分布計測の数
 */
#ifndef TNUM_HIST
#define TNUM_HIST		10
#endif /* TNUM_HIST */

/*
 *  ターゲット依存部で設定変更するためのマクロ
 */
#ifndef HISTTIM						/* 実行時間計測用の時刻のデータ型 */
#define HISTTIM			SYSUTM
#endif /* HISTTIM */

#ifndef HIST_GET_TIM				/* 実行時間計測用の現在時刻の取得 */
#ifndef TOPPERS_SUPPORT_GET_UTM
#error get_utm is not supported.
#endif /* TOPPERS_SUPPORT_GET_UTM */
#define HIST_GET_TIM(p_time)	((void) get_utm(p_time))
#endif /* HIST_GET_TIM */

#ifndef HIST_CONV_TIM				/* 時刻の差から実行時間への変換 */
#define HIST_CONV_TIM(time)		((uint_t)(time))
#endif /* HIST_CONV_TIM */

#ifndef HIST_BM_HOOK				/* 実行時間計測直前に行うべき処理 */
#define HIST_BM_HOOK()			((void) 0)
#endif

/*
 *  実行時間分布計測管理ブロック
 */
typedef struct histogram_control_block {
	HISTTIM		begin_time;			/* 計測開始時刻 */
	uint_t		maxval;				/* 分布を記録する最大時間 */
	uint_t		*histarea;			/* 分布を記録するメモリ領域 */
	uint_t		over;				/* 最大時間を超えた度数 */
	uint_t		under;				/* 時間の逆転が疑われる度数 */
} HISTCB;

/*
 *  実行時間分布計測管理ブロックのエリア
 */
static HISTCB	histcb_table[TNUM_HIST];

/*
 *  実行時間分布計測IDの最小値と最大値
 */
#define TMIN_HISTID		1
#define TMAX_HISTID		(TMIN_HISTID + TNUM_HIST - 1)

/*
 *  実行時間分布計測の初期化
 */
void
init_hist(ID histid, uint_t maxval, uint_t histarea[])
{
	HISTCB	*p_histcb;
	uint_t	i;

	assert(TMIN_HISTID <= histid && histid <= TMAX_HISTID);
	p_histcb = &(histcb_table[histid - TMIN_HISTID]);

	for (i = 0; i <= maxval; i++) {
		histarea[i] = 0U;
	}
	p_histcb->maxval = maxval;
	p_histcb->histarea = histarea;
	p_histcb->over = 0U;
	p_histcb->under = 0U;
}

/*
 *  実行時間計測の開始
 */
void
begin_measure(ID histid)
{
	HISTCB	*p_histcb;

	assert(TMIN_HISTID <= histid && histid <= TMAX_HISTID);
	p_histcb = &(histcb_table[histid - TMIN_HISTID]);

	HIST_BM_HOOK();
	HIST_GET_TIM(&(p_histcb->begin_time));
}

/*
 *  実行時間計測の終了
 */
void
end_measure(ID histid)
{
	HISTCB	*p_histcb;
	HISTTIM	end_time;
	uint_t	val;

	HIST_GET_TIM(&end_time);

	assert(TMIN_HISTID <= histid && histid <= TMAX_HISTID);
	p_histcb = &(histcb_table[histid - TMIN_HISTID]);

	val = HIST_CONV_TIM(end_time - p_histcb->begin_time);
	if (val <= p_histcb->maxval) {
		p_histcb->histarea[val]++;
	}
	else if (val <= ((uint_t) INT_MAX)) {
		p_histcb->over++;
	}
	else {
		p_histcb->under++;
	}
}

/*
 *  実行時間分布計測の表示
 */
void
print_hist(ID histid)
{
	HISTCB	*p_histcb;
	uint_t	i;

	assert(TMIN_HISTID <= histid && histid <= TMAX_HISTID);
	p_histcb = &(histcb_table[histid - TMIN_HISTID]);

	for (i = 0; i <= p_histcb->maxval; i++) {
		if (p_histcb->histarea[i] > 0) {
			syslog_2(LOG_NOTICE, "%d : %d", i, p_histcb->histarea[i]);
			syslog_flush();
		}
	}
	if (p_histcb->over > 0) {
		syslog_2(LOG_NOTICE, "> %d : %d", p_histcb->maxval, p_histcb->over);
	}
	if (p_histcb->under > 0) {
		syslog_1(LOG_NOTICE, "> INT_MAX : %d", p_histcb->under);
	}
	syslog_flush();
}
