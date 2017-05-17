/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
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
 *  $Id: target_timer.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		タイマドライバ（Mac OS X用）
 */

#include "kernel_impl.h"
#include "time_event.h"
#include "target_timer.h"
#include <sys/time.h>

#ifndef TOPPERS_SUPPORT_OVRHDR

/*
 *		オーバランハンドラ機能をサポートしない場合
 */

#if 0
/*
 *		インターバルタイマを周期タイマとして使用する方法
 *
 *  インターバルタイマを周期タイマとして使用するのが素直であるが，現在
 *  のMac OS Xでは，SIGALRMの配送が遅延することがあるためか，タイマが次
 *  の周期に進んだにもかかわらず，SIGALRMの配送がsigpendingで検出できな
 *  い場合がある．これにより，性能評価用システム時刻の参照が正しく動作
 *  しないため，このコードは使用していない．
 */

/*
 *  タイマの起動処理
 */
void
target_timer_initialize(intptr_t exinf)
{
	CLOCK	cyc = TO_CLOCK(TIC_NUME, TIC_DENO);
	struct itimerval	val;

	/*
	 *  タイマ周期を設定し，タイマの動作を開始する．
	 */
	assert(cyc <= MAX_CLOCK);
	val.it_interval.tv_sec = 0;
	val.it_interval.tv_usec = cyc;
	val.it_value = val.it_interval;
	setitimer(ITIMER_REAL, &val, NULL);
}

/*
 *  タイマの停止処理
 */
void
target_timer_terminate(intptr_t exinf)
{
	struct itimerval	val;

	/*
	 *  タイマの動作を停止する．
	 */
	val.it_interval.tv_sec = 0;
	val.it_interval.tv_usec = 0;
	val.it_value = val.it_interval;
	setitimer(ITIMER_REAL, &val, NULL);
}

/*
 *  タイマの現在値の読出し
 */
CLOCK
target_timer_get_current(void)
{
	struct itimerval	val;

	getitimer(ITIMER_REAL, &val);
	return(TO_CLOCK(TIC_NUME, TIC_DENO) - val.it_value.tv_usec);
}

/*
 *  タイマ割込み要求のチェック
 */
bool_t
target_timer_probe_int(void)
{
	return(x_probe_int(INTNO_TIMER));
}

/*
 *  タイマ割込みハンドラ
 */
void
target_timer_handler(void)
{
	i_begin_int(INTNO_TIMER);
	signal_time();					/* タイムティックの供給 */
	i_end_int(INTNO_TIMER);
}

#else /* 0 */
/*
 *		インターバルタイマをワンショットタイマとして使用する方法
 */

/*
 *  タイマの動作開始
 */
Inline void
itimer_start(void)
{
	CLOCK	cyc = TO_CLOCK(TIC_NUME, TIC_DENO);
	struct itimerval	val;

	/*
	 *  タイマ周期を設定し，タイマの動作を開始する．
	 */
	assert(cyc <= MAX_CLOCK);
	val.it_interval.tv_sec = 0;
	val.it_interval.tv_usec = 0;
	val.it_value.tv_sec = 0;
	val.it_value.tv_usec = cyc;
	setitimer(ITIMER_REAL, &val, NULL);
}

/*
 *  タイマの起動処理
 */
void
target_timer_initialize(intptr_t exinf)
{
	itimer_start();
}

/*
 *  タイマの停止処理
 */
void
target_timer_terminate(intptr_t exinf)
{
	struct itimerval	val;

	/*
	 *  タイマの動作を停止する．
	 */
	val.it_interval.tv_sec = 0;
	val.it_interval.tv_usec = 0;
	val.it_value = val.it_interval;
	setitimer(ITIMER_REAL, &val, NULL);
}

/*
 *  タイマの現在値の読出し
 */
CLOCK
target_timer_get_current(void)
{
	struct itimerval	val;

	getitimer(ITIMER_REAL, &val);
	return(TO_CLOCK(TIC_NUME, TIC_DENO) - val.it_value.tv_usec);
}

/*
 *  タイマ割込み要求のチェック
 */
bool_t
target_timer_probe_int(void)
{
	return(false);
}

/*
 *  タイマ割込みハンドラ
 */
void
target_timer_handler(void)
{
	i_begin_int(INTNO_TIMER);
	i_lock_cpu();
	itimer_start();
	i_unlock_cpu();
	signal_time();					/* タイムティックの供給 */
	i_end_int(INTNO_TIMER);
}

#endif /* 0 */
#else /* TOPPERS_SUPPORT_OVRHDR */

/*
 *		オーバランハンドラ機能をサポートする場合
 *
 *  タイムティック供給のためのタイマ（ティックタイマ）とオーバランハン
 *  ドラ機能のためのタイマ（オーバランタイマ）を，1つのインターバルタイ
 *  マを多重化して実現している．この方法は，ティックの周期がずれるため
 *  に推奨できないが，マイクロ秒精度のタイマが1つしかなく，シミュレーショ
 *  ン環境でティック周期のずれは大きい問題ではないため，この方法を採用
 *  している．
 */
#include "task.h"
#include "overrun.h"

static CLOCK	ticktimer_cyc;		/* ティックタイマの周期 */

static bool_t	itimer_ticktimer;	/* インターバルタイマに
											ティックタイマが設定されている */
static CLOCK	ticktimer_left;		/* ティックタイマの残り時間 */
static bool_t	ovrtimer_active;	/* オーバランタイマが有効 */
static OVRTIM	ovrtimer_left;		/* オーバランタイマの残り時間 */

#define CLOCK_TO_OVRTIM(clock)		((OVRTIM) clock)
#define OVRTIM_TO_CLOCK(ovrtim)		((CLOCK) ovrtim)

static const struct itimerval		itimerval_stop = {{ 0, 0 }, { 0, 0 }};

/*
 *  タイマの動作開始
 */
Inline void
itimer_start(void)
{
	struct itimerval	val;

	/*
	 *  タイマを設定し，タイマの動作を開始する．
	 */
	if (!ovrtimer_active || CLOCK_TO_OVRTIM(ticktimer_left) <= ovrtimer_left) {
		val.it_value.tv_sec = 0;
		val.it_value.tv_usec = ticktimer_left;
		itimer_ticktimer = true;
	}
	else {
		val.it_value.tv_sec = 0;
		val.it_value.tv_usec = OVRTIM_TO_CLOCK(ovrtimer_left);
		itimer_ticktimer = false;
	}
	val.it_interval.tv_sec = 0;
	val.it_interval.tv_usec = 0;
	setitimer(ITIMER_REAL, &val, NULL);
}

/*
 *  タイマの動作停止
 */
Inline void
itimer_stop(void)
{
	struct itimerval	val;
	CLOCK				left;

	/*
	 *  タイマの動作を停止し，その時点の残り時間をleftに取り出す．残り
	 *  時間が0になっていれば1に読み換える．この時に，スプリアス割込み
	 *  を発生させる可能性がある．
	 */
	setitimer(ITIMER_REAL, &itimerval_stop, &val);
	left = val.it_value.tv_usec;
	if (left == 0) {
		left = 1;
	}

	/*
	 *  タイマの残り時間から，ティックタイマとオーバランタイマの残り時
	 *  間を設定し直す．
	 */
	if (itimer_ticktimer) {
		if (ovrtimer_active) {
			ovrtimer_left -= CLOCK_TO_OVRTIM(ticktimer_left - left);
		}
		ticktimer_left = left;
	}
	else {
		ticktimer_left -= (OVRTIM_TO_CLOCK(ovrtimer_left) - left);
		ovrtimer_left = CLOCK_TO_OVRTIM(left);
	}
}

/*
 *  タイマの起動処理
 */
void
target_timer_initialize(intptr_t exinf)
{
	ticktimer_cyc = TO_CLOCK(TIC_NUME, TIC_DENO);
	assert(ticktimer_cyc <= MAX_CLOCK);

	ticktimer_left = ticktimer_cyc;
	ovrtimer_active = false;
	itimer_start();
}

/*
 *  タイマの停止処理
 */
void
target_timer_terminate(intptr_t exinf)
{
	/*
	 *  タイマの動作を停止する．
	 */
	setitimer(ITIMER_REAL, &itimerval_stop, NULL);
}

/*
 *  タイマの現在値の読出し
 */
CLOCK
target_timer_get_current(void)
{
	struct itimerval	val;

	if (itimer_ticktimer) {
		getitimer(ITIMER_REAL, &val);
		if (val.it_value.tv_usec == 0) {
			return(0);
		}
		else {
			return(TO_CLOCK(TIC_NUME, TIC_DENO) - val.it_value.tv_usec);
		}
	}
	else {
		getitimer(ITIMER_REAL, &val);
		return(ticktimer_left - (OVRTIM_TO_CLOCK(ovrtimer_left)
												- val.it_value.tv_usec));
	}
}

/*
 *  タイマ割込み要求のチェック
 */
bool_t
target_timer_probe_int(void)
{
	struct itimerval	val;

	if (itimer_ticktimer) {
		getitimer(ITIMER_REAL, &val);
		return(val.it_value.tv_usec == 0);
	}
	else {
		return(false);
	}
}

/*
 *  オーバランタイマの初期化処理
 *
 *  必要な処理をすべてtarget_timer_initializeで行っているので，こちらで
 *  は何もしない．
 */
void
target_ovrtimer_initialize(intptr_t exinf)
{
}

/*
 *  オーバランタイマの停止処理
 *
 *  必要な処理をすべてtarget_timer_terminateで行っているので，こちらで
 *  は何もしない．
 */
void
target_ovrtimer_terminate(intptr_t exinf)
{
}

/*
 *  オーバランタイマの動作開始
 */
void
target_ovrtimer_start(OVRTIM ovrtim)
{
	assert(!ovrtimer_active);
	itimer_stop();
	ovrtimer_active = true;
	ovrtimer_left = ovrtim;
	itimer_start();
}

/*
 *  オーバランタイマの停止
 */
OVRTIM
target_ovrtimer_stop(void)
{
	struct itimerval	val;

	assert(ovrtimer_active);
	if (itimer_ticktimer) {
		getitimer(ITIMER_REAL, &val);
		ovrtimer_left -= CLOCK_TO_OVRTIM(ticktimer_left
												- val.it_value.tv_usec);
		if (ovrtimer_left == 0) {
			ovrtimer_left = 1;
		}
		ovrtimer_active = false;
	}
	else {
		itimer_stop();
		ovrtimer_active = false;
		itimer_start();
	}
	return(ovrtimer_left);
}

/*
 *  オーバランタイマの現在値の読出し
 */
OVRTIM
target_ovrtimer_get_current(void)
{
	struct itimerval	val;
	OVRTIM				ovrtimer_current;

	assert(ovrtimer_active);
	if (itimer_ticktimer) {
		getitimer(ITIMER_REAL, &val);
		ovrtimer_current = ovrtimer_left
					- CLOCK_TO_OVRTIM(ticktimer_left - val.it_value.tv_usec);
	}
	else {
		getitimer(ITIMER_REAL, &val);
		ovrtimer_current = CLOCK_TO_OVRTIM(val.it_value.tv_usec);
	}
	return(ovrtimer_current);
}

/*
 *  タイマ割込みハンドラ
 *
 *  タイマ割込みハンドラの入口処理で，ovrtimer_stopが呼び出される．オー
 *  バランタイマが動作していた場合には，そこからtarget_ovrtimer_stopが
 *  呼び出される．インターバルタイマにオーバランタイマが設定されていた
 *  場合には，そこからitimer_stopとitimer_startが呼び出されて，インター
 *  バルタイマにティックタイマが設定され，インターバルタイマが動作開始
 *  される．逆に，インターバルタイマにティックタイマが設定されていた場
 *  合には，インターバルタイマは停止したままとなっている．
 */
void
target_timer_handler(void)
{
	struct itimerval			val;

	i_begin_int(INTNO_TIMER);
	i_lock_cpu();
	getitimer(ITIMER_REAL, &val);
	if (val.it_value.tv_usec == 0) {
		/*
		 *  インターバルタイマが停止しているのは，インターバルタイマに
		 *  ティックタイマが設定されていた場合である．
		 */
		ticktimer_left = ticktimer_cyc;

		val.it_value.tv_sec = 0;
		val.it_value.tv_usec = ticktimer_left;
		val.it_interval.tv_sec = 0;
		val.it_interval.tv_usec = 0;
		setitimer(ITIMER_REAL, &val, NULL);

		i_unlock_cpu();
		signal_time();				/* タイムティックの供給 */
	}
	else {
		/*
		 *  インターバルタイマが動作しているのは，インターバルタイマに
		 *  オーバランタイマが設定されていた場合である．
		 */
		if (p_runtsk->leftotm == 1) {
			/*
			 *  スプリアス割込みでない場合に，オーバランタイマを起動す
			 *  る．
			 */
			i_unlock_cpu();
			call_ovrhdr();			/* オーバランハンドラの起動 */
		}
		else {
			i_unlock_cpu();
		}
	}
	i_end_int(INTNO_TIMER);
}

#endif /* TOPPERS_SUPPORT_OVRHDR */
