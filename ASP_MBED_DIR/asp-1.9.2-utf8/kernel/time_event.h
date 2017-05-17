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
 *  $Id: time_event.h 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		タイムイベント管理モジュール
 */

#ifndef TOPPERS_TIME_EVENT_H
#define TOPPERS_TIME_EVENT_H

/*
 *  イベント発生時刻のデータ型の定義
 *
 *  EVTTIMは，RELTIMとして指定できる範囲よりも広い範囲を表現できる必要
 *  がある．μITRON4.0仕様のスタンダードプロファイルでは，RELTIMが16ビッ
 *  ト以上でなければならないため，EVTTIMは17ビット以上であることが必要
 *  である．そのため，16ビットになる場合があるuint_tではなく，ulong_tに
 *  定義している．
 */
typedef ulong_t	EVTTIM;

/* 
 *  タイムイベントブロックのデータ型の定義
 */
typedef void	(*CBACK)(void *);	/* コールバック関数の型 */

typedef struct time_event_block {
	uint_t	index;			/* タイムイベントヒープ中での位置 */
	CBACK	callback;		/* コールバック関数 */
	void	*arg;			/* コールバック関数へ渡す引数 */
} TMEVTB;

/*
 *  タイムイベントヒープ中のノードのデータ型の定義
 */
typedef struct time_event_node {
	EVTTIM	time;			/* イベント発生時刻 */
	TMEVTB	*p_tmevtb;		/* 対応するタイムイベントブロック */
} TMEVTN;

/*
 *  タイムイベントヒープ（kernel_cfg.c）
 */
extern TMEVTN	tmevt_heap[];

/*
 *  現在のシステム時刻（単位: 1ミリ秒）
 *
 *  システム起動時に0に初期化され，以降，タイムティックが供給される度に
 *  単調に増加する．
 */
extern EVTTIM	current_time;

/*
 *  タイムイベントヒープ中で有効な最小のシステム時刻（単位: 1ミリ秒）
 */
extern EVTTIM	min_time;

/*
 *  次のタイムティックのシステム時刻（単位: 1ミリ秒）
 */
extern EVTTIM	next_time;

/*
 *  システム時刻積算用変数（単位: 1/TIC_DENOミリ秒）
 *
 *  次のタイムティックのシステム時刻の下位桁を示す（上位桁はnext_time）．
 *  TIC_DENOが1の時は，下位桁は常に0であるため，この変数は必要ない．
 */
#if TIC_DENO != 1U
extern uint_t	next_subtime;
#endif /* TIC_DENO != 1U */

/*
 *  相対時間の基準時刻（単位: 1ミリ秒）
 *
 *  次のタイムティックのシステム時刻を切り上げた時刻．TIC_DENOが1の時
 *  は，next_timeに一致する．
 */
#if TIC_DENO == 1U
#define	base_time	(next_time)
#else /* TIC_DENO == 1U */
#define	base_time	(next_time + (next_subtime > 0U ? 1U : 0U))
#endif /* TIC_DENO == 1U */

/*
 *  タイムイベントヒープの最後の使用領域のインデックス
 *
 *  タイムイベントヒープに登録されているタイムイベントの数に一致する．
 */
extern uint_t	last_index;

/*
 *  タイムイベント管理モジュールの初期化
 */
extern void	initialize_tmevt(void);

/*
 *  タイムイベントの挿入位置の探索
 */
extern uint_t	tmevt_up(uint_t index, EVTTIM time);
extern uint_t	tmevt_down(uint_t index, EVTTIM time);

/*
 *  タイムイベントヒープへの登録と削除
 */
extern void	tmevtb_insert(TMEVTB *p_tmevtb, EVTTIM time);
extern void	tmevtb_delete(TMEVTB *p_tmevtb);

/*
 *  タイムイベントブロックの登録（相対時間指定）
 *
 *  timeで指定した相対時間が経過した後に，argを引数としてcallbackが呼
 *  び出されるように，p_tmevtbで指定したタイムイベントブロックを登録す
 *  る．
 *  
 */
Inline void
tmevtb_enqueue(TMEVTB *p_tmevtb, RELTIM time, CBACK callback, void *arg)
{
	assert(time <= TMAX_RELTIM);

	p_tmevtb->callback = callback;
	p_tmevtb->arg = arg;
	tmevtb_insert(p_tmevtb, base_time + time);
}

/*
 *  タイムイベントブロックの登録（イベント発生時刻指定）
 *
 *  timeで指定したイベント発生時刻に，argを引数としてcallbackが呼び出
 *  されるように，p_tmevtbで指定したタイムイベントブロックを登録する．
 */
Inline void
tmevtb_enqueue_evttim(TMEVTB *p_tmevtb, EVTTIM time, CBACK callback, void *arg)
{
	p_tmevtb->callback = callback;
	p_tmevtb->arg = arg;
	tmevtb_insert(p_tmevtb, time);
}

/*
 *  タイムイベントブロックの登録解除
 */
Inline void
tmevtb_dequeue(TMEVTB *p_tmevtb)
{
	tmevtb_delete(p_tmevtb);
}

/*
 *  タイムイベントまでの残り時間の計算
 */
extern RELTIM	tmevt_lefttim(TMEVTB *p_tmevtb);

/*
 *  タイムティックの供給
 */
extern void	signal_time(void);

#endif /* TOPPERS_TIME_EVENT_H */
