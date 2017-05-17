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
 *  @(#) $Id: overrun.h 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		オーバランハンドラ機能
 */

#ifndef TOPPERS_OVERRUN_H
#define TOPPERS_OVERRUN_H

#ifdef TOPPERS_SUPPORT_OVRHDR
#include "target_timer.h"

/*
 *  オーバランハンドラ初期化ブロック
 */
typedef struct overrun_handler_initialization_block {
	ATR			ovratr;			/* オーバランハンドラ属性 */
	OVRHDR		ovrhdr;			/* オーバランハンドラの起動番地 */
} OVRINIB;

/*
 *  オーバランハンドラ初期化ブロックのエリア（kernel_cfg.c）
 */
extern const OVRINIB	ovrinib;

/*
 *  オーバランタイマが動作中かを示すフラグ
 */
extern bool_t	ovrtimer_flag;

/*
 *  オーバランハンドラ機能の初期化
 */
extern void	initialize_overrun(void);

/*
 *  オーバランハンドラ用タイマの動作開始
 */
extern void	ovrtimer_start(void);

/*
 *  オーバランハンドラ用タイマの停止
 */
#ifndef OMIT_OVRTIMER_STOP

extern void	ovrtimer_stop(void);

#else /* OMIT_OVRTIMER_STOP */

Inline void
ovrtimer_stop(void)
{
	if (p_runtsk->leftotm > 0U) {
		p_runtsk->leftotm = target_ovrtimer_stop();
		ovrtimer_flag = false;
	}
}

#endif /* OMIT_OVRTIMER_STOP */

/*
 *  オーバランハンドラ起動ルーチン
 */
extern void	call_ovrhdr(void);

#endif /* TOPPERS_SUPPORT_OVRHDR */
#endif /* TOPPERS_OVERRUN_H */
