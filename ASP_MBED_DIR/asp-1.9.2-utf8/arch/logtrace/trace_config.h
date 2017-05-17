/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2012 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: trace_config.h 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		トレースログに関する設定
 *
 *  このインクルードファイルは，target_config.hおよびtarget_syssvc.hの
 *  みからインクルードされる．また，トレースログ機能の初期化や記録の開
 *  始／停止，トレースログのダンプを行うプログラムからインクルードする
 *  ことを想定している．
 */

#ifndef TOPPERS_TRACE_CONFIG_H
#define TOPPERS_TRACE_CONFIG_H

/*
 *  トレースログバッファのサイズ
 */
#ifndef TCNT_TRACE_BUFFER
#define TCNT_TRACE_BUFFER	1024
#endif /* TCNT_TRACE_BUFFER */

/*
 *  トレース時刻の取得方法
 */
#ifndef TRACE_GET_TIM
#define TRACE_GET_TIM()		(current_time)
#endif /* TRACE_GET_TIM */

#ifndef TOPPERS_MACRO_ONLY

/*
 *  トレースログのデータ構造
 *
 *  システムログ機能のログ情報のデータ構造と同じものを用いる．
 */
#include <t_syslog.h>
typedef	SYSLOG	TRACE;

/*
 *  トレースログバッファとそれにアクセスするためのポインタ
 */
extern TRACE	trace_buffer[];		/* トレースログバッファ */
extern uint_t	trace_count;		/* トレースログバッファ中のログの数 */
extern uint_t	trace_head;			/* 先頭のトレースログの格納位置 */
extern uint_t	trace_tail;			/* 次のトレースログの格納位置 */
extern uint_t	trace_lost;			/* 失われたトレースの数 */

#endif /* TOPPERS_MACRO_ONLY */

/*
 *  トレースモードの定義
 */
#define TRACE_STOP			UINT_C(0x00)	/* トレース停止 */
#define TRACE_RINGBUF		UINT_C(0x01)	/* リングバッファモード */
#define TRACE_AUTOSTOP		UINT_C(0x02)	/* 自動停止モード */
#define TRACE_CLEAR			UINT_C(0x04)	/* トレースログのクリア */

#ifndef TOPPERS_MACRO_ONLY

/*
 *  トレースログ機能の初期化
 *
 *  トレースログ機能を初期化する．初期化ルーチンとして登録することを想
 *  定している．引数により次の動作を行う．
 *
 *  TRACE_STOP：初期化のみでトレースは開始しない．
 *  TRACE_RINGBUF：リングバッファモードでトレースを開始．
 *  TRACE_AUTOSTOP：自動停止モードでトレースを開始．
 */
extern void	trace_initialize(intptr_t exinf);

/*
 *  トレースログの開始
 *
 *  トレースログの記録を開始／停止する．引数により次の動作を行う．
 *
 *  TRACE_STOP：トレースを停止．
 *  TRACE_RINGBUF：リングバッファモードでトレースを開始．
 *  TRACE_AUTOSTOP：自動停止モードでトレースを開始．
 *  TRACE_CLEAR：トレースログをクリア．
 */
extern ER	trace_sta_log(MODE mode);

/*
 *  トレースログの書込み
 */
extern ER	trace_wri_log(TRACE *p_trace);

/*
 *  トレースログの読出し
 */
extern ER	trace_rea_log(TRACE *p_trace);

/* 
 *  トレースログのダンプ（trace_dump.c）
 *
 *  トレースログをダンプする．終了処理ルーチンとして登録することも想定
 *  している．引数として，ダンプ先となる文字出力関数へのポインタを渡す．
 *  ターゲット依存の低レベル文字出力を利用する場合には，target_putcを渡
 *  す．
 */
extern void	trace_dump(intptr_t exinf);

/*
 *  トレースログを出力するためのライブラリ関数
 */
extern void	trace_write_0(uint_t type);
extern void	trace_write_1(uint_t type, intptr_t arg1);
extern void	trace_write_2(uint_t type, intptr_t arg1, intptr_t arg2);
extern void	trace_write_3(uint_t type, intptr_t arg1, intptr_t arg2,
														intptr_t arg3);

/*
 *  トレースログを出力するためのマクロ
 */

#define trace_0(type) \
				trace_write_0(type)

#define trace_1(type, arg1) \
				trace_write_1(type, (intptr_t)(arg1))

#define trace_2(type, arg1, arg2) \
				trace_write_2(type, (intptr_t)(arg1), (intptr_t)(arg2))

#define trace_3(type, arg1, arg2, arg3) \
				trace_write_3(type, (intptr_t)(arg1), (intptr_t)(arg2), \
						(intptr_t)(arg3))

#endif /* TOPPERS_MACRO_ONLY */

/*
 *  トレースログ方法の設定
 */
#define LOG_TSKSTAT(p_tcb)		trace_2(LOG_TYPE_TSKSTAT, p_tcb, p_tcb->tstat)

#define LOG_DSP_LEAVE(p_tcb)	trace_1(LOG_TYPE_DSP|LOG_LEAVE, p_tcb)

#define LOG_SYSLOG_WRI_LOG_ENTER(prio, p_syslog) \
								trace_wri_log((TRACE *) p_syslog)

#endif /* TOPPERS_TRACE_CONFIG_H */
