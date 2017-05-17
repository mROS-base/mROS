/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
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
 *  $Id: trace_config.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		トレースログ機能
 */

#include "kernel_impl.h"
#include "task.h"
#include "time_event.h"
#include <sil.h>

/*
 *  トレースログバッファとそれにアクセスするためのポインタ
 */
SYSLOG	trace_buffer[TCNT_TRACE_BUFFER];	/* トレースログバッファ */
uint_t	trace_count;				/* トレースログバッファ中のログの数 */
uint_t	trace_head;					/* 先頭のトレースログの格納位置 */
uint_t	trace_tail;					/* 次のトレースログの格納位置 */
MODE	trace_mode;					/* トレースモード */

/*
 *  トレースログ機能の初期化
 */
void
trace_initialize(intptr_t exinf)
{
	MODE	mode = ((MODE) exinf);

	trace_count = 0U;
	trace_head = 0U;
	trace_tail = 0U;
	trace_mode = mode;
}     

/*
 *  トレースログの開始
 */
ER
trace_sta_log(MODE mode)
{
	if ((mode & TRACE_CLEAR) != 0U) {
		trace_count = 0U;
		trace_head = 0U;
		trace_tail = 0U;
	}
	trace_mode = mode;
	return(E_OK);
}     

/* 
 *  トレースログの書込み
 */
ER
trace_wri_log(TRACE *p_trace)
{
	SIL_PRE_LOC;

	if (trace_mode != TRACE_STOP) {
		SIL_LOC_INT();

		/*
		 *  トレース時刻の設定
		 *
		 *  LOG_WRI_LOG_ENTERから呼ばれた場合にp_trace->logtimを書き換
		 *  えてしまうのは気持ちが悪いが，wri_logの方で上書きするため問
		 *  題はない．
		 */
		p_trace->logtim = TRACE_GET_TIM();

		/*
		 *  トレースバッファに記録
		 */
		trace_buffer[trace_tail] = *p_trace;
		trace_tail++;
		if (trace_tail >= TCNT_TRACE_BUFFER) {
			trace_tail = 0U;
		}
		if (trace_count < TCNT_TRACE_BUFFER) {
			trace_count++;
			if (trace_count >= TCNT_TRACE_BUFFER
						&& (trace_mode & TRACE_AUTOSTOP) != 0U) {
				trace_mode = TRACE_STOP;
			}
		}
		else {
			trace_head = trace_tail;
		}

		SIL_UNL_INT();
	}
	return(E_OK);
}

/*
 *  トレースログの読出し
 */
ER
trace_rea_log(TRACE *p_trace)
{
	ER_UINT	ercd;
	SIL_PRE_LOC;

	SIL_LOC_INT();

	/*
	 *  トレースログバッファからの取出し
	 */
	if (trace_count > 0U) {
		*p_trace = trace_buffer[trace_head];
		trace_count--;
		trace_head++;
		if (trace_head >= TCNT_TRACE_BUFFER) {
			trace_head = 0U;
		}
		ercd = E_OK;
	}
	else {
		ercd = E_OBJ;
	}

	SIL_UNL_INT();
	return(ercd);
}

/*
 *  トレースログを出力するためのライブラリ関数
 */

void
trace_write_0(uint_t type)
{
	TRACE	trace;

	trace.logtype = type;
	(void) trace_wri_log(&trace);
}

void
trace_write_1(uint_t type, intptr_t arg1)
{
	TRACE	trace;

	trace.logtype = type;
	trace.loginfo[0] = arg1;
	(void) trace_wri_log(&trace);
}

void
trace_write_2(uint_t type, intptr_t arg1, intptr_t arg2)
{
	TRACE	trace;

	trace.logtype = type;
	trace.loginfo[0] = arg1;
	trace.loginfo[1] = arg2;
	(void) trace_wri_log(&trace);
}

void
trace_write_3(uint_t type, intptr_t arg1, intptr_t arg2, intptr_t arg3)
{
	TRACE	trace;

	trace.logtype = type;
	trace.loginfo[0] = arg1;
	trace.loginfo[1] = arg2;
	trace.loginfo[2] = arg3;
	(void) trace_wri_log(&trace);
}

/*
 *  アセンブリ言語で記述されるコードからトレースログを出力するための関
 *  数
 */

void
log_dsp_enter(TCB *p_tcb)
{
	trace_1(LOG_TYPE_DSP|LOG_ENTER, p_tcb);
}

void
log_dsp_leave(TCB *p_tcb)
{
	trace_1(LOG_TYPE_DSP|LOG_LEAVE, p_tcb);
}

void
log_inh_enter(INHNO inhno)
{
	trace_1(LOG_TYPE_INH|LOG_ENTER, inhno);
}

void
log_inh_leave(INHNO inhno)
{
	trace_1(LOG_TYPE_INH|LOG_LEAVE, inhno);
}

void
log_exc_enter(EXCNO excno)
{
	trace_1(LOG_TYPE_EXC|LOG_ENTER, excno);
}

void
log_exc_leave(EXCNO excno)
{
	trace_1(LOG_TYPE_EXC|LOG_LEAVE, excno);
}
