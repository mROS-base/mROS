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
 *  $Id: syslog.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		システムログ機能
 */

#include <sil.h>
#undef TOPPERS_OMIT_SYSLOG
#include <t_syslog.h>
#include <log_output.h>
#include "target_syssvc.h"
#include "syslog.h"

/*
 *  トレースログマクロのデフォルト定義
 */
#ifndef LOG_SYSLOG_WRI_LOG_ENTER
#define LOG_SYSLOG_WRI_LOG_ENTER(prio, p_syslog)
#endif /* LOG_SYSLOG_WRI_LOG_ENTER */

#ifndef LOG_SYSLOG_WRI_LOG_LEAVE
#define LOG_SYSLOG_WRI_LOG_LEAVE(ercd)
#endif /* LOG_SYSLOG_WRI_LOG_LEAVE */

#ifndef LOG_SYSLOG_REA_LOG_ENTER
#define LOG_SYSLOG_REA_LOG_ENTER(p_syslog)
#endif /* LOG_SYSLOG_REA_LOG_ENTER */

#ifndef LOG_SYSLOG_REA_LOG_LEAVE
#define LOG_SYSLOG_REA_LOG_LEAVE(ercd, p_syslog)
#endif /* LOG_SYSLOG_REA_LOG_LEAVE */

#ifndef LOG_SYSLOG_MSK_LOG_ENTER
#define LOG_SYSLOG_MSK_LOG_ENTER(logmask, lowmask)
#endif /* LOG_SYSLOG_MSK_LOG_ENTER */

#ifndef LOG_SYSLOG_MSK_LOG_LEAVE
#define LOG_SYSLOG_MSK_LOG_LEAVE(ercd)
#endif /* LOG_SYSLOG_MSK_LOG_LEAVE */

#ifndef LOG_SYSLOG_REF_LOG_ENTER
#define LOG_SYSLOG_REF_LOG_ENTER(pk_rlog)
#endif /* LOG_SYSLOG_REF_LOG_ENTER */

#ifndef LOG_SYSLOG_REF_LOG_LEAVE
#define LOG_SYSLOG_REF_LOG_LEAVE(pk_rlog)
#endif /* LOG_SYSLOG_REF_LOG_LEAVE */

/*
 *  現在のシステム時刻
 *
 *  ログ時刻を取り出すために，やむなく，カーネルの内部変数を参照してい
 *  る．
 */
extern ulong_t	_kernel_current_time;

/*
 *  ログバッファとそれにアクセスするためのポインタ
 */
static SYSLOG	syslog_buffer[TCNT_SYSLOG_BUFFER];	/* ログバッファ */
static uint_t	syslog_count;			/* ログバッファ中のログの数 */
static uint_t	syslog_head;			/* 先頭のログの格納位置 */
static uint_t	syslog_tail;			/* 次のログの格納位置 */
static uint_t	syslog_lost;			/* 失われたログの数 */

/*
 *  出力すべきログ情報の重要度（ビットマップ）
 */
static uint_t	syslog_logmask;			/* ログバッファに記録すべき重要度 */
static uint_t	syslog_lowmask_not;		/* 低レベル出力すべき重要度（反転）*/

/*
 *  システムログ機能の初期化
 */
void
syslog_initialize(intptr_t exinf)
{
	syslog_count = 0U;
	syslog_head = 0U;
	syslog_tail = 0U;
	syslog_lost = 0U;
	syslog_logmask = 0U;
	syslog_lowmask_not = 0U;
}     

/* 
 *  ログ情報の出力
 *
 *  CPUロック状態や実行コンテキストによらず動作できるように実装してある．
 */
ER
syslog_wri_log(uint_t prio, const SYSLOG *p_syslog)
{
	SIL_PRE_LOC;

	LOG_SYSLOG_WRI_LOG_ENTER(prio, p_syslog);
	SIL_LOC_INT();

	/*
	 *  ログ時刻の設定
	 */
	((SYSLOG *) p_syslog)->logtim = _kernel_current_time;

	/*
	 *  ログバッファに記録
	 */
	if ((syslog_logmask & LOG_MASK(prio)) != 0U) {
		syslog_buffer[syslog_tail] = *p_syslog;
		syslog_tail++;
		if (syslog_tail >= TCNT_SYSLOG_BUFFER) {
			syslog_tail = 0U;
		}
		if (syslog_count < TCNT_SYSLOG_BUFFER) {
			syslog_count++;
		}
		else {
			syslog_head = syslog_tail;
			syslog_lost++;
		}
	}

	/*
	 *  低レベル出力
	 */
	if (((~syslog_lowmask_not) & LOG_MASK(prio)) != 0U) {
		syslog_print(p_syslog, target_fput_log);
		target_fput_log('\n');
	}

	SIL_UNL_INT();
	LOG_SYSLOG_WRI_LOG_LEAVE(E_OK);
	return(E_OK);
}

/*
 *  ログバッファからの読出し
 *
 *  CPUロック状態や実行コンテキストによらず動作できるように実装してある．
 */
ER_UINT
syslog_rea_log(SYSLOG *p_syslog)
{
	ER_UINT	ercd;
	SIL_PRE_LOC;

	LOG_SYSLOG_REA_LOG_ENTER(p_syslog);
	SIL_LOC_INT();

	/*
	 *  ログバッファからの取出し
	 */
	if (syslog_count > 0U) {
		*p_syslog = syslog_buffer[syslog_head];
		syslog_count--;
		syslog_head++;
		if (syslog_head >= TCNT_SYSLOG_BUFFER) {
			syslog_head = 0U;
		}
		ercd = (ER_UINT) syslog_lost;
		syslog_lost = 0U;
	}
	else {
		ercd = E_OBJ;
	}

	SIL_UNL_INT();
	LOG_SYSLOG_REA_LOG_LEAVE(ercd, p_syslog);
	return(ercd);
}

/* 
 *  出力すべきログ情報の重要度の設定
 */
ER
syslog_msk_log(uint_t logmask, uint_t lowmask)
{
	LOG_SYSLOG_MSK_LOG_ENTER(logmask, lowmask);
	syslog_logmask = logmask;
	syslog_lowmask_not = ~lowmask;
	LOG_SYSLOG_MSK_LOG_LEAVE(E_OK);
	return(E_OK);
}

/*
 *  ログバッファの状態参照
 */
ER
syslog_ref_log(T_SYSLOG_RLOG *pk_rlog)
{
	SIL_PRE_LOC;

	LOG_SYSLOG_REF_LOG_ENTER(pk_rlog);
	SIL_LOC_INT();

	pk_rlog->count = syslog_count;
	pk_rlog->lost = syslog_lost;
	pk_rlog->logmask = syslog_logmask;
	pk_rlog->lowmask = ~syslog_lowmask_not;

	SIL_UNL_INT();
	LOG_SYSLOG_REF_LOG_LEAVE(pk_rlog);
	return(E_OK);
}
