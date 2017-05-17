/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2005-2009 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: prc_config.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		プロセッサ依存モジュール（M68040用）
 */

#include "kernel_impl.h"
#include "check.h"
#include "task.h"

/*
 *  CPUロックフラグ実現のための変数
 */
volatile bool_t		lock_flag;		/* CPUロックフラグの値を保持する変数 */
volatile uint16_t	saved_iipm;		/* 割込み優先度マスクを保存する変数 */

/*
 *  プロセッサ依存の初期化
 */
void
prc_initialize(void)
{
	/*
	 *  CPUロックフラグ実現のための変数の初期化
	 */
	lock_flag = true;
	saved_iipm = IIPM_ENAALL;

	/*
	 *  例外ベクタテーブルの初期化
	 */
#ifdef EXCVT_KERNEL
	memcpy(EXCVT_KERNEL, EXCVT_ORIG, EXCVT_LEN);
	set_vbr(EXCVT_KERNEL);
#endif /* EXCVT_KERNEL */
}

/*
 *  プロセッサ依存の終了処理
 */
void
prc_terminate(void)
{
	extern void	software_term_hook(void);
	void (*volatile fp)(void) = software_term_hook;

	/*
	 *  software_term_hookへのポインタを，一旦volatile指定のあるfpに代
	 *  入してから使うのは，0との比較が最適化で削除されないようにするた
	 *  めである．
	 */
	if (fp != 0) {
		(*fp)();
	}

#ifdef EXCVT_KERNEL
	set_vbr(EXCVT_ORIG);
#endif /* EXCVT_KERNEL */
}

/*
 *  CPU例外の発生状況のログ出力
 *
 *  CPU例外ハンドラの中から，CPU例外情報ポインタ（p_excinf）を引数とし
 *  て呼び出すことで，CPU例外の発生状況をシステムログに出力する．
 */
#ifdef SUPPORT_XLOG_SYS

void
xlog_sys(void *p_excinf)
{
	char	*excsp = (char *) p_excinf;
	uint_t	format;

	syslog_0(LOG_ERROR, "CPU Exception Information:");
	syslog_4(LOG_ERROR, "SR = %04x (M = %d, S = %d, IPM = %d)",
				*((uint16_t *) excsp),
				(*((uint16_t *) excsp) & 0x1000U) >> 12,
				(*((uint16_t *) excsp) & 0x2000U) >> 13,
				(*((uint16_t *) excsp) & 0x0700U) >> 8);
	syslog_1(LOG_ERROR, "PC = %08x", *((uint32_t *)(excsp + 2)));
	format = (*((uint16_t *)(excsp + 6)) & 0xf000U) >> 12;
	syslog_2(LOG_ERROR, "Format = %d, Vector Offset = %03x",
				format, (*((uint16_t *)(excsp + 6)) & 0x0fffU));
	if (format >= 2U) {
		syslog_1(LOG_ERROR, "ADR = %08x", *((uint32_t *)(excsp + 8)));
	}
}

#endif /* SUPPORT_XLOG_SYS */
