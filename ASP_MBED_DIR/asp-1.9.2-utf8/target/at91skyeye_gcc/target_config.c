/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2006 by GJ Business Division RICOH COMPANY,LTD. JAPAN
 *  Copyright (C) 2007-2016 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: target_config.c 2743 2016-01-09 05:47:54Z ertl-honda $
 */

/*
 * チップ依存モジュール（AT91SKYEYE用）
 */

#include "kernel_impl.h"
#include <sil.h>
#include "target_serial.h"
#include <stdlib.h>

#ifdef TOPPERS_ENABLE_TRACE
#include <stdio.h>
static FILE *trace_log_file;
#endif /* TOPPERS_ENABLE_TRACE */

/*
 * 各割込みの割込み要求禁止フラグの状態
 */
uint32_t tsim_idf;

/*
 * 現在の割込み優先度マスクの値
 */
PRI tsim_ipm;
 
/*
 *  割込み属性が設定されているかを判別するための変数
 */
uint32_t bitpat_cfgint;

/*
 *  文字列の出力（バージョンチェック用）
 */
static void
target_fput_str(char *c) {
	while(*c != '\0') {
		at91skyeye_putc(*c++);
	}
}

/*
 *  数字 to 文字変換(4bit用)
 */
static char
num_to_char(uint8_t num) {
	if (num <= 9) {
		return ('0' + num);
	}
	else {
		return ('a' + (num - 10));
	}
}

/*
 *  バージョン番号の出力
 */
static void
version_put(uint16_t version) {
	char c;

	c = num_to_char((version >> 12) & 0x0f);
	at91skyeye_putc(c);
	at91skyeye_putc('.');
	c = num_to_char((version >> 8) & 0x0f);
	at91skyeye_putc(c);
	c = num_to_char((version >> 4) & 0x0f);
	at91skyeye_putc(c);
	at91skyeye_putc('.');
	c = num_to_char((version) & 0x0f);
	at91skyeye_putc(c);
	at91skyeye_putc('.');
}

/*
 *  バージョンチェック
 */
static void
version_check(void) {

	/* SkyEye のバージョンを確認 */
	if (sil_rew_mem((void *)(SKYEYE_VER_REG)) != SUPPORT_SKYEYE_VER) {
		target_fput_str("SkyEye version is mismatch!!");
		at91skyeye_putc('\n');
		target_fput_str("Suppoted version is ");
		version_put(SUPPORT_SKYEYE_VER);
		at91skyeye_putc('\n');
		target_fput_str("Tool version is ");
		version_put(sil_rew_mem((void *)(SKYEYE_VER_REG)));
		at91skyeye_putc('\n');
		target_fput_str("Kernel Exit...");
		at91skyeye_putc('\n');
		target_exit();
	}

	/* DeviceManger Extension のバージョンを確認 */
	if ((sil_rew_mem((void *)(DEVM_VER_REG)) & 0xfff0)
		!= (SUPPORT_DEVM_VER & 0xfff0)) {
		target_fput_str("DeviceManager Extension version is mismatch!!");
		at91skyeye_putc('\n');
		target_fput_str("Suppoted version is ");
		version_put(SUPPORT_DEVM_VER);
		at91skyeye_putc('\n');
		target_fput_str("Tool version is ");
		version_put((sil_rew_mem((void *)(DEVM_VER_REG)) & 0xfff0));
		at91skyeye_putc('\n');
		target_fput_str("Kernel Exit...");
		at91skyeye_putc('\n');
		target_exit();
	}
}

/*
 *  ターゲット依存の初期化
 */
void
target_initialize(void)
{
	/*
	 *  target_fput_logが使えるようにUARTを初期化
	 */
	at91skyeye_init_uart();

	/*
	 *  バージョンの確認
	 */
	version_check();

	/*
	 *  ARM依存の初期化
	 */
	core_initialize();
    
	/*
	 * 各割込みの割込み要求禁止フラグ全禁止
	 */
	tsim_idf = ~0U;

	/*
	 * 割込み優先度マスクは0
	 */ 
	tsim_ipm = 0U;

	/*
	 * 全ての割込みをマスク
	 */ 
	at91skyeye_disable_int(~0U);

	/*
	 * 全ての割込み要因をクリア
	 */
	at91skyeye_clear_int(~0U);

	/*
	 *  割込み属性が設定されているかを判別するための変数を初期化する．
	 */
	bitpat_cfgint = 0U;

#ifdef TOPPERS_ENABLE_TRACE
	trace_log_file = fopen("kernel_trace.log","w");
#endif /* TOPPERS_ENABLE_TRACE */
}

/*
 *  ターゲット依存の終了処理
 */
void
target_exit(void)
{
	extern void    software_term_hook(void);
	void (*volatile fp)(void) = software_term_hook;

	/*
	 *  software_term_hookへのポインタを，一旦volatile指定のあるfpに代
	 *  入してから使うのは，0との比較が最適化で削除されないようにするた
	 *  めである．
	 */
	if (fp != 0) {
		(*fp)();
	}

	/*
	 *  すべての割込みをマスクする．
	 */
	at91skyeye_disable_int(~0U);

	/*
	 *  ARM依存の終了処理
	 */
	core_terminate();

	/*
	 *  開発環境依存の終了処理
	 */
	at91skyeye_exit();

#ifdef TOPPERS_ENABLE_TRACE
	fclose(trace_log_file);
#endif /* TOPPERS_ENABLE_TRACE */

#ifdef TOPPERS_ENABLE_GCOV_PART
	/*
	 * 一部取得の場合は終了時に .gcda ファイルを出力しないように_exit()を
	 * 呼び出す．
	 */
	extern void _exit(int) NoReturn;;
	_exit(0);
#else /* TOPPERS_ENABLE_GCOV_FULL */
	exit(0);
#endif /* TOPPERS_ENABLE_GCOV_PART */
	/* ここには来ないはず */
	while(1);
}

/*
 *  システムログの低レベル出力のための文字出力
 */
void
target_fput_log(char c)
{
	if (c == '\n') {
		at91skyeye_putc('\r');
	}
	at91skyeye_putc(c);
}

#ifdef TOPPERS_ENABLE_TRACE
/*
 *  トレースログのファイル出力
 */
void
target_fput_log_file(char c)
{
	if (c == '\n') {
		fputc('\r', trace_log_file);
	}
	fputc(c, trace_log_file);
}
#endif /* TOPPERS_ENABLE_TRACE */

/*
 *  割込み要求ラインの属性の設定
 *
 *  ASPカーネルでの利用を想定して，パラメータエラーはアサーションでチェッ
 *  クしている．FI4カーネルに利用する場合には，エラーを返すようにすべき
 *  であろう．
 *
 */
void
x_config_int(INTNO intno, ATR intatr, PRI intpri)
{
	assert(VALID_INTNO(intno));
	assert(TMIN_INTPRI <= intpri && intpri <= TMAX_INTPRI);

	/*
	 *  割込み属性が設定されているかを判別するための変数の設定
	 */
	bitpat_cfgint |= INTNO_BITPAT(intno);
    
	/* 
	 * いったん割込みを禁止する
	 */    
	x_disable_int(intno);

	if ((intatr & TA_ENAINT) != 0U){
		(void)x_enable_int(intno);
	}    
}

#ifndef OMIT_DEFAULT_INT_HANDLER
/*
 * 未定義の割込みが入った場合の処理
 */
void
default_int_handler(void){
	syslog_0(LOG_EMERG, "Unregistered Interrupt occurs.");
	target_exit();
}
#endif /* OMIT_DEFAULT_INT_HANDLER */

/*
 *  コンパイラのスタートアップルーチンから呼び出される．
 *  sta_ker()を呼び出してカーネルをスタートさせる． 
 */
int
main(void)
{
	extern void software_init_hook(void);
	void (*volatile fp)(void) = software_init_hook;

	/*
	 *  software_init_hook を呼出し（0 でない場合）
	 *
	 *  ソフトウェア環境（特にライブラリ）に依存して必要な初期化処
	 *  理がある場合は，software_init_hook という関数を用意すれば
	 *  よい．
	 *  software_term_hook と同様の理由で一旦volatile指定のあるfpに
	 *  代入する． 
	 */
	if (fp != 0) {
		(*fp)();
	}

	/*
	 * カーネルを起動する
	 */
	sta_ker();

	return 1;
}

#ifdef TOPPERS_ENABLE_GCOV_PART
/*
 *  GCOV一部取得用ライブラリ
 */

extern void __gcov_flush();

/*
 *  GCOV初期化関数
 *   カバレッジを.gcdaに出力し，SkyEyeで*.gcdaファイルを削除する．
 */
void
gcov_init(void)
{
	SIL_PRE_LOC;

	SIL_LOC_INT();
	__gcov_flush();
	sil_wrw_mem((void*)0xFFFE1020, 0);
	SIL_UNL_INT();
}

/*
 *  GCOV中断関数
 *   カバレッジを.gcdaに出力し，SkyEyeで*.gcdaファイルを*.gcda.bakへリネームする．
 */
void
gcov_pause(void)
{
	SIL_PRE_LOC;

	SIL_LOC_INT();
	__gcov_flush();
	sil_wrw_mem((void*)0xFFFE1024, 0);
	SIL_UNL_INT();
}

/*
 *  GCOV再開関数
 *   カバレッジを.gcdaに出力し，SkyEyeで*.gcda.bakファイルを*.gcdaへリネームする．
 */
void
gcov_resume(void)
{
	SIL_PRE_LOC;

	SIL_LOC_INT();
	__gcov_flush();
	sil_wrw_mem((void*)0xFFFE1028, 0);
	SIL_UNL_INT();
}

/*
 *  GCOV出力関数
 *   カバレッジを.gcdaに出力する．
 */
void
gcov_dump(void)
{
	SIL_PRE_LOC;

	SIL_LOC_INT();
	__gcov_flush();
	SIL_UNL_INT();
}
#endif /* TOPPERS_ENABLE_GCOV_PART */
