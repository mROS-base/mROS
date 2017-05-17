/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2005-2013 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: test_lib.h 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/* 
 *		テストプログラム用ライブラリ
 */

#ifndef TOPPERS_TEST_LIB_H
#define TOPPERS_TEST_LIB_H

#ifdef __cplusplus
extern "C" {
#endif

#include <t_stddef.h>

/*
 *  ターゲット依存の定義
 */
#include "target_test.h"

/*
 *	自己診断関数の型
 */
typedef ER (*BIT_FUNC)(void);

/*
 *	自己診断関数の設定
 */
extern void	set_bit_func(BIT_FUNC bit_func);

/*
 *  テストプログラムの開始
 */
extern void	test_start(char *progname);

/*
 *  システムログの出力処理
 */
extern void	syslog_flush(void);

/*
 *	テストプログラムの終了
 */
extern void	test_finish(void);

/*
 *	チェックポイント
 */
extern void	check_point(uint_t count);

/*
 *	完了チェックポイント
 */
extern void	check_finish(uint_t count);

/*
 *	条件チェック
 */
extern void	_check_assert(const char *expr, const char *file, int_t line);
#define check_assert(exp) \
	((void)(!(exp) ? (_check_assert(#exp, __FILE__, __LINE__), 0) : 0))

/*
 *	エラーコードチェック
 */
extern void	_check_ercd(ER ercd, const char *file, int_t line);
#define check_ercd(ercd, expected_ercd) \
	((void)((ercd) != (expected_ercd) ? \
					(_check_ercd(ercd, __FILE__, __LINE__), 0) : 0))

/*
 *	システム状態のチェック
 */
Inline void
check_state(bool_t ctx, bool_t loc, PRI ipm, bool_t dsp,
										bool_t dpn, bool_t tex)
{
	PRI		intpri;
	ER		ercd;

	check_assert(sns_ctx() == ctx);
	check_assert(sns_loc() == loc);
	if (!loc) {
		/*
		 *  IPMのチェックは，CPUロック解除状態の場合にのみ行う．
		 */
		ercd = get_ipm(&intpri);
		check_ercd(ercd, E_OK);
		check_assert(intpri == ipm);
	}
	check_assert(sns_dsp() == dsp);
	check_assert(sns_dpn() == dpn);
	check_assert(sns_tex() == tex);
}

/*
 *	システム状態のチェック（非タスクコンテキスト用）
 */
Inline void
check_state_i(bool_t ctx, bool_t loc, bool_t dsp, bool_t dpn, bool_t tex)
{
	check_assert(sns_ctx() == ctx);
	check_assert(sns_loc() == loc);
	check_assert(sns_dsp() == dsp);
	check_assert(sns_dpn() == dpn);
	check_assert(sns_tex() == tex);
}

#ifdef __cplusplus
}
#endif

#endif /* TOPPERS_TEST_LIB_H */
