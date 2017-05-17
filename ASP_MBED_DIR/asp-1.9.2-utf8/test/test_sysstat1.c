/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2006-2013 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: test_sysstat1.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/* 
 *		システム状態に関するテスト(1)
 *
 *  テストシーケンス：
 *
 *	== TASK1（優先度：10）==
 *	1:	初期状態のチェック
 *	2:	loc_cpu() ... システム状態をランダムに変化させる
 *		chg_ipm(TMAX_INTPRI) -> E_CTX
 *	3:	unl_cpu()
 *	4:	chg_ipm(TMAX_INTPRI)
 *	5:	dis_dsp()
 *	6:	ena_tex()
 *	7:	chg_ipm(TIPM_ENAALL)
 *	8:	ena_dsp()
 *	9:	dis_tex()
 *	10:	dis_dsp() ... タスク例外処理ルーチンを呼び出す準備
 *		ena_tex()
 *	11:	ras_tex(TSK_SELF, 0x0001)
 *	== TASK1タスク例外処理ルーチン（1回目）==
 *	12:	初期状態のチェック
 *	13:	ena_dsp() ... 3つの状態を変化させ，リターンで元にもどるか調べる
 *		chg_ipm(TMIN_INTPRI)
 *		ena_tex()
 *		リターン
 *	== TASK1（続き）==
 *	14:	戻ってきた状態のチェック
 *	15:	loc_cpu() ... CPUロック状態のままタスクを終了させる
 *	16:	リターン（＝タスク終了）
 *	== TASK2（優先度：10）	==
 *	17:	初期状態のチェック
 *	18:	終了
 */

#include <kernel.h>
#include <test_lib.h>
#include <t_syslog.h>
#include "kernel_cfg.h"
#include "test_sysstat1.h"

void
tex_task1(TEXPTN texptn, intptr_t exinf)
{
	ER		ercd;

	switch (texptn) {
	case 0x0001:
		check_point(12);
		check_state(false, false, TIPM_ENAALL, true, true, true);

		/*
		 *  ディスパッチ許可，割込み優先度マスク変更，タスク例外処理許可
		 */
		check_point(13);
		ercd = ena_dsp();
		check_ercd(ercd, E_OK);
		ercd = chg_ipm(TMIN_INTPRI);
		check_ercd(ercd, E_OK);
		ercd = ena_tex();
		check_ercd(ercd, E_OK);
		check_state(false, false, TMIN_INTPRI, false, true, false);
		break;

	default:
		check_point(0);
		break;
	}
}

void
task1(intptr_t exinf)
{
	ER		ercd;

	test_start(__FILE__);

	/*
	 *  初期状態のチェック
	 */
	check_point(1);
	check_state(false, false, TIPM_ENAALL, false, false, true);

	/*
	 *  CPUロック状態のチェック
	 */
	check_point(2);
	ercd = loc_cpu();
	check_ercd(ercd, E_OK);
	check_state(false, true, TIPM_ENAALL, false, true, true);

	/*
	 *  割込み優先度マスク変更のチェック
	 */
	ercd = chg_ipm(TMAX_INTPRI);
	check_ercd(ercd, E_CTX);
	check_state(false, true, TIPM_ENAALL, false, true, true);

	/*
	 *  CPUロック解除のチェック
	 */
	check_point(3);
	ercd = unl_cpu();
	check_ercd(ercd, E_OK);
	check_state(false, false, TIPM_ENAALL, false, false, true);

	/*
	 *  割込み優先度マスク変更のチェック
	 */
	check_point(4);
	ercd = chg_ipm(TMAX_INTPRI);
	check_ercd(ercd, E_OK);
	check_state(false, false, TMAX_INTPRI, false, true, true);

	/*
	 *  ディスパッチ禁止のチェック
	 */
	check_point(5);
	ercd = dis_dsp();
	check_ercd(ercd, E_OK);
	check_state(false, false, TMAX_INTPRI, true, true, true);

	/*
	 *  タスク例外処理許可のチェック
	 */
	check_point(6);
	ercd = ena_tex();
	check_ercd(ercd, E_OK);
	check_state(false, false, TMAX_INTPRI, true, true, false);

	/*
	 *  割込み優先度マスク全解除のチェック
	 */
	check_point(7);
	ercd = chg_ipm(TIPM_ENAALL);
	check_ercd(ercd, E_OK);
	check_state(false, false, TIPM_ENAALL, true, true, false);

	/*
	 *  ディスパッチ許可のチェック
	 */
	check_point(8);
	ercd = ena_dsp();
	check_ercd(ercd, E_OK);
	check_state(false, false, TIPM_ENAALL, false, false, false);

	/*
	 *  タスク例外処理禁止のチェック
	 */
	check_point(9);
	ercd = dis_tex();
	check_ercd(ercd, E_OK);
	check_state(false, false, TIPM_ENAALL, false, false, true);

	/*
	 *  ディスパッチ禁止，タスク例外処理許可
	 */
	check_point(10);
	ercd = dis_dsp();
	check_ercd(ercd, E_OK);
	ercd = ena_tex();
	check_ercd(ercd, E_OK);
	check_state(false, false, TIPM_ENAALL, true, true, false);

	/*
	 *  タスク例外処理を要求
	 */
	check_point(11);
	ercd = ras_tex(TSK_SELF, 0x0001);
	/* ここでタスク例外処理ルーチンが動作する */
	check_ercd(ercd, E_OK);

	/*
	 *  タスク例外処理からのリターンにより元の状態に戻っていることを
	 *  チェック
	 */
	check_point(14);
	check_state(false, false, TIPM_ENAALL, true, true, false);

	/*
	 *  CPUロック状態に
	 */
	check_point(15);
	ercd = loc_cpu();
	check_ercd(ercd, E_OK);

	/*
	 *  そのままタスク終了
	 */
	check_point(16);
}

void
task2(intptr_t exinf)
{
	/*
	 *  初期状態のチェック
	 */
	check_point(17);
	check_state(false, false, TIPM_ENAALL, false, false, true);

	check_finish(18);
}
