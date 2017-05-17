/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2010-2013 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: test_rstr1.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/* 
 *		制約タスクのテスト(1)
 *
 * 【テストの目的】
 *
 *  制約タスクの基本的な振舞い（ras_priサービスコール以外の動作）をテス
 *  トする．
 *
 * 【テスト項目】
 *
 *	(A) 制約タスクが，自タスクを待ち状態にする可能性のあるサービスコー
 *      ルを呼び出した場合，E_NOSPTエラーとなる
 *		(A-1) slp_tsk
 *		(A-2) tslp_tsk
 *		(A-3) dly_tsk
 *      ※ 他にもあるが，これだけに留める
 *	(B) 制約タスクを対象として，chg_pri，wup_tsk，iwup_tsk，can_wup，
 *	    rel_wai，irel_wai，sus_tsk，rsm_tskを呼び出した場合，E_NOSPTエ
 *	    ラーとなる
 *		(B-1) chg_pri
 *		(B-2) wup_tsk
 *		(B-3) iwup_tsk
 *		(B-4) can_wup
 *		(B-5) rel_wai
 *		(B-6) irel_wai
 *		(B-7) sus_tsk
 *		(B-8) rsm_tsk
 *  (C) rot_rdq，irot_rdqは，対象優先度を持つ実行できる状態のタスクの中
 *      で最も優先順位が高いタスクが制約タスクである場合，E_NOSPTエラー
 *      となる
 *		(C-1) rot_rdqが正しく動作する
 *		(C-2) rot_rdqがE_NOSPTを返す
 *		(C-3) irot_rdqが正しく動作する
 *		(C-4) irot_rdqがE_NOSPTを返す
 *
 * 【使用リソース】
 *
 *	TASK1: 中優先度タスク，メインタスク，最初から起動
 *	TASK2: 高優先度タスク，制約タスク
 *	TASK3: 中優先度タスク，制約タスク
 *	TASK4: 中優先度タスク，制約タスク
 *	ALM1:  アラームハンドラ
 *
 * 【テストシーケンス】
 *
 *	== TASK1（優先度：中）==
 *	1:	act_tsk(TASK2)
 *	== TASK2-1（優先度：高）==
 *	2:	slp_tsk() -> E_NOSPT						... (A-1)
 *	3:	tslp_tsk(10) -> E_NOSPT						... (A-2)
 *	4:	dly_tsk(10) -> E_NOSPT						... (A-3)
 *  5:	ext_tsk()
 *	== TASK1（続き）==
 *	6:	chg_pri(TASK3, HIGH_PRIORITY) -> E_NOSPT	... (B-1)
 *	7:	wup_tsk(TASK3) -> E_NOSPT					... (B-2)
 *	8:	can_wup(TASK3) -> E_NOSPT					... (B-4)
 *	9:	rel_wai(TASK3) -> E_NOSPT					... (B-5)
 *	10:	sus_tsk(TASK3) -> E_NOSPT					... (B-7)
 *	11:	rsm_tsk(TASK3) -> E_NOSPT					... (B-8)
 *	12:	sta_alm(ALM1, 10)
 *	13:	slp_tsk()
 *	== ALM1 ==
 *	14:	iwup_tsk(TASK3) -> E_NOSPT					... (B-3)
 *	15:	irel_wai(TASK3) -> E_NOSPT					... (B-6)
 *	16:	iwup_tsk(TASK1)
 *	17:	iact_tsk(TASK3)
 *	18:	iact_tsk(TASK4)
 *	19:	irot_rdq(MID_PRIORITY)						... (C-3)
 *	20:	irot_rdq(MID_PRIORITY) -> E_NOSPT			... (C-4)
 *	21:	RETURN
 *	== TASK3-1（優先度：中）==
 *	22:	act_tsk(TASK2)
 *	== TASK2-2（優先度：高）2回め ==
 *	23:	rot_rdq(MID_PRIORITY) -> E_NOSPT			... (C-2)
 *	24:	ext_tsk()
 *	== TASK3-1（続き）==
 *	25:	ext_tsk()
 *	== TASK4-1（優先度：中）1回め ==
 *	26:	ext_tsk()
 *	== TASK1（続き）==
 *	27:	act_tsk(TASK3)
 *	28:	act_tsk(TASK4)
 *	29:	act_tsk(TASK2)
 *	== TASK2-3（優先度：高）3回め ==
 *	30:	rot_rdq(MID_PRIORITY)						... (C-1)
 *	31:	ext_tsk()
 *	== TASK3-2（優先度：中）2回め ==
 *	32:	ext_tsk()
 *	== TASK4-2（優先度：中）2回め ==
 *	33:	ext_tsk()
 *	== TASK1（続き）==
 *	34: END
 */

#include <kernel.h>
#include <test_lib.h>
#include <t_syslog.h>
#include "kernel_cfg.h"
#include "test_rstr1.h"

/* DO NOT DELETE THIS LINE -- gentest depends on it. */

void
alarm1_handler(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(14);
	ercd = iwup_tsk(TASK3);
	check_ercd(ercd, E_NOSPT);

	check_point(15);
	ercd = irel_wai(TASK3);
	check_ercd(ercd, E_NOSPT);

	check_point(16);
	ercd = iwup_tsk(TASK1);
	check_ercd(ercd, E_OK);

	check_point(17);
	ercd = iact_tsk(TASK3);
	check_ercd(ercd, E_OK);

	check_point(18);
	ercd = iact_tsk(TASK4);
	check_ercd(ercd, E_OK);

	check_point(19);
	ercd = irot_rdq(MID_PRIORITY);
	check_ercd(ercd, E_OK);

	check_point(20);
	ercd = irot_rdq(MID_PRIORITY);
	check_ercd(ercd, E_NOSPT);

	check_point(21);
	return;

	check_point(0);
}

void
task1(intptr_t exinf)
{
	ER_UINT	ercd;

	test_start(__FILE__);

	check_point(1);
	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(6);
	ercd = chg_pri(TASK3, HIGH_PRIORITY);
	check_ercd(ercd, E_NOSPT);

	check_point(7);
	ercd = wup_tsk(TASK3);
	check_ercd(ercd, E_NOSPT);

	check_point(8);
	ercd = can_wup(TASK3);
	check_ercd(ercd, E_NOSPT);

	check_point(9);
	ercd = rel_wai(TASK3);
	check_ercd(ercd, E_NOSPT);

	check_point(10);
	ercd = sus_tsk(TASK3);
	check_ercd(ercd, E_NOSPT);

	check_point(11);
	ercd = rsm_tsk(TASK3);
	check_ercd(ercd, E_NOSPT);

	check_point(12);
	ercd = sta_alm(ALM1, 10);
	check_ercd(ercd, E_OK);

	check_point(13);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(27);
	ercd = act_tsk(TASK3);
	check_ercd(ercd, E_OK);

	check_point(28);
	ercd = act_tsk(TASK4);
	check_ercd(ercd, E_OK);

	check_point(29);
	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_finish(34);
	check_point(0);
}

static uint_t	task2_count = 0;

void
task2(intptr_t exinf)
{
	ER_UINT	ercd;

	switch (++task2_count) {
	case 1:
		check_point(2);
		ercd = slp_tsk();
		check_ercd(ercd, E_NOSPT);

		check_point(3);
		ercd = tslp_tsk(10);
		check_ercd(ercd, E_NOSPT);

		check_point(4);
		ercd = dly_tsk(10);
		check_ercd(ercd, E_NOSPT);

		check_point(5);
		ercd = ext_tsk();
		check_ercd(ercd, E_OK);

		check_point(0);

	case 2:
		check_point(23);
		ercd = rot_rdq(MID_PRIORITY);
		check_ercd(ercd, E_NOSPT);

		check_point(24);
		ercd = ext_tsk();
		check_ercd(ercd, E_OK);

		check_point(0);

	case 3:
		check_point(30);
		ercd = rot_rdq(MID_PRIORITY);
		check_ercd(ercd, E_OK);

		check_point(31);
		ercd = ext_tsk();
		check_ercd(ercd, E_OK);

		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
}

static uint_t	task3_count = 0;

void
task3(intptr_t exinf)
{
	ER_UINT	ercd;

	switch (++task3_count) {
	case 1:
		check_point(22);
		ercd = act_tsk(TASK2);
		check_ercd(ercd, E_OK);

		check_point(25);
		ercd = ext_tsk();
		check_ercd(ercd, E_OK);

		check_point(0);

	case 2:
		check_point(32);
		ercd = ext_tsk();
		check_ercd(ercd, E_OK);

		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
}

static uint_t	task4_count = 0;

void
task4(intptr_t exinf)
{
	ER_UINT	ercd;

	switch (++task4_count) {
	case 1:
		check_point(26);
		ercd = ext_tsk();
		check_ercd(ercd, E_OK);

		check_point(0);

	case 2:
		check_point(33);
		ercd = ext_tsk();
		check_ercd(ercd, E_OK);

		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
}
