/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2007-2013 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: test_tex2.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/* 
 *		タスク例外処理に関するテスト(2)
 *
 * 【テストの目的】
 *
 *  割込みハンドラ（アラームハンドラ）およびCPU例外ハンドラからタスク例
 *  外処理ルーチンを起動する処理を網羅的にテストする．
 *
 * 【テスト項目】
 *
 *	(A) iras_texのエラー検出
 *		(A-1) 対象タスクが休止状態
 *		(A-2) 対象タスクのタスク例外処理ルーチンが定義されていない
 *	(B) 割込みハンドラから呼ばれたiras_texの正常処理
 *		(B-1) 対象タスクが実行状態のタスクかつタスク例外処理許可
 *		(B-2) 対象タスクが実行状態のタスクでない
 *		(B-3) 対象タスクが実行状態のタスクだがタスク例外処理禁止
 *	(C) CPU例外ハンドラから呼ばれたiras_texの正常処理
 *		(C-1) 対象タスクが実行状態のタスクかつタスク例外処理許可
 *		(C-2) 対象タスクが実行状態のタスクでない
 *		(C-3) 対象タスクが実行状態のタスクだがタスク例外処理禁止
 *				→ 実施しない（ターゲット非依存に実現できない）
 *	(D) 割込みハンドラの出口処理による起動
 *		(D-1) ディスパッチ後のタスクがタスク例外許可でタスク例外処理要
 *			  求あり
 *	(E) CPU例外ハンドラの出口処理による起動
 *		(E-1) ディスパッチ後のタスクがタスク例外許可でタスク例外処理要
 *			  求あり
 *	(F) sns_texで実行状態のタスクがない
 *
 * 【使用リソース】
 *
 *	TASK1: メインのタスク．実行状態のタスクに対してタスク例外処理を要求す
 *		   る対象タスク
 *	TASK2: 実行状態でないタスクに対してタスク例外処理を要求する対象タスク
 *	TASK3: タスク例外処理ルーチンが定義されていないタスク
 *	TASK4: 休止状態のタスク
 *	ALM1:  アラームハンドラ1
 *	ALM2:  アラームハンドラ2
 *	ALM3:  アラームハンドラ3
 *	CPUEXC: CPU例外ハンドラ
 *
 * 【テストシーケンス】
 *
 *	== TASK1（優先度：10）==
 *	1:	state(false, false, TIPM_ENAALL, false, false, true)
 *		ref_tex(TSK_SELF, &rtex)
 *		assert((rtex.texstat & TTEX_DIS) != 0U)
 *		assert(rtex.pndptn == 0U)
 *	2:	sta_alm(ALM1, 1U)
 *		DO(while (!(alm1_flag)))	... アラームハンドラ1の実行を待つ
 *	== ALM1 ==
 *	3:	state_i(true, false, false, true, true)
 *		iras_tex(TASK3, 0x0001) -> E_OBJ		... (A-2)
 *		iras_tex(TASK4, 0x0001) -> E_OBJ		... (A-1)
 *		iras_tex(TASK2, 0x0001)					... (B-2)
 *		iras_tex(TASK1, 0x0001)					... (B-3)
 *		DO(alm1_flag = true)
 *		RETURN
 *	== TASK1（続き）==
 *	4:	ena_tex()
 *	== TASK1-TEX-1（1回目）==
 *	5:	assert(texptn == 0x0001)
 *		state(false, false, TIPM_ENAALL, false, false, true)
 *	6:	dis_dsp() ... 4つの状態をそれぞれ変化させる
 *		chg_ipm(TMAX_INTPRI)
 *		ena_tex()
 *		loc_cpu()
 *		state(false, true, TMAX_INTPRI, true, true, false)
 *		RETURN
 *	== TASK1（続き）==
 *	7:	state(false, false, TIPM_ENAALL, false, false, false)
 *		ref_tex(TSK_SELF, &rtex)
 *		assert((rtex.texstat & TTEX_ENA) != 0U)
 *		assert(rtex.pndptn == 0U)
 *	8:	sta_alm(ALM2, 1U)
 *		DO(while (!(alm2_flag)))
 *	== ALM2 ==
 *	9:	state_i(true, false, false, true, false)
 *		iras_tex(TASK1, 0x0002)					... (B-1)
 *		DO(alm2_flag = true)
 *		RETURN									... (D-1)
 *	== TASK1-TEX-2（2回目）==
 *	10:	assert(texptn == 0x0002)
 *		state(false, false, TIPM_ENAALL, false, false, true)
 *		RETURN
 *	== TASK1（続き）==
 *	11:	sus_tsk(TASK2)
 *		sus_tsk(TASK3)
 *	12:	sta_alm(ALM3, 10U)
 *	13:	dly_tsk(50U)
 *	== ALM3 ==
 *	14:	state_i(true, false, false, true, true)	... (F)［sns_tex()を含む］
 *		iget_tid(&tskid)
 *		assert(tskid == TSK_NONE)
 *		iras_tex(TASK1, 0x0004)
 *		DO(alm3_flag = true)
 *		RETURN
 *	== TASK1-TEX-3（3回目）==
 *	15:	assert(texptn == 0x0004)
 *		state(false, false, TIPM_ENAALL, false, false, true)
 *		RETURN
 *	== TASK1（続き）==
 *	16:	rsm_tsk(TASK2)
 *		rsm_tsk(TASK3)
 *		dis_dsp()
 *	17:	DO(RAISE_CPU_EXCEPTION)
 *	== CPUEXC ==
 *	18:	state_i(true, false, true, true, false)
 *		assert(xsns_xpn(p_excinf) == false)
 *		iras_tex(TASK3, 0x0010) -> E_OBJ		... (A-2)
 *		iras_tex(TASK4, 0x0010) -> E_OBJ		... (A-1)
 *		iras_tex(TASK2, 0x0010)					... (C-2)
 *		iras_tex(TASK1, 0x0010)					... (C-1)
 *		RETURN									... (E-1)
 *	== TASK1-TEX-4（4回目）==
 *	19:	assert(texptn == 0x0010)
 *		state(false, false, TIPM_ENAALL, true, true, true)
 *	20:	ext_tsk()
 *	== TASK2（優先度：10）==
 *	21:	state(false, false, TIPM_ENAALL, false, false, true)
 *	22:	ena_tex()
 *	== TASK2-TEX ==
 *	23:	assert(texptn == 0x0011)
 *		state(false, false, TIPM_ENAALL, false, false, true)
 *		RETURN
 *	== TASK2（続き）==
 *	24:	sus_tsk(TASK3)
 *	25:	END
 */

#include <kernel.h>
#include <test_lib.h>
#include <t_syslog.h>
#include "kernel_cfg.h"
#include "test_tex2.h"

volatile bool_t	alm1_flag = false;
volatile bool_t	alm2_flag = false;
volatile bool_t	alm3_flag = false;

/*
 *  task3，task4とtex_task4は生成されない
 */

void
task3(intptr_t exinf)
{
	check_point(0);
}

void
task4(intptr_t exinf)
{
	check_point(0);
}

void
tex_task4(TEXPTN texptn, intptr_t exinf)
{
	check_point(0);
}

/* DO NOT DELETE THIS LINE -- gentest depends on it. */

void
alarm1_handler(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(3);
	check_state_i(true, false, false, true, true);

	ercd = iras_tex(TASK3, 0x0001);
	check_ercd(ercd, E_OBJ);

	ercd = iras_tex(TASK4, 0x0001);
	check_ercd(ercd, E_OBJ);

	ercd = iras_tex(TASK2, 0x0001);
	check_ercd(ercd, E_OK);

	ercd = iras_tex(TASK1, 0x0001);
	check_ercd(ercd, E_OK);

	alm1_flag = true;

	return;

	check_point(0);
}

void
alarm2_handler(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(9);
	check_state_i(true, false, false, true, false);

	ercd = iras_tex(TASK1, 0x0002);
	check_ercd(ercd, E_OK);

	alm2_flag = true;

	return;

	check_point(0);
}

void
alarm3_handler(intptr_t exinf)
{
	ID		tskid;
	ER_UINT	ercd;

	check_point(14);
	check_state_i(true, false, false, true, true);

	ercd = iget_tid(&tskid);
	check_ercd(ercd, E_OK);

	check_assert(tskid == TSK_NONE);

	ercd = iras_tex(TASK1, 0x0004);
	check_ercd(ercd, E_OK);

	alm3_flag = true;

	return;

	check_point(0);
}

void
cpuexc_handler(void *p_excinf)
{
	ER_UINT	ercd;

	check_point(18);
	check_state_i(true, false, true, true, false);

	check_assert(xsns_xpn(p_excinf) == false);

	ercd = iras_tex(TASK3, 0x0010);
	check_ercd(ercd, E_OBJ);

	ercd = iras_tex(TASK4, 0x0010);
	check_ercd(ercd, E_OBJ);

	ercd = iras_tex(TASK2, 0x0010);
	check_ercd(ercd, E_OK);

	ercd = iras_tex(TASK1, 0x0010);
	check_ercd(ercd, E_OK);

	return;

	check_point(0);
}

void
task1(intptr_t exinf)
{
	ER_UINT	ercd;
	T_RTEX	rtex;

	test_start(__FILE__);

	check_point(1);
	check_state(false, false, TIPM_ENAALL, false, false, true);

	ercd = ref_tex(TSK_SELF, &rtex);
	check_ercd(ercd, E_OK);

	check_assert((rtex.texstat & TTEX_DIS) != 0U);

	check_assert(rtex.pndptn == 0U);

	check_point(2);
	ercd = sta_alm(ALM1, 1U);
	check_ercd(ercd, E_OK);

	while (!(alm1_flag));

	check_point(4);
	ercd = ena_tex();
	check_ercd(ercd, E_OK);

	check_point(7);
	check_state(false, false, TIPM_ENAALL, false, false, false);

	ercd = ref_tex(TSK_SELF, &rtex);
	check_ercd(ercd, E_OK);

	check_assert((rtex.texstat & TTEX_ENA) != 0U);

	check_assert(rtex.pndptn == 0U);

	check_point(8);
	ercd = sta_alm(ALM2, 1U);
	check_ercd(ercd, E_OK);

	while (!(alm2_flag));

	check_point(11);
	ercd = sus_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = sus_tsk(TASK3);
	check_ercd(ercd, E_OK);

	check_point(12);
	ercd = sta_alm(ALM3, 10U);
	check_ercd(ercd, E_OK);

	check_point(13);
	ercd = dly_tsk(50U);
	check_ercd(ercd, E_OK);

	check_point(16);
	ercd = rsm_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = rsm_tsk(TASK3);
	check_ercd(ercd, E_OK);

	ercd = dis_dsp();
	check_ercd(ercd, E_OK);

	check_point(17);
	RAISE_CPU_EXCEPTION;

	check_point(0);
}

static uint_t	tex_task1_count = 0;

void
tex_task1(TEXPTN texptn, intptr_t exinf)
{
	ER_UINT	ercd;

	switch (++tex_task1_count) {
	case 1:
		check_point(5);
		check_assert(texptn == 0x0001);

		check_state(false, false, TIPM_ENAALL, false, false, true);

		check_point(6);
		ercd = dis_dsp();
		check_ercd(ercd, E_OK);

		ercd = chg_ipm(TMAX_INTPRI);
		check_ercd(ercd, E_OK);

		ercd = ena_tex();
		check_ercd(ercd, E_OK);

		ercd = loc_cpu();
		check_ercd(ercd, E_OK);

		check_state(false, true, TMAX_INTPRI, true, true, false);

		return;

		check_point(0);

	case 2:
		check_point(10);
		check_assert(texptn == 0x0002);

		check_state(false, false, TIPM_ENAALL, false, false, true);

		return;

		check_point(0);

	case 3:
		check_point(15);
		check_assert(texptn == 0x0004);

		check_state(false, false, TIPM_ENAALL, false, false, true);

		return;

		check_point(0);

	case 4:
		check_point(19);
		check_assert(texptn == 0x0010);

		check_state(false, false, TIPM_ENAALL, true, true, true);

		check_point(20);
		ercd = ext_tsk();
		check_ercd(ercd, E_OK);

		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
}

void
task2(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(21);
	check_state(false, false, TIPM_ENAALL, false, false, true);

	check_point(22);
	ercd = ena_tex();
	check_ercd(ercd, E_OK);

	check_point(24);
	ercd = sus_tsk(TASK3);
	check_ercd(ercd, E_OK);

	check_finish(25);
	check_point(0);
}

void
tex_task2(TEXPTN texptn, intptr_t exinf)
{

	check_point(23);
	check_assert(texptn == 0x0011);

	check_state(false, false, TIPM_ENAALL, false, false, true);

	return;

	check_point(0);
}
