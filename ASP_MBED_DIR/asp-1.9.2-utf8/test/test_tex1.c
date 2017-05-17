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
 *  $Id: test_tex1.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/* 
 *		タスク例外処理に関するテスト(1)
 *
 * 【テストの目的】
 *
 *  タスクからタスク例外処理ルーチンを起動する処理を網羅的にテストする．
 *
 * 【テスト項目】
 *
 *	(A) ras_texのエラー検出
 *		(A-1) 対象タスクが休止状態［NGKI1413］
 *		(A-2) 対象タスクのタスク例外処理ルーチンが定義されていない［NGKI1414］
 *	(B) dis_texのエラー検出
 *		(B-1) 自タスクのタスク例外処理ルーチンが定義されていない［NGKI1421］
 *	(C) ena_texのエラー検出
 *		(C-1) 自タスクのタスク例外処理ルーチンが定義されていない［NGKI1426］
 *	(D) ref_texのエラー検出
 *		(D-1) 対象タスクが休止状態［NGKI1439］
 *		(D-2) 対象タスクのタスク例外処理ルーチンが定義されていない［NGKI1440］
 *	(E) ras_texの正常処理［NGKI1415］
 *		(E-1) 対象タスクが自タスク，タスク例外処理許可，かつ割込み優先
 *			  度マスク全解除状態で，すぐに実行開始
 *		(E-2) 対象タスクが自タスクでない
 *		(E-3) 対象タスクが自タスクだが，タスク例外処理禁止
 *		(E-4) 対象タスクが自タスクでタスク例外処理許可だが，割込み優先
 *			  度マスクが全解除でない
 *	(F) ena_texの正常処理［NGKI1427］
 *		(F-1) タスク例外処理要求があり，かつ割込み優先度マスク全解除状
 *			  態で，すぐに実行開始
 *		(F-2) タスク例外処理要求がない
 *		(F-3) タスク例外処理要求があるが，割込み優先度マスクが全解除で
 *			  ない
 *	(G) chg_ipmの正常処理
 *		(G-1) タスク例外処理要求があり，かつタスク例外処理許可で，すぐ
 *			  に実行開始
 *		(G-2) タスク例外処理要求がない
 *		(G-3) タスク例外処理要求があるが，タスク例外処理禁止
 *		(G-4) タスク例外処理要求があるが，割込み優先度マスクが全解除で
 *			  ない
 *	(H) タスクディスパッチャによる起動
 *		(H-1) ディスパッチ後のタスクがタスク例外許可でタスク例外処理要
 *			  求あり
 *	(I) タスク例外処理ルーチンからのリターンによる起動（連続起動）
 *	(J) タスク例外処理ルーチンからの戻り時による状態復帰
 *		(J-1) タスクに戻ってくる時
 *		(J-2) タスク例外処理ルーチンが連続起動される時
 *	(K) タスク例外処理ルーチンの多重起動
 *	(L) タスク例外処理ルーチンからの戻り時のタスク切換え
 *
 * 【使用リソース】
 *
 *	TASK1: メインのタスク．自タスクに対してタスク例外処理を要求する
 *	TASK2: 他タスクに対してタスク例外処理を要求する対象タスク
 *	TASK3: タスク例外処理ルーチンが定義されていないタスク
 *	TASK4: 休止状態のタスク
 *
 * 【テストシーケンス】
 *
 *	== TASK1（優先度：10）==
 *	1:	state(false, false, TIPM_ENAALL, false, false, true)
 *		ref_tex(TSK_SELF, &rtex)
 *		assert((rtex.texstat & TTEX_DIS) != 0U)
 *		assert(rtex.pndptn == 0U)
 *		ras_tex(TASK3, 0x0001) -> E_OBJ		... (A-2)
 *		ras_tex(TASK4, 0x0001) -> E_OBJ		... (A-1)
 *		ref_tex(TASK3, &rtex) -> E_OBJ		... (D-2)
 *		ref_tex(TASK4, &rtex) -> E_OBJ		... (D-1)
 *	2:	ena_tex()							... (F-2)
 *		state(false, false, TIPM_ENAALL, false, false, false)
 *		ref_tex(TSK_SELF, &rtex)
 *		assert((rtex.texstat & TTEX_ENA) != 0U)
 *		assert(rtex.pndptn == 0U)
 *	3:	ras_tex(TSK_SELF, 0x0001)			... (E-1)
 *	== TASK1-TEX-1（1回目）==
 *	4:	assert(texptn == 0x0001)
 *		state(false, false, TIPM_ENAALL, false, false, true)
 *	5:	dis_dsp() ... 4つの状態をそれぞれ変化させる
 *		chg_ipm(TMAX_INTPRI)
 *		ena_tex()
 *		loc_cpu()
 *		state(false, true, TMAX_INTPRI, true, true, false)
 *		RETURN
 *	== TASK1（続き）==
 *	6:	state(false, false, TIPM_ENAALL, false, false, false)	... (J-1)
 *	7:	dis_dsp() ... ディスパッチ禁止，タスク例外処理禁止
 *		dis_tex()
 *		state(false, false, TIPM_ENAALL, true, true, true)
 *	8:	ras_tex(TASK1, 0x0002)				... (E-3)
 *		ref_tex(TSK_SELF, &rtex)
 *		assert((rtex.texstat & TTEX_DIS) != 0)
 *		assert(rtex.pndptn == 0x0002)
 *	9:	ena_tex()							... (F-1)
 *	== TASK1-TEX-2（2回目）==
 *	10:	assert(texptn == 0x0002)
 *		state(false, false, TIPM_ENAALL, true, true, true)
 *	11:	ras_tex(TASK1, 0x0001)				... (E-3)
 *		ras_tex(TASK1, 0x0002)				... (E-3)
 *	12:	ena_dsp() ... 3つの状態をそれぞれ変化させる
 *		chg_ipm(TMAX_INTPRI)
 *		loc_cpu()
 *		state(false, true, TMAX_INTPRI, false, true, true)
 *		RETURN								... (I)
 *	== TASK1-TEX-3（3回目）==
 *	13:	assert(texptn == 0x0003)
 *		state(false, false, TIPM_ENAALL, true, true, true)	... (J-2)
 *	14:	ena_dsp() ... ディスパッチ許可，タスク例外許可
 *		chg_ipm(TMAX_INTPRI)
 *		ena_tex()
 *		state(false, false, TMAX_INTPRI, false, true, false)
 *		chg_ipm(TIPM_ENAALL)				... (G-2)
 *		chg_ipm(TMAX_INTPRI)
 *	15: ras_tex(TSK_SELF, 0x0004)			... (E-4)
 *		chg_ipm(TMAX_INTPRI)				... (G-4)
 *		dis_tex()
 *		chg_ipm(TIPM_ENAALL)				... (G-3)
 *		chg_ipm(TMAX_INTPRI)
 *		ena_tex()							... (F-3)
 *	16:	chg_ipm(TIPM_ENAALL)				... (G-1)(K)
 *	== TASK1-TEX-4（4回目）==
 *	17:	assert(texptn == 0x0004)
 *		state(false, false, TIPM_ENAALL, false, false, true)
 *	18:	dis_dsp() ... 3つの状態をそれぞれ変化させる
 *		chg_ipm(TMAX_INTPRI)
 *		loc_cpu()
 *		state(false, true, TMAX_INTPRI, true, true, true)
 *		RETURN
 *	== TASK1-TEX-3（3回目続き）==
 *	19:	state(false, false, TIPM_ENAALL, false, false, false)
 *		RETURN
 *	== TASK1（続き）==
 *	20:	state(false, false, TIPM_ENAALL, true, true, false)	... (J-1)
 *	21: ena_dsp()
 *		rot_rdq(TPRI_SELF)
 *	== TASK2（優先度：10）	==
 *	22:	state(false, false, TIPM_ENAALL, false, false, true)
 *	23:	ena_tex()
 *		state(false, false, TIPM_ENAALL, false, false, false)
 *		rot_rdq(TPRI_SELF)
 *	== TASK3（優先度：10）	==
 *	24:	state(false, false, TIPM_ENAALL, false, false, true)
 *	25:	ena_tex() -> E_OBJ					... (C-1)
 *		state(false, false, TIPM_ENAALL, false, false, true)
 *	26:	dis_tex() -> E_OBJ					... (B-1)
 *		state(false, false, TIPM_ENAALL, false, false, true)
 *		ext_tsk()
 *	== TASK1（続き）==
 *	27: ras_tex(TASK2, 0x0001)				... (E-2)
 *		ref_tex(TASK2, &rtex)
 *	28:	rot_rdq(TPRI_SELF)					... (H-1)
 *	== TASK2-TEX-1（1回目）==
 *	29:	assert(texptn == 0x0001)
 *		state(false, false, TIPM_ENAALL, false, false, true)
 *		RETURN
 *	== TASK2（続き）==
 *	30: ras_tex(TSK_SELF, 0x0002)
 *	== TASK2-TEX-2（2回目）==
 *	31:	assert(texptn == 0x0002)
 *		state(false, false, TIPM_ENAALL, false, false, true)
 *	32:	dis_dsp()
 *		rot_rdq(TPRI_SELF)
 *	33:	RETURN
 *	== TASK1（続き）==
 *	34:	RETURN
 *	== TASK2（続き）==
 *	35:	END
 */

#include <kernel.h>
#include <test_lib.h>
#include <t_syslog.h>
#include "kernel_cfg.h"
#include "test_tex1.h"

/*
 *  task4とtex_task4は生成されない
 */

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

	ercd = ras_tex(TASK3, 0x0001);
	check_ercd(ercd, E_OBJ);

	ercd = ras_tex(TASK4, 0x0001);
	check_ercd(ercd, E_OBJ);

	ercd = ref_tex(TASK3, &rtex);
	check_ercd(ercd, E_OBJ);

	ercd = ref_tex(TASK4, &rtex);
	check_ercd(ercd, E_OBJ);

	check_point(2);
	ercd = ena_tex();
	check_ercd(ercd, E_OK);

	check_state(false, false, TIPM_ENAALL, false, false, false);

	ercd = ref_tex(TSK_SELF, &rtex);
	check_ercd(ercd, E_OK);

	check_assert((rtex.texstat & TTEX_ENA) != 0U);

	check_assert(rtex.pndptn == 0U);

	check_point(3);
	ercd = ras_tex(TSK_SELF, 0x0001);
	check_ercd(ercd, E_OK);

	check_point(6);
	check_state(false, false, TIPM_ENAALL, false, false, false);

	check_point(7);
	ercd = dis_dsp();
	check_ercd(ercd, E_OK);

	ercd = dis_tex();
	check_ercd(ercd, E_OK);

	check_state(false, false, TIPM_ENAALL, true, true, true);

	check_point(8);
	ercd = ras_tex(TASK1, 0x0002);
	check_ercd(ercd, E_OK);

	ercd = ref_tex(TSK_SELF, &rtex);
	check_ercd(ercd, E_OK);

	check_assert((rtex.texstat & TTEX_DIS) != 0);

	check_assert(rtex.pndptn == 0x0002);

	check_point(9);
	ercd = ena_tex();
	check_ercd(ercd, E_OK);

	check_point(20);
	check_state(false, false, TIPM_ENAALL, true, true, false);

	check_point(21);
	ercd = ena_dsp();
	check_ercd(ercd, E_OK);

	ercd = rot_rdq(TPRI_SELF);
	check_ercd(ercd, E_OK);

	check_point(27);
	ercd = ras_tex(TASK2, 0x0001);
	check_ercd(ercd, E_OK);

	ercd = ref_tex(TASK2, &rtex);
	check_ercd(ercd, E_OK);

	check_point(28);
	ercd = rot_rdq(TPRI_SELF);
	check_ercd(ercd, E_OK);

	check_point(34);
	return;

	check_point(0);
}

static uint_t	tex_task1_count = 0;

void
tex_task1(TEXPTN texptn, intptr_t exinf)
{
	ER_UINT	ercd;

	switch (++tex_task1_count) {
	case 1:
		check_point(4);
		check_assert(texptn == 0x0001);

		check_state(false, false, TIPM_ENAALL, false, false, true);

		check_point(5);
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

		check_state(false, false, TIPM_ENAALL, true, true, true);

		check_point(11);
		ercd = ras_tex(TASK1, 0x0001);
		check_ercd(ercd, E_OK);

		ercd = ras_tex(TASK1, 0x0002);
		check_ercd(ercd, E_OK);

		check_point(12);
		ercd = ena_dsp();
		check_ercd(ercd, E_OK);

		ercd = chg_ipm(TMAX_INTPRI);
		check_ercd(ercd, E_OK);

		ercd = loc_cpu();
		check_ercd(ercd, E_OK);

		check_state(false, true, TMAX_INTPRI, false, true, true);

		return;

		check_point(0);

	case 3:
		check_point(13);
		check_assert(texptn == 0x0003);

		check_state(false, false, TIPM_ENAALL, true, true, true);

		check_point(14);
		ercd = ena_dsp();
		check_ercd(ercd, E_OK);

		ercd = chg_ipm(TMAX_INTPRI);
		check_ercd(ercd, E_OK);

		ercd = ena_tex();
		check_ercd(ercd, E_OK);

		check_state(false, false, TMAX_INTPRI, false, true, false);

		ercd = chg_ipm(TIPM_ENAALL);
		check_ercd(ercd, E_OK);

		ercd = chg_ipm(TMAX_INTPRI);
		check_ercd(ercd, E_OK);

		check_point(15);
		ercd = ras_tex(TSK_SELF, 0x0004);
		check_ercd(ercd, E_OK);

		ercd = chg_ipm(TMAX_INTPRI);
		check_ercd(ercd, E_OK);

		ercd = dis_tex();
		check_ercd(ercd, E_OK);

		ercd = chg_ipm(TIPM_ENAALL);
		check_ercd(ercd, E_OK);

		ercd = chg_ipm(TMAX_INTPRI);
		check_ercd(ercd, E_OK);

		ercd = ena_tex();
		check_ercd(ercd, E_OK);

		check_point(16);
		ercd = chg_ipm(TIPM_ENAALL);
		check_ercd(ercd, E_OK);

		check_point(19);
		check_state(false, false, TIPM_ENAALL, false, false, false);

		return;

		check_point(0);

	case 4:
		check_point(17);
		check_assert(texptn == 0x0004);

		check_state(false, false, TIPM_ENAALL, false, false, true);

		check_point(18);
		ercd = dis_dsp();
		check_ercd(ercd, E_OK);

		ercd = chg_ipm(TMAX_INTPRI);
		check_ercd(ercd, E_OK);

		ercd = loc_cpu();
		check_ercd(ercd, E_OK);

		check_state(false, true, TMAX_INTPRI, true, true, true);

		return;

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

	check_point(22);
	check_state(false, false, TIPM_ENAALL, false, false, true);

	check_point(23);
	ercd = ena_tex();
	check_ercd(ercd, E_OK);

	check_state(false, false, TIPM_ENAALL, false, false, false);

	ercd = rot_rdq(TPRI_SELF);
	check_ercd(ercd, E_OK);

	check_point(30);
	ercd = ras_tex(TSK_SELF, 0x0002);
	check_ercd(ercd, E_OK);

	check_finish(35);
	check_point(0);
}

static uint_t	tex_task2_count = 0;

void
tex_task2(TEXPTN texptn, intptr_t exinf)
{
	ER_UINT	ercd;

	switch (++tex_task2_count) {
	case 1:
		check_point(29);
		check_assert(texptn == 0x0001);

		check_state(false, false, TIPM_ENAALL, false, false, true);

		return;

		check_point(0);

	case 2:
		check_point(31);
		check_assert(texptn == 0x0002);

		check_state(false, false, TIPM_ENAALL, false, false, true);

		check_point(32);
		ercd = dis_dsp();
		check_ercd(ercd, E_OK);

		ercd = rot_rdq(TPRI_SELF);
		check_ercd(ercd, E_OK);

		check_point(33);
		return;

		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
}

void
task3(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(24);
	check_state(false, false, TIPM_ENAALL, false, false, true);

	check_point(25);
	ercd = ena_tex();
	check_ercd(ercd, E_OBJ);

	check_state(false, false, TIPM_ENAALL, false, false, true);

	check_point(26);
	ercd = dis_tex();
	check_ercd(ercd, E_OBJ);

	check_state(false, false, TIPM_ENAALL, false, false, true);

	ercd = ext_tsk();
	check_ercd(ercd, E_OK);

	check_point(0);
}
