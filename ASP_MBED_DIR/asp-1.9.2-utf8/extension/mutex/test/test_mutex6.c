/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2007-2014 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: test_mutex6.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/* 
 *		ミューテックスのテスト(6)
 *
 * 【テストの目的】
 *
 *  優先度上限ミューテックスに対して，タスクの終了時，タスクの優先順位
 *  の回転時のミューテックス関連の処理を網羅的にテストする．
 *
 * 【テスト項目】
 *
 *	(A) タスクの終了時（ext_tsk）
 *		(A-1) ロックしていたミューテックス（1つ）がロック解除されること．
 *		(A-2) ロックしていたミューテックス（1つ）がロック解除され，ロッ
 *			  クを待っていたタスクがそれをロックし，優先度が変化し，待
 *			  ち解除されること．
 *		(A-3) ロックしていたミューテックス（複数）がすべてロック解除さ
 *			  れること．
 *		(A-4) ロックしていたミューテックス（複数）がロック解除され，ロッ
 *			  クを待っていたタスク（複数）がそれをロックし，優先度が変
 *			  化し，待ち解除されること．その時に，後でミューテックスを
 *			  ロックしたタスク（先にロックしていたミューテックスを待っ
 *			  ていたタスク）の方が，優先順位が高くなること．
 *	(B) タスクの優先順位の回転（rot_rdq）
 *		(B-1) TPRI_SELFを指定した時に，タスクのベース優先度の優先順位が
 *		　　　回転すること．
 *		(B-2) TPRI_SELFを指定した時に，タスクの現在優先度の優先順位が回
 *		　　　転しないこと．
 *
 * 【テスト項目の実現方法】
 *
 *	(A-1)
 *		低優先度タスク（TASK2）にミューテックス（MTX1）をロックさせ，
 *		ext_tskすると，ロックが解除されることを確認する．
 *	(A-2)
 *		低優先度タスク（TASK2）に高優先度上限ミューテックス（MTX1）をロッ
 *		クさせ，別の低優先度タスク（TASK3）にMTX1を待たせた状態で，
 *		TASK2 がext_tskすると，TASK3が高優先度になって待ち解除されるこ
 *		とを確認する．
 *	(A-3)
 *		低優先度タスク（TASK2）にミューテックスを2つ（MTX1，MTX2）ロッ
 *		クさせ，ext_tskすると，両方のロックが解除されることを確認する．
 *	(A-4)
 *		低優先度タスク（TASK2）に高優先度上限ミューテックス2つ（MTX1，
 *		MTX2）をこの順でロックさせ，別の低優先度タスク2つ（TASK3，
 *		TASK4）にそれぞれのロックを待たせた状態で，TASK2をext_tskすると，
 *		TASK3とTASK4が高優先度になって待ち解除されることを確認する．ま
 *		た，先にロックしていたミューテックス（MTX1）を待っていたタスク
 *		（TASK3）が，TASK4よりも優先順位が高くなることを確認する．
 *	(B-1)
 *		低優先度タスクが3つ（TASK2，TASK3，TASK4）が実行できる状態の時
 *		に，1つの低優先度タスク（TASK2）に高優先度上限ミューテックスを
 *		ロックさせ，rot_rdq(TPRI_SELF)すると，残りの2つの低優先度タスク
 *		の優先順位が回転することを確認する．
 *	(B-2)
 *		上と同じ状況で，高優先度タスクの優先順位が回転しないことを確認
 *		する．
 *
 * 【使用リソース】
 *
 *	TASK1: 低優先度タスク，メインタスク，最初から起動
 *	TASK2: 低優先度タスク
 *	TASK3: 低優先度タスク
 *	TASK4: 低優先度タスク
 *	MTX1: ミューテックス（TA_CEILING属性，上限は高優先度）
 *	MTX2: ミューテックス（TA_CEILING属性，上限は高優先度）
 *
 * 【テストシーケンス】
 *
 *	== TASK1（優先度：低）==
 *		call(set_bit_func(bit_mutex))
 *	1:	act_tsk(TASK2)
 *	2:	rot_rdq(TPRI_SELF)
 *	//		低：TASK2→TASK1
 *	== TASK2-1（優先度：低）1回め ==
 *	3:	loc_mtx(MTX1)
 *	//		高：TASK2，低：TASK1，MTX1：TASK2
 *	4:	ext_tsk() -> noreturn			... (A-1)
 *	//		低：TASK1
 *	== TASK1（続き）==
 *	5:	ref_mtx(MTX1, &rmtx)
 *		assert(rmtx.htskid == TSK_NONE)
 *		assert(rmtx.wtskid == TSK_NONE)
 *
 *	6:	act_tsk(TASK2)
 *		act_tsk(TASK3)
 *	7:	rot_rdq(TPRI_SELF)
 *	//		低：TASK2→TASK3→TASK1
 *	== TASK2-2（優先度：低）2回め ==
 *	8:	loc_mtx(MTX1)
 *	//		高：TASK2，低：TASK3→TASK1，MTX1：TASK2
 *	9:	slp_tsk()
 *	//		低：TASK3→TASK1，MTX1：TASK2
 *	== TASK3（優先度：低）==
 *	10:	loc_mtx(MTX1)
 *	//		低：TASK1，MTX1：TASK2→TASK3
 *	== TASK1（続き）==
 *	11:	wup_tsk(TASK2)
 *	//		高：TASK2，低：TASK1，MTX1：TASK2→TASK3
 *	== TASK2-2（続き）==
 *	12:	ext_tsk() -> noreturn			... (A-2)
 *	//		高：TASK3，低：TASK1，MTX1：TASK3
 *	== TASK3（続き）==
 *	13:	ref_mtx(MTX1, &rmtx)
 *		assert(rmtx.htskid == TASK3)
 *		assert(rmtx.wtskid == TSK_NONE)
 *		get_pri(TPRI_SELF, &tskpri)
 *		assert(tskpri == HIGH_PRIORITY)
 *		unl_mtx(MTX1)
 *		slp_tsk()
 *	//		低：TASK1
 *
 *	== TASK1（続き）==
 *	14:	act_tsk(TASK2)
 *	15:	rot_rdq(TPRI_SELF)
 *	//		低：TASK2→TASK1
 *	== TASK2-3（優先度：低）3回め ==
 *	16:	loc_mtx(MTX1)
 *		loc_mtx(MTX2)
 *	//		高：TASK2，低：TASK1，MTX1：TASK2，MTX2：TASK2
 *	17:	ext_tsk() -> noreturn			... (A-3)
 *	//		低：TASK1
 *	== TASK1（続き）==
 *	18:	ref_mtx(MTX1, &rmtx)
 *		assert(rmtx.htskid == TSK_NONE)
 *		assert(rmtx.wtskid == TSK_NONE)
 *		ref_mtx(MTX2, &rmtx)
 *		assert(rmtx.htskid == TSK_NONE)
 *		assert(rmtx.wtskid == TSK_NONE)
 *
 *	19:	act_tsk(TASK2)
 *		wup_tsk(TASK3)
 *		act_tsk(TASK4)
 *	20:	rot_rdq(TPRI_SELF)
 *	//		低：TASK2→TASK3→TASK4→TASK1
 *	== TASK2-4（優先度：低）4回め ==
 *	21:	loc_mtx(MTX1)
 *		loc_mtx(MTX2)
 *	//		高：TASK2，低：TASK3→TASK4→TASK1，MTX1：TASK2，MTX2：TASK2
 *	22:	chg_pri(TASK1, HIGH_PRIORITY)
 *	//		高：TASK2→TASK1，低：TASK3→TASK4，MTX1：TASK2，MTX2：TASK2
 *	23:	rot_rdq(TPRI_SELF)				... (B-1)(B-2)
 *	//		高：TASK2→TASK1，低：TASK4→TASK3，MTX1：TASK2，MTX2：TASK2
 *	24:	slp_tsk()
 *	//		高：TASK1，低：TASK4→TASK3，MTX1：TASK2，MTX2：TASK2
 *	== TASK1（続き）==
 *	25:	chg_pri(TSK_SELF, TPRI_INI)
 *	//		低：TASK4→TASK3→TASK1，MTX1：TASK2，MTX2：TASK2
 *	== TASK4（優先度：低）==
 *	26:	loc_mtx(MTX2)
 *	//		低：TASK3→TASK1，MTX1：TASK2，MTX2：TASK2→TASK4
 *	== TASK3（続き）==
 *	27:	loc_mtx(MTX1)
 *	//		低：TASK1，MTX1：TASK2→TASK3，MTX2：TASK2→TASK4
 *	== TASK1（続き）==
 *	28:	wup_tsk(TASK2)
 *	//		高：TASK2，低：TASK1，MTX1：TASK2→TASK3，MTX2：TASK2→TASK4
 *	== TASK2-4（続き）==
 *	29:	ext_tsk() -> noreturn			... (A-4)
 *	//		高：TASK4→TASK3，低：TASK1，MTX1：TASK3，MTX2：TASK4
 *	== TASK4（続き）==
 *	30:	ref_mtx(MTX1, &rmtx)
 *		assert(rmtx.htskid == TASK3)
 *		assert(rmtx.wtskid == TSK_NONE)
 *		ref_mtx(MTX2, &rmtx)
 *		assert(rmtx.htskid == TASK4)
 *		assert(rmtx.wtskid == TSK_NONE)
 *		get_pri(TPRI_SELF, &tskpri)
 *		assert(tskpri == HIGH_PRIORITY)
 *		ext_tsk() -> noreturn
 *	//		高：TASK3，低：TASK1，MTX2：TASK4
 *	== TASK3（続き）==
 *	31:	get_pri(TPRI_SELF, &tskpri)
 *		assert(tskpri == HIGH_PRIORITY)
 *		ext_tsk() -> noreturn
 *	//		低：TASK1
 *	== TASK1（続き）==
 *	32:	END
 */

#include <kernel.h>
#include <t_syslog.h>
#include "kernel_cfg.h"
#include "test_lib.h"
#include "test_mutex6.h"

extern ER	bit_mutex(void);

/* DO NOT DELETE THIS LINE -- gentest depends on it. */

void
task1(intptr_t exinf)
{
	ER_UINT	ercd;
	T_RMTX	rmtx;

	test_start(__FILE__);

	set_bit_func(bit_mutex);

	check_point(1);
	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(2);
	ercd = rot_rdq(TPRI_SELF);
	check_ercd(ercd, E_OK);

	check_point(5);
	ercd = ref_mtx(MTX1, &rmtx);
	check_ercd(ercd, E_OK);

	check_assert(rmtx.htskid == TSK_NONE);

	check_assert(rmtx.wtskid == TSK_NONE);

	check_point(6);
	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = act_tsk(TASK3);
	check_ercd(ercd, E_OK);

	check_point(7);
	ercd = rot_rdq(TPRI_SELF);
	check_ercd(ercd, E_OK);

	check_point(11);
	ercd = wup_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(14);
	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(15);
	ercd = rot_rdq(TPRI_SELF);
	check_ercd(ercd, E_OK);

	check_point(18);
	ercd = ref_mtx(MTX1, &rmtx);
	check_ercd(ercd, E_OK);

	check_assert(rmtx.htskid == TSK_NONE);

	check_assert(rmtx.wtskid == TSK_NONE);

	ercd = ref_mtx(MTX2, &rmtx);
	check_ercd(ercd, E_OK);

	check_assert(rmtx.htskid == TSK_NONE);

	check_assert(rmtx.wtskid == TSK_NONE);

	check_point(19);
	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = wup_tsk(TASK3);
	check_ercd(ercd, E_OK);

	ercd = act_tsk(TASK4);
	check_ercd(ercd, E_OK);

	check_point(20);
	ercd = rot_rdq(TPRI_SELF);
	check_ercd(ercd, E_OK);

	check_point(25);
	ercd = chg_pri(TSK_SELF, TPRI_INI);
	check_ercd(ercd, E_OK);

	check_point(28);
	ercd = wup_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_finish(32);
	check_point(0);
}

static uint_t	task2_count = 0;

void
task2(intptr_t exinf)
{
	ER_UINT	ercd;

	switch (++task2_count) {
	case 1:
		check_point(3);
		ercd = loc_mtx(MTX1);
		check_ercd(ercd, E_OK);

		check_point(4);
		ercd = ext_tsk();

		check_point(0);

	case 2:
		check_point(8);
		ercd = loc_mtx(MTX1);
		check_ercd(ercd, E_OK);

		check_point(9);
		ercd = slp_tsk();
		check_ercd(ercd, E_OK);

		check_point(12);
		ercd = ext_tsk();

		check_point(0);

	case 3:
		check_point(16);
		ercd = loc_mtx(MTX1);
		check_ercd(ercd, E_OK);

		ercd = loc_mtx(MTX2);
		check_ercd(ercd, E_OK);

		check_point(17);
		ercd = ext_tsk();

		check_point(0);

	case 4:
		check_point(21);
		ercd = loc_mtx(MTX1);
		check_ercd(ercd, E_OK);

		ercd = loc_mtx(MTX2);
		check_ercd(ercd, E_OK);

		check_point(22);
		ercd = chg_pri(TASK1, HIGH_PRIORITY);
		check_ercd(ercd, E_OK);

		check_point(23);
		ercd = rot_rdq(TPRI_SELF);
		check_ercd(ercd, E_OK);

		check_point(24);
		ercd = slp_tsk();
		check_ercd(ercd, E_OK);

		check_point(29);
		ercd = ext_tsk();

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
	PRI		tskpri;
	T_RMTX	rmtx;

	check_point(10);
	ercd = loc_mtx(MTX1);
	check_ercd(ercd, E_OK);

	check_point(13);
	ercd = ref_mtx(MTX1, &rmtx);
	check_ercd(ercd, E_OK);

	check_assert(rmtx.htskid == TASK3);

	check_assert(rmtx.wtskid == TSK_NONE);

	ercd = get_pri(TPRI_SELF, &tskpri);
	check_ercd(ercd, E_OK);

	check_assert(tskpri == HIGH_PRIORITY);

	ercd = unl_mtx(MTX1);
	check_ercd(ercd, E_OK);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(27);
	ercd = loc_mtx(MTX1);
	check_ercd(ercd, E_OK);

	check_point(31);
	ercd = get_pri(TPRI_SELF, &tskpri);
	check_ercd(ercd, E_OK);

	check_assert(tskpri == HIGH_PRIORITY);

	ercd = ext_tsk();

	check_point(0);
}

void
task4(intptr_t exinf)
{
	ER_UINT	ercd;
	PRI		tskpri;
	T_RMTX	rmtx;

	check_point(26);
	ercd = loc_mtx(MTX2);
	check_ercd(ercd, E_OK);

	check_point(30);
	ercd = ref_mtx(MTX1, &rmtx);
	check_ercd(ercd, E_OK);

	check_assert(rmtx.htskid == TASK3);

	check_assert(rmtx.wtskid == TSK_NONE);

	ercd = ref_mtx(MTX2, &rmtx);
	check_ercd(ercd, E_OK);

	check_assert(rmtx.htskid == TASK4);

	check_assert(rmtx.wtskid == TSK_NONE);

	ercd = get_pri(TPRI_SELF, &tskpri);
	check_ercd(ercd, E_OK);

	check_assert(tskpri == HIGH_PRIORITY);

	ercd = ext_tsk();

	check_point(0);
}
