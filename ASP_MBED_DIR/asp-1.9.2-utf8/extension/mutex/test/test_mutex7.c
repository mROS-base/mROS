/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2008-2014 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: test_mutex7.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/* 
 *		ミューテックスのテスト(7)
 *
 * 【テストの目的】
 *
 *  優先度上限ミューテックスに対して，タスクの強制終了時のミューテック
 *  ス関連の処理を網羅的にテストする．
 *
 * 【テスト項目】
 *
 *	(A) タスクの強制終了時（ter_tsk）
 *		(A-1) ロックしていたミューテックス（1つ）がロック解除されること．
 *		(A-2) ロックしていたミューテックス（1つ）がロック解除され，ロッ
 *			  クを待っていたタスクがそれをロックし，優先度が変化し，待
 *			  ち解除されること．
 *		(A-3) (A-2)の結果，タスクディスパッチが起こること．
 *		(A-4) ロックしていたミューテックス（複数）がすべてロック解除さ
 *			  れること．
 *		(A-5) ロックしていたミューテックス（複数）がロック解除され，ロッ
 *			  クを待っていたタスク（複数）がそれをロックし，優先度が変
 *			  化し，待ち解除されること．その時に，後でミューテックスを
 *			  ロックしたタスク（先にロックしていたミューテックスを待っ
 *			  ていたタスク）の方が，優先順位が高くなること．
 *		(A-6) (A-5)の結果，タスクディスパッチが起こること．
 *
 * 【テスト項目の実現方法】
 *
 *	(A-1)
 *		低優先度タスク（TASK2）にミューテックス（MTX1）をロックさせ，別
 *		のタスク（TASK1）からTASK2をter_tskすると，ロックが解除されるこ
 *		とを確認する．
 *	(A-2)
 *		低優先度タスク（TASK2）に中優先度上限ミューテックス（MTX1）をロッ
 *		クさせ，別の低優先度タスク（TASK3）にMTX1を待たせた状態で，高優
 *		先度タスク（TASK1）からTASK2をter_tskすると，TASK3が中優先度に
 *		なって待ち解除されることを確認する．
 *	(A-3)
 *		低優先度タスク（TASK2）に中優先度上限ミューテックス（MTX1）をロッ
 *		クさせ，別の低優先度タスク（TASK3）にMTX1を待たせた状態で，別の
 *		低優先度タスク（TASK1）からTASK2をter_tskすると，TASK3が中優先
 *		度になって待ち解除され，TASK3に切り換わることを確認する．
 *	(A-4)
 *		低優先度タスク（TASK2）にミューテックスを2つ（MTX1，MTX2）ロッ
 *		クさせ，別のタスク（TASK1）からTASK2をter_tskすると，両方のロッ
 *		クが解除されることを確認する．
 *	(A-5)
 *		低優先度タスク（TASK2）に高優先度上限ミューテックス2つ（MTX1，
 *		MTX2）をこの順でロックさせ，別の低優先度タスク2つ（TASK3，
 *		TASK4）にそれぞれのロックを待たせた状態で，高優先度タスク
 *		（TASK1）からTASK2をter_tskすると，TASK3とTASK4が中優先度になっ
 *		て待ち解除されることを確認する．また，先にロックしていたミュー
 *		テックス（MTX1）を待っていたタスク（TASK3）が，TASK4よりも優先
 *		順位が高くなることを確認する．
 *	(A-6)
 *		低優先度タスク（TASK2）に高優先度上限ミューテックス2つ（MTX1，
 *		MTX2）をこの順でロックさせ，別の低優先度タスク2つ（TASK3，
 *		TASK4）にそれぞれのロックを待たせた状態で，別の低優先度タスク
 *		（TASK1）からTASK2 をter_tskすると，TASK3とTASK4が中優先度になっ
 *		て待ち解除され，TASK3に切り換わることを確認する．
 *
 * 【使用リソース】
 *
 *	TASK1: 高優先度タスク，メインタスク，最初から起動
 *	TASK2: 低優先度タスク
 *	TASK3: 低優先度タスク
 *	TASK4: 低優先度タスク
 *	MTX1: ミューテックス（TA_CEILING属性，上限は中優先度）
 *	MTX2: ミューテックス（TA_CEILING属性，上限は中優先度）
 *
 * 【テストシーケンス】
 *
 *	== TASK1（優先度：高）==
 *		call(set_bit_func(bit_mutex))
 *	1:	act_tsk(TASK2)
 *	2:	slp_tsk()
 *	//		低：TASK2
 *	== TASK2-1（優先度：低）==
 *	3:	loc_mtx(MTX1)
 *	//		中：TASK2，MTX1：TASK2
 *	4:	wup_tsk(TASK1)
 *	//		高：TASK1，中：TASK2，MTX1：TASK2
 *	== TASK1（続き）==
 *	5:	ter_tsk(TASK2)			... (A-1)
 *	//		高：TASK1
 *	6:	ref_mtx(MTX1, &rmtx)
 *		assert(rmtx.htskid == TSK_NONE)
 *		assert(rmtx.wtskid == TSK_NONE)
 *
 *	7:	act_tsk(TASK2)
 *		act_tsk(TASK3)
 *	8:	slp_tsk()
 *	//		低：TASK2→TASK3
 *	== TASK2-2（優先度：低）==
 *	9:	loc_mtx(MTX1)
 *	//		中：TASK2，低：TASK3，MTX1：TASK2
 *	10:	tslp_tsk(10) -> E_TMOUT
 *	//		低：TASK3，MTX1：TASK2
 *	== TASK3-1（優先度：低）==
 *	11:	loc_mtx(MTX1)
 *	//		MTX1：TASK2→TASK3
 *	//		タイムアウト後
 *	//		中：TASK2，MTX1：TASK2→TASK3
 *	== TASK2-2（続き）==
 *	12:	wup_tsk(TASK1)
 *	//		高：TASK1，中：TASK2，MTX1：TASK2→TASK3
 *	== TASK1（続き）==
 *	13:	ter_tsk(TASK2)			... (A-2)
 *	//		高：TASK1，中：TASK3，MTX1：TASK3
 *	14:	ref_mtx(MTX1, &rmtx)
 *		assert(rmtx.htskid == TASK3)
 *		assert(rmtx.wtskid == TSK_NONE)
 *		get_pri(TASK3, &tskpri)
 *		assert(tskpri == MID_PRIORITY)
 *	15:	ter_tsk(TASK3)
 *	//		高：TASK1
 *
 *	16:	act_tsk(TASK2)
 *		act_tsk(TASK3)
 *	17:	chg_pri(TSK_SELF, LOW_PRIORITY)
 *	//		低：TASK2→TASK3→TASK1
 *	== TASK2-3（優先度：低）==
 *	18:	loc_mtx(MTX1)
 *	//		中：TASK2，低：TASK3→TASK1，MTX1：TASK2
 *	19:	slp_tsk()
 *	//		低：TASK3→TASK1，MTX1：TASK2
 *	== TASK3-2（優先度：低）==
 *	20:	loc_mtx(MTX1)
 *	//		低：TASK1，MTX1：TASK2→TASK3
 *	== TASK1（続き）==
 *	21:	ter_tsk(TASK2)			... (A-3)
 *	//		中：TASK3，低：TASK1，MTX1：TASK3
 *	== TASK3-2（続き）==
 *	22:	ref_mtx(MTX1, &rmtx)
 *		assert(rmtx.htskid == TASK3)
 *		assert(rmtx.wtskid == TSK_NONE)
 *		get_pri(TASK3, &tskpri)
 *		assert(tskpri == MID_PRIORITY)
 *	23:	ext_tsk() -> noreturn
 *	//		低：TASK1
 *	== TASK1（続き）==
 *	24:	chg_pri(TSK_SELF, HIGH_PRIORITY)
 *	//		高：TASK1
 *
 *	25:	act_tsk(TASK2)
 *	26:	slp_tsk()
 *	//		低：TASK2
 *	== TASK2-4（優先度：低）==
 *	27:	loc_mtx(MTX1)
 *		loc_mtx(MTX2)
 *	//		中：TASK2，MTX1：TASK2，MTX2：TASK2
 *	28:	wup_tsk(TASK1)
 *	//		高：TASK1，中：TASK2，MTX1：TASK2，MTX2：TASK2
 *	== TASK1（続き）==
 *	29:	ter_tsk(TASK2)			... (A-4)
 *	//		高：TASK1
 *	30:	ref_mtx(MTX1, &rmtx)
 *		assert(rmtx.htskid == TSK_NONE)
 *		assert(rmtx.wtskid == TSK_NONE)
 *		ref_mtx(MTX2, &rmtx)
 *		assert(rmtx.htskid == TSK_NONE)
 *		assert(rmtx.wtskid == TSK_NONE)
 *
 *	31:	act_tsk(TASK2)
 *		act_tsk(TASK3)
 *		act_tsk(TASK4)
 *	32:	slp_tsk()
 *	//		低：TASK2→TASK3→TASK4
 *	== TASK2-5（優先度：低）==
 *	33:	loc_mtx(MTX1)
 *		loc_mtx(MTX2)
 *	//		中：TASK2，低：TASK3→TASK4，MTX1：TASK2，MTX2：TASK2
 *	34:	tslp_tsk(10) -> E_TMOUT
 *	//		低：TASK3→TASK4，MTX1：TASK2，MTX2：TASK2
 *	== TASK3-3（優先度：低）==
 *	35:	loc_mtx(MTX1)
 *	//		低：TASK4，MTX1：TASK2→TASK3，MTX2：TASK2
 *	== TASK4-1（優先度：低）==
 *	36:	loc_mtx(MTX2)
 *	//		MTX1：TASK2→TASK3，MTX2：TASK2→TASK4
 *	//		タイムアウト後
 *	//		中：TASK2，MTX1：TASK2→TASK3，MTX2：TASK2→TASK4
 *	== TASK2-5（続き）==
 *	37:	wup_tsk(TASK1)
 *	//		高：TASK1，中：TASK2，MTX1：TASK2→TASK3，MTX2：TASK2→TASK4
 *	== TASK1（続き）==
 *	38:	ter_tsk(TASK2)			... (A-5)
 *	//		高：TASK1，中：TASK4→TASK3，MTX1：TASK3，MTX2：TASK4
 *	39:	ref_mtx(MTX1, &rmtx)
 *		assert(rmtx.htskid == TASK3)
 *		assert(rmtx.wtskid == TSK_NONE)
 *		ref_mtx(MTX2, &rmtx)
 *		assert(rmtx.htskid == TASK4)
 *		assert(rmtx.wtskid == TSK_NONE)
 *		get_pri(TASK3, &tskpri)
 *		assert(tskpri == MID_PRIORITY)
 *		get_pri(TASK4, &tskpri)
 *		assert(tskpri == MID_PRIORITY)
 *	40:	chg_pri(TSK_SELF, LOW_PRIORITY)
 *	//		中：TASK4→TASK3，低：TASK1，MTX1：TASK3，MTX2：TASK4
 *	== TASK4-1（続き）==
 *	41:	ext_tsk() -> noreturn
 *	//		中：TASK3，低：TASK1，MTX1：TASK3
 *	== TASK3-3（続き）==
 *	42:	ext_tsk() -> noreturn
 *	//		低：TASK1
 *	== TASK1（続き）==
 *	43:	act_tsk(TASK2)
 *		act_tsk(TASK3)
 *		act_tsk(TASK4)
 *	//		低：TASK1→TASK2→TASK3→TASK4
 *	44:	chg_pri(TSK_SELF, LOW_PRIORITY)
 *	//		低：TASK2→TASK3→TASK4→TASK1
 *	== TASK2-6（優先度：低）==
 *	45:	loc_mtx(MTX1)
 *		loc_mtx(MTX2)
 *	//		中：TASK2，低：TASK3→TASK4→TASK1，MTX1：TASK2，MTX2：TASK2
 *	46:	slp_tsk()
 *	//		低：TASK3→TASK4→TASK1，MTX1：TASK2，MTX2：TASK2
 *	== TASK3-4（優先度：低）==
 *	47:	loc_mtx(MTX1)
 *	//		低：TASK4→TASK1，MTX1：TASK2→TASK3，MTX2：TASK2
 *	== TASK4-2（優先度：低）==
 *	48:	loc_mtx(MTX2)
 *	//		低：TASK1，MTX1：TASK2→TASK3，MTX2：TASK2→TASK4
 *	== TASK1（続き）==
 *	49:	ter_tsk(TASK2)			... (A-6)
 *	//		中：TASK4→TASK3，低：TASK1，MTX1：TASK3，MTX2：TASK4
 *	== TASK4-2（続き）==
 *	50:	ref_mtx(MTX1, &rmtx)
 *		assert(rmtx.htskid == TASK3)
 *		assert(rmtx.wtskid == TSK_NONE)
 *		ref_mtx(MTX2, &rmtx)
 *		assert(rmtx.htskid == TASK4)
 *		assert(rmtx.wtskid == TSK_NONE)
 *		get_pri(TASK3, &tskpri)
 *		assert(tskpri == MID_PRIORITY)
 *		get_pri(TASK4, &tskpri)
 *		assert(tskpri == MID_PRIORITY)
 *	51:	ext_tsk() -> noreturn
 *	//		中：TASK3，低：TASK1，MTX1：TASK3
 *	== TASK3-4（続き）==
 *	52:	ext_tsk() -> noreturn
 *	//		低：TASK1
 *	== TASK1（続き）==
 *	53:	END
 */

#include <kernel.h>
#include <t_syslog.h>
#include "kernel_cfg.h"
#include "test_lib.h"
#include "test_mutex7.h"

extern ER	bit_mutex(void);

/* DO NOT DELETE THIS LINE -- gentest depends on it. */

void
task1(intptr_t exinf)
{
	ER_UINT	ercd;
	PRI		tskpri;
	T_RMTX	rmtx;

	test_start(__FILE__);

	set_bit_func(bit_mutex);

	check_point(1);
	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(2);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(5);
	ercd = ter_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(6);
	ercd = ref_mtx(MTX1, &rmtx);
	check_ercd(ercd, E_OK);

	check_assert(rmtx.htskid == TSK_NONE);

	check_assert(rmtx.wtskid == TSK_NONE);

	check_point(7);
	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = act_tsk(TASK3);
	check_ercd(ercd, E_OK);

	check_point(8);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(13);
	ercd = ter_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(14);
	ercd = ref_mtx(MTX1, &rmtx);
	check_ercd(ercd, E_OK);

	check_assert(rmtx.htskid == TASK3);

	check_assert(rmtx.wtskid == TSK_NONE);

	ercd = get_pri(TASK3, &tskpri);
	check_ercd(ercd, E_OK);

	check_assert(tskpri == MID_PRIORITY);

	check_point(15);
	ercd = ter_tsk(TASK3);
	check_ercd(ercd, E_OK);

	check_point(16);
	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = act_tsk(TASK3);
	check_ercd(ercd, E_OK);

	check_point(17);
	ercd = chg_pri(TSK_SELF, LOW_PRIORITY);
	check_ercd(ercd, E_OK);

	check_point(21);
	ercd = ter_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(24);
	ercd = chg_pri(TSK_SELF, HIGH_PRIORITY);
	check_ercd(ercd, E_OK);

	check_point(25);
	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(26);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(29);
	ercd = ter_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(30);
	ercd = ref_mtx(MTX1, &rmtx);
	check_ercd(ercd, E_OK);

	check_assert(rmtx.htskid == TSK_NONE);

	check_assert(rmtx.wtskid == TSK_NONE);

	ercd = ref_mtx(MTX2, &rmtx);
	check_ercd(ercd, E_OK);

	check_assert(rmtx.htskid == TSK_NONE);

	check_assert(rmtx.wtskid == TSK_NONE);

	check_point(31);
	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = act_tsk(TASK3);
	check_ercd(ercd, E_OK);

	ercd = act_tsk(TASK4);
	check_ercd(ercd, E_OK);

	check_point(32);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(38);
	ercd = ter_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(39);
	ercd = ref_mtx(MTX1, &rmtx);
	check_ercd(ercd, E_OK);

	check_assert(rmtx.htskid == TASK3);

	check_assert(rmtx.wtskid == TSK_NONE);

	ercd = ref_mtx(MTX2, &rmtx);
	check_ercd(ercd, E_OK);

	check_assert(rmtx.htskid == TASK4);

	check_assert(rmtx.wtskid == TSK_NONE);

	ercd = get_pri(TASK3, &tskpri);
	check_ercd(ercd, E_OK);

	check_assert(tskpri == MID_PRIORITY);

	ercd = get_pri(TASK4, &tskpri);
	check_ercd(ercd, E_OK);

	check_assert(tskpri == MID_PRIORITY);

	check_point(40);
	ercd = chg_pri(TSK_SELF, LOW_PRIORITY);
	check_ercd(ercd, E_OK);

	check_point(43);
	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = act_tsk(TASK3);
	check_ercd(ercd, E_OK);

	ercd = act_tsk(TASK4);
	check_ercd(ercd, E_OK);

	check_point(44);
	ercd = chg_pri(TSK_SELF, LOW_PRIORITY);
	check_ercd(ercd, E_OK);

	check_point(49);
	ercd = ter_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_finish(53);
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
		ercd = wup_tsk(TASK1);
		check_ercd(ercd, E_OK);

		check_point(0);

	case 2:
		check_point(9);
		ercd = loc_mtx(MTX1);
		check_ercd(ercd, E_OK);

		check_point(10);
		ercd = tslp_tsk(10);
		check_ercd(ercd, E_TMOUT);

		check_point(12);
		ercd = wup_tsk(TASK1);
		check_ercd(ercd, E_OK);

		check_point(0);

	case 3:
		check_point(18);
		ercd = loc_mtx(MTX1);
		check_ercd(ercd, E_OK);

		check_point(19);
		ercd = slp_tsk();
		check_ercd(ercd, E_OK);

		check_point(0);

	case 4:
		check_point(27);
		ercd = loc_mtx(MTX1);
		check_ercd(ercd, E_OK);

		ercd = loc_mtx(MTX2);
		check_ercd(ercd, E_OK);

		check_point(28);
		ercd = wup_tsk(TASK1);
		check_ercd(ercd, E_OK);

		check_point(0);

	case 5:
		check_point(33);
		ercd = loc_mtx(MTX1);
		check_ercd(ercd, E_OK);

		ercd = loc_mtx(MTX2);
		check_ercd(ercd, E_OK);

		check_point(34);
		ercd = tslp_tsk(10);
		check_ercd(ercd, E_TMOUT);

		check_point(37);
		ercd = wup_tsk(TASK1);
		check_ercd(ercd, E_OK);

		check_point(0);

	case 6:
		check_point(45);
		ercd = loc_mtx(MTX1);
		check_ercd(ercd, E_OK);

		ercd = loc_mtx(MTX2);
		check_ercd(ercd, E_OK);

		check_point(46);
		ercd = slp_tsk();
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
	PRI		tskpri;
	T_RMTX	rmtx;

	switch (++task3_count) {
	case 1:
		check_point(11);
		ercd = loc_mtx(MTX1);
		check_ercd(ercd, E_OK);

		check_point(0);

	case 2:
		check_point(20);
		ercd = loc_mtx(MTX1);
		check_ercd(ercd, E_OK);

		check_point(22);
		ercd = ref_mtx(MTX1, &rmtx);
		check_ercd(ercd, E_OK);

		check_assert(rmtx.htskid == TASK3);

		check_assert(rmtx.wtskid == TSK_NONE);

		ercd = get_pri(TASK3, &tskpri);
		check_ercd(ercd, E_OK);

		check_assert(tskpri == MID_PRIORITY);

		check_point(23);
		ercd = ext_tsk();

		check_point(0);

	case 3:
		check_point(35);
		ercd = loc_mtx(MTX1);
		check_ercd(ercd, E_OK);

		check_point(42);
		ercd = ext_tsk();

		check_point(0);

	case 4:
		check_point(47);
		ercd = loc_mtx(MTX1);
		check_ercd(ercd, E_OK);

		check_point(52);
		ercd = ext_tsk();

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
	PRI		tskpri;
	T_RMTX	rmtx;

	switch (++task4_count) {
	case 1:
		check_point(36);
		ercd = loc_mtx(MTX2);
		check_ercd(ercd, E_OK);

		check_point(41);
		ercd = ext_tsk();

		check_point(0);

	case 2:
		check_point(48);
		ercd = loc_mtx(MTX2);
		check_ercd(ercd, E_OK);

		check_point(50);
		ercd = ref_mtx(MTX1, &rmtx);
		check_ercd(ercd, E_OK);

		check_assert(rmtx.htskid == TASK3);

		check_assert(rmtx.wtskid == TSK_NONE);

		ercd = ref_mtx(MTX2, &rmtx);
		check_ercd(ercd, E_OK);

		check_assert(rmtx.htskid == TASK4);

		check_assert(rmtx.wtskid == TSK_NONE);

		ercd = get_pri(TASK3, &tskpri);
		check_ercd(ercd, E_OK);

		check_assert(tskpri == MID_PRIORITY);

		ercd = get_pri(TASK4, &tskpri);
		check_ercd(ercd, E_OK);

		check_assert(tskpri == MID_PRIORITY);

		check_point(51);
		ercd = ext_tsk();

		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
}
