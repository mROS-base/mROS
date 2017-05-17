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
 *  $Id: test_mutex4.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/* 
 *		ミューテックスのテスト(4)
 *
 * 【テストの目的】
 *
 *  優先度上限ミューテックスに対して，loc_mtxとunl_mtxに伴う優先度の変
 *  更処理を網羅的にテストする．ただし，change_priorityと
 *  mutex_calc_priorityの内容には踏み込まない．
 *
 * 【テスト項目】
 *
 *	(A) ミューテックスのロック処理（loc_mtx）に伴う優先度変更
 *		(A-1) ロックしたミューテックスの優先度上限が自タスクの現在優先
 *			  度よりも高い場合に，自タスクの優先度がその優先度上限まで
 *			  上がること．また，同じ優先度内での優先順位が最高になるこ
 *			  と
 *		(A-2) ロックしたミューテックスの優先度上限が自タスクの現在優先
 *			  度と同じ場合に，自タスクの優先度が変わらないこと．また，
 *			  同じ優先度内での優先順位が変わらないこと
 *		(A-3) ロックしたミューテックスの優先度上限が自タスクの現在優先
 *			  度よりも低い場合に，自タスクの優先度が変わらないこと．ま
 *			  た，同じ優先度内での優先順位が変わらないこと
 *	(B) ミューテックスのロック解除処理（unl_mtx）に伴うロック解除した
 * 		タスクの優先度変更
 *		(B-1) ロック解除したミューテックスの上限優先度が自タスクの現在
 *			  優先度と同じで，ミューテックスのロック解除で優先度が下が
 *			  るべき場合に，自タスクの優先度が適切に下げられること．ま
 *			  た，同じ優先度内での優先順位が最高になること
 *		(B-2) ロック解除したミューテックスの上限優先度が自タスクの現在
 *			  優先度と同じで，ミューテックスのロック解除で優先度が変わ
 *			  るべきでない場合に，自タスクの優先度が変わらないこと．ま
 *			  た，同じ優先度内での優先順位が変わらないこと
 *		(B-3) ロック解除したミューテックスの上限優先度が自タスクの現在
 *			  優先度よりも低い場合に，自タスクの優先度が変わらないこと．
 *			  また，同じ優先度内での優先順位が変わらないこと
 *	(C) ミューテックスのロック解除処理（unl_mtx）に伴いミューテックスを
 *		ロックしたタスクの優先度変更
 *		(C-1) ミューテックスの優先度上限が新たにミューテックスをロック
 *			  したタスクの現在優先度よりも高い場合に，当該タスクの優先
 *			  度がその優先度上限まで上がること．また，同じ優先度内での
 *			  優先順位が最低になること
 *		(C-2) ミューテックスの優先度上限が新たにミューテックスをロック
 *			  したタスクの現在優先度と同じ場合に，当該タスクの優先度が
 *			  変わらないこと．また，同じ優先度内での優先順位が最低にな
 *			  ること
 *		(C-3) ミューテックスの優先度上限が新たにミューテックスをロック
 *			  したタスクの現在優先度よりも低い場合に，当該タスクの優先
 *			  度が変わらないこと．また，同じ優先度内での優先順位が最低
 *			  になること
 *	(D) ミューテックスのロック処理（loc_mtx）のE_ILUSEエラー
 *		(D-1) 自タスクのベース優先度が，ロックしようとしたミューテック
 *			  スの上限優先度よりも高い時，E_ILUSEエラーになること
 *		(D-2) 自タスクのベース優先度が，ロックしようとしたミューテック
 *			  スの上限優先度と同じかそれより低ければ，自タスクの現在優
 *			  先度がそれより高くても，E_ILUSEエラーにならないこと
 *
 * 【テスト項目の実現方法】
 *
 *	(A-1)
 *		中優先度タスク（TASK1）に，高優先度上限ミューテックス（MTX2）を
 *		ロックさせ，高優先度になることを確認する．また，実行可能状態の
 *		高優先度タスク（TASK4）よりも，優先順位が高くなることを確認する．
 *		ディスパッチ禁止状態で実施する．
 *	(A-2)
 *		中優先度タスク（TASK1）に，中優先度上限ミューテックス（MTX1）を
 *		ロックさせ，優先度が変わらないことを確認する．また，実行可能状
 *		態の中優先度タスクを2つ（TASK2，TASK3）を用意しておき，優先順位
 *		が変わらないことを確認する．ディスパッチ禁止状態で実施する．
 *	(A-3)
 *		中優先度タスク（TASK1）が高優先度上限ミューテックス（MTX2）をロッ
 *		クして高優先度になっている状態で，中優先度上限ミューテックス
 *		（MTX1）をロックさせ，優先度が変わらないことを確認する．また，
 *		実行可能状態の高優先度タスクを2つ（TASK4，TASK5）を用意しておき，
 *		優先順位が変わらないことを確認する．ディスパッチ禁止状態で実施
 *		する．
 *	(B-1)
 *		中優先度タスク（TASK1）に高優先度上限ミューテックス（MTX2）のみ
 *		をロックさせている状態で，それをロック解除させ，中優先度になる
 *		ことを確認する．また，実行可能状態の中優先度タスク（TASK2）より
 *		も，優先順位が高くなることを確認する．
 *	(B-2)
 *		中優先度タスク（TASK1）に中優先度上限ミューテックス（MTX1）のみ
 *		をロックさせている状態で，それをロック解除させ，優先度が変わら
 *		ないことを確認する．また，実行可能状態の中優先度タスクを2つ
 *		（TASK2，TASK3）を用意しておき，優先順位が変わらないことを確認
 *		する．ディスパッチ禁止状態で実施する．
 *	(B-3)
 *		中優先度タスク（TASK1）に高優先度上限ミューテックス（MTX2）と中
 *		優先度上限ミューテックス（MTX1）をロックさせている状態で，中優
 *		先度上限ミューテックス（MTX1）をロック解除させ，優先度が変わら
 *		ないことを確認する．また，実行可能状態の高優先度タスクを2つ
 *		（TASK4，TASK5）を用意しておき，優先順位が変わらないことを確認
 *		する．ディスパッチ禁止状態で実施する．
 *	(C-1)
 *		中優先度タスク（TASK1）に高優先度上限ミューテックスを2つ（MTX2，
 *		MTX3）ロックさせ，別の中優先度タスク（TASK2）がMTX2を待っている
 *		状態で，TASK1にMTX2をロック解除させ，TASK2の優先度が高優先度に
 *		なることを確認する．また，TASK2の優先順位が，TASK1よりも低くな
 *		ることを確認する．
 *	(C-2)
 *		中優先度タスク（TASK2）に中優先度上限ミューテックス（MTX1）をロッ
 *		クさせ，別の中優先度タスク（TASK1）がMTX1を待っている状態で，
 *		TASK2にMTX1をロック解除させ，TASK1の優先度が中優先度のまま変化
 *		しないことを確認する．また，実行可能状態の中優先度タスクをもう
 *		1つ（TASK3）用意しておき，TASK1の優先順位がTASK3よりも低くなる
 *		ことを確認する．
 *	(C-3)
 *		中優先度タスク（TASK1）に中優先度上限ミューテックス（MTX1）と高
 *		優先度上限ミューテックス（MTX3）をロックさせ，別の中優先度タス
 *		ク（TASK2）が別の高優先度上限ミューテックス（MTX2）をロックして
 *		MTX1を待っている状態で，TASK1にMTX1をロック解除させ，TASK2の優
 *		先度が高優先度のまま変化しないことを確認する．また，TASK2の優先
 *		順位が，TASK1よりも低くなることを確認する．
 *	(D-1)
 *		高優先度タスク（TASK4）に，中優先度上限ミューテックス（MTX1）を
 *		ロックさせ，E_ILUSEエラーになることを確認する．
 *	(D-2)
 *		中優先度タスク（TASK1）が高優先度上限ミューテックス（MTX2）をロッ
 *		クして高優先度になっている状態で，中優先度上限ミューテックス
 *		（MTX1）をロックさせ，E_ILUSEエラーになるないことを確認する．
 *
 * 【使用リソース】
 *
 *	TASK1: 中優先度タスク，メインタスク，最初から起動
 *	TASK2: 中優先度タスク
 *	TASK3: 中優先度タスク
 *	TASK4: 高優先度タスク
 *	TASK5: 高優先度タスク
 *	MTX1: ミューテックス（TA_CEILING属性，上限は中優先度）
 *	MTX2: ミューテックス（TA_CEILING属性，上限は高優先度）
 *	MTX3: ミューテックス（TA_CEILING属性，上限は高優先度）
 *
 * 【テストシーケンス】
 *
 *	== TASK1（優先度：中）==
 *		call(set_bit_func(bit_mutex))
 *	1:	dis_dsp()
 *	2:	act_tsk(TASK2)
 *		act_tsk(TASK4)
 *	//		高：TASK4，中：TASK1→TASK2
 *	3:	ploc_mtx(MTX2)								... (A-1)
 *	//		高：TASK1→TASK4，中：TASK2，MTX2：TASK1
 *		get_pri(TSK_SELF, &tskpri)
 *		assert(tskpri == HIGH_PRIORITY)
 *	4:	ena_dsp()
 *	5:	unl_mtx(MTX2)								... (B-1)
 *	//		高：TASK4，中：TASK1→TASK2
 *	== TASK4（優先度：高）==
 *	6:	loc_mtx(MTX1) -> E_ILUSE					... (D-1)
 *	7:	slp_tsk()
 *	//		中：TASK1→TASK2
 *	== TASK1（続き）==
 *	8:	get_pri(TSK_SELF, &tskpri)
 *		assert(tskpri == MID_PRIORITY)
 *
 *	9:	dis_dsp()
 *	10:	rot_rdq(MID_PRIORITY)
 *	//		中：TASK2→TASK1
 *	11:	act_tsk(TASK3)
 *	//		中：TASK2→TASK1→TASK3
 *	12:	ploc_mtx(MTX1)								... (A-2)
 *	//		中：TASK2→TASK1→TASK3，MTX1：TASK1
 *		get_pri(TSK_SELF, &tskpri)
 *		assert(tskpri == MID_PRIORITY)
 *	13:	ena_dsp()
 *	== TASK2（優先度：中）==
 *	14:	slp_tsk()
 *	//		中：TASK1→TASK3，MTX1：TASK1
 *	== TASK1（続き）==
 *	15:	slp_tsk()
 *	//		中：TASK3，MTX1：TASK1
 *	== TASK3（優先度：中）==
 *	16:	wup_tsk(TASK1)
 *	//		中：TASK3→TASK1，MTX1：TASK1
 *	17:	slp_tsk()
 *	//		中：TASK1，MTX1：TASK1
 *
 *	== TASK1（続き）==
 *	18:	wup_tsk(TASK2)
 *	//		中：TASK1→TASK2，MTX1：TASK1
 *	19:	dis_dsp()
 *	20:	rot_rdq(MID_PRIORITY)
 *	//		中：TASK2→TASK1，MTX1：TASK1
 *	21:	wup_tsk(TASK3)
 *	//		中：TASK2→TASK1→TASK3，MTX1：TASK1
 *	22:	unl_mtx(MTX1)								... (B-2)
 *	//		中：TASK2→TASK1→TASK3
 *		get_pri(TSK_SELF, &tskpri)
 *		assert(tskpri == MID_PRIORITY)
 *	23:	ena_dsp()
 *	== TASK2（優先度：中）==
 *	24:	slp_tsk()
 *	//		中：TASK1→TASK3
 *	== TASK1（続き）==
 *	25:	slp_tsk()
 *	//		中：TASK3
 *	== TASK3（優先度：中）==
 *	26:	wup_tsk(TASK1)
 *	//		中：TASK3→TASK1
 *	27:	slp_tsk()
 *	//		中：TASK1
 *
 *	== TASK1（続き）==
 *	28:	dis_dsp()
 *	29:	wup_tsk(TASK4)
 *	//		高：TASK4，中：TASK1
 *	30:	ploc_mtx(MTX2)
 *	//		高：TASK1→TASK4，MTX2：TASK1
 *	31:	rot_rdq(HIGH_PRIORITY)
 *	//		高：TASK4→TASK1，MTX2：TASK1
 *	32:	act_tsk(TASK5)
 *	//		高：TASK4→TASK1→TASK5，MTX2：TASK1
 *	33:	ploc_mtx(MTX1)								... (A-3)(D-2)
 *	//		高：TASK4→TASK1→TASK5，MTX2：TASK1，MTX1：TASK1
 *		get_pri(TSK_SELF, &tskpri)
 *		assert(tskpri == HIGH_PRIORITY)
 *	34:	ena_dsp()
 *	== TASK4（続き）==
 *	35:	slp_tsk()
 *	//		高：TASK1→TASK5，MTX2：TASK1，MTX1：TASK1
 *	== TASK1（続き）==
 *	36:	slp_tsk()
 *	//		高：TASK5，MTX2：TASK1，MTX1：TASK1
 *	== TASK5（優先度：高）==
 *	37:	wup_tsk(TASK1)
 *	//		高：TASK5→TASK1，MTX2：TASK1，MTX1：TASK1
 *	38:	slp_tsk()
 *	//		高：TASK1，MTX2：TASK1，MTX1：TASK1
 *
 *	== TASK1（続き）==
 *	39:	dis_dsp()
 *	40:	wup_tsk(TASK4)
 *	//		高：TASK1→TASK4，MTX2：TASK1，MTX1：TASK1
 *	41:	rot_rdq(HIGH_PRIORITY)
 *	//		高：TASK4→TASK1，MTX2：TASK1，MTX1：TASK1
 *	42:	wup_tsk(TASK5)
 *	//		高：TASK4→TASK1→TASK5，MTX2：TASK1，MTX1：TASK1
 *	43:	unl_mtx(MTX1)								... (B-3)
 *	//		高：TASK4→TASK1→TASK5，MTX2：TASK1
 *		get_pri(TSK_SELF, &tskpri)
 *		assert(tskpri == HIGH_PRIORITY)
 *	44:	ena_dsp()
 *	== TASK4（続き）==
 *	45:	slp_tsk()
 *	//		高：TASK1→TASK5，MTX2：TASK1
 *	== TASK1（続き）==
 *	46:	slp_tsk()
 *	//		高：TASK5，MTX2：TASK1
 *	== TASK5（続き）==
 *	47:	wup_tsk(TASK1)
 *	//		高：TASK5→TASK1，MTX2：TASK1
 *	48:	slp_tsk()
 *	//		高：TASK1，MTX2：TASK1
 *
 *	== TASK1（続き）==
 *	49:	wup_tsk(TASK2)
 *		wup_tsk(TASK3)
 *		slp_tsk()
 *	//		中：TASK2→TASK3，MTX2：TASK1
 *	== TASK2（続き）==
 *	50:	loc_mtx(MTX2)
 *	//		中：TASK3，MTX2：TASK1→TASK2
 *	== TASK3（続き）==
 *	51:	wup_tsk(TASK1)
 *	//		高：TASK1，中：TASK3，MTX2：TASK1→TASK2
 *	== TASK1（続き）==
 *	52:	loc_mtx(MTX3)
 *	//		高：TASK1，中：TASK3，MTX2：TASK1→TASK2，MTX3：TASK1
 *	53:	unl_mtx(MTX2)								... (C-1)
 *	//		高：TASK1→TASK2，中：TASK3，MTX2：TASK2，MTX3：TASK1
 *		get_pri(TASK2, &tskpri)
 *		assert(tskpri == HIGH_PRIORITY)
 *	54:	loc_mtx(MTX1)
 *	//		高：TASK1→TASK2，中：TASK3，MTX1：TASK1，MTX2：TASK2，MTX3：TASK1
 *	55:	slp_tsk()
 *	//		高：TASK2，中：TASK3，MTX1：TASK1，MTX2：TASK2，MTX3：TASK1
 *
 *	== TASK2（続き）==
 *	56:	wup_tsk(TASK1)
 *	//		高：TASK2→TASK1，中：TASK3，MTX1：TASK1，MTX2：TASK2，MTX3：TASK1
 *	57:	loc_mtx(MTX1)
 *	//		高：TASK1，中：TASK3，MTX1：TASK1→TASK2，MTX2：TASK2，MTX3：TASK1
 *	== TASK1（続き）==
 *	58:	unl_mtx(MTX1)								... (C-3)
 *	//		高：TASK1→TASK2，中：TASK3，MTX1：TASK2，MTX2：TASK2，MTX3：TASK1
 *		get_pri(TASK2, &tskpri)
 *		assert(tskpri == HIGH_PRIORITY)
 *	59:	unl_mtx(MTX3)
 *	//		高：TASK2，中：TASK1→TASK3，MTX1：TASK2，MTX2：TASK2
 *	== TASK2（続き）==
 *	60:	unl_mtx(MTX2)
 *	//		中：TASK2→TASK1→TASK3，MTX1：TASK2
 *
 *	61:	rot_rdq(MID_PRIORITY)
 *	//		中：TASK1→TASK3→TASK2，MTX1：TASK2
 *	== TASK1（続き）==
 *	62:	loc_mtx(MTX1)
 *	//		中：TASK3→TASK2，MTX1：TASK2→TASK1
 *	== TASK3（続き）==
 *	63:	rot_rdq(MID_PRIORITY)
 *	//		中：TASK2→TASK3，MTX1：TASK2→TASK1
 *	== TASK2（続き）==
 *	64:	unl_mtx(MTX1)								... (C-2)
 *	//		中：TASK2→TASK3→TASK1，MTX1：TASK1
 *		get_pri(TASK1, &tskpri)
 *		assert(tskpri == MID_PRIORITY)
 *	== TASK2（続き）==
 *	65:	ext_tsk() -> noreturn
 *	//		中：TASK3→TASK1，MTX1：TASK1
 *	== TASK3（続き）==
 *	66:	ext_tsk() -> noreturn
 *	//		中：TASK1，MTX1：TASK1
 *	== TASK1（続き）==
 *	67:	END
 */

#include <kernel.h>
#include <t_syslog.h>
#include "kernel_cfg.h"
#include "test_lib.h"
#include "test_mutex4.h"

extern ER	bit_mutex(void);

/* DO NOT DELETE THIS LINE -- gentest depends on it. */

void
task1(intptr_t exinf)
{
	ER_UINT	ercd;
	PRI		tskpri;

	test_start(__FILE__);

	set_bit_func(bit_mutex);

	check_point(1);
	ercd = dis_dsp();
	check_ercd(ercd, E_OK);

	check_point(2);
	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = act_tsk(TASK4);
	check_ercd(ercd, E_OK);

	check_point(3);
	ercd = ploc_mtx(MTX2);
	check_ercd(ercd, E_OK);

	ercd = get_pri(TSK_SELF, &tskpri);
	check_ercd(ercd, E_OK);

	check_assert(tskpri == HIGH_PRIORITY);

	check_point(4);
	ercd = ena_dsp();
	check_ercd(ercd, E_OK);

	check_point(5);
	ercd = unl_mtx(MTX2);
	check_ercd(ercd, E_OK);

	check_point(8);
	ercd = get_pri(TSK_SELF, &tskpri);
	check_ercd(ercd, E_OK);

	check_assert(tskpri == MID_PRIORITY);

	check_point(9);
	ercd = dis_dsp();
	check_ercd(ercd, E_OK);

	check_point(10);
	ercd = rot_rdq(MID_PRIORITY);
	check_ercd(ercd, E_OK);

	check_point(11);
	ercd = act_tsk(TASK3);
	check_ercd(ercd, E_OK);

	check_point(12);
	ercd = ploc_mtx(MTX1);
	check_ercd(ercd, E_OK);

	ercd = get_pri(TSK_SELF, &tskpri);
	check_ercd(ercd, E_OK);

	check_assert(tskpri == MID_PRIORITY);

	check_point(13);
	ercd = ena_dsp();
	check_ercd(ercd, E_OK);

	check_point(15);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(18);
	ercd = wup_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(19);
	ercd = dis_dsp();
	check_ercd(ercd, E_OK);

	check_point(20);
	ercd = rot_rdq(MID_PRIORITY);
	check_ercd(ercd, E_OK);

	check_point(21);
	ercd = wup_tsk(TASK3);
	check_ercd(ercd, E_OK);

	check_point(22);
	ercd = unl_mtx(MTX1);
	check_ercd(ercd, E_OK);

	ercd = get_pri(TSK_SELF, &tskpri);
	check_ercd(ercd, E_OK);

	check_assert(tskpri == MID_PRIORITY);

	check_point(23);
	ercd = ena_dsp();
	check_ercd(ercd, E_OK);

	check_point(25);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(28);
	ercd = dis_dsp();
	check_ercd(ercd, E_OK);

	check_point(29);
	ercd = wup_tsk(TASK4);
	check_ercd(ercd, E_OK);

	check_point(30);
	ercd = ploc_mtx(MTX2);
	check_ercd(ercd, E_OK);

	check_point(31);
	ercd = rot_rdq(HIGH_PRIORITY);
	check_ercd(ercd, E_OK);

	check_point(32);
	ercd = act_tsk(TASK5);
	check_ercd(ercd, E_OK);

	check_point(33);
	ercd = ploc_mtx(MTX1);
	check_ercd(ercd, E_OK);

	ercd = get_pri(TSK_SELF, &tskpri);
	check_ercd(ercd, E_OK);

	check_assert(tskpri == HIGH_PRIORITY);

	check_point(34);
	ercd = ena_dsp();
	check_ercd(ercd, E_OK);

	check_point(36);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(39);
	ercd = dis_dsp();
	check_ercd(ercd, E_OK);

	check_point(40);
	ercd = wup_tsk(TASK4);
	check_ercd(ercd, E_OK);

	check_point(41);
	ercd = rot_rdq(HIGH_PRIORITY);
	check_ercd(ercd, E_OK);

	check_point(42);
	ercd = wup_tsk(TASK5);
	check_ercd(ercd, E_OK);

	check_point(43);
	ercd = unl_mtx(MTX1);
	check_ercd(ercd, E_OK);

	ercd = get_pri(TSK_SELF, &tskpri);
	check_ercd(ercd, E_OK);

	check_assert(tskpri == HIGH_PRIORITY);

	check_point(44);
	ercd = ena_dsp();
	check_ercd(ercd, E_OK);

	check_point(46);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(49);
	ercd = wup_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = wup_tsk(TASK3);
	check_ercd(ercd, E_OK);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(52);
	ercd = loc_mtx(MTX3);
	check_ercd(ercd, E_OK);

	check_point(53);
	ercd = unl_mtx(MTX2);
	check_ercd(ercd, E_OK);

	ercd = get_pri(TASK2, &tskpri);
	check_ercd(ercd, E_OK);

	check_assert(tskpri == HIGH_PRIORITY);

	check_point(54);
	ercd = loc_mtx(MTX1);
	check_ercd(ercd, E_OK);

	check_point(55);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(58);
	ercd = unl_mtx(MTX1);
	check_ercd(ercd, E_OK);

	ercd = get_pri(TASK2, &tskpri);
	check_ercd(ercd, E_OK);

	check_assert(tskpri == HIGH_PRIORITY);

	check_point(59);
	ercd = unl_mtx(MTX3);
	check_ercd(ercd, E_OK);

	check_point(62);
	ercd = loc_mtx(MTX1);
	check_ercd(ercd, E_OK);

	check_finish(67);
	check_point(0);
}

void
task2(intptr_t exinf)
{
	ER_UINT	ercd;
	PRI		tskpri;

	check_point(14);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(24);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(50);
	ercd = loc_mtx(MTX2);
	check_ercd(ercd, E_OK);

	check_point(56);
	ercd = wup_tsk(TASK1);
	check_ercd(ercd, E_OK);

	check_point(57);
	ercd = loc_mtx(MTX1);
	check_ercd(ercd, E_OK);

	check_point(60);
	ercd = unl_mtx(MTX2);
	check_ercd(ercd, E_OK);

	check_point(61);
	ercd = rot_rdq(MID_PRIORITY);
	check_ercd(ercd, E_OK);

	check_point(64);
	ercd = unl_mtx(MTX1);
	check_ercd(ercd, E_OK);

	ercd = get_pri(TASK1, &tskpri);
	check_ercd(ercd, E_OK);

	check_assert(tskpri == MID_PRIORITY);

	check_point(65);
	ercd = ext_tsk();

	check_point(0);
}

void
task3(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(16);
	ercd = wup_tsk(TASK1);
	check_ercd(ercd, E_OK);

	check_point(17);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(26);
	ercd = wup_tsk(TASK1);
	check_ercd(ercd, E_OK);

	check_point(27);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(51);
	ercd = wup_tsk(TASK1);
	check_ercd(ercd, E_OK);

	check_point(63);
	ercd = rot_rdq(MID_PRIORITY);
	check_ercd(ercd, E_OK);

	check_point(66);
	ercd = ext_tsk();

	check_point(0);
}

void
task4(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(6);
	ercd = loc_mtx(MTX1);
	check_ercd(ercd, E_ILUSE);

	check_point(7);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(35);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(45);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(0);
}

void
task5(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(37);
	ercd = wup_tsk(TASK1);
	check_ercd(ercd, E_OK);

	check_point(38);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(47);
	ercd = wup_tsk(TASK1);
	check_ercd(ercd, E_OK);

	check_point(48);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(0);
}
