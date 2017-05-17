/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2009-2013 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: test_mutex8.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/* 
 *		ミューテックスのテスト(8)
 *
 * 【テストの目的】
 *
 *  優先度上限ミューテックスに対して，chg_priに伴うミューテックス関連の
 *  優先度変更処理を網羅的にテストする．ただし，change_priorityと
 *  mutex_calc_priorityの内容には踏み込まない．
 *
 * 【テスト項目】
 *
 *	(A) タスクのベース優先度の変更処理（chg_pri）
 *		(A-1) 対象タスクが実行できる状態で，ミューテックスをロックして
 *			  いない場合に，同じ優先度内での優先順位が最低になること
 *		(A-2) 対象タスクが実行できる状態で，優先度上限ミューテックス以
 *			  外のミューテックスのみをロックしている場合に，同じ優先度
 *			  内での優先順位が最低になること
 *		(A-3) 対象タスクが実行できる状態で，優先度上限ミューテックスを
 *			  ロックしている場合に，同じ優先度内での優先順位が変わらな
 *			  いこと
 *		(A-4) 対象タスクが待ち状態で，優先度順の待ち行列につながれてお
 *			  り，ミューテックスをロックしていない場合に，優先度が正し
 *			  く変更され，同じ優先度内での順序が最後になること
 *		(A-5) 対象タスクが待ち状態で，優先度順の待ち行列につながれてお
 *			  り，優先度上限ミューテックス以外のミューテックスのみをロッ
 *			  クしている場合に，優先度が正しく変更され，同じ優先度内で
 *			  の順序が最後になること
 *		(A-6) 対象タスクが待ち状態で，優先度順の待ち行列につながれてお
 *			  り，優先度上限ミューテックスをロックしている場合に，同じ
 *			  優先度内での順序が変わらないこと
 *	(B) タスクのベース優先度の変更処理（chg_pri）のE_ILUSEエラー
 *		(B-1) 対象タスクがロックしているミューテックスの上限優先度より
 *			  も，ベース優先度を高くしようとした時，E_ILUSEエラーになる
 *			  こと
 *		(B-2) 対象タスクが優先度上限ミューテックスをロックしていても，
 *			  ロックしているミューテックスの上限優先度よりも，ベース優
 *			  先度を高くしようとしない場合には，E_ILUSEエラーにならない
 *			  こと
 *		(B-3) 対象タスクがロックを待っているミューテックスの上限優先度
 *			  よりも，ベース優先度を高くしようとした時，E_ILUSEエラーに
 *			  なること
 *		(B-4) 対象タスクが優先度上限ミューテックス待ちであっても，ロッ
 *			  クを待っているミューテックスの上限優先度よりも，ベース優
 *			  先度を高くしようとしない場合には，E_ILUSEエラーにならない
 *			  こと
 *
 * 【テスト項目の実現方法】
 *
 *	(A-1)
 *		中優先度タスク（TASK1）が実行可能状態の時に，高優先度タスク
 *		（TASK5）からTASK1を低優先度にchg_priすると，実行可能状態の他の
 *		低優先度タスク（TASK4）よりも，優先順位が低くなることを確認する．
 *	(A-2)
 *		中優先度タスク（TASK1）が優先度順ミューテックス（MTX3）をロック
 *		し，実行可能状態の時に，高優先度タスク（TASK5）からTASK1を低優
 *		先度にchg_priすると，実行可能状態の他の低優先度タスク（TASK4）
 *		よりも，優先順位が低くなることを確認する．
 *	(A-3)
 *		中優先度タスク（TASK1）が中優先度上限ミューテックス（MTX1）をロッ
 *		クし，実行可能状態の時に，高優先度タスク（TASK5）からTASK1を低
 *		優先度にchg_priすると，TASK1の優先度が変化しないことを確認する．
 *		また，実行可能状態の中優先度タスクを2つ（TASK2，TASK3）を用意し
 *		ておき，優先順位が変わらないことを確認する．
 *	(A-4)
 *		中優先度タスク（TASK1）が高優先度上限ミューテックス（MTX2）を待っ
 *		ている時に，高優先度タスク（TASK5）からTASK1を低優先度に
 *		chg_priすると，MTX2を待っている他の低優先度タスク（TASK4）より
 *		も，待ち行列中での順序が後になることを確認する．
 *	(A-5)
 *		中優先度タスク（TASK1）が優先度順ミューテックス（MTX3）をロック
 *		し，高優先度上限ミューテックス（MTX2）を待っている時に，高優先度タ
 *		スク（TASK5）からTASK1を低優先度にchg_priすると，MTX2を待ってい
 *		る他の低優先度タスク（TASK4）よりも，待ち行列中での順序が変わら
 *		ないことを確認する．
 *	(A-6)
 *		中優先度タスク（TASK1）が中優先度上限ミューテックス（MTX1）をロッ
 *		クし，高優先度上限ミューテックス（MTX2）を待っている時に，高優
 *		先度タスク（TASK5）からTASK1を低優先度にchg_priすると，TASK1の
 *		優先度が変化しないことを確認する．また，MTX2を待っている中優先
 *		度タスクを2つ（TASK2，TASK3）を用意しておき，待ち行列中での順序
 *		が変わらないことを確認する．
 *	(B-1)
 *		中優先度タスク（TASK1）が中優先度上限ミューテックス（MTX1）をロッ
 *		クし，実行可能状態の時に，高優先度タスク（TASK5）からTASK1を高
 *		優先度にchg_priすると，E_ILUSEエラーになることを確認する．
 *	(B-2)
 *		(A-3)のテスト項目で，chg_priがE_ILUSEエラーにならないことで確認
 *		できている．
 *	(B-3)
 *		中優先度タスク（TASK2）が中優先度上限ミューテックス（MTX1）のロッ
 *		クを待っている時に，高優先度タスク（TASK5）からTASK2を高優先度
 *		にchg_priすると，E_ILUSEエラーになることを確認する．
 *	(B-4)
 *		(A-6)のテスト項目で，chg_priがE_ILUSEエラーにならないことで確認
 *		できている．
 *
 * 【使用リソース】
 *
 *	TASK1: 中優先度タスク，メインタスク，最初から起動
 *	TASK2: 中優先度タスク
 *	TASK3: 中優先度タスク
 *	TASK4: 低優先度タスク
 *	TASK5: 高優先度タスク
 *	MTX1: ミューテックス（TA_CEILING属性，上限は中優先度）
 *	MTX2: ミューテックス（TA_CEILING属性，上限は高優先度）
 *	MTX3: ミューテックス（TA_TPRI属性）
 *
 * 【テストシーケンス】
 *
 *	== TASK1（優先度：中）==
 *		call(set_bit_func(bit_mutex))
 *	1:	act_tsk(TASK4)
 *	2:	act_tsk(TASK5)
 *	//		高：TASK5，中：TASK1，低：TASK4
 *	== TASK5（優先度：高）==
 *	3:	chg_pri(TASK1, LOW_PRIORITY)		... (A-1)
 *	//		高：TASK5，低：TASK4→TASK1
 *	4:	slp_tsk()
 *	== TASK4（優先度：低）==
 *	5:	slp_tsk()
 *	== TASK1（続き）==
 *	6:	chg_pri(TSK_SELF, TPRI_INI)
 *
 *	7:	loc_mtx(MTX3)
 *	8:	wup_tsk(TASK4)
 *	9:	wup_tsk(TASK5)
 *	//		高：TASK5，中：TASK1，低：TASK4，MTX3：TASK1
 *	== TASK5（続き）==
 *	10:	chg_pri(TASK1, LOW_PRIORITY)		... (A-2)
 *	//		高：TASK5，低：TASK4→TASK1，MTX3：TASK1
 *	11:	slp_tsk()
 *	== TASK4（続き）==
 *	12:	slp_tsk()
 *	== TASK1（続き）==
 *	13:	unl_mtx(MTX3)
 *		chg_pri(TSK_SELF, TPRI_INI)
 *
 *	14:	loc_mtx(MTX1)
 *	15:	act_tsk(TASK2)
 *	16:	dis_dsp()
 *	17:	rot_rdq(MID_PRIORITY)
 *	18:	act_tsk(TASK3)
 *	//		中：TASK2→TASK1→TASK3，MTX1：TASK1
 *	19:	wup_tsk(TASK5)
 *	//		高：TASK5，中：TASK2→TASK1→TASK3，MTX1：TASK1
 *	20:	ena_dsp()
 *	== TASK5（続き）==
 *	21:	chg_pri(TASK1, LOW_PRIORITY)		... (A-3)(B-2)
 *	//		高：TASK5，中：TASK2→TASK1→TASK3，MTX1：TASK1
 *		get_pri(TASK1, &tskpri)
 *		assert(tskpri == MID_PRIORITY)
 *	22:	slp_tsk()
 *	//		中：TASK2→TASK1→TASK3，MTX1：TASK1
 *	== TASK2（続き）==
 *	23:	slp_tsk()
 *	//		中：TASK1→TASK3，MTX1：TASK1
 *	== TASK1（続き）==
 *	24:	unl_mtx(MTX1)
 *	//		中：TASK3，低：TASK1
 *	== TASK3（続き）==
 *	25:	slp_tsk()
 *	== TASK1（続き）==
 *	26:	chg_pri(TSK_SELF, TPRI_INI)
 *
 *	27:	wup_tsk(TASK5)
 *	== TASK5（続き）==
 *	28:	loc_mtx(MTX2)
 *	29:	tslp_tsk(10) -> E_TMOUT
 *	== TASK1（続き）==
 *	30:	wup_tsk(TASK4)
 *	31:	loc_mtx(MTX2)
 *	== TASK4（続き）==
 *	32:	loc_mtx(MTX2)
 *	//		MTX2：TASK5→TASK1→TASK4
 *	//		タイムアウト待ち
 *	//		高：TASK5，MTX2：TASK5→TASK1→TASK4
 *	== TASK5（続き）==
 *	33:	chg_pri(TASK1, LOW_PRIORITY)		... (A-4)
 *	//		高：TASK5，MTX2：TASK5→TASK4→TASK1
 *	34:	unl_mtx(MTX2)
 *	//		高：TASK5→TASK4，MTX2：TASK4→TASK1
 *	35:	slp_tsk()
 *	//		高：TASK4，MTX2：TASK4→TASK1
 *	== TASK4（続き）==
 *	36:	unl_mtx(MTX2)
 *	//		高：TASK1，低：TASK4，MTX2：TASK1
 *	== TASK1（続き）==
 *	37:	unl_mtx(MTX2)
 *	//		低：TASK1→TASK4
 *	38:	chg_pri(TSK_SELF, TPRI_INI)
 *
 *	39:	loc_mtx(MTX3)
 *	40:	wup_tsk(TASK5)
 *	//		高：TASK5，中：TASK1，低：TASK4，MTX3：TASK1
 *	== TASK5（続き）==
 *	41:	loc_mtx(MTX2)
 *	42:	tslp_tsk(10) -> E_TMOUT
 *	== TASK1（続き）==
 *	43:	loc_mtx(MTX2)
 *	== TASK4（続き）==
 *	44:	loc_mtx(MTX2)
 *	//		MTX2：TASK5→TASK1→TASK4，MTX3：TASK1
 *	//		タイムアウト待ち
 *	//		高：TASK5，MTX2：TASK5→TASK1→TASK4，MTX3：TASK1
 *	== TASK5（続き）==
 *	45:	chg_pri(TASK1, LOW_PRIORITY)		... (A-5)
 *	//		高：TASK5，MTX2：TASK5→TASK4→TASK1，MTX3：TASK1
 *	46:	unl_mtx(MTX2)
 *	//		高：TASK5→TASK4，MTX2：TASK4→TASK1，MTX3：TASK1
 *	47:	slp_tsk()
 *	//		高：TASK4，MTX2：TASK4→TASK1，MTX3：TASK1
 *	== TASK4（続き）==
 *	48:	unl_mtx(MTX2)
 *	//		高：TASK1，低：TASK4，MTX2：TASK1，MTX3：TASK1
 *	== TASK1（続き）==
 *	49:	unl_mtx(MTX2)
 *	//		低：TASK1→TASK4，MTX3：TASK1
 *	50:	unl_mtx(MTX3)
 *	//		低：TASK1→TASK4
 *	51:	ter_tsk(TASK4)
 *	52:	chg_pri(TSK_SELF, TPRI_INI)
 *
 *	53:	loc_mtx(MTX1)
 *	54:	wup_tsk(TASK5)
 *	//		高：TASK5，中：TASK1，MTX1：TASK1
 *	== TASK5（続き）==
 *	55:	loc_mtx(MTX2)
 *	56:	tslp_tsk(10) -> E_TMOUT
 *	//		中：TASK1，MTX1：TASK1，MTX2：TASK5
 *	== TASK1（続き）==
 *	57:	wup_tsk(TASK2)
 *	58:	rot_rdq(MID_PRIORITY)
 *	== TASK2（続き）==
 *	59:	loc_mtx(MTX2)
 *	//		中：TASK1，MTX1：TASK1，MTX2：TASK5→TASK2
 *	== TASK1（続き）==
 *	60:	wup_tsk(TASK3)
 *	61:	loc_mtx(MTX2)
 *	//		中：TASK3，MTX1：TASK1，MTX2：TASK5→TASK2→TASK1
 *	== TASK3（続き）==
 *	62:	loc_mtx(MTX2)
 *	//		MTX1：TASK1，MTX2：TASK5→TASK2→TASK1→TASK3
 *	//		タイムアウト待ち
 *	//		高：TASK5，MTX1：TASK1，MTX2：TASK5→TASK2→TASK1→TASK3
 *	== TASK5（続き）==
 *	63:	chg_pri(TASK1, LOW_PRIORITY)		... (A-6)(B-4)
 *	//		高：TASK5，MTX1：TASK1，MTX2：TASK5→TASK2→TASK1→TASK3
 *		get_pri(TASK1, &tskpri)
 *		assert(tskpri == MID_PRIORITY)
 *	64:	unl_mtx(MTX2)
 *	//		高：TASK5→TASK2，MTX1：TASK1，MTX2：TASK2→TASK1→TASK3
 *	65:	slp_tsk()
 *	//		高：TASK2，MTX1：TASK1，MTX2：TASK2→TASK1→TASK3
 *	== TASK2（続き）==
 *	66:	unl_mtx(MTX2)
 *	//		高：TASK1，中：TASK2，MTX1：TASK1，MTX2：TASK1→TASK3
 *	== TASK1（続き）==
 *	67:	unl_mtx(MTX2)
 *	//		高：TASK3，中：TASK1→TASK2，MTX1：TASK1，MTX2：TASK3
 *	== TASK3（続き）==
 *	68:	unl_mtx(MTX2)
 *	//		中：TASK3→TASK1→TASK2，MTX1：TASK1
 *	69:	ext_tsk() -> noreturn
 *	//		中：TASK1→TASK2，MTX1：TASK1
 *	== TASK1（続き）==
 *	70:	chg_pri(TSK_SELF, TPRI_INI)
 *	//		中：TASK1→TASK2，MTX1：TASK1
 *
 *	71:	rot_rdq(MID_PRIORITY)
 *	//		中：TASK2→TASK1，MTX1：TASK1
 *	== TASK2（続き）==
 *	72:	loc_mtx(MTX1)
 *	//		中：TASK1，MTX1：TASK1→TASK2
 *	== TASK1（続き）==
 *	73:	wup_tsk(TASK5)
 *	//		高：TASK5，中：TASK1，MTX1：TASK1→TASK2
 *	== TASK5（続き）==
 *	74:	chg_pri(TASK1, HIGH_PRIORITY) -> E_ILUSE	... (B-1)
 *	75:	chg_pri(TASK2, HIGH_PRIORITY) -> E_ILUSE	... (B-3)
 *	76:	ext_tsk() -> noreturn
 *	//		中：TASK1，MTX1：TASK1→TASK2
 *	== TASK1（続き）==
 *	77:	unl_mtx(MTX1)
 *	//		中：TASK1→TASK2，MTX1：TASK2
 *	78:	ext_tsk() -> noreturn
 *	== TASK2（続き）==
 *	79:	unl_mtx(MTX1)
 *	80:	END
 */

#include <kernel.h>
#include <t_syslog.h>
#include "kernel_cfg.h"
#include "test_lib.h"
#include "test_mutex8.h"

extern ER	bit_mutex(void);

/* DO NOT DELETE THIS LINE -- gentest depends on it. */

void
task1(intptr_t exinf)
{
	ER_UINT	ercd;

	test_start(__FILE__);

	set_bit_func(bit_mutex);

	check_point(1);
	ercd = act_tsk(TASK4);
	check_ercd(ercd, E_OK);

	check_point(2);
	ercd = act_tsk(TASK5);
	check_ercd(ercd, E_OK);

	check_point(6);
	ercd = chg_pri(TSK_SELF, TPRI_INI);
	check_ercd(ercd, E_OK);

	check_point(7);
	ercd = loc_mtx(MTX3);
	check_ercd(ercd, E_OK);

	check_point(8);
	ercd = wup_tsk(TASK4);
	check_ercd(ercd, E_OK);

	check_point(9);
	ercd = wup_tsk(TASK5);
	check_ercd(ercd, E_OK);

	check_point(13);
	ercd = unl_mtx(MTX3);
	check_ercd(ercd, E_OK);

	ercd = chg_pri(TSK_SELF, TPRI_INI);
	check_ercd(ercd, E_OK);

	check_point(14);
	ercd = loc_mtx(MTX1);
	check_ercd(ercd, E_OK);

	check_point(15);
	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(16);
	ercd = dis_dsp();
	check_ercd(ercd, E_OK);

	check_point(17);
	ercd = rot_rdq(MID_PRIORITY);
	check_ercd(ercd, E_OK);

	check_point(18);
	ercd = act_tsk(TASK3);
	check_ercd(ercd, E_OK);

	check_point(19);
	ercd = wup_tsk(TASK5);
	check_ercd(ercd, E_OK);

	check_point(20);
	ercd = ena_dsp();
	check_ercd(ercd, E_OK);

	check_point(24);
	ercd = unl_mtx(MTX1);
	check_ercd(ercd, E_OK);

	check_point(26);
	ercd = chg_pri(TSK_SELF, TPRI_INI);
	check_ercd(ercd, E_OK);

	check_point(27);
	ercd = wup_tsk(TASK5);
	check_ercd(ercd, E_OK);

	check_point(30);
	ercd = wup_tsk(TASK4);
	check_ercd(ercd, E_OK);

	check_point(31);
	ercd = loc_mtx(MTX2);
	check_ercd(ercd, E_OK);

	check_point(37);
	ercd = unl_mtx(MTX2);
	check_ercd(ercd, E_OK);

	check_point(38);
	ercd = chg_pri(TSK_SELF, TPRI_INI);
	check_ercd(ercd, E_OK);

	check_point(39);
	ercd = loc_mtx(MTX3);
	check_ercd(ercd, E_OK);

	check_point(40);
	ercd = wup_tsk(TASK5);
	check_ercd(ercd, E_OK);

	check_point(43);
	ercd = loc_mtx(MTX2);
	check_ercd(ercd, E_OK);

	check_point(49);
	ercd = unl_mtx(MTX2);
	check_ercd(ercd, E_OK);

	check_point(50);
	ercd = unl_mtx(MTX3);
	check_ercd(ercd, E_OK);

	check_point(51);
	ercd = ter_tsk(TASK4);
	check_ercd(ercd, E_OK);

	check_point(52);
	ercd = chg_pri(TSK_SELF, TPRI_INI);
	check_ercd(ercd, E_OK);

	check_point(53);
	ercd = loc_mtx(MTX1);
	check_ercd(ercd, E_OK);

	check_point(54);
	ercd = wup_tsk(TASK5);
	check_ercd(ercd, E_OK);

	check_point(57);
	ercd = wup_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(58);
	ercd = rot_rdq(MID_PRIORITY);
	check_ercd(ercd, E_OK);

	check_point(60);
	ercd = wup_tsk(TASK3);
	check_ercd(ercd, E_OK);

	check_point(61);
	ercd = loc_mtx(MTX2);
	check_ercd(ercd, E_OK);

	check_point(67);
	ercd = unl_mtx(MTX2);
	check_ercd(ercd, E_OK);

	check_point(70);
	ercd = chg_pri(TSK_SELF, TPRI_INI);
	check_ercd(ercd, E_OK);

	check_point(71);
	ercd = rot_rdq(MID_PRIORITY);
	check_ercd(ercd, E_OK);

	check_point(73);
	ercd = wup_tsk(TASK5);
	check_ercd(ercd, E_OK);

	check_point(77);
	ercd = unl_mtx(MTX1);
	check_ercd(ercd, E_OK);

	check_point(78);
	ercd = ext_tsk();

	check_point(0);
}

void
task2(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(23);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(59);
	ercd = loc_mtx(MTX2);
	check_ercd(ercd, E_OK);

	check_point(66);
	ercd = unl_mtx(MTX2);
	check_ercd(ercd, E_OK);

	check_point(72);
	ercd = loc_mtx(MTX1);
	check_ercd(ercd, E_OK);

	check_point(79);
	ercd = unl_mtx(MTX1);
	check_ercd(ercd, E_OK);

	check_finish(80);
	check_point(0);
}

void
task3(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(25);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(62);
	ercd = loc_mtx(MTX2);
	check_ercd(ercd, E_OK);

	check_point(68);
	ercd = unl_mtx(MTX2);
	check_ercd(ercd, E_OK);

	check_point(69);
	ercd = ext_tsk();

	check_point(0);
}

void
task4(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(5);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(12);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(32);
	ercd = loc_mtx(MTX2);
	check_ercd(ercd, E_OK);

	check_point(36);
	ercd = unl_mtx(MTX2);
	check_ercd(ercd, E_OK);

	check_point(44);
	ercd = loc_mtx(MTX2);
	check_ercd(ercd, E_OK);

	check_point(48);
	ercd = unl_mtx(MTX2);
	check_ercd(ercd, E_OK);

	check_point(0);
}

void
task5(intptr_t exinf)
{
	ER_UINT	ercd;
	PRI		tskpri;

	check_point(3);
	ercd = chg_pri(TASK1, LOW_PRIORITY);
	check_ercd(ercd, E_OK);

	check_point(4);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(10);
	ercd = chg_pri(TASK1, LOW_PRIORITY);
	check_ercd(ercd, E_OK);

	check_point(11);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(21);
	ercd = chg_pri(TASK1, LOW_PRIORITY);
	check_ercd(ercd, E_OK);

	ercd = get_pri(TASK1, &tskpri);
	check_ercd(ercd, E_OK);

	check_assert(tskpri == MID_PRIORITY);

	check_point(22);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(28);
	ercd = loc_mtx(MTX2);
	check_ercd(ercd, E_OK);

	check_point(29);
	ercd = tslp_tsk(10);
	check_ercd(ercd, E_TMOUT);

	check_point(33);
	ercd = chg_pri(TASK1, LOW_PRIORITY);
	check_ercd(ercd, E_OK);

	check_point(34);
	ercd = unl_mtx(MTX2);
	check_ercd(ercd, E_OK);

	check_point(35);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(41);
	ercd = loc_mtx(MTX2);
	check_ercd(ercd, E_OK);

	check_point(42);
	ercd = tslp_tsk(10);
	check_ercd(ercd, E_TMOUT);

	check_point(45);
	ercd = chg_pri(TASK1, LOW_PRIORITY);
	check_ercd(ercd, E_OK);

	check_point(46);
	ercd = unl_mtx(MTX2);
	check_ercd(ercd, E_OK);

	check_point(47);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(55);
	ercd = loc_mtx(MTX2);
	check_ercd(ercd, E_OK);

	check_point(56);
	ercd = tslp_tsk(10);
	check_ercd(ercd, E_TMOUT);

	check_point(63);
	ercd = chg_pri(TASK1, LOW_PRIORITY);
	check_ercd(ercd, E_OK);

	ercd = get_pri(TASK1, &tskpri);
	check_ercd(ercd, E_OK);

	check_assert(tskpri == MID_PRIORITY);

	check_point(64);
	ercd = unl_mtx(MTX2);
	check_ercd(ercd, E_OK);

	check_point(65);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(74);
	ercd = chg_pri(TASK1, HIGH_PRIORITY);
	check_ercd(ercd, E_ILUSE);

	check_point(75);
	ercd = chg_pri(TASK2, HIGH_PRIORITY);
	check_ercd(ercd, E_ILUSE);

	check_point(76);
	ercd = ext_tsk();

	check_point(0);
}
