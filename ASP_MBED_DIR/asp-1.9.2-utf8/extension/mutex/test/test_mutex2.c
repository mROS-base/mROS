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
 *  $Id: test_mutex2.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/* 
 *		ミューテックスのテスト(2)
 *
 * 【テストの目的】
 *
 *  優先度順ミューテックスを，ロックする処理とロック解除する処理を一通
 *  りテストする．
 *
 * 【テスト項目】
 *
 *	(A) ミューテックスのロック処理（loc_mtx）
 *		(A-1) ロックされていない場合には，すぐにロックできること
 *		(A-2) 多重にロックしようとすると，E_OBJエラーになること
 *		(A-3) ロックされている場合には，優先度順で待ち状態になること
 *	(B) ミューテックスのロック解除処理（unl_mtx）
 *		(B-1) 他タスクがロックしているミューテックスを解放しようとすると
 *		　　　E_OBJエラーになること
 *		(B-2) 待ちタスクがないと，単にロック解除すること
 *		(B-3) 待ちタスクにロックを渡すこと
 *		(B-4) 待ちタスクにロックを渡して，ディスパッチが起こること
 *	(C) ミューテックスのロック処理（ploc_mtx）
 *		(C-1) ロックされている場合には，すぐにE_TMOUTエラーになること
 *	(D) ミューテックスのロック処理（tloc_mtx）
 *		(D-1) ロックされている場合には，タイムアウト付きの待ち状態にな
 *		　　　ること
 *	(E) おまけ
 *		(E-1) タスクを終了すると，ミューテックスをロック解除すること
 *
 * 【使用リソース】
 *
 *	TASK1: 低優先度タスク，メインタスク，最初から起動
 *	TASK2: 中優先度タスク
 *	TASK3: 高優先度タスク
 *	MTX1: ミューテックス（TA_TPRI属性）
 *
 * 【テストシーケンス】
 *
 *	== TASK1（優先度：低）==
 *		call(set_bit_func(bit_mutex))
 *	1:	ref_mtx(MTX1, &rmtx)
 *		assert(rmtx.htskid == TSK_NONE)
 *		assert(rmtx.wtskid == TSK_NONE)
 *		loc_mtx(MTX1)					... (A-1)
 *	2:	loc_mtx(MTX1) -> E_OBJ			... (A-2)
 *	3:	ref_mtx(MTX1, &rmtx)
 *		assert(rmtx.htskid == TASK1)
 *		assert(rmtx.wtskid == TSK_NONE)
 *		act_tsk(TASK2)
 *	== TASK2（優先度：中）==
 *	4:	ploc_mtx(MTX1) -> E_TMOUT		... (C-1)
 *		loc_mtx(MTX1)					... (A-3)
 *	== TASK1（続き）==
 *	5:	ref_mtx(MTX1, &rmtx)
 *		assert(rmtx.htskid == TASK1)
 *		assert(rmtx.wtskid == TASK2)
 *		act_tsk(TASK3)
 *	== TASK3（優先度：高）==
 *	6:	unl_mtx(MTX1) -> E_OBJ			... (B-1)
 *	7:	loc_mtx(MTX1)					... (A-3)
 *	== TASK1（続き）==
 *	8:	ref_mtx(MTX1, &rmtx)
 *		assert(rmtx.htskid == TASK1)
 *		assert(rmtx.wtskid == TASK3)
 *		dis_dsp()
 *		unl_mtx(MTX1)					... (B-3)
 *	9:	ref_mtx(MTX1, &rmtx)
 *		assert(rmtx.htskid == TASK3)
 *		assert(rmtx.wtskid == TASK2)
 *		ena_dsp()
 *	== TASK3（続き）==
 *	10:	ext_tsk() -> noreturn			... (E-1)
 *	== TASK2（続き）==
 *	11:	unl_mtx(MTX1)					... (B-4)
 *	12:	loc_mtx(MTX1)
 *	13:	slp_tsk()
 *	== TASK1（続き）==
 *	14:	ref_mtx(MTX1, &rmtx)
 *		assert(rmtx.htskid == TASK2)
 *		assert(rmtx.wtskid == TSK_NONE)
 *		tloc_mtx(MTX1, 10) -> E_TMOUT	... (D-1)
 *	15:	wup_tsk(TASK2)
 *	== TASK2（続き）==
 *	16:	unl_mtx(MTX1)					... (B-2)
 *	17:	ext_tsk() -> noreturn
 *	== TASK1（続き）==
 *	18:	END
 */

#include <kernel.h>
#include <t_syslog.h>
#include "kernel_cfg.h"
#include "test_lib.h"
#include "test_mutex.h"

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
	ercd = ref_mtx(MTX1, &rmtx);
	check_ercd(ercd, E_OK);

	check_assert(rmtx.htskid == TSK_NONE);

	check_assert(rmtx.wtskid == TSK_NONE);

	ercd = loc_mtx(MTX1);
	check_ercd(ercd, E_OK);

	check_point(2);
	ercd = loc_mtx(MTX1);
	check_ercd(ercd, E_OBJ);

	check_point(3);
	ercd = ref_mtx(MTX1, &rmtx);
	check_ercd(ercd, E_OK);

	check_assert(rmtx.htskid == TASK1);

	check_assert(rmtx.wtskid == TSK_NONE);

	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(5);
	ercd = ref_mtx(MTX1, &rmtx);
	check_ercd(ercd, E_OK);

	check_assert(rmtx.htskid == TASK1);

	check_assert(rmtx.wtskid == TASK2);

	ercd = act_tsk(TASK3);
	check_ercd(ercd, E_OK);

	check_point(8);
	ercd = ref_mtx(MTX1, &rmtx);
	check_ercd(ercd, E_OK);

	check_assert(rmtx.htskid == TASK1);

	check_assert(rmtx.wtskid == TASK3);

	ercd = dis_dsp();
	check_ercd(ercd, E_OK);

	ercd = unl_mtx(MTX1);
	check_ercd(ercd, E_OK);

	check_point(9);
	ercd = ref_mtx(MTX1, &rmtx);
	check_ercd(ercd, E_OK);

	check_assert(rmtx.htskid == TASK3);

	check_assert(rmtx.wtskid == TASK2);

	ercd = ena_dsp();
	check_ercd(ercd, E_OK);

	check_point(14);
	ercd = ref_mtx(MTX1, &rmtx);
	check_ercd(ercd, E_OK);

	check_assert(rmtx.htskid == TASK2);

	check_assert(rmtx.wtskid == TSK_NONE);

	ercd = tloc_mtx(MTX1, 10);
	check_ercd(ercd, E_TMOUT);

	check_point(15);
	ercd = wup_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_finish(18);
	check_point(0);
}

void
task2(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(4);
	ercd = ploc_mtx(MTX1);
	check_ercd(ercd, E_TMOUT);

	ercd = loc_mtx(MTX1);
	check_ercd(ercd, E_OK);

	check_point(11);
	ercd = unl_mtx(MTX1);
	check_ercd(ercd, E_OK);

	check_point(12);
	ercd = loc_mtx(MTX1);
	check_ercd(ercd, E_OK);

	check_point(13);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(16);
	ercd = unl_mtx(MTX1);
	check_ercd(ercd, E_OK);

	check_point(17);
	ercd = ext_tsk();

	check_point(0);
}

void
task3(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(6);
	ercd = unl_mtx(MTX1);
	check_ercd(ercd, E_OBJ);

	check_point(7);
	ercd = loc_mtx(MTX1);
	check_ercd(ercd, E_OK);

	check_point(10);
	ercd = ext_tsk();

	check_point(0);
}
