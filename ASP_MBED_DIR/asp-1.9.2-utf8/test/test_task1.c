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
 *  $Id: test_task1.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/* 
 *		タスク管理モジュールのテスト(1)
 *
 * 【テストの目的】
 *
 *  make_runnableとmake_non_runnableを網羅的にテストする．
 *
 * 【テスト項目】
 *
 *	(A) 実行状態のタスクよりも高い優先度のタスクを実行できる状態にする
 *		(A-1) ディスパッチ保留状態ではない場合
 *		(A-2) ディスパッチ保留状態の場合
 *	(B) 実行状態のタスクと同じ優先度のタスクを実行できる状態にする
 *		！同じ優先度のタスクの中の末尾に入ることを確認する
 *	(C) 実行状態のタスクよりも低い優先度のタスクを実行できる状態にする
 *		！同じ優先度のタスクの中の末尾に入ることを確認する
 *	(D) 実行できる状態のタスクがない状態で，タスクを実行できる状態にする
 *		！この場合は，ディスパッチ保留状態ではない
 *	(E) ディスパッチ保留状態で，実行状態になるべきだが実行可能状態タス
 *		クよりも高い優先度のタスクを，実行できる状態にする
 *	(F) ディスパッチ保留状態で，実行状態になるべきだが実行可能状態タス
 *		クと同じ優先度のタスクを，実行できる状態にする
 *	(G) ディスパッチ保留状態で，実行状態になるべきだが実行可能状態タス
 *		クよりも低い優先度のタスクを，実行できる状態にする
 *	(H) 実行状態のタスクを，実行できる状態でなくす
 *		(H-1) 実行できる状態のタスクがなくなる場合
 *		(H-2) 同一優先度のタスクがなくなる場合
 *		(H-3) 同一優先度のタスクがなくならない場合
 *	(I) 実行可能状態のタスクを，実行できる状態でなくす
 *		(I-1) 同一優先度のタスクがなくなる場合
 *		(I-2) 同一優先度のタスクがなくならない場合
 *	(J) ディスパッチ保留状態で，実行状態になるべきだが実行可能状態となっ
 *		ているタスクを，実行できる状態でなくす
 *		(J-1) 同一優先度のタスクがなくなる場合
 *		(J-2) 同一優先度のタスクがなくならない場合
 *	(K) ディスパッチ保留状態で，実行可能状態になるべきだが実行状態のタ
 *		スクを，実行できる状態でなくす
 *		！ディスパッチ保留では，実行状態のタスクが実行できる状態でなく
 *		なるような遷移を起こすことができないため，この状況は起こらない．
 *
 * 【使用リソース】
 *
 *	TASK1: 中優先度タスク，メインタスク，最初から起動
 *	TASK2: 高優先度タスク
 *	TASK3: 中優先度タスク
 *	TASK4: 中優先度タスク
 *	TASK5: 低優先度タスク
 *	TASK6: 低優先度タスク
 *	ALM1:  アラームハンドラ
 *
 * 【テストシーケンス】
 *
 *	== TASK1（優先度：中）==
 *	1:	act_tsk(TASK2)					... (A-1)
 *	== TASK2（優先度：高）==
 *	2:	slp_tsk()						... (H-2)
 *	== TASK1（続き）==
 *	3:	act_tsk(TASK3)					... (B)
 *	4:	act_tsk(TASK4)					... (B)
 *	5:	slp_tsk()						... (H-3)
 *	== TASK3（優先度：中）==
 *	6:	slp_tsk()						... (H-3)
 *	== TASK4（優先度：中）==
 *	7:	wup_tsk(TASK1)
 *	8:	slp_tsk()						... (H-3)
 *	== TASK1（続き）==
 *	9:	act_tsk(TASK5)					... (C)
 *	10:	act_tsk(TASK6)					... (C)
 *	11:	sus_tsk(TASK6)					... (I-2)
 *	12:	sus_tsk(TASK5)					... (I-1)
 *	13:	rsm_tsk(TASK5)					... (C)
 *	14:	rsm_tsk(TASK6)					... (C)
 *	15:	slp_tsk()						... (H-2)
 *	== TASK5（優先度：低）==
 *	16:	slp_tsk()						... (H-3)
 *	== TASK6（優先度：低）==
 *	17:	dis_dsp()
 *	18:	wup_tsk(TASK1)					... (A-2)
 *	19:	wup_tsk(TASK2)					... (E)
 *	20:	sus_tsk(TASK2)					... (J-1)
 *	21:	wup_tsk(TASK3)					... (F)
 *	22:	wup_tsk(TASK4)					... (F)
 *	23:	sus_tsk(TASK1)					... (J-2)
 *	24:	ena_dsp()
 *	== TASK3（続き）==
 *	25:	slp_tsk()						... (H-3)
 *	== TASK4（続き）==
 *	26:	slp_tsk()						... (H-2)
 *	== TASK6（続き）==
 *	27:	dis_dsp()
 *	28:	rsm_tsk(TASK1)					... (A-2)
 *	29:	wup_tsk(TASK5)					... (G)
 *	30:	ena_dsp()
 *	== TASK1（続き）==
 *	31:	slp_tsk()						... (H-2)
 *	== TASK6（続き）==
 *	32:	slp_tsk()						... (H-3)
 *	== TASK5（続き）==
 *	33:	sta_alm(ALM1, 10)
 *	34:	slp_tsk()						... (H-1)
 *	== ALM1 ==
 *	35:	iget_tid(&tskid)
 *		assert(tskid == TSK_NONE)
 *	36:	iwup_tsk(TASK1)					... (D)
 *	37:	RETURN
 *	== TASK1（続き）==
 *	38: END
 */

#include <kernel.h>
#include <test_lib.h>
#include <t_syslog.h>
#include "kernel_cfg.h"
#include "test_task1.h"

/* DO NOT DELETE THIS LINE -- gentest depends on it. */

void
alarm1_handler(intptr_t exinf)
{
	ID		tskid;
	ER_UINT	ercd;

	check_point(35);
	ercd = iget_tid(&tskid);
	check_ercd(ercd, E_OK);

	check_assert(tskid == TSK_NONE);

	check_point(36);
	ercd = iwup_tsk(TASK1);
	check_ercd(ercd, E_OK);

	check_point(37);
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

	check_point(3);
	ercd = act_tsk(TASK3);
	check_ercd(ercd, E_OK);

	check_point(4);
	ercd = act_tsk(TASK4);
	check_ercd(ercd, E_OK);

	check_point(5);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(9);
	ercd = act_tsk(TASK5);
	check_ercd(ercd, E_OK);

	check_point(10);
	ercd = act_tsk(TASK6);
	check_ercd(ercd, E_OK);

	check_point(11);
	ercd = sus_tsk(TASK6);
	check_ercd(ercd, E_OK);

	check_point(12);
	ercd = sus_tsk(TASK5);
	check_ercd(ercd, E_OK);

	check_point(13);
	ercd = rsm_tsk(TASK5);
	check_ercd(ercd, E_OK);

	check_point(14);
	ercd = rsm_tsk(TASK6);
	check_ercd(ercd, E_OK);

	check_point(15);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(31);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_finish(38);
	check_point(0);
}

void
task2(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(2);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(0);
}

void
task3(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(6);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(25);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(0);
}

void
task4(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(7);
	ercd = wup_tsk(TASK1);
	check_ercd(ercd, E_OK);

	check_point(8);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(26);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(0);
}

void
task5(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(16);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(33);
	ercd = sta_alm(ALM1, 10);
	check_ercd(ercd, E_OK);

	check_point(34);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(0);
}

void
task6(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(17);
	ercd = dis_dsp();
	check_ercd(ercd, E_OK);

	check_point(18);
	ercd = wup_tsk(TASK1);
	check_ercd(ercd, E_OK);

	check_point(19);
	ercd = wup_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(20);
	ercd = sus_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(21);
	ercd = wup_tsk(TASK3);
	check_ercd(ercd, E_OK);

	check_point(22);
	ercd = wup_tsk(TASK4);
	check_ercd(ercd, E_OK);

	check_point(23);
	ercd = sus_tsk(TASK1);
	check_ercd(ercd, E_OK);

	check_point(24);
	ercd = ena_dsp();
	check_ercd(ercd, E_OK);

	check_point(27);
	ercd = dis_dsp();
	check_ercd(ercd, E_OK);

	check_point(28);
	ercd = rsm_tsk(TASK1);
	check_ercd(ercd, E_OK);

	check_point(29);
	ercd = wup_tsk(TASK5);
	check_ercd(ercd, E_OK);

	check_point(30);
	ercd = ena_dsp();
	check_ercd(ercd, E_OK);

	check_point(32);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(0);
}
