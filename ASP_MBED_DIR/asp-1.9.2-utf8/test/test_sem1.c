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
 *  $Id: test_sem1.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/* 
 *		セマフォ機能のテスト(1)
 *
 * 【テストの目的】
 *
 *  sig_sem，wai_sem，CRE_SEMを網羅的にテストする．
 *  ただし，CRE_SEMのエラーのテストは除く．
 *
 * 【テスト項目】
 *
 *	(A) sig_semの静的エラーのテスト
 *		(A-1) 非タスクコンテキストからの呼出し
 *		(A-2) CPUロック状態からの呼出し
 *		(A-3) semidが不正（小さすぎる）
 *		(A-4) semidが不正（大きすぎる）
 *	(B) sig_semによりセマフォ待ち状態のタスクが待ち解除される
 *		(B-1) 待ち解除されたタスクに切り換わる
 *		(B-2) ディスパッチ保留状態で，切り換わらない
 *		(B-3) 待ち解除されたタスクが強制待ち状態で，切り換わらない
 *		(B-4) 待ち解除されたタスクが優先度が低く，切り換わらない
 *	(C) sig_semによりセマフォの資源数が1増える
 *		(C-1) セマフォの資源数が0から1になる
 *		(C-2) セマフォの資源数が1から2になる
 *	(D) sig_semがE_QOVRエラーとなる
 *		(D-1) セマフォの最大資源数が1の時
 *		(D-2) セマフォの最大資源数が2の時
 *	(E) wai_semの静的エラーのテスト
 *		(E-1) 非タスクコンテキストからの呼出し
 *		(E-2) CPUロック状態からの呼出し
 *		(E-3) ディスパッチ禁止状態からの呼出し
 *		(E-4) 割込み優先度マスク全解除でない状態からの呼出し
 *		(E-5) semidが不正（小さすぎる）
 *		(E-6) semidが不正（大きすぎる）
 *	(F) wai_semによりセマフォの資源数が1減る
 *		(F-1) セマフォの資源数が1から0になる
 *		(F-2) セマフォの資源数が2から1になる
 *	(G) wai_semによりセマフォ待ち状態になる
 *		(G-1) TA_TNULL属性のセマフォで，待っているタスクがなかった場合
 *		(G-2) TA_TNULL属性のセマフォで，待っているタスクがあった場合
 *		(G-3) TA_TPRI属性のセマフォで，待っているタスクがなかった場合
 *		(G-4) TA_TPRI属性のセマフォで，優先度が高いタスクが待っている場合
 *		(G-5) TA_TPRI属性のセマフォで，優先度が同じタスクが待っている場合
 *		(G-6) TA_TPRI属性のセマフォで，優先度が低いタスクが待っている場合
 *	(H) セマフォ待ち状態が強制解除される
 *	(I) セマフォ待ち状態の間にセマフォが初期化される
 *	(J) セマフォの資源数の初期値が正しく設定される
 *		(J-1) セマフォの資源数の初期値が0
 *		(J-2) セマフォの資源数の初期値が1
 *		(J-3) セマフォの資源数の初期値が2
 *
 * 【使用リソース】
 *
 *	TASK1: 中優先度タスク，TA_ACT属性
 *	TASK2: 高優先度タスク
 *	TASK3: 低優先度タスク
 *	TASK4: 中優先度タスク
 *	TASK5: 中優先度タスク
 *	ALM1:  アラームハンドラ
 *  SEM1:  TA_NULL属性，初期資源数1，最大資源数1
 *  SEM2:  TA_NULL属性，初期資源数2，最大資源数2
 *  SEM3:  TA_TPRI属性，初期資源数0，最大資源数1
 *
 * 【テストシーケンス】
 *
 *	== TASK1（優先度：中）==
 *  1:	ref_sem(SEM1, &rsem)
 *		assert(rsem.wtskid == TSK_NONE)
 *		assert(rsem.semcnt == 1)			... (J-2)
 *  	ref_sem(SEM2, &rsem)
 *		assert(rsem.wtskid == TSK_NONE)
 *		assert(rsem.semcnt == 2)			... (J-3)
 *  	ref_sem(SEM3, &rsem)
 *		assert(rsem.wtskid == TSK_NONE)
 *		assert(rsem.semcnt == 0)			... (J-1)
 *	2:	loc_cpu()
 *		sig_sem(SEM1) -> E_CTX				... (A-2)
 *		wai_sem(SEM1) -> E_CTX				... (E-2)
 *		unl_cpu()
 *		dis_dsp()
 *		wai_sem(SEM1) -> E_CTX				... (E-3)
 *		ena_dsp()
 *		chg_ipm(TMAX_INTPRI)
 *		wai_sem(SEM1) -> E_CTX				... (E-4)
 *		chg_ipm(TIPM_ENAALL)
 *		sig_sem(0) -> E_ID					... (A-3)
 *		wai_sem(0) -> E_ID					... (E-5)
 *		sig_sem(TNUM_SEMID+1) -> E_ID		... (A-4)
 *		wai_sem(TNUM_SEMID+1) -> E_ID		... (E-6)
 *	3:	act_tsk(TASK3)
 *	4:	slp_tsk()
 *	== TASK3（優先度：低）==
 *	5:	wai_sem(SEM1)						... (F-1)
 *  6:	ref_sem(SEM1, &rsem)
 *		assert(rsem.wtskid == TSK_NONE)
 *		assert(rsem.semcnt == 0)
 *	7:	sta_alm(ALM1, 10)
 *	8:	wai_sem(SEM1)						... (G-1)
 *	== ALM1 ==
 *	9:	sig_sem(SEM1) -> E_CTX				... (A-1)
 *		wai_sem(SEM1) -> E_CTX				... (E-1)
 *	10:	iwup_tsk(TASK1)
 *	11:	RETURN
 *	== TASK1（続き）==
 *	12:	act_tsk(TASK2)
 *	== TASK2（優先度：高）==
 *	13:	wai_sem(SEM1)						... (G-2)
 *	== TASK1（続き）==
 *  14:	ref_sem(SEM1, &rsem)
 *		assert(rsem.wtskid == TASK3)
 *		assert(rsem.semcnt == 0)
 *	15:	sig_sem(SEM1)						... (B-4)
 *	16:	sig_sem(SEM1)						... (B-1)
 *	== TASK2（続き）==
 *	17:	wai_sem(SEM1)						... (G-1)
 *	== TASK1（続き）==
 *	18: dis_dsp()
 *	19:	sig_sem(SEM1)						... (B-2)
 *	20:	ena_dsp()
 *	== TASK2（続き）==
 *	21:	wai_sem(SEM1)						... (G-1)
 *	== TASK1（続き）==
 *	22: sus_tsk(TASK2)
 *	23:	sig_sem(SEM1)						... (B-3)
 *	24: sig_sem(SEM1)						... (C-1)
 *	25: sig_sem(SEM1) -> E_QOVR				... (D-1)
 *  26:	ref_sem(SEM1, &rsem)
 *		assert(rsem.wtskid == TSK_NONE)
 *		assert(rsem.semcnt == 1)
 *	27:	rsm_tsk(TASK2)
 *	== TASK2（続き）==
 *	28:	wai_sem(SEM2)						... (F-2)
 *  29:	ref_sem(SEM2, &rsem)
 *		assert(rsem.wtskid == TSK_NONE)
 *		assert(rsem.semcnt == 1)
 *	30:	wai_sem(SEM2)						... (F-1)
 *	31: wai_sem(SEM2)						... (G-1)
 *	== TASK1（続き）==
 *	32:	sig_sem(SEM2)						... (B-1)
 *	== TASK2（続き）==
 *	33:	wai_sem(SEM3)						... (G-3)
 *	== TASK1（続き）==
 *	34:	sig_sem(SEM2)						... (C-1)
 *	35:	sig_sem(SEM2)						... (C-2)
 *  36:	ref_sem(SEM2, &rsem)
 *		assert(rsem.wtskid == TSK_NONE)
 *		assert(rsem.semcnt == 2)
 *	37:	sig_sem(SEM2) -> E_QOVR				... (D-2)
 *  38:	ref_sem(SEM2, &rsem)
 *		assert(rsem.wtskid == TSK_NONE)
 *		assert(rsem.semcnt == 2)
 *	39:	tslp_tsk(10) -> E_TMOUT
 *	== TASK3（続き）==
 *	40:	wai_sem(SEM3)						... (G-4)
 *	== TASK1（続き）==
 *	41:	act_tsk(TASK4)
 *	42:	act_tsk(TASK5)
 *	43:	rot_rdq(TPRI_SELF)
 *	== TASK4（優先度：中）==
 *	44:	wai_sem(SEM3)						... (G-6)
 *	== TASK5（優先度：中）==
 *	45:	wai_sem(SEM3)						... (G-5)
 *	== TASK1（続き）==
 *	46:	sig_sem(SEM3)						... (B-1)
 *	== TASK2（続き）==
 *	47:	wai_sem(SEM1)
 *		wai_sem(SEM1) -> E_RLWAI
 *	== TASK1（続き）==
 *	48:	sig_sem(SEM3)						... (B-4)
 *	49:	tslp_tsk(10) -> E_TMOUT
 *	== TASK4（続き）==
 *	50:	ext_tsk() -> noreturn
 *	== TASK1（続き）==
 *	51:	sig_sem(SEM3)						... (B-4)
 *	52:	tslp_tsk(10) -> E_TMOUT
 *	== TASK5（続き）==
 *	53:	ext_tsk() -> noreturn
 *	== TASK1（続き）==
 *	54:	sig_sem(SEM3)						... (B-4)
 *	55:	tslp_tsk(10) -> E_TMOUT
 *	== TASK3（続き）==
 *	56:	ext_tsk() -> noreturn
 *	== TASK1（続き）==
 *	57: rel_wai(TASK2)						... (H)
 *	== TASK2（続き）==
 *	58:	wai_sem(SEM1) -> E_DLT
 *	== TASK1（続き）==
 *	59: ini_sem(SEM1)						... (I)
 *	== TASK2（続き）==
 *	60: ext_tsk() -> noreturn
 *	== TASK1（続き）==
 *	61: END
 */

#include <kernel.h>
#include <test_lib.h>
#include <t_syslog.h>
#include "kernel_cfg.h"
#include "test_sem1.h"

/* DO NOT DELETE THIS LINE -- gentest depends on it. */

void
alarm1_handler(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(9);
	ercd = sig_sem(SEM1);
	check_ercd(ercd, E_CTX);

	ercd = wai_sem(SEM1);
	check_ercd(ercd, E_CTX);

	check_point(10);
	ercd = iwup_tsk(TASK1);
	check_ercd(ercd, E_OK);

	check_point(11);
	return;

	check_point(0);
}

void
task1(intptr_t exinf)
{
	ER_UINT	ercd;
	T_RSEM	rsem;

	test_start(__FILE__);

	check_point(1);
	ercd = ref_sem(SEM1, &rsem);
	check_ercd(ercd, E_OK);

	check_assert(rsem.wtskid == TSK_NONE);

	check_assert(rsem.semcnt == 1);

	ercd = ref_sem(SEM2, &rsem);
	check_ercd(ercd, E_OK);

	check_assert(rsem.wtskid == TSK_NONE);

	check_assert(rsem.semcnt == 2);

	ercd = ref_sem(SEM3, &rsem);
	check_ercd(ercd, E_OK);

	check_assert(rsem.wtskid == TSK_NONE);

	check_assert(rsem.semcnt == 0);

	check_point(2);
	ercd = loc_cpu();
	check_ercd(ercd, E_OK);

	ercd = sig_sem(SEM1);
	check_ercd(ercd, E_CTX);

	ercd = wai_sem(SEM1);
	check_ercd(ercd, E_CTX);

	ercd = unl_cpu();
	check_ercd(ercd, E_OK);

	ercd = dis_dsp();
	check_ercd(ercd, E_OK);

	ercd = wai_sem(SEM1);
	check_ercd(ercd, E_CTX);

	ercd = ena_dsp();
	check_ercd(ercd, E_OK);

	ercd = chg_ipm(TMAX_INTPRI);
	check_ercd(ercd, E_OK);

	ercd = wai_sem(SEM1);
	check_ercd(ercd, E_CTX);

	ercd = chg_ipm(TIPM_ENAALL);
	check_ercd(ercd, E_OK);

	ercd = sig_sem(0);
	check_ercd(ercd, E_ID);

	ercd = wai_sem(0);
	check_ercd(ercd, E_ID);

	ercd = sig_sem(TNUM_SEMID+1);
	check_ercd(ercd, E_ID);

	ercd = wai_sem(TNUM_SEMID+1);
	check_ercd(ercd, E_ID);

	check_point(3);
	ercd = act_tsk(TASK3);
	check_ercd(ercd, E_OK);

	check_point(4);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(12);
	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(14);
	ercd = ref_sem(SEM1, &rsem);
	check_ercd(ercd, E_OK);

	check_assert(rsem.wtskid == TASK3);

	check_assert(rsem.semcnt == 0);

	check_point(15);
	ercd = sig_sem(SEM1);
	check_ercd(ercd, E_OK);

	check_point(16);
	ercd = sig_sem(SEM1);
	check_ercd(ercd, E_OK);

	check_point(18);
	ercd = dis_dsp();
	check_ercd(ercd, E_OK);

	check_point(19);
	ercd = sig_sem(SEM1);
	check_ercd(ercd, E_OK);

	check_point(20);
	ercd = ena_dsp();
	check_ercd(ercd, E_OK);

	check_point(22);
	ercd = sus_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(23);
	ercd = sig_sem(SEM1);
	check_ercd(ercd, E_OK);

	check_point(24);
	ercd = sig_sem(SEM1);
	check_ercd(ercd, E_OK);

	check_point(25);
	ercd = sig_sem(SEM1);
	check_ercd(ercd, E_QOVR);

	check_point(26);
	ercd = ref_sem(SEM1, &rsem);
	check_ercd(ercd, E_OK);

	check_assert(rsem.wtskid == TSK_NONE);

	check_assert(rsem.semcnt == 1);

	check_point(27);
	ercd = rsm_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(32);
	ercd = sig_sem(SEM2);
	check_ercd(ercd, E_OK);

	check_point(34);
	ercd = sig_sem(SEM2);
	check_ercd(ercd, E_OK);

	check_point(35);
	ercd = sig_sem(SEM2);
	check_ercd(ercd, E_OK);

	check_point(36);
	ercd = ref_sem(SEM2, &rsem);
	check_ercd(ercd, E_OK);

	check_assert(rsem.wtskid == TSK_NONE);

	check_assert(rsem.semcnt == 2);

	check_point(37);
	ercd = sig_sem(SEM2);
	check_ercd(ercd, E_QOVR);

	check_point(38);
	ercd = ref_sem(SEM2, &rsem);
	check_ercd(ercd, E_OK);

	check_assert(rsem.wtskid == TSK_NONE);

	check_assert(rsem.semcnt == 2);

	check_point(39);
	ercd = tslp_tsk(10);
	check_ercd(ercd, E_TMOUT);

	check_point(41);
	ercd = act_tsk(TASK4);
	check_ercd(ercd, E_OK);

	check_point(42);
	ercd = act_tsk(TASK5);
	check_ercd(ercd, E_OK);

	check_point(43);
	ercd = rot_rdq(TPRI_SELF);
	check_ercd(ercd, E_OK);

	check_point(46);
	ercd = sig_sem(SEM3);
	check_ercd(ercd, E_OK);

	check_point(48);
	ercd = sig_sem(SEM3);
	check_ercd(ercd, E_OK);

	check_point(49);
	ercd = tslp_tsk(10);
	check_ercd(ercd, E_TMOUT);

	check_point(51);
	ercd = sig_sem(SEM3);
	check_ercd(ercd, E_OK);

	check_point(52);
	ercd = tslp_tsk(10);
	check_ercd(ercd, E_TMOUT);

	check_point(54);
	ercd = sig_sem(SEM3);
	check_ercd(ercd, E_OK);

	check_point(55);
	ercd = tslp_tsk(10);
	check_ercd(ercd, E_TMOUT);

	check_point(57);
	ercd = rel_wai(TASK2);
	check_ercd(ercd, E_OK);

	check_point(59);
	ercd = ini_sem(SEM1);
	check_ercd(ercd, E_OK);

	check_finish(61);
	check_point(0);
}

void
task2(intptr_t exinf)
{
	ER_UINT	ercd;
	T_RSEM	rsem;

	check_point(13);
	ercd = wai_sem(SEM1);
	check_ercd(ercd, E_OK);

	check_point(17);
	ercd = wai_sem(SEM1);
	check_ercd(ercd, E_OK);

	check_point(21);
	ercd = wai_sem(SEM1);
	check_ercd(ercd, E_OK);

	check_point(28);
	ercd = wai_sem(SEM2);
	check_ercd(ercd, E_OK);

	check_point(29);
	ercd = ref_sem(SEM2, &rsem);
	check_ercd(ercd, E_OK);

	check_assert(rsem.wtskid == TSK_NONE);

	check_assert(rsem.semcnt == 1);

	check_point(30);
	ercd = wai_sem(SEM2);
	check_ercd(ercd, E_OK);

	check_point(31);
	ercd = wai_sem(SEM2);
	check_ercd(ercd, E_OK);

	check_point(33);
	ercd = wai_sem(SEM3);
	check_ercd(ercd, E_OK);

	check_point(47);
	ercd = wai_sem(SEM1);
	check_ercd(ercd, E_OK);

	ercd = wai_sem(SEM1);
	check_ercd(ercd, E_RLWAI);

	check_point(58);
	ercd = wai_sem(SEM1);
	check_ercd(ercd, E_DLT);

	check_point(60);
	ercd = ext_tsk();

	check_point(0);
}

void
task3(intptr_t exinf)
{
	ER_UINT	ercd;
	T_RSEM	rsem;

	check_point(5);
	ercd = wai_sem(SEM1);
	check_ercd(ercd, E_OK);

	check_point(6);
	ercd = ref_sem(SEM1, &rsem);
	check_ercd(ercd, E_OK);

	check_assert(rsem.wtskid == TSK_NONE);

	check_assert(rsem.semcnt == 0);

	check_point(7);
	ercd = sta_alm(ALM1, 10);
	check_ercd(ercd, E_OK);

	check_point(8);
	ercd = wai_sem(SEM1);
	check_ercd(ercd, E_OK);

	check_point(40);
	ercd = wai_sem(SEM3);
	check_ercd(ercd, E_OK);

	check_point(56);
	ercd = ext_tsk();

	check_point(0);
}

void
task4(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(44);
	ercd = wai_sem(SEM3);
	check_ercd(ercd, E_OK);

	check_point(50);
	ercd = ext_tsk();

	check_point(0);
}

void
task5(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(45);
	ercd = wai_sem(SEM3);
	check_ercd(ercd, E_OK);

	check_point(53);
	ercd = ext_tsk();

	check_point(0);
}
