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
 *  $Id: test_cpuexc6.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/* 
 *		CPU例外処理のテスト(6)
 *
 * 【テストの目的】
 *
 *  割込み優先度マスク＝TIPM_ENAALL，ディスパッチ許可状態，タスク例外禁
 *  止状態で発生したCPU例外におけるシステム状態のテスト．タスク切換えに
 *  よりリカバリーできることもテストする．
 *
 * 【テスト項目】
 *
 *  いずれも，割込み優先度マスク＝TIPM_ENAALL，ディスパッチ許可状態，タ
 *  スク例外禁止状態で発生したCPU例外において，
 *
 *	(A) CPU例外ハンドラ実行開始時にCPUロックフラグが変化しないこと
 *	(B) CPU例外ハンドラ実行開始時に割込み優先度マスクが変化しないこと
 *		！CPU例外ハンドラ中で割込み優先度マスクを読めないため，テストで
 *		きない．
 *	(C) CPU例外ハンドラ実行開始時にディスパッチ禁止フラグが変化しないこと
 *	(D) CPU例外ハンドラ実行開始時にタスク例外処理禁止フラグが変化しないこと
 *	(E) CPU例外ハンドラリターン時にCPUロックフラグが元に戻ること
 *	(F) CPU例外ハンドラリターン時に割込み優先度マスクが元に戻ること
 *	(G) CPU例外ハンドラリターン時にディスパッチ禁止フラグが変化しないこと
 *	(H) CPU例外ハンドラリターン時にタスク例外処理禁止フラグが変化しないこと
 *	(I) xsns_xpnがtrueを返すこと
 *	(J) xsns_dpnがfalseを返すこと
 *	(K) タスク切換えによるリカバリーができること
 *
 * 【使用リソース】
 *
 *	TASK1: TA_ACT，中優先度，タスク例外処理ルーチン登録
 *	TASK2: TA_NULL，高優先度
 *	CPUEXC1: TA_NULL
 *
 * 【テストシーケンス】
 *
 *	== TASK1（中優先度，1回目）==
 *	1:	状態のチェック
 *	2:	RAISE_CPU_EXCEPTION
 *	== CPUEXC1 ==
 *	3:	状態のチェック				... (A),(C),(D)
 *		xsns_xpn() == true			... (I)
 *		xsns_dpn() == false			... (J)
 *  4:	iact_tsk(TASK2)
 *		iloc_cpu()
 *  	リターン
 *	== TASK2（高優先度）==
 *	5:	状態のチェック				... (E),(F),(G),(H)
 *	6:	ter_tsk(TASK1)				... (K)
 *	7:	act_tsk(TASK1)				... (K)
 *	8:	ext_tsk()
 *	== TASK1（中優先度，2回目）==
 *	9:	状態のチェック				... (K)
 *	10:	テスト終了
 */

#include <kernel.h>
#include <test_lib.h>
#include <t_syslog.h>
#include "kernel_cfg.h"
#include "test_cpuexc.h"

int_t	task1_count = 0;

void
task1(intptr_t exinf)
{
	switch (++task1_count) {
	case 1:
		test_start(__FILE__);

		check_point(1);
		check_state(false, false, TIPM_ENAALL, false, false, true);

		check_point(2);
		RAISE_CPU_EXCEPTION;

		check_point(0);
		break;

	case 2:
		check_point(9);
		check_state(false, false, TIPM_ENAALL, false, false, true);

		check_finish(10);
		break;

	default:
		check_point(0);
		break;
	}
}

void
tex_task1(TEXPTN texptn, intptr_t exinf)
{
	check_point(0);
}

void
task2(intptr_t exinf)
{
	ER		ercd;

	check_point(5);
	check_state(false, false, TIPM_ENAALL, false, false, true);

	check_point(6);
	ercd = ter_tsk(TASK1);
	check_ercd(ercd, E_OK);

	check_point(7);
	ercd = act_tsk(TASK1);
	check_ercd(ercd, E_OK);

	check_point(8);
	ercd = ext_tsk();
	check_ercd(ercd, E_OK);

	check_point(0);
}

void
alarm1_handler(intptr_t exinf)
{
	check_point(0);
}

void
cpuexc_handler(void *p_excinf)
{
	ER		ercd;

	check_point(3);
	check_state_i(true, false, false, true, true);
	check_assert(xsns_xpn(p_excinf) == true);
	check_assert(xsns_dpn(p_excinf) == false);

	check_point(4);
	ercd = iact_tsk(TASK2);
	check_ercd(ercd, E_OK);
	ercd = iloc_cpu();
	check_ercd(ercd, E_OK);
}
