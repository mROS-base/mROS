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
 *  $Id: test_cpuexc3.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/* 
 *		CPU例外処理のテスト(3)
 *
 * 【テストの目的】
 *
 *  非タスクコンテキスト，割込ロック解除，CPUロック解除で発生したCPU例
 *  外におけるシステム状態のテスト．割込み優先度マスク＝TIPM_ENAALL，ディ
 *  スパッチ禁止状態，タスク例外許可状態でテストする．
 *
 * 【テスト項目】
 *
 *  いずれも，非タスクコンテキスト，割込ロック解除，CPUロック解除で発生
 *  したCPU例外において，
 *
 *	(A) CPU例外ハンドラ実行開始時にCPUロックフラグが変化しないこと
 *	(B) CPU例外ハンドラ実行開始時に割込み優先度マスクが変化しないこと
 *		！CPU例外ハンドラ中で割込み優先度マスクを読めないため，テストで
 *		きない．
 *	(C) CPU例外ハンドラ実行開始時にディスパッチ禁止フラグが変化しないこと
 *	(D) CPU例外ハンドラ実行開始時にタスク例外処理禁止フラグが変化しないこと
 *	(E) CPU例外ハンドラリターン時にCPUロックフラグが元に戻ること
 *		！CPU例外ハンドラからリターンできる場合のみテストする．
 *	(F) CPU例外ハンドラリターン時に割込み優先度マスクが元に戻ること
 *		！CPU例外ハンドラからリターンできる場合のみテストする．
 *	(G) CPU例外ハンドラリターン時にディスパッチ禁止フラグが変化しないこと
 *		！CPU例外ハンドラからリターンできる場合のみテストする．
 *	(H) CPU例外ハンドラリターン時にタスク例外処理禁止フラグが変化しないこと
 *		！CPU例外ハンドラからリターンできる場合のみテストする．
 *	(I) xsns_xpnがtrueを返すこと
 *	(J) xsns_dpnがtrueを返すこと
 *
 * 【使用リソース】
 *
 *	TASK1: TA_ACT，中優先度，タスク例外処理ルーチン登録
 *	CPUEXC1: TA_NULL
 *	ALM1: TA_NULL
 *
 * 【テストシーケンス】
 *
 *	== TASK1（中優先度）==
 *	1:	状態のチェック
 *		dis_dsp()
 *		ena_tex()
 *	2:	状態のチェック
 *		sta_alm(ALM1, 1U)
 *		アラームハンドラ1の実行を待つ
 *	== ALM1 ==
 *	3:	状態のチェック
 *		RAISE_CPU_EXCEPTION
 *	== CPUEXC1 ==
 *	4:	状態のチェック				... (A)(C)(D)
 *		xsns_xpn() == true			... (I)
 *		xsns_dpn() == true			... (J)
 *	5:	CPU例外ハンドラからリターンできない場合は，ここで終了
 *	5:	リターン
 *	== ALM1（続き）==
 *	6:	状態のチェック				... (E)(G)(H)
 *		リターン
 *	== TASK1（続き）==
 *	7:	状態のチェック				... (F)
 *	8:	テスト終了
 */

#include <kernel.h>
#include <test_lib.h>
#include <t_syslog.h>
#include "kernel_cfg.h"
#include "test_cpuexc.h"

volatile bool_t	alm1_flag = false;

void
task1(intptr_t exinf)
{
	ER		ercd;

	test_start(__FILE__);

	check_point(1);
	check_state(false, false, TIPM_ENAALL, false, false, true);
	ercd = dis_dsp();
	check_ercd(ercd, E_OK);
	ercd = ena_tex();
	check_ercd(ercd, E_OK);

	check_point(2);
	check_state(false, false, TIPM_ENAALL, true, true, false);
	ercd = sta_alm(ALM1, 1U);
	check_ercd(ercd, E_OK);

	while (!(alm1_flag));

	check_point(7);
	check_state(false, false, TIPM_ENAALL, true, true, false);

	check_finish(8);
}

void
tex_task1(TEXPTN texptn, intptr_t exinf)
{
	check_point(0);
}

void
task2(intptr_t exinf)
{
	check_point(0);
}

void
alarm1_handler(intptr_t exinf)
{
	check_point(3);
	check_state_i(true, false, true, true, false);
	RAISE_CPU_EXCEPTION;

	check_point(6);
	check_state_i(true, false, true, true, false);
	alm1_flag = true;
}

void
cpuexc_handler(void *p_excinf)
{
	check_point(4);
	check_state_i(true, false, true, true, false);
	check_assert(xsns_xpn(p_excinf) == true);
	check_assert(xsns_dpn(p_excinf) == true);

#ifdef CANNOT_RETURN_CPUEXC
	check_finish(5);
#endif /* CANNOT_RETURN_CPUEXC */

	check_point(5);
}
