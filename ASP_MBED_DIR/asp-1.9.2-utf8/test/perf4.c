/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2006-2009 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: perf4.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		カーネル性能評価プログラム(4)
 *
 *  act_tsk，iact_tskの処理時間とタスク切換え時間を計測するためのプログ
 *  ラム．以下の3つの時間を測定する．
 *
 *  (1) タスク切換えを起こさないact_tskの処理時間．自タスクよりも優先度
 *      の低いタスクに対してact_tskを発行し，休止状態から実行できる状態
 *      に遷移させる処理の時間．
 *
 *  (2) タスク切換えを起こすact_tskの処理時間．自タスクよりも優先度の高
 *      いタスクに対してact_tskを発行し，休止状態から実行できる状態に遷
 *      移させ，タスク切換えを起こして，高い優先度のタスクの実行が始ま
 *      るまでの時間．
 *
 *  (3) タスク切換えを起こすiact_tskの処理時間．周期ハンドラから，実行
 *      状態のタスクよりも高い優先度のタスクに対してact_tskを発行し，休
 *      止状態から実行できる状態に遷移させたあとに周期ハンドラからリター
 *      ンし，タスク切換えを起こして，高い優先度のタスクの実行が始まる
 *      までの時間．
 */

#include <kernel.h>
#include <t_syslog.h>
#include <test_lib.h>
#include <histogram.h>
#include "kernel_cfg.h"
#include "perf4.h"

/*
 *  計測回数と実行時間分布を記録する最大時間
 */
#define NO_MEASURE	10000U			/* 計測回数 */
#define MAX_TIME	1000U			/* 実行時間分布を記録する最大時間 */

/*
 *  実行時間分布を記録するメモリ領域
 */
static uint_t	histarea1[MAX_TIME + 1];
static uint_t	histarea2[MAX_TIME + 1];
static uint_t	histarea3[MAX_TIME + 1];

/*
 *  計測タスク1（高優先度）
 */
void task1(intptr_t exinf)
{
	end_measure(2);
	ext_tsk();
}

/*
 *  計測タスク2とメインタスクの共有変数
 */
volatile uint_t		task2_count;

/*
 *  計測タスク2（高優先度）
 */
void task2(intptr_t exinf)
{
	end_measure(3);
	task2_count++;
	ext_tsk();
}

/*
 *  計測タスク3（低優先度）
 */
void task3(intptr_t exinf)
{
	ext_tsk();
}

/*
 *  計測タスク4（最低優先度）
 */
void task4(intptr_t exinf)
{
	while (true) {
		wup_tsk(MAIN_TASK);
	}
}

/*
 *  周期ハンドラ
 */
void cyclic_handler(intptr_t exinf)
{
	begin_measure(3);
	iact_tsk(TASK2);
}

/*
 *  メインタスク（中優先度）
 */
void main_task(intptr_t exinf)
{
	uint_t	i;

	syslog_0(LOG_NOTICE, "Performance evaluation program (4)");
	init_hist(1, MAX_TIME, histarea1);
	init_hist(2, MAX_TIME, histarea2);
	init_hist(3, MAX_TIME, histarea3);
	syslog_flush();

	/*
	 *  タスク切換えを起こさないact_tskの処理時間の測定
	 */
	for (i = 0; i < NO_MEASURE; i++) {
		begin_measure(1);
		act_tsk(TASK3);
		end_measure(1);
		slp_tsk();
	}

	/*
	 *  タスク切換えを起こすact_tskの処理時間の測定
	 */
	for (i = 0; i < NO_MEASURE; i++) {
		begin_measure(2);
		act_tsk(TASK1);
	}

	/*
	 *  タスク切換えを起こすiact_tskの処理時間の測定（測定回数は10分の1）
	 */
	task2_count = 0;
	sta_cyc(CYC1);
	while (task2_count < NO_MEASURE / 10) ;
	stp_cyc(CYC1);

	syslog_0(LOG_NOTICE, "Execution times of act_tsk without task switch");
	print_hist(1);
	syslog_0(LOG_NOTICE, "Execution times of act_tsk with task switch");
	print_hist(2);
	syslog_0(LOG_NOTICE, "Execution times of iact_tsk with task switch");
	print_hist(3);
	test_finish();
}
