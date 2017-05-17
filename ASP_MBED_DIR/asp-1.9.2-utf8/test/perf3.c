/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2007-2009 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: perf3.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		カーネル性能評価プログラム(3)
 *
 *  待ち解除するタスクの数により，set_flgの処理時間がどのように変化する
 *  かを計測するためのプログラム．
 */

#include <kernel.h>
#include <t_syslog.h>
#include <test_lib.h>
#include <histogram.h>
#include "kernel_cfg.h"
#include "perf3.h"

/*
 *  計測回数と実行時間分布を記録する最大時間
 */
#define NO_MEASURE	10000U			/* 計測回数 */
#define MAX_TIME	1000U			/* 実行時間分布を記録する最大時間 */

/*
 *  実行時間分布を記録するメモリ領域
 */
static uint_t	histarea1[MAX_TIME + 1];

/*
 *  計測タスクのリスト
 */
static ID task_list[20] = {
	TASK1, TASK2, TASK3, TASK4, TASK5,
	TASK6, TASK7, TASK8, TASK9, TASK10,
	TASK11, TASK12, TASK13, TASK14, TASK15,
	TASK16, TASK17, TASK18, TASK19, TASK20
};

/*
 *  計測タスク（中優先度）
 */
void task(intptr_t exinf)
{
	FLGPTN	flgptn;

	wai_flg(FLG1, 0x01U, TWF_ORW, &flgptn);
}

/*
 *  計測ルーチン
 */
void
perf_eval(uint_t n)
{
	uint_t		i, j;

	init_hist(1, MAX_TIME, histarea1);

	for (i = 0; i < NO_MEASURE; i++) {
		ini_flg(FLG1);
		for (j = 0; j < n; j++) {
			act_tsk(task_list[j]);
		}
		chg_pri(TSK_SELF, MAIN_PRIORITY_LOW);
		/* タスクが待ち状態に入るのを待つ */
		chg_pri(TSK_SELF, TPRI_INI);

		begin_measure(1);
		set_flg(FLG1, 0x01U);
		end_measure(1);

		chg_pri(TSK_SELF, MAIN_PRIORITY_LOW);
		/* タスクが終了するのを待つ */
		chg_pri(TSK_SELF, TPRI_INI);
	}

	syslog_1(LOG_NOTICE, "Execution times of set_flg"
							" when %d tasks are released from waiting.", n);
	print_hist(1);
	syslog_flush();
}

/*
 *  メインタスク（高優先度）
 */
void main_task(intptr_t exinf)
{
	syslog_0(LOG_NOTICE, "Performance evaluation program (3)");
	syslog_flush();

	perf_eval(0);
	perf_eval(1);
	perf_eval(2);
	perf_eval(3);
	perf_eval(4);
	perf_eval(5);
	perf_eval(10);
	perf_eval(20);
	test_finish();
}
