/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2014 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: test_messagebuf3.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/* 
 *		メッセージバッファのテスト(3)
 *
 * 【テストの目的】
 *
 *  タスク優先度順メッセージバッファの，FIFO順メッセージバッファと異な
 *  る振舞いをテストする．
 *
 * 【テスト項目】
 *
 *	(A) 送信待ちキューがタスク優先度であること
 *		(A-1) 後から到着した高優先度タスクが前につながれること
 *		(A-2) 同じ優先度のタスクはFIFO順であること
 *	(B) 送信待ちキューにタスクがあるために送信待ち状態になる条件の違い
 *		(B-1) 送信待ちキューが空の場合
 *		(B-2) 送信待ちキューに低い優先度のタスクのみがある場合
 *		(B-3) 送信待ちキューに同じか高い優先度のタスクがある場合
 *	(C) 送信待ちキューの先頭タスクの優先度を下げて先頭タスクが変わる場合
 *		(C-1) 送信待ちタスクの待ち解除なし
 *		(C-2) 送信待ちタスク（1つまたは複数）が待ち解除（タスク切換えなし）
 *		(C-3) 送信待ちタスク（1つまたは複数）が待ち解除（タスク切換えあり）
 *	(D) 送信待ちキューの途中タスクの優先度を上げて先頭タスクが変わる場合
 *		(D-1) 送信待ちタスクの待ち解除なし
 *		(D-2) 送信待ちタスク（1つまたは複数）が待ち解除（タスク切換えなし）
 *		(D-3) 送信待ちタスク（1つまたは複数）が待ち解除（タスク切換えあり）
 *
 * 【使用リソース】
 *
 *	TASK1: 高優先度タスク，メインタスク，最初から起動
 *	TASK2: 中優先度タスク
 *	TASK3: 中優先度タスク
 *	TASK4: 低優先度タスク
 *	MBF1: メッセージバッファ（TA_TPRI属性，最大メッセージサイズ：26，メッ
 *		  セージバッファ管理領域のサイズ：26→実際には28）
 *
 * 【テストシーケンス】
 *
 *	== TASK1（優先度：高）==
 *		call(set_bit_func(bit_kernel))
 *	1:	act_tsk(TASK2)
 *		act_tsk(TASK4)
 *		slp_tsk()
 *	== TASK2（優先度：中）==
 *	2:	snd_mbf(MBF1, string1, 26)
 *	== TASK4（優先度：低）==
 *	3:	wup_tsk(TASK1)
 *	== TASK1（続き）==
 *	4:	snd_mbf(MBF1, string2, 25)						... (A-1)
 *	== TASK4（続き）==
 *	5:	ref_mbf(MBF1, &rmbf)
 *		assert(rmbf.stskid == TASK1)
 *		assert(rmbf.rtskid == TSK_NONE)
 *		assert(rmbf.smbfcnt == 0)
 *		rcv_mbf(MBF1, buf1) -> 25
 *	== TASK1（続き）==
 *	6:	assert(strncmp(buf1, string2, 25) == 0)
 *		act_tsk(TASK3)
 *		slp_tsk()
 *	== TASK3（優先度：中）==
 *	7:	snd_mbf(MBF1, string2, 26)						... (A-2)
 *	== TASK4（続き）==
 *	8:	rcv_mbf(MBF1, buf1) -> 26
 *	== TASK2（続き）==
 *	9:	assert(strncmp(buf1, string1, 26) == 0)
 *		rcv_mbf(MBF1, buf1) -> 26
 *		assert(strncmp(buf1, string2, 26) == 0)
 *	10:	snd_mbf(MBF1, string3, 10)						... (B-1)
 *		rcv_mbf(MBF1, buf1) -> 10
 *		assert(strncmp(buf1, string3, 10) == 0)
 *	11:	snd_mbf(MBF1, string1, 26)
 *	== TASK3（続き）==
 *	12:	wup_tsk(TASK1)
 *	== TASK1（続き）==
 *	13:	snd_mbf(MBF1, string2, 10)						... (B-2)
 *		rcv_mbf(MBF1, buf1) -> 10
 *		assert(strncmp(buf1, string2, 10) == 0)
 *		slp_tsk()
 *	== TASK3（続き）==
 *	14:	snd_mbf(MBF1, string3, 10)						... (B-3)
 *	== TASK4（続き）==
 *	15:	rcv_mbf(MBF1, buf1) -> 26
 *	== TASK2（続き）==
 *	16:	assert(strncmp(buf1, string1, 26) == 0)
 *		rcv_mbf(MBF1, buf1) -> 10
 *		assert(strncmp(buf1, string3, 10) == 0)
 *	17:	snd_mbf(MBF1, string1, 26)
 *	== TASK3（続き）==
 *	18:	snd_mbf(MBF1, string2, 26)
 *	== TASK4（続き）==
 *	19:	chg_pri(TASK2, LOW_PRIORITY)					... (C-1)
 *		rcv_mbf(MBF1, buf1) -> 26
 *	== TASK3（続き）==
 *	20:	assert(strncmp(buf1, string2, 26) == 0)
 *		rcv_mbf(MBF1, buf1) -> 26
 *		assert(strncmp(buf1, string1, 26) == 0)
 *		chg_pri(TASK2, TPRI_INI)						... 元に戻す
 *	21:	snd_mbf(MBF1, string1, 26)
 *	== TASK2（続き）==
 *	22:	snd_mbf(MBF1, string2, 10)
 *	== TASK4（続き）==
 *	23:	wup_tsk(TASK1)
 *	== TASK1（続き）==
 *	24:	chg_pri(TASK3, LOW_PRIORITY)					... (C-2)
 *		slp_tsk()
 *	== TASK2（続き）==
 *	25:	rcv_mbf(MBF1, buf1) -> 10
 *		assert(strncmp(buf1, string2, 10) == 0)
 *		chg_pri(TASK3, TPRI_INI)						... 元に戻す
 *	26:	snd_mbf(MBF1, string3, 10)
 *	== TASK4（続き）==
 *	27:	chg_pri(TASK3, LOW_PRIORITY)					... (C-3)
 *	== TASK2（続き）==
 *	28:	rcv_mbf(MBF1, buf1) -> 10
 *		assert(strncmp(buf1, string3, 10) == 0)
 *		chg_pri(TASK3, TPRI_INI)						... 元に戻す
 *	29:	snd_mbf(MBF1, string1, 26)
 *	== TASK4（続き）==
 *	30:	chg_pri(TASK2, HIGH_PRIORITY)					... (D-1)
 *		rcv_mbf(MBF1, buf1) -> 26
 *	== TASK2（続き）==
 *	31:	assert(strncmp(buf1, string1, 26) == 0)
 *		chg_pri(TSK_SELF, TPRI_INI)						... 元に戻す
 *	32:	snd_mbf(MBF1, string2, 10)
 *	== TASK4（続き）==
 *	33:	wup_tsk(TASK1)
 *	== TASK1（続き）==
 *	34:	chg_pri(TASK2, HIGH_PRIORITY)					... (D-2)
 *		slp_tsk()
 *	== TASK2（続き）==
 *	35:	rcv_mbf(MBF1, buf1) -> 10
 *		assert(strncmp(buf1, string2, 10) == 0)
 *		chg_pri(TSK_SELF, TPRI_INI)						... 元に戻す
 *	36:	snd_mbf(MBF1, string3, 10)
 *	== TASK4（続き）==
 *	37:	chg_pri(TASK2, HIGH_PRIORITY)					... (D-3)
 *	== TASK2（続き）==
 *	38:	rcv_mbf(MBF1, buf1) -> 10
 *		assert(strncmp(buf1, string3, 10) == 0)
 *		rcv_mbf(MBF1, buf1) -> 26
 *		assert(strncmp(buf1, string1, 26) == 0)
 *	39:	END
 */

#include <kernel.h>
#include <t_syslog.h>
#include "kernel_cfg.h"
#include "test_lib.h"
#include "test_messagebuf3.h"
#include <string.h>

const char string1[26] = "abcdefghijklmnopqrstuvwxyz";
const char string2[26] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
const char string3[16] = "0123456789abcdef";

char buf1[26];

extern ER	bit_kernel(void);

/* DO NOT DELETE THIS LINE -- gentest depends on it. */

void
task1(intptr_t exinf)
{
	ER_UINT	ercd;

	test_start(__FILE__);

	set_bit_func(bit_kernel);

	check_point(1);
	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = act_tsk(TASK4);
	check_ercd(ercd, E_OK);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(4);
	ercd = snd_mbf(MBF1, string2, 25);
	check_ercd(ercd, E_OK);

	check_point(6);
	check_assert(strncmp(buf1, string2, 25) == 0);

	ercd = act_tsk(TASK3);
	check_ercd(ercd, E_OK);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(13);
	ercd = snd_mbf(MBF1, string2, 10);
	check_ercd(ercd, E_OK);

	ercd = rcv_mbf(MBF1, buf1);
	check_ercd(ercd, 10);

	check_assert(strncmp(buf1, string2, 10) == 0);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(24);
	ercd = chg_pri(TASK3, LOW_PRIORITY);
	check_ercd(ercd, E_OK);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(34);
	ercd = chg_pri(TASK2, HIGH_PRIORITY);
	check_ercd(ercd, E_OK);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(0);
}

void
task2(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(2);
	ercd = snd_mbf(MBF1, string1, 26);
	check_ercd(ercd, E_OK);

	check_point(9);
	check_assert(strncmp(buf1, string1, 26) == 0);

	ercd = rcv_mbf(MBF1, buf1);
	check_ercd(ercd, 26);

	check_assert(strncmp(buf1, string2, 26) == 0);

	check_point(10);
	ercd = snd_mbf(MBF1, string3, 10);
	check_ercd(ercd, E_OK);

	ercd = rcv_mbf(MBF1, buf1);
	check_ercd(ercd, 10);

	check_assert(strncmp(buf1, string3, 10) == 0);

	check_point(11);
	ercd = snd_mbf(MBF1, string1, 26);
	check_ercd(ercd, E_OK);

	check_point(16);
	check_assert(strncmp(buf1, string1, 26) == 0);

	ercd = rcv_mbf(MBF1, buf1);
	check_ercd(ercd, 10);

	check_assert(strncmp(buf1, string3, 10) == 0);

	check_point(17);
	ercd = snd_mbf(MBF1, string1, 26);
	check_ercd(ercd, E_OK);

	check_point(22);
	ercd = snd_mbf(MBF1, string2, 10);
	check_ercd(ercd, E_OK);

	check_point(25);
	ercd = rcv_mbf(MBF1, buf1);
	check_ercd(ercd, 10);

	check_assert(strncmp(buf1, string2, 10) == 0);

	ercd = chg_pri(TASK3, TPRI_INI);
	check_ercd(ercd, E_OK);

	check_point(26);
	ercd = snd_mbf(MBF1, string3, 10);
	check_ercd(ercd, E_OK);

	check_point(28);
	ercd = rcv_mbf(MBF1, buf1);
	check_ercd(ercd, 10);

	check_assert(strncmp(buf1, string3, 10) == 0);

	ercd = chg_pri(TASK3, TPRI_INI);
	check_ercd(ercd, E_OK);

	check_point(29);
	ercd = snd_mbf(MBF1, string1, 26);
	check_ercd(ercd, E_OK);

	check_point(31);
	check_assert(strncmp(buf1, string1, 26) == 0);

	ercd = chg_pri(TSK_SELF, TPRI_INI);
	check_ercd(ercd, E_OK);

	check_point(32);
	ercd = snd_mbf(MBF1, string2, 10);
	check_ercd(ercd, E_OK);

	check_point(35);
	ercd = rcv_mbf(MBF1, buf1);
	check_ercd(ercd, 10);

	check_assert(strncmp(buf1, string2, 10) == 0);

	ercd = chg_pri(TSK_SELF, TPRI_INI);
	check_ercd(ercd, E_OK);

	check_point(36);
	ercd = snd_mbf(MBF1, string3, 10);
	check_ercd(ercd, E_OK);

	check_point(38);
	ercd = rcv_mbf(MBF1, buf1);
	check_ercd(ercd, 10);

	check_assert(strncmp(buf1, string3, 10) == 0);

	ercd = rcv_mbf(MBF1, buf1);
	check_ercd(ercd, 26);

	check_assert(strncmp(buf1, string1, 26) == 0);

	check_finish(39);
	check_point(0);
}

void
task3(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(7);
	ercd = snd_mbf(MBF1, string2, 26);
	check_ercd(ercd, E_OK);

	check_point(12);
	ercd = wup_tsk(TASK1);
	check_ercd(ercd, E_OK);

	check_point(14);
	ercd = snd_mbf(MBF1, string3, 10);
	check_ercd(ercd, E_OK);

	check_point(18);
	ercd = snd_mbf(MBF1, string2, 26);
	check_ercd(ercd, E_OK);

	check_point(20);
	check_assert(strncmp(buf1, string2, 26) == 0);

	ercd = rcv_mbf(MBF1, buf1);
	check_ercd(ercd, 26);

	check_assert(strncmp(buf1, string1, 26) == 0);

	ercd = chg_pri(TASK2, TPRI_INI);
	check_ercd(ercd, E_OK);

	check_point(21);
	ercd = snd_mbf(MBF1, string1, 26);
	check_ercd(ercd, E_OK);

	check_point(0);
}

void
task4(intptr_t exinf)
{
	ER_UINT	ercd;
	T_RMBF	rmbf;

	check_point(3);
	ercd = wup_tsk(TASK1);
	check_ercd(ercd, E_OK);

	check_point(5);
	ercd = ref_mbf(MBF1, &rmbf);
	check_ercd(ercd, E_OK);

	check_assert(rmbf.stskid == TASK1);

	check_assert(rmbf.rtskid == TSK_NONE);

	check_assert(rmbf.smbfcnt == 0);

	ercd = rcv_mbf(MBF1, buf1);
	check_ercd(ercd, 25);

	check_point(8);
	ercd = rcv_mbf(MBF1, buf1);
	check_ercd(ercd, 26);

	check_point(15);
	ercd = rcv_mbf(MBF1, buf1);
	check_ercd(ercd, 26);

	check_point(19);
	ercd = chg_pri(TASK2, LOW_PRIORITY);
	check_ercd(ercd, E_OK);

	ercd = rcv_mbf(MBF1, buf1);
	check_ercd(ercd, 26);

	check_point(23);
	ercd = wup_tsk(TASK1);
	check_ercd(ercd, E_OK);

	check_point(27);
	ercd = chg_pri(TASK3, LOW_PRIORITY);
	check_ercd(ercd, E_OK);

	check_point(30);
	ercd = chg_pri(TASK2, HIGH_PRIORITY);
	check_ercd(ercd, E_OK);

	ercd = rcv_mbf(MBF1, buf1);
	check_ercd(ercd, 26);

	check_point(33);
	ercd = wup_tsk(TASK1);
	check_ercd(ercd, E_OK);

	check_point(37);
	ercd = chg_pri(TASK2, HIGH_PRIORITY);
	check_ercd(ercd, E_OK);

	check_point(0);
}
