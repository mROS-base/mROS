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
 *  $Id: test_messagebuf2.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/* 
 *		メッセージバッファのテスト(2)
 *
 * 【テストの目的】
 *
 *  FIFO順メッセージバッファの送信待ちタスクが，強制的に待ち解除された
 *  場合を一通りテストする．
 *
 * 【テスト項目】
 *
 *	(A) 送信待ちタスクが強制終了（ter_tsk）
 *		(A-1) 送信待ちタスクの待ち解除なし
 *		(A-2) 送信待ちタスク（1つまたは複数）が待ち解除（タスク切換えなし）
 *		(A-3) 送信待ちタスク（1つまたは複数）が待ち解除（タスク切換えあり）
 *	(B) 送信待ちタスクが強制待ち解除（rel_wai）
 *		(B-1) 送信待ちタスクの待ち解除なし
 *		(B-2) 送信待ちタスク（1つまたは複数）が待ち解除（タスク切換えなし）
 *		(B-3) 送信待ちタスク（1つまたは複数）が待ち解除（タスク切換えあり）
 *	(C) 送信待ちタスクが強制待ち解除（irel_wai）
 *		(C-1) 送信待ちタスクの待ち解除なし
 *		(C-2) 送信待ちタスク（1つまたは複数）が待ち解除（タスク切換えなし）
 *		(C-3) 送信待ちタスク（1つまたは複数）が待ち解除（タスク切換えあり）
 *	(D) 送信待ちタスクがタイムアウトで待ち解除
 *		(D-1) 送信待ちタスクの待ち解除なし
 *		(D-2) 送信待ちタスク（1つまたは複数）が待ち解除（タスク切換えなし）
 *		(D-3) 送信待ちタスク（1つまたは複数）が待ち解除（タスク切換えあり）
 *
 * 【使用リソース】
 *
 *	TASK1: 高優先度タスク，メインタスク，最初から起動
 *	TASK2: 中優先度タスク
 *	TASK3: 低優先度タスク
 *	MBF1: メッセージバッファ（TA_NULL属性，最大メッセージサイズ：26，メッ
 *		  セージバッファ管理領域のサイズ：26→実際には28）
 *
 * 【テストシーケンス】
 *
 *	== TASK1（優先度：高）==
 *		call(set_bit_func(bit_kernel))
 *	1:	act_tsk(TASK2)
 *		act_tsk(TASK3)
 *		tslp_tsk(1) -> E_TMOUT
 *	== TASK2-1（優先度：中）1回め ==
 *	2:	snd_mbf(MBF1, string1, 26)
 *	== TASK3（優先度：低）==
 *	3:	snd_mbf(MBF1, string2, 26)
 *	== TASK1（続き）==
 *	4:	ter_tsk(TASK2)									... (A-1)
 *		tslp_tsk(1) -> E_TMOUT
 *	== TASK1（続き）==
 *	5:	rcv_mbf(MBF1, buf1) -> 26
 *		assert(strncmp(buf1, string2, 26) == 0)
 *		act_tsk(TASK2)
 *		tslp_tsk(1) -> E_TMOUT
 *	== TASK2-2（優先度：中）2回め ==
 *	6:	snd_mbf(MBF1, string1, 26)
 *	== TASK3（続き）==
 *	7:	snd_mbf(MBF1, string3, 10)
 *	== TASK1（続き）==
 *	8:	ter_tsk(TASK2)									... (A-2)
 *		slp_tsk()
 *	== TASK3（続き）==
 *	9:	rcv_mbf(MBF1, buf1) -> 10
 *		assert(strncmp(buf1, string3, 10) == 0)
 *	10:	act_tsk(TASK2)
 *	== TASK2-3（優先度：中）3回め ==
 *	11:	snd_mbf(MBF1, string1, 26)
 *	== TASK3（続き）==
 *	12:	wup_tsk(TASK1)
 *	== TASK1（続き）==
 *	13:	snd_mbf(MBF1, string3, 10)
 *	== TASK3（続き）==
 *	14:	ter_tsk(TASK2)									... (A-3)
 *	== TASK1（続き）==
 *	15:	rcv_mbf(MBF1, buf1) -> 10
 *		assert(strncmp(buf1, string3, 10) == 0)
 *		act_tsk(TASK2)
 *		tslp_tsk(1) -> E_TMOUT
 *	== TASK2-4（優先度：中）4回め ==
 *	16:	snd_mbf(MBF1, string1, 26) -> E_RLWAI
 *	== TASK3（続き）==
 *	17:	snd_mbf(MBF1, string2, 26)
 *	== TASK1（続き）==
 *	18:	rel_wai(TASK2)									... (B-1)
 *		tslp_tsk(1) -> E_TMOUT
 *	== TASK2-4（続き）==
 *	19:	slp_tsk()
 *	== TASK1（続き）==
 *	20:	rcv_mbf(MBF1, buf1) -> 26
 *		assert(strncmp(buf1, string2, 26) == 0)
 *		wup_tsk(TASK2)
 *		tslp_tsk(1) -> E_TMOUT
 *	== TASK2-4（続き）==
 *	21:	snd_mbf(MBF1, string1, 26) -> E_RLWAI
 *	== TASK3（続き）==
 *	22:	snd_mbf(MBF1, string3, 10)
 *	== TASK1（続き）==
 *	23:	rel_wai(TASK2)									... (B-2)
 *		slp_tsk()
 *	== TASK2-4（続き）==
 *	24:	slp_tsk()
 *	== TASK3（続き）==
 *	25:	rcv_mbf(MBF1, buf1) -> 10
 *		assert(strncmp(buf1, string3, 10) == 0)
 *	26:	wup_tsk(TASK2)
 *	== TASK2-4（続き）==
 *	27:	snd_mbf(MBF1, string1, 26) -> E_RLWAI
 *	== TASK3（続き）==
 *	28:	wup_tsk(TASK1)
 *	== TASK1（続き）==
 *	29:	snd_mbf(MBF1, string3, 10)
 *	== TASK3（続き）==
 *	30:	rel_wai(TASK2)									... (B-3)
 *	== TASK1（続き）==
 *	31:	rcv_mbf(MBF1, buf1) -> 10
 *		assert(strncmp(buf1, string3, 10) == 0)
 *		tslp_tsk(1) -> E_TMOUT
 *	== TASK2-4（続き）==
 *	32:	snd_mbf(MBF1, string1, 26) -> E_RLWAI
 *	== TASK3（続き）==
 *	33:	snd_mbf(MBF1, string2, 26)
 *	== TASK1（続き）==
 *	34:	sta_alm(ALM1, 1U)
 *		slp_tsk()
 *	== ALM1-1 ==
 *	35:	irel_wai(TASK2)									... (C-1)
 *		iwup_tsk(TASK1)
 *		RETURN
 *	== TASK1（続き）==
 *	36:	tslp_tsk(1) -> E_TMOUT
 *	== TASK2-4（続き）==
 *	37:	slp_tsk()
 *	== TASK1（続き）==
 *	38:	rcv_mbf(MBF1, buf1) -> 26
 *		assert(strncmp(buf1, string2, 26) == 0)
 *		wup_tsk(TASK2)
 *		tslp_tsk(1) -> E_TMOUT
 *	== TASK2-4（続き）==
 *	39:	snd_mbf(MBF1, string1, 26) -> E_RLWAI
 *	== TASK3（続き）==
 *	40:	snd_mbf(MBF1, string3, 10)
 *	== TASK1（続き）==
 *	41:	sta_alm(ALM1, 1U)
 *		slp_tsk()
 *	== ALM1-2 ==
 *	42:	irel_wai(TASK2)									... (C-2)
 *		RETURN
 *	== TASK2-4（続き）==
 *	43:	slp_tsk()
 *	== TASK3（続き）==
 *	44:	rcv_mbf(MBF1, buf1) -> 10
 *		assert(strncmp(buf1, string3, 10) == 0)
 *	45:	wup_tsk(TASK2)
 *	== TASK2-4（続き）==
 *	46:	snd_mbf(MBF1, string1, 26) -> E_RLWAI
 *	== TASK3（続き）==
 *	47:	wup_tsk(TASK1)
 *	== TASK1（続き）==
 *	48:	snd_mbf(MBF1, string3, 10)
 *	== TASK3（続き）==
 *	49:	sta_alm(ALM1, 1U)
 *		slp_tsk()
 *	== ALM1-3 ==
 *	50:	irel_wai(TASK2)									... (C-3)
 *		RETURN
 *	== TASK1（続き）==
 *	51:	rcv_mbf(MBF1, buf1) -> 10
 *		assert(strncmp(buf1, string3, 10) == 0)
 *		wup_tsk(TASK3)
 *		slp_tsk()
 *	== TASK2-4（続き）==
  *	52:	tsnd_mbf(MBF1, string1, 26, 1) -> E_TMOUT		... (D-1)
 *	== TASK3（続き）==
 *	53:	snd_mbf(MBF1, string2, 26)
 *	== TASK2-4（続き）==
 *	54:	tslp_tsk(1) -> E_TMOUT
 *	== TASK2-4（続き）==
 *	55:	rcv_mbf(MBF1, buf1) -> 26
 *		assert(strncmp(buf1, string2, 26) == 0)
 *	56:	tsnd_mbf(MBF1, string1, 26, 1) -> E_TMOUT		... (D-2)
 *	== TASK3（続き）==
 *	57:	snd_mbf(MBF1, string3, 10)
 *	== TASK2-4（続き）==
 *	58:	slp_tsk()
 *	== TASK3（続き）==
 *	59:	rcv_mbf(MBF1, buf1) -> 10
 *		assert(strncmp(buf1, string3, 10) == 0)
 *	60:	wup_tsk(TASK2)
 *	== TASK2-4（続き）==
 *	61:	tsnd_mbf(MBF1, string1, 26, 1) -> E_TMOUT		... (D-3)
 *	== TASK3（続き）==
 *	62:	wup_tsk(TASK1)
 *	== TASK1（続き）==
 *	63:	snd_mbf(MBF1, string3, 10)
 *	== TASK3（続き）==
 *	64:	slp_tsk()
 *	== TASK1（続き）==
 *	65:	rcv_mbf(MBF1, buf1) -> 10
 *		assert(strncmp(buf1, string3, 10) == 0)
 *		slp_tsk()
 *	== TASK2-4（続き）==
 *	66:	END
 */

#include <kernel.h>
#include <t_syslog.h>
#include "kernel_cfg.h"
#include "test_lib.h"
#include "test_messagebuf2.h"
#include <string.h>

const char string1[26] = "abcdefghijklmnopqrstuvwxyz";
const char string2[26] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
const char string3[16] = "0123456789abcdef";

char buf1[26];

extern ER	bit_kernel(void);

/* DO NOT DELETE THIS LINE -- gentest depends on it. */

static uint_t	alarm1_count = 0;

void
alarm1_handler(intptr_t exinf)
{
	ER_UINT	ercd;

	switch (++alarm1_count) {
	case 1:
		check_point(35);
		ercd = irel_wai(TASK2);
		check_ercd(ercd, E_OK);

		ercd = iwup_tsk(TASK1);
		check_ercd(ercd, E_OK);

		return;

		check_point(0);

	case 2:
		check_point(42);
		ercd = irel_wai(TASK2);
		check_ercd(ercd, E_OK);

		return;

		check_point(0);

	case 3:
		check_point(50);
		ercd = irel_wai(TASK2);
		check_ercd(ercd, E_OK);

		return;

		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
}

void
task1(intptr_t exinf)
{
	ER_UINT	ercd;

	test_start(__FILE__);

	set_bit_func(bit_kernel);

	check_point(1);
	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = act_tsk(TASK3);
	check_ercd(ercd, E_OK);

	ercd = tslp_tsk(1);
	check_ercd(ercd, E_TMOUT);

	check_point(4);
	ercd = ter_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = tslp_tsk(1);
	check_ercd(ercd, E_TMOUT);

	check_point(5);
	ercd = rcv_mbf(MBF1, buf1);
	check_ercd(ercd, 26);

	check_assert(strncmp(buf1, string2, 26) == 0);

	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = tslp_tsk(1);
	check_ercd(ercd, E_TMOUT);

	check_point(8);
	ercd = ter_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(13);
	ercd = snd_mbf(MBF1, string3, 10);
	check_ercd(ercd, E_OK);

	check_point(15);
	ercd = rcv_mbf(MBF1, buf1);
	check_ercd(ercd, 10);

	check_assert(strncmp(buf1, string3, 10) == 0);

	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = tslp_tsk(1);
	check_ercd(ercd, E_TMOUT);

	check_point(18);
	ercd = rel_wai(TASK2);
	check_ercd(ercd, E_OK);

	ercd = tslp_tsk(1);
	check_ercd(ercd, E_TMOUT);

	check_point(20);
	ercd = rcv_mbf(MBF1, buf1);
	check_ercd(ercd, 26);

	check_assert(strncmp(buf1, string2, 26) == 0);

	ercd = wup_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = tslp_tsk(1);
	check_ercd(ercd, E_TMOUT);

	check_point(23);
	ercd = rel_wai(TASK2);
	check_ercd(ercd, E_OK);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(29);
	ercd = snd_mbf(MBF1, string3, 10);
	check_ercd(ercd, E_OK);

	check_point(31);
	ercd = rcv_mbf(MBF1, buf1);
	check_ercd(ercd, 10);

	check_assert(strncmp(buf1, string3, 10) == 0);

	ercd = tslp_tsk(1);
	check_ercd(ercd, E_TMOUT);

	check_point(34);
	ercd = sta_alm(ALM1, 1U);
	check_ercd(ercd, E_OK);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(36);
	ercd = tslp_tsk(1);
	check_ercd(ercd, E_TMOUT);

	check_point(38);
	ercd = rcv_mbf(MBF1, buf1);
	check_ercd(ercd, 26);

	check_assert(strncmp(buf1, string2, 26) == 0);

	ercd = wup_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = tslp_tsk(1);
	check_ercd(ercd, E_TMOUT);

	check_point(41);
	ercd = sta_alm(ALM1, 1U);
	check_ercd(ercd, E_OK);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(48);
	ercd = snd_mbf(MBF1, string3, 10);
	check_ercd(ercd, E_OK);

	check_point(51);
	ercd = rcv_mbf(MBF1, buf1);
	check_ercd(ercd, 10);

	check_assert(strncmp(buf1, string3, 10) == 0);

	ercd = wup_tsk(TASK3);
	check_ercd(ercd, E_OK);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(63);
	ercd = snd_mbf(MBF1, string3, 10);
	check_ercd(ercd, E_OK);

	check_point(65);
	ercd = rcv_mbf(MBF1, buf1);
	check_ercd(ercd, 10);

	check_assert(strncmp(buf1, string3, 10) == 0);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(0);
}

static uint_t	task2_count = 0;

void
task2(intptr_t exinf)
{
	ER_UINT	ercd;

	switch (++task2_count) {
	case 1:
		check_point(2);
		ercd = snd_mbf(MBF1, string1, 26);
		check_ercd(ercd, E_OK);

		check_point(0);

	case 2:
		check_point(6);
		ercd = snd_mbf(MBF1, string1, 26);
		check_ercd(ercd, E_OK);

		check_point(0);

	case 3:
		check_point(11);
		ercd = snd_mbf(MBF1, string1, 26);
		check_ercd(ercd, E_OK);

		check_point(0);

	case 4:
		check_point(16);
		ercd = snd_mbf(MBF1, string1, 26);
		check_ercd(ercd, E_RLWAI);

		check_point(19);
		ercd = slp_tsk();
		check_ercd(ercd, E_OK);

		check_point(21);
		ercd = snd_mbf(MBF1, string1, 26);
		check_ercd(ercd, E_RLWAI);

		check_point(24);
		ercd = slp_tsk();
		check_ercd(ercd, E_OK);

		check_point(27);
		ercd = snd_mbf(MBF1, string1, 26);
		check_ercd(ercd, E_RLWAI);

		check_point(32);
		ercd = snd_mbf(MBF1, string1, 26);
		check_ercd(ercd, E_RLWAI);

		check_point(37);
		ercd = slp_tsk();
		check_ercd(ercd, E_OK);

		check_point(39);
		ercd = snd_mbf(MBF1, string1, 26);
		check_ercd(ercd, E_RLWAI);

		check_point(43);
		ercd = slp_tsk();
		check_ercd(ercd, E_OK);

		check_point(46);
		ercd = snd_mbf(MBF1, string1, 26);
		check_ercd(ercd, E_RLWAI);

		check_point(52);
		ercd = tsnd_mbf(MBF1, string1, 26, 1);
		check_ercd(ercd, E_TMOUT);

		check_point(54);
		ercd = tslp_tsk(1);
		check_ercd(ercd, E_TMOUT);

		check_point(55);
		ercd = rcv_mbf(MBF1, buf1);
		check_ercd(ercd, 26);

		check_assert(strncmp(buf1, string2, 26) == 0);

		check_point(56);
		ercd = tsnd_mbf(MBF1, string1, 26, 1);
		check_ercd(ercd, E_TMOUT);

		check_point(58);
		ercd = slp_tsk();
		check_ercd(ercd, E_OK);

		check_point(61);
		ercd = tsnd_mbf(MBF1, string1, 26, 1);
		check_ercd(ercd, E_TMOUT);

		check_finish(66);
		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
}

void
task3(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(3);
	ercd = snd_mbf(MBF1, string2, 26);
	check_ercd(ercd, E_OK);

	check_point(7);
	ercd = snd_mbf(MBF1, string3, 10);
	check_ercd(ercd, E_OK);

	check_point(9);
	ercd = rcv_mbf(MBF1, buf1);
	check_ercd(ercd, 10);

	check_assert(strncmp(buf1, string3, 10) == 0);

	check_point(10);
	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(12);
	ercd = wup_tsk(TASK1);
	check_ercd(ercd, E_OK);

	check_point(14);
	ercd = ter_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(17);
	ercd = snd_mbf(MBF1, string2, 26);
	check_ercd(ercd, E_OK);

	check_point(22);
	ercd = snd_mbf(MBF1, string3, 10);
	check_ercd(ercd, E_OK);

	check_point(25);
	ercd = rcv_mbf(MBF1, buf1);
	check_ercd(ercd, 10);

	check_assert(strncmp(buf1, string3, 10) == 0);

	check_point(26);
	ercd = wup_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(28);
	ercd = wup_tsk(TASK1);
	check_ercd(ercd, E_OK);

	check_point(30);
	ercd = rel_wai(TASK2);
	check_ercd(ercd, E_OK);

	check_point(33);
	ercd = snd_mbf(MBF1, string2, 26);
	check_ercd(ercd, E_OK);

	check_point(40);
	ercd = snd_mbf(MBF1, string3, 10);
	check_ercd(ercd, E_OK);

	check_point(44);
	ercd = rcv_mbf(MBF1, buf1);
	check_ercd(ercd, 10);

	check_assert(strncmp(buf1, string3, 10) == 0);

	check_point(45);
	ercd = wup_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(47);
	ercd = wup_tsk(TASK1);
	check_ercd(ercd, E_OK);

	check_point(49);
	ercd = sta_alm(ALM1, 1U);
	check_ercd(ercd, E_OK);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(53);
	ercd = snd_mbf(MBF1, string2, 26);
	check_ercd(ercd, E_OK);

	check_point(57);
	ercd = snd_mbf(MBF1, string3, 10);
	check_ercd(ercd, E_OK);

	check_point(59);
	ercd = rcv_mbf(MBF1, buf1);
	check_ercd(ercd, 10);

	check_assert(strncmp(buf1, string3, 10) == 0);

	check_point(60);
	ercd = wup_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(62);
	ercd = wup_tsk(TASK1);
	check_ercd(ercd, E_OK);

	check_point(64);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(0);
}
