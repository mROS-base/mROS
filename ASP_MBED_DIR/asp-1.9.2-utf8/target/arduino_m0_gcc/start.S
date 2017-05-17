/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2008,2015 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2013-2015 by Naoki Saito
 *             Nagoya Municipal Industrial Research Institute, JAPAN
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
 *  @(#) $Id: start.S 2728 2015-12-30 01:46:11Z ertl-honda $
 */
                
/*
 *   カーネル用のスタートアップモジュール(ARM-M用)
 */
#define TOPPERS_MACRO_ONLY
#define UINT_C(val)		(val)		/* uint_t型の定数を作るマクロ */
#define ULONG_C(val)	(val)		/* ulong_t型の定数を作るマクロ */
#define CAST(type, val)	(val)		/* 型キャストを行うマクロ */

#include "kernel_impl.h"
#include "arm_m.h"

/*
 *  スタートアップルーチン
 *
 *  Threadモードで呼び出されることを前提としてる．
 */
	.text
	.align 2
	.code 16
	.syntax unified
	.globl Reset_Handler
	.globl _start
	.type Reset_Handler, function
	.type _start, function
Reset_Handler:
_start:
	cpsid i                 /* 割込みロック状態へ */

#ifdef INIT_MSP
	/*
	 * MSPの設定
	 */
	mov   r0, #CONTROL_MSP  /* MSPを有効に  */
	msr   control, r0
	isb                     /* control の操作後に必要 */
	ldr  r0,=_kernel_istkpt
	ldr  r1,[r0]
	msr  msp, r1
#endif /* INIT_MSP */

	/*
	 * hardware_init_hookの呼び出し （0でない場合）
	 *
	 * ターゲットハードウェアに依存して必要な初期化処理がある場合
	 * は，hardware_init_hook という関数を用意する
	 */
	ldr  r0, =hardware_init_hook
	cmp  r0, #0
	beq  start_1
	blx  r0

start_1:
#ifndef OMIT_BSS_INIT
	/*
	 *  BSSセクション初期化
	 */
	movs  r0, #0
	ldr  r1, =__bss_start__
	ldr  r2, =__bss_end__
	cmp  r1, r2
	bhs  start_3
start_2:
	str  r0, [r1]
	adds r1,r1, #4
	cmp  r1, r2
	blo  start_2
#endif /* OMIT_BSS_INIT */


start_3:

#ifndef OMIT_DATA_INIT
	/*
	 *  DATAセクション初期化
	 *  __idata_start から __idata_end までを，__data_start 以降に
	 *  コピーする
	 */
	ldr  r1, =__etext
	ldr  r3, =__data_start__
	ldr  r2, =__data_end__
	cmp  r3, r2
	bhs  start_5
start_4:
	ldr  r0, [r1]
	str  r0, [r3]
	adds  r1, #4
	adds  r3, #4
	cmp  r3, r2
	blo  start_4
#endif /* OMIT_DATA_INIT */

	/*
	 *  software_init_hook を呼出し（0 でない場合）
	 *
	 *  ソフトウェア環境（特にライブラリ）に依存して必要な初期化処
	 *  理がある場合は，software_init_hook という関数を用意すれば
	 *  よい．
	 */
start_5:
	ldr  r0, =software_init_hook
	cmp  r0, #0
	beq  start_6
	blx  r0

start_6:
	bl   sta_ker

l_end:
	b   l_end
