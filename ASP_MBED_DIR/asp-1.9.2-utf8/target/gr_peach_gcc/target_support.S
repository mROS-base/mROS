/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2006-2012 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: target_support.S 2761 2016-03-11 03:30:48Z ertl-honda $
 */

/*
 *  ターゲット依存モジュール アセンブリ言語部（R7S72100用）
 */

#define  TOPPERS_MACRO_ONLY
#define UINT_C(val)		(val)		/* uint_t型の定数を作るマクロ */
#define ULONG_C(val)	(val)		/* ulong_t型の定数を作るマクロ */

#include "kernel_impl.h"

/*
 *  低レベルのターゲットシステム依存の初期化
 *
 *  スタートアップモジュールの中で，メモリ初期化の前に呼び出される．
 */
	.text
	.align 2
	.global hardware_init_hook
hardware_init_hook:
	/*
	 *  SRAM書き込み許可
	 */
	ldr	r2, =(ASP_SYSCR3)
	ldr	r0, =0x0F
	strb	r0, [r2]		/* RRAM page0-3 write enable */
	ldrb	r0, [r2]		/* dummy read */

	/* 
	 * クロック関係の初期化
	 */
	/* L2$をスタンバイモードへ */
	ldr	r2, =(ASP_PL310_POWER_CTRL)
	ldr	r0, [r2]
	orr	r0, r0, #0x01     /* enable stand-by mode */
	str	r0, [r2]

	/* CPUクロック= *1 (400MHz) */
set_frqcr:
	ldr	r2, =(ASP_FRQCR)
	ldr	r0, =0x1035
	strh	r0, [r2]
	ldrh	r1, [r2]
	cmp	r0, r1
	bne	set_frqcr

#ifdef TOPPERS_RZA1H
	/* 
	 * 画像処理クロック= *2/3
	 * Clockin = 13.33MHz, CKIO = 66.67MHz
	 * I  Clock = 400.00MHz,                
	 * G  Clock = 266.67MHz, 
	 * B  Clock = 133.33MHz, 
	 * P1 Clock =  66.67MHz,      
	 *  P0 Clock =  33.33MHz  
	 */
set_frqcr2:
	ldr	r2, =(ASP_FRQCR2)
	ldr	r0, =0x0001
	strh	r0, [r2]
	ldrh	r1, [r2]
	cmp	r0, r1
	bne	set_frqcr2
#endif /* TOPPERS_RZA1H */

wait_isbusy:
	ldr	r2, =(ASP_CPUSTS)
	ldrb	r0, [r2]
	tst	r0, #0x10
	bne	wait_isbusy

	ldr	r2, =(ASP_PL310_POWER_CTRL)
	ldr	r0, [r2]
	bfc	r0, #0, #1    /* disable stand-by mode */
	str	r0, [r2]

#if 0
/* バスステートコントローラの初期化 */
	ldr	r2, =(ASP_CS0BCR)
	ldr	r0, =0x10000C00		/* IWW=1 wait cycle, 16bit幅 */
	str	r0, [r2]

	ldr	r2, =(ASP_CS0WCR)
	ldr	r0, =0x00000B40		/* SW=1.5cycle, WR=6cycle, WM=1, HW=0.5cycle */
	str	r0, [r2]

	ldr	r2, =(ASP_CS1BCR)
	ldr	r0, =0x10000C00		/* IWW=1 wait cycle, 16bit幅 */
	str	r0, [r2]
	ldr	r2, =(ASP_CS1WCR)
	ldr	r0, =0x00000B40		/* SW=1.5cycle, WR=6cycle, WM=1, HW=0.5cycle */
	str	r0, [r2]

	ldr	r2, =(ASP_CS2BCR)
	ldr	r0, =0x00004C00		/* SDRAM, 16bit幅 */
	str	r0, [r2]

	ldr	r2, =(ASP_CS2WCR)
	ldr	r0, =0x00000480		/* CAS latency 2 cycle */
	str	r0, [r2]

	ldr	r2, =(ASP_CS3BCR)
	ldr	r0, =0x00004C00		/* SDRAM, 16bit幅 */
	str	r0, [r2]

	ldr	r2, =(ASP_CS3WCR)
	ldr	r0, =0x00002492		/* WTRP=1cycle, WTRCD=1cycle, CAS latency 2 cycle, */
	str	r0, [r2]

	ldr	r2, =(ASP_SDRAM_MODE_CS2)
	eor	r0, r0
	strh	r0, [r2]

	ldr	r2, =(ASP_SDCR)
	ldr	r0, =0x00120812		/* Row address 13bit, column address 10bit, do refresh, */
					/* auto refresh, auto pre-charge */
	str	r0, [r2]

	ldr	r2, =(ASP_RTCOR)
	ldr	r0, =0xA55A0020		/* 7.813usec /240nsec = 32(0x20)cycles per refresh */
	str	r0, [r2]

	ldr	r2, =(ASP_RTCSR)
	ldr	r0, =0xA55A0010		/* clock=CKIO/16 */
	str	r0, [r2]

	ldr	r2, =(ASP_SDRAM_MODE_CS3)
	eor	r0, r0
	strh	r0, [r2]
#endif

	bx	lr

#ifdef TOPPERS_EXECUTE_ON_ROM  
    ldr     r0, sf_boot
loop:
    b       loop

sf_boot: 
    .word   boot_loader
#endif /* TOPPERS_EXECUTE_ON_ROM   */
