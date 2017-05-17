/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2008-2015 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: arm_m.h 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *  ARMVx-Mのハードウェア資源の定義
 */

#ifndef ARM_M_H
#define ARM_M_H

#if !((__TARGET_ARCH_THUMB == 4) || (__TARGET_ARCH_THUMB == 3))
#error __TARGET_ARCH_THUMB is not defined or other than 4(ARMv7)/3(ARMv6)
#endif /* __TARGET_ARCH_THUMB == 4 */

/*
 *  EPSRのTビット
 */
#define EPSR_T   0x01000000

/*
 * IPSRの ISR NUMBER
 */
#define IPSR_ISR_NUMBER 0x1ff

/*
 *  例外・割込み発生時にスタック上に積まれる保存領域のサイズ
 *  本カーネルでは例外フレームと呼ぶ
 */
#define EXC_FRAME_SIZE         (8*4)

/*
 *  例外・割込み発生時にLRに設定されるEXC_RETURNの値
 */
#define EXC_RETURN_PSP          0x04
#define EXC_RETURN_HANDLER_MSP  0x00
#define EXC_RETURN_TREAD_MSP    0x09
#define EXC_RETURN_TREAD_PSP    0x0d
#define EXC_RETURN_FP           0x10
#define EXC_RETURN_FP_USED      0x00
#define EXC_RETURN_FP_NONUSED   0x10
#define EXC_RETURN_OTHER        0xFFFFFFE0

/*
 *  CONTROLレジスタ
 */
#define CONTROL_PSP            0x02
#define CONTROL_MSP            0x00
#define CONTROL_FPCA           0x04

/*
 *  例外番号
 */
#define EXCNO_NMI         2
#define EXCNO_HARD        3
#define EXCNO_MPU         4
#define EXCNO_BUS         5
#define EXCNO_USAGE       6
#define EXCNO_SVCALL     11          
#define EXCNO_DEBUG      12
#define EXCNO_PENDSV     14

/*
 *  例外番号の最小値と最大値
 */
#define TMIN_EXCNO   2
#define TMAX_EXCNO  14

/*
 *  割込み番号
 */
#define IRQNO_SYSTICK    15

/*
 *  割込み番号の最小値
 */
#define TMIN_INTNO       15

/*
 *  例外フレームのオフセット
 */
#define P_EXCINF_OFFSET_EXC_RETURN  0x01
#define P_EXCINF_OFFSET_IIPM        0x00
#define P_EXCINF_OFFSET_XPSR        0x09
#define P_EXCINF_OFFSET_PC          0x08

/*
 *  NVIC関連
 */

/*
 *  システムハンドラーコントロールレジスタ
 */
#define NVIC_SYS_HND_CTRL   0xE000ED24

/*
 *  各例外の許可ビット
 */
#define NVIC_SYS_HND_CTRL_USAGE 0x00040000
#define NVIC_SYS_HND_CTRL_BUS   0x00020000
#define NVIC_SYS_HND_CTRL_MEM   0x00010000

/*
 *  優先度設定レジスタ
 */
#define NVIC_SYS_PRI1       0xE000ED18  // Sys. Handlers 4 to 7 Priority
#define NVIC_SYS_PRI2       0xE000ED1C  // Sys. Handlers 8 to 11 Priority
#define NVIC_SYS_PRI3       0xE000ED20  // Sys. Handlers 12 to 15 Priority
#define NVIC_PRI0           0xE000E400  // IRQ 0 to 3 Priority Register

/*
 *  割込み許可レジスタ
 */
#define NVIC_SETENA0        0xE000E100  // IRQ 0 to 31 Set Enable Register

/*
 *  割込み禁止レジスタ
 */
#define NVIC_CLRENA0        0xE000E180  // IRQ 0 to 31 Set Disable Register

/*
 *  割込みセットペンディングレジスタ
 */
#define NVIC_ISER0          0xE000E200  // IRQ 0 to 31 Set-Pending Register

/*
 *  割込みクリアペンディングレジスタ
 */
#define NVIC_ICER0          0xE000E280  // IRQ 0 to 31 Clear-Pending Register

/*
 *  ベクタテーブルオフセットレジスタ
 */
#define NVIC_VECTTBL        0xE000ED08

/*
 *  割り込み制御と状態レジスタ
 */
#define NVIC_ICSR           0xE000ED04
#define NVIC_PENDSVSET          (1 << 28)       /*  PenvSVC例外 */
#define NVIC_PENDSTSET          (1 << 26)       /*  SYSTick例外 */


/*
 *  SYSTIC関連レジスタ
 */
#define SYSTIC_CONTROL_STATUS 0xE000E010
#define SYSTIC_RELOAD_VALUE   0xE000E014
#define SYSTIC_CURRENT_VALUE  0xE000E018
#define SYSTIC_CALIBRATION    0xE000E01C

#define SYSTIC_ENABLE    0x01
#define SYSTIC_TICINT    0x02
#define SYSTIC_CLKSOURCE 0x04
#define SYSTIC_COUNTFLAG 0x10000

#define SYSTIC_SKEW      0x40000000
#define SYSTIC_NOREF     0x80000000
#define SYSTIC_TENMS     0x00ffffff

/*
 * FPU関連レジスタ
 */
#if __TARGET_ARCH_THUMB == 4

#define CPACR 0xE000ED88
#define FPCCR 0xE000EF34

#define CPACR_FPU_ENABLE 0x00f00000
#define FPCCR_NO_PRESERV       0x00000000
#define FPCCR_NO_LAZYSTACKING  0x80000000
#define FPCCR_LAZYSTACKING     0xC0000000
#define FPCCR_LSPACT           0x00000001

#endif /* __TARGET_ARCH_THUMB == 4 */


#if defined(TOPPERS_CORTEX_M0) || defined(TOPPERS_CORTEX_M0PLUS)
/*
 *  M0/M0+固有の内容
 */

/*
 *  割込み優先度の範囲
 */
#define TMIN_INTPRI		(-4)	/* 割込み優先度の最小値（最高値）*/
#define TMAX_INTPRI		(-1)	/* 割込み優先度の最大値（最低値）*/

/*
 *  割込み優先度のビット幅
 */
#define TBITW_IPRI		2

#endif /* defined(TOPPERS_CORTEX_M0) || defined(TOPPERS_CORTEX_M0PLUS) */

#endif  /* ARM_M_H */
