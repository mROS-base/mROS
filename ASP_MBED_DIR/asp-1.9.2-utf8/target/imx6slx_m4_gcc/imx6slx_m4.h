/*
 *  TOPPERS/FMP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Flexible MultiProcessor Kernel
 *
 *  Copyright (C) 2015 by Embedded and Real-Time Systems Laboratory
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
 */
#ifndef TOPPERS_IMX6SLX_H
#define TOPPERS_IMX6SLX_H

#include <sil.h>

/*
 *  UART
 */
#define UART1_BASE   0x42020000
#define UART2_BASE   0x421E8000

/*
 *  Interrupt Number
 */
#define IRQNO_UART1 (26 + 16)
#define IRQNO_UART2 (27 + 16)

/*
 *  IOMUXC
 */
#define IOMUXC_BASE 0x420E0000

#define IOMUXC_PAD_CTL_PKE		(1 << 12)
#define IOMUXC_PAD_CTL_PUE		(1 << 13 | IOMUXC_PAD_CTL_PKE)
#define IOMUXC_PAD_CTL_PUS_100K_UP	(2 << 14 | IOMUXC_PAD_CTL_PUE)
#define IOMUXC_PAD_CTL_SPEED_MED	(2 << 6)
#define IOMUXC_PAD_CTL_DSE_40ohm	(6 << 3)
#define IOMUXC_PAD_CTL_SRE_FAST	(1 << 0)
#define IOMUXC_PAD_CTL_HYS		(1 << 16)

#define UART_UBIR_VAL   0x000f
#define UART_UBMR_VAL   0x0068

#define UART1_TX_IOMUX 0x00024,0,0x0830,0,0x036C,UART_PAD_CTL_VAL
#define UART1_RX_IOMUX 0x0028,0,0x0830,1,0x0370,UART_PAD_CTL_VAL
#define UART2_TX_IOMUX 0x002C,0,0x0838,0,0x0374,UART_PAD_CTL_VAL
#define UART2_RX_IOMUX 0x0030,0,0x0838,1,0x0378,UART_PAD_CTL_VAL

/*
 *  割込み数
 */
#define TMAX_INTNO (128 + 16)

/*
 *  クロック
 */
#define SYS_CLOCK		227000000


#endif /* TOPPERS_IMX6SLX_H */
