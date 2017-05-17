/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel  
 *
 *  Copyright (C) 2006-2016 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: rza1.h 2765 2016-03-13 00:12:41Z ertl-honda $
 */

/*
 *  RZ-A1 のハードウェア資源の定義
 */

#ifndef TOPPERS_RZA1_H
#define TOPPERS_RZA1_H

#include <sil.h>

#ifdef TOPPERS_RZA1H
/*
 *  サポートするシリアルポートの個数
 */
#define TNUM_SIOP  8

#ifndef GIC_TMAX_INTNO
#define GIC_TMAX_INTNO    586U
#endif /* GIC_TMAX_INTNO */

#ifndef GIC_TNUM_INT
#define GIC_TNUM_INT      587U
#endif /* GIC_TNUM_INT */

#elif defined(TOPPERS_RZ_A1L)

/*
 *  サポートするシリアルポートの個数
 */
#define TNUM_SIOP  5

#ifndef GIC_TMAX_INTNO
#define GIC_TMAX_INTNO    537U
#endif /* GIC_TMAX_INTNO */

#ifndef GIC_TNUM_INT
#define GIC_TNUM_INT      538U
#endif /* GIC_TNUM_INT */

#else /* TOPPERS_RZA1L */
#error Define either TOPPERS_RZA1H or TOPPERS_RZA1L
#endif /* TOPPERS_RZA1H */

/*
 *  割込みコントローラのベースアドレス
 */
#define GICC_BASE           0xE8202000
#define GICD_BASE           0xE8201000

/*
 *	UARTのベースアドレス
 */
#define UART1_BASE      0xE8007000
#define UART2_BASE      0xE8007800
#define UART3_BASE      0xE8008000
#define UART4_BASE      0xE8008800
#define UART5_BASE      0xE8009000
#define UART6_BASE      0xE8009800
#define UART7_BASE      0xE800A000
#define UART8_BASE      0xE800A800

/*
 *	割込み番号
 */
#define TOPPERS_INTID_IRQ5               37     /* 割り込みID(IRQ5) */
#define TOPPERS_INTID_OSTM0             134     /* 割り込みID(OSTM0) */
#define TOPPERS_INTID_OSTM1             135     /* 割り込みID(OSTM1) */
#define TOPPERS_INTID_SCIF_BRI_1        221     /* 割り込みID(BRI0) */
#define TOPPERS_INTID_SCIF_ERI_1        222     /* 割り込みID(ERI0) */
#define TOPPERS_INTID_SCIF_RXI_1        223     /* 割り込みID(RXI0) */
#define TOPPERS_INTID_SCIF_TXI_1        224     /* 割り込みID(TXI0) */
#define TOPPERS_INTID_SCIF_BRI_2        225     /* 割り込みID(BRI1) */
#define TOPPERS_INTID_SCIF_ERI_2        226     /* 割り込みID(ERI1) */
#define TOPPERS_INTID_SCIF_RXI_2        227     /* 割り込みID(RXI1) */
#define TOPPERS_INTID_SCIF_TXI_2        228     /* 割り込みID(TXI1) */
#define TOPPERS_INTID_SCIF_BRI_3        229     /* 割り込みID(BRI2) */
#define TOPPERS_INTID_SCIF_ERI_3        230     /* 割り込みID(ERI2) */
#define TOPPERS_INTID_SCIF_RXI_3        231     /* 割り込みID(RXI2) */
#define TOPPERS_INTID_SCIF_TXI_3        232     /* 割り込みID(TXI2) */
#define TOPPERS_INTID_SCIF_BRI_4        233     /* 割り込みID(BRI3) */
#define TOPPERS_INTID_SCIF_ERI_4        234     /* 割り込みID(ERI3) */
#define TOPPERS_INTID_SCIF_RXI_4        235     /* 割り込みID(RXI3) */
#define TOPPERS_INTID_SCIF_TXI_4        236     /* 割り込みID(TXI3) */
#define TOPPERS_INTID_SCIF_BRI_5        237     /* 割り込みID(BRI4) */
#define TOPPERS_INTID_SCIF_ERI_5        238     /* 割り込みID(ERI4) */
#define TOPPERS_INTID_SCIF_RXI_5        239     /* 割り込みID(RXI4) */
#define TOPPERS_INTID_SCIF_TXI_5        240     /* 割り込みID(TXI4) */
#define TOPPERS_INTID_SCIF_BRI_6        241     /* 割り込みID(BRI5) */
#define TOPPERS_INTID_SCIF_ERI_6        242     /* 割り込みID(ERI5) */
#define TOPPERS_INTID_SCIF_RXI_6        243     /* 割り込みID(RXI5) */
#define TOPPERS_INTID_SCIF_TXI_6        244     /* 割り込みID(TXI5) */
#define TOPPERS_INTID_SCIF_BRI_7        245     /* 割り込みID(BRI6) */
#define TOPPERS_INTID_SCIF_ERI_7        246     /* 割り込みID(ERI6) */
#define TOPPERS_INTID_SCIF_RXI_7        247     /* 割り込みID(RXI6) */
#define TOPPERS_INTID_SCIF_TXI_7        248     /* 割り込みID(TXI6) */
#define TOPPERS_INTID_SCIF_BRI_8        249     /* 割り込みID(BRI7) */
#define TOPPERS_INTID_SCIF_ERI_8        250     /* 割り込みID(ERI7) */
#define TOPPERS_INTID_SCIF_RXI_8        251     /* 割り込みID(RXI7) */
#define TOPPERS_INTID_SCIF_TXI_8        252     /* 割り込みID(TXI7) */

/*
 *  割込み関連のレジスタ
 */
#define ICR0       0xFCFEF800
#define ICR1       0xFCFEF802
#define IRQRR      0xFCFEF804

/*
 *	GICに関する設定
 */
#define GIC_PRI_LEVEL		  32 //優先度の個数,256,128,64,32,16

/*
 *  割込み優先度に関する設定
 */
#define TMIN_INTPRI   (-(GIC_PRI_LEVEL - 1))   /* 割込み優先度の最小値（最高値）*/
#define TMAX_INTPRI   (-1)    /* 割込み優先度の最大値（最低値）*/

/* メモリマップ。MMU設定用 */
#define ASP_CS0_BASE        0x00000000          /* CS0領域（NOR Flash） */
#define ASP_CS0_SIZE        0x04000000
#define ASP_CS1_BASE        0x04000000          /* CS1領域（NOR Flash） */
#define ASP_CS1_SIZE        0x04000000
#define ASP_CS2_BASE        0x08000000          /* CS2領域（SDRAM） */
#define ASP_CS2_SIZE        0x04000000
#define ASP_CS3_BASE        0x0C000000          /* CS3領域（SDRAM） */
#define ASP_CS3_SIZE        0x04000000
#define ASP_CS4_BASE        0x10000000          /* CS4領域 */
#define ASP_CS4_SIZE        0x04000000
#define ASP_CS5_BASE        0x14000000          /* CS5領域 */
#define ASP_CS5_SIZE        0x04000000
#define ASP_SPI_BASE        0x18000000          /* シリアルフラッシュメモリ */
#define ASP_SPI_SIZE        0x08000000
#define ASP_SRAM_BASE       0x20000000          /* 内蔵RAM */
#define ASP_SRAM_SIZE       0x00A00000
#define ASP_IO1_BASE        0x20A00000          /* I/O及び未使用領域 */
#define ASP_IO1_SIZE        0x1F600000

#define ASP_CS0M_BASE       0x40000000          /* CS0領域（NOR Flash）（ミラー） */
#define ASP_CS0M_SIZE       0x04000000
#define ASP_CS1M_BASE       0x44000000          /* CS1領域（NOR Flash）（ミラー） */
#define ASP_CS1M_SIZE       0x04000000
#define ASP_CS2M_BASE       0x48000000          /* CS2領域（SDRAM）（ミラー） */
#define ASP_CS2M_SIZE       0x04000000
#define ASP_CS3M_BASE       0x4C000000          /* CS3領域（SDRAM）（ミラー） */
#define ASP_CS3M_SIZE       0x04000000
#define ASP_CS4M_BASE       0x50000000          /* CS4領域（ミラー） */
#define ASP_CS4M_SIZE       0x04000000
#define ASP_CS5M_BASE       0x54000000          /* CS5領域（ミラー） */
#define ASP_CS5M_SIZE       0x04000000
#define ASP_SPIM_BASE       0x58000000          /* シリアルフラッシュメモリ（ミラー） */
#define ASP_SPIM_SIZE       0x08000000
#define ASP_SRAMM_BASE      0x60000000          /* 内蔵RAM（ミラー） */
#define ASP_SRAMM_SIZE      0x00A00000
#define ASP_IO2_BASE        0x60A00000          /* I/O及び未使用領域 */
#define ASP_IO2_SIZE        0x9F600000

/* Clock Pulse Generator */
#define ASP_CPG_BASE            0xFCFE0000
#define ASP_FRQCR               (ASP_CPG_BASE + 0x0010)
#define ASP_FRQCR2              (ASP_CPG_BASE + 0x0014)

/* 低消費電力モード */
#define ASP_LOWPWR_BASE         0xFCFE0000
#define ASP_STBCR1              (ASP_LOWPWR_BASE + 0x0020)
#define ASP_STBCR2              (ASP_LOWPWR_BASE + 0x0024)
#define ASP_STBCR3              (ASP_LOWPWR_BASE + 0x0420)
#define ASP_STBCR4              (ASP_LOWPWR_BASE + 0x0424)
#define ASP_STBCR5              (ASP_LOWPWR_BASE + 0x0428)
#define ASP_STBCR6              (ASP_LOWPWR_BASE + 0x042C)
#define ASP_STBCR7              (ASP_LOWPWR_BASE + 0x0430)
#define ASP_STBCR8              (ASP_LOWPWR_BASE + 0x0434)
#define ASP_STBCR9              (ASP_LOWPWR_BASE + 0x0438)
#define ASP_STBCR10             (ASP_LOWPWR_BASE + 0x043C)
#define ASP_STBCR11             (ASP_LOWPWR_BASE + 0x0440)
#define ASP_STBCR12             (ASP_LOWPWR_BASE + 0x0444)
#define ASP_STBCR13             (ASP_LOWPWR_BASE + 0x0470)
#define ASP_SYSCR3              (ASP_LOWPWR_BASE + 0x0408)
#define ASP_CPUSTS              (ASP_LOWPWR_BASE + 0x0018)

/* 汎用入出力ポート */
#define ASP_PORT_BASE           0xFCFE3000
#define ASP_PORT_P_OFFSET       0x0000
#define ASP_PORT_PSR_OFFSET     0x0200
#define ASP_PORT_PPR_OFFSET     0x0200
#define ASP_PORT_PM_OFFSET      0x0300
#define ASP_PORT_PMC_OFFSET     0x0400
#define ASP_PORT_PFC_OFFSET     0x0500
#define ASP_PORT_PFCE_OFFSET    0x0600
#define ASP_PORT_PFCAE_OFFSET   0x0A00
#define ASP_PORT_PIBC_OFFSET    0x4000
#define ASP_PORT_PBDC_OFFSET    0x4100
#define ASP_PORT_PIPC_OFFSET    0x4200
#define ASP_PORT_P(n)           (ASP_PORT_BASE + ASP_PORT_P_OFFSET     + (n) * 4)
#define ASP_PORT_PSR(n)         (ASP_PORT_BASE + ASP_PORT_PSR_OFFSET   + (n) * 4)
#define ASP_PORT_PPR(n)         (ASP_PORT_BASE + ASP_PORT_PPR_OFFSET   + (n) * 4)
#define ASP_PORT_PM(n)          (ASP_PORT_BASE + ASP_PORT_PM_OFFSET    + (n) * 4)
#define ASP_PORT_PMC(n)         (ASP_PORT_BASE + ASP_PORT_PMC_OFFSET   + (n) * 4)
#define ASP_PORT_PFC(n)         (ASP_PORT_BASE + ASP_PORT_PFC_OFFSET   + (n) * 4)
#define ASP_PORT_PFCE(n)        (ASP_PORT_BASE + ASP_PORT_PFCE_OFFSET  + (n) * 4)
#define ASP_PORT_PFCAE(n)       (ASP_PORT_BASE + ASP_PORT_PFCAE_OFFSET + (n) * 4)
#define ASP_PORT_PIBC(n)        (ASP_PORT_BASE + ASP_PORT_PIBC_OFFSET  + (n) * 4)
#define ASP_PORT_PBDC(n)        (ASP_PORT_BASE + ASP_PORT_PBDC_OFFSET  + (n) * 4)
#define ASP_PORT_PIPC(n)        (ASP_PORT_BASE + ASP_PORT_PIPC_OFFSET  + (n) * 4)

/* バスステートコントローラ */
#define ASP_BSC_BASE            0x3FFFC000
#define ASP_CMNCR               (ASP_BSC_BASE)
#define ASP_CS0BCR              (ASP_BSC_BASE + 0x0004)
#define ASP_CS1BCR              (ASP_BSC_BASE + 0x0008)
#define ASP_CS2BCR              (ASP_BSC_BASE + 0x000C)
#define ASP_CS3BCR              (ASP_BSC_BASE + 0x0010)
#define ASP_CS4BCR              (ASP_BSC_BASE + 0x0014)
#define ASP_CS5BCR              (ASP_BSC_BASE + 0x0018)
#define ASP_CS0WCR              (ASP_BSC_BASE + 0x0028)
#define ASP_CS1WCR              (ASP_BSC_BASE + 0x002C)
#define ASP_CS2WCR              (ASP_BSC_BASE + 0x0030)
#define ASP_CS3WCR              (ASP_BSC_BASE + 0x0034)
#define ASP_CS4WCR              (ASP_BSC_BASE + 0x0038)
#define ASP_CS5WCR              (ASP_BSC_BASE + 0x003C)
#define ASP_SDCR                (ASP_BSC_BASE + 0x004C)
#define ASP_RTCSR               (ASP_BSC_BASE + 0x0050)
#define ASP_RTCNT               (ASP_BSC_BASE + 0x0054)
#define ASP_RTCOR               (ASP_BSC_BASE + 0x0058)

/* PL310 L2 cache controller */
#define ASP_PL310_BASE          0x3FFFF000
#define ASP_PL310_POWER_CTRL    (ASP_PL310_BASE + 0xF80)

#endif /* TOPPERS_RZA1_H */
