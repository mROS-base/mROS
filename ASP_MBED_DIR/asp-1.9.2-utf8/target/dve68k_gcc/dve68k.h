/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2005-2011 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: dve68k.h 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		DVE-68K/40 CPUボードのハードウェア資源の定義
 */

#ifndef TOPPERS_DVE68K_H
#define TOPPERS_DVE68K_H

#include <sil.h>

/*
 *  割込み番号（intno）の定義
 */
#define TINTNO_ACF		1U			/* ACFAIL割込み */
#define TINTNO_ABT		2U			/* アボート割込み */
#define TINTNO_SF		3U			/* SYSFAIL割込み */
#define TINTNO_BER		4U			/* バスエラー割込み */
#define TINTNO_IAK		5U			/* IAK割込み */
#define TINTNO_SRQ		6U			/* SRQ割込み */
#define TINTNO_SAK		7U			/* SAK割込み */
#define TINTNO_GP7		9U			/* GP7割込み */
#define TINTNO_DMA		10U			/* DMA割込み */
#define TINTNO_TT1		11U			/* タイマ1割込み */
#define TINTNO_TT0		12U			/* タイマ0割込み */
#define TINTNO_GP3		13U			/* フラッシュメモリ割込み */
#define TINTNO_GP2		14U			/* SCSI割込み */
#define TINTNO_GP1		15U			/* Ethernet割込み */
#define TINTNO_GP0		16U			/* シリアルI/O割込み */
#define TINTNO_SWI7		17U			/* ソフトウェア割込み7 */
#define TINTNO_SWI6		18U			/* ソフトウェア割込み6 */
#define TINTNO_SWI5		19U			/* ソフトウェア割込み5 */
#define TINTNO_SWI4		20U			/* ソフトウェア割込み4 */
#define TINTNO_SWI3		21U			/* ソフトウェア割込み3 */
#define TINTNO_SWI2		22U			/* ソフトウェア割込み2 */
#define TINTNO_SWI1		23U			/* ソフトウェア割込み1 */
#define TINTNO_SWI0		24U			/* ソフトウェア割込み0 */
#define TINTNO_VM7		25U			/* VME割込み7 */
#define TINTNO_VM6		26U			/* VME割込み6 */
#define TINTNO_VM5		27U			/* VME割込み5 */
#define TINTNO_VM4		28U			/* VME割込み4 */
#define TINTNO_VM3		29U			/* VME割込み3 */
#define TINTNO_VM2		30U			/* VME割込み2 */
#define TINTNO_VM1		31U			/* VME割込み1 */

/*
 *  割込みベクトルの設定値の定義
 */
#define TVEC_G0I		0x40U		/* グループ0割込みベクトル */
#define TVEC_G1I		0x48U		/* グループ1割込みベクトル */
#define TVEC_SWI		0x50U		/* ソフトウェア割込みベクトル */
#define TVEC_SPRI		0x40U		/* スプリアス割込みベクトル */

/*
 *  割込みハンドラ番号（inhno）の定義
 */
#define TINHNO_ACF		0x47U		/* ACFAIL割込み */
#define TINHNO_ABT		0x46U		/* アボート割込み */
#define TINHNO_SF		0x45U		/* SYSFAIL割込み */
#define TINHNO_BER		0x44U		/* バスエラー割込み */
#define TINHNO_IAK		0x43U		/* IAK割込み */
#define TINHNO_SRQ		0x42U		/* SRQ割込み */
#define TINHNO_SAK		0x41U		/* SAK割込み */
#define TINHNO_GP7		0x4fU		/* GP7割込み */
#define TINHNO_DMA		0x4eU		/* DMA割込み */
#define TINHNO_TT1		0x4dU		/* タイマ1割込み */
#define TINHNO_TT0		0x4cU		/* タイマ0割込み */
#define TINHNO_GP3		0x4bU		/* フラッシュメモリ割込み */
#define TINHNO_GP2		0x4aU		/* SCSI割込み */
#define TINHNO_GP1		0x49U		/* Ethernet割込み */
#define TINHNO_GP0		0x48U		/* シリアルI/O割込み */
#define TINHNO_SWI7		0x57U		/* ソフトウェア割込み7 */
#define TINHNO_SWI6		0x56U		/* ソフトウェア割込み6 */
#define TINHNO_SWI5		0x55U		/* ソフトウェア割込み5 */
#define TINHNO_SWI4		0x54U		/* ソフトウェア割込み4 */
#define TINHNO_SWI3		0x53U		/* ソフトウェア割込み3 */
#define TINHNO_SWI2		0x52U		/* ソフトウェア割込み2 */
#define TINHNO_SWI1		0x51U		/* ソフトウェア割込み1 */
#define TINHNO_SWI0		0x50U		/* ソフトウェア割込み0 */
#define TINHNO_SPRI		0x40U		/* スプリアス割込み */

/*
 *  CPUボード上のレジスタ
 */
#define TADR_BOARD_REG0		0xfff48000
#define TADR_BOARD_REG1		0xfff48004
#define TADR_BOARD_REG2		0xfff48008

/*
 *  DGA-001のレジスタのアドレス
 */
#define TADR_DGA_CSR0		0xfff44000
#define TADR_DGA_CSR1		0xfff44004
#define TADR_DGA_CSR3		0xfff4400c
#define TADR_DGA_CSR4		0xfff44010
#define TADR_DGA_CSR5		0xfff44014
#define TADR_DGA_CSR12		0xfff44030
#define TADR_DGA_CSR13		0xfff44034
#define TADR_DGA_CSR14		0xfff44038
#define TADR_DGA_CSR15		0xfff4403c
#define TADR_DGA_CSR18		0xfff44048
#define TADR_DGA_CSR19		0xfff4404c
#define TADR_DGA_CSR20		0xfff44050
#define TADR_DGA_CSR21		0xfff44054
#define TADR_DGA_CSR23		0xfff4405c
#define TADR_DGA_CSR24		0xfff44060
#define TADR_DGA_IFR0		0xfff44070
#define TADR_DGA_IFR3		0xfff4407c

/*
 *  DGA-001の割込み優先度設定のための定義
 */
#define TIRQ_NMI		(-7)		/* ノンマスカブル割込み */
#define TIRQ_LEVEL6		(-6)		/* 割込みレベル6 */
#define TIRQ_LEVEL5		(-5)		/* 割込みレベル5 */
#define TIRQ_LEVEL4		(-4)		/* 割込みレベル4 */
#define TIRQ_LEVEL3		(-3)		/* 割込みレベル3 */
#define TIRQ_LEVEL2		(-2)		/* 割込みレベル2 */
#define TIRQ_LEVEL1		(-1)		/* 割込みレベル1 */

/*
 *  DGAへのアクセス関数
 */
#define dga_rew_reg(addr)			sil_rew_mem(((uint32_t *) addr))
#define dga_wrw_reg(addr, val)		sil_wrw_mem(((uint32_t *) addr), val)

/*
 *  DGAのレジスタへのアクセス関数
 */
#ifndef TOPPERS_MACRO_ONLY

Inline uint32_t
dga_read(void *addr)
{
	return(dga_rew_reg(addr));
}

Inline void
dga_write(void *addr, uint32_t val)
{
	dga_wrw_reg(addr, val);
}

Inline void
dga_bit_or(void *addr, uint32_t bitpat)
{
	dga_write(addr, dga_read(addr) | bitpat);
}

Inline void
dga_bit_and(void *addr, uint32_t bitpat)
{
	dga_write(addr, dga_read(addr) & bitpat);
}

Inline void
dga_set_ilv(void *addr, uint_t shift, uint_t level)
{
	dga_write(addr, (dga_read(addr) & ~(0x07 << shift)) | (level << shift));
} 

#endif /* TOPPERS_MACRO_ONLY */

/*
 *  μPD72001（MPSC）のレジスタのアドレス
 */
#define TADR_UPD72001_DATAA		0xfff45003U
#define TADR_UPD72001_CTRLA		0xfff45007U
#define TADR_UPD72001_DATAB		0xfff4500bU
#define TADR_UPD72001_CTRLB		0xfff4500fU

/*
 *  μPD72001へのアクセス関数
 */
#define upd72001_reb_reg(addr)			sil_reb_mem(((uint8_t *) addr))
#define upd72001_wrb_reg(addr, val)		sil_wrb_mem(((uint8_t *) addr), val)

/*
 *  開発環境依存の処理
 */
#ifndef TOPPERS_MACRO_ONLY
#ifdef TOPPERS_GDB_STUB				/* GDBスタブ */

Inline void
dve68k_exit(void)
{
	Asm("trap #2");
}

Inline void
dve68k_putc(char c)
{
	Asm("move.l %0, %%d1; trap #3"
	  : /* no output */
	  : "g"(c)
	  : "d0", "d1", "d2", "d6", "d7");
}

#else /* TOPPERS_GDB_STUB */		/* その他の開発環境 */

extern void		dve68k_exit(void) NoReturn;
extern void		dve68k_putc(char c);

#endif /* TOPPERS_GDB_STUB */
#endif /* TOPPERS_MACRO_ONLY */
#endif /* TOPPERS_DVE68K_H */
