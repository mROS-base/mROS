/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 *
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
 *  @(#) $Id: gic.c 2758 2016-03-10 15:15:26Z ertl-honda $
 */

#include "kernel_impl.h"

/*
 *  CPU Interface 関連
 */

/*
 *  CPU Interface の初期化
 */
void
gicc_init(void)
{
	/* CPUインタフェースを無効に */
	sil_wrw_mem((void *)ASP_GICC_ICCICR, 0);

	/* 最低優先度に設定 */
	gicc_set_priority(GIC_MIN_PRIORITY);

	/* 割込み優先度の全ビット有効に */
	gicc_set_bp(0);

	/* ペンディングしている可能性があるので，EOI によりクリア */
	sil_wrw_mem((void *)ASP_GICC_ICCEOIR, sil_rew_mem((void *)ASP_GICC_ICCIAR));

	/* CPUインタフェースを有効に */
#ifdef GIC_USE_FIQ
	sil_wrw_mem((void *)ASP_GICC_ICCICR, (GICC_CTLR_FIQEn|GICC_CTLR_ENABLEGRP1|GICC_CTLR_ENABLEGRP0));
#else /* GIC_USE_FIQ */
	//sil_wrw_mem((void *)ASP_GICC_ICCICR, GICC_CTLR_EN);
	sil_wrw_mem((void *)ASP_GICC_ICCICR, (GICC_CTLR_ENABLEGRP1|GICC_CTLR_ENABLEGRP0));
#endif /* GIC_USE_FIQ */
}

/*
 *  CPU Interface の終了
 */
void
gicc_stop(void)
{
#ifndef GICC_NO_INIT
	sil_wrw_mem((void *)(ASP_GICC_ICCICR), 0);
#endif /* GICC_NO_INIT */
}

/*
 *  Distoributor 関連
 */
static const uint32_t intc_icdicfrn_table[] =	 /* Initial value table of Interrupt Configuration Registers */
{						  /*		   Interrupt ID */
	0xAAAAAAAA,			/* ICDICFR0  :  15 to   0 */
	0x00000055,			/* ICDICFR1  :  19 to  16 */
	0xFFFD5555,			/* ICDICFR2  :  47 to  32 */
	0x555FFFFF,			/* ICDICFR3  :  63 to  48 */
	0x55555555,			/* ICDICFR4  :  79 to  64 */
	0x55555555,			/* ICDICFR5  :  95 to  80 */
	0x55555555,			/* ICDICFR6  : 111 to  96 */
	0x55555555,			/* ICDICFR7  : 127 to 112 */
	0x5555F555,			/* ICDICFR8  : 143 to 128 */
	0x55555555,			/* ICDICFR9  : 159 to 144 */
	0x55555555,			/* ICDICFR10 : 175 to 160 */
	0xF5555555,			/* ICDICFR11 : 191 to 176 */
	0xF555F555,			/* ICDICFR12 : 207 to 192 */
	0x5555F555,			/* ICDICFR13 : 223 to 208 */
	0x55555555,			/* ICDICFR14 : 239 to 224 */
	0x55555555,			/* ICDICFR15 : 255 to 240 */
	0x55555555,			/* ICDICFR16 : 271 to 256 */
	0xFD555555,			/* ICDICFR17 : 287 to 272 */
	0x55555557,			/* ICDICFR18 : 303 to 288 */
	0x55555555,			/* ICDICFR19 : 319 to 304 */
#ifdef TOPPERS_RZA1H
	0x55555555,			/* ICDICFR20 : 335 to 320 */
#else /* TOPPERS_RZA1L */
	0x7FD55555,			/* ICDICFR20 : 335 to 320 */
#endif /* TOPPERS_RZA1H */
	0x5F555555,			/* ICDICFR21 : 351 to 336 */
	0xFD55555F,			/* ICDICFR22 : 367 to 352 */
	0x55555557,			/* ICDICFR23 : 383 to 368 */
	0x55555555,			/* ICDICFR24 : 399 to 384 */
	0x55555555,			/* ICDICFR25 : 415 to 400 */
	0x55555555,			/* ICDICFR26 : 431 to 416 */
	0x55555555,			/* ICDICFR27 : 447 to 432 */
	0x55555555,			/* ICDICFR28 : 463 to 448 */
	0x55555555,			/* ICDICFR29 : 479 to 464 */
	0x55555555,			/* ICDICFR30 : 495 to 480 */
	0x55555555,			/* ICDICFR31 : 511 to 496 */
	0x55555555,			/* ICDICFR32 : 527 to 512 */
	0x55555555,			/* ICDICFR33 : 543 to 528 */
#ifdef TOPPERS_RZA1H
	0x55555555,			/* ICDICFR34 : 559 to 544 */
	0x55555555,			/* ICDICFR35 : 575 to 560 */
	0x00155555			 /* ICDICFR36 : 586 to 576 */
#endif /* TOPPERS_RZA1H */
};

/*
 *  割込み禁止
 */
void
gicd_disable_int(uint8_t id)
{
	uint16_t offset_addr;
	uint16_t offset_bit;

	offset_addr = (id / 32) * 4;
	offset_bit  = id % 32;

	sil_wrw_mem((void *)(ASP_GICD_ICDICER + offset_addr), (1 << offset_bit));
}

/*
 *  割込み許可
 */
void
gicd_enable_int(uint8_t id)
{
	uint16_t offset_addr;
	uint16_t offset_bit;

	offset_addr = (id / 32) * 4;
	offset_bit  = id % 32;

	sil_wrw_mem((void *)(ASP_GICD_ICDISER + offset_addr), (1 << offset_bit));
}

/*
 *  割込みペンディングクリア
 */
void
gicd_clear_pending(uint8_t id)
{
	uint16_t offset_addr;
	uint16_t offset_bit;

	offset_addr = (id / 32) * 4;
	offset_bit  = id % 32;

	sil_wrw_mem((void *)(ASP_GICD_ICDICPR + offset_addr), (1 << offset_bit));
}

/*
 *  割込みペンディングセット
 */
void
gicd_set_pending(uint8_t id)
{
	uint16_t offset_addr;
	uint16_t offset_bit;

	offset_addr = (id / 32) * 4;
	offset_bit  = id % 32;

	sil_wrw_mem((void *)(ASP_GICD_ICDISPR + offset_addr), (1 << offset_bit));
}

/*
 *  割込み要求のチェック
 */
bool_t
gicd_probe_int(uint8_t id)
{
	uint32_t state;
	uint16_t offset_addr;
	uint16_t offset_bit;

	offset_addr = (id / 32) * 4;
	offset_bit  = id % 32;

	state = sil_rew_mem((void *)(ASP_GICD_ICDISPR + offset_addr));

	if ((state & (1 << offset_bit)) == (1 << offset_bit)) {
		return(true);
	}
	else {
		return(false);
	}
}

/*
 *  割込みコンフィギュレーション設定
 */
void
gicd_config(uint8_t id,  bool_t is_edge, bool_t is_1_n)
{
	uint16_t offset_addr;
	uint16_t offset_bit;
	uint32_t cfgr_reg_val;
	uint8_t  config;

	if (id < 416) return;       /* ID 0～415は設定変更禁止。変更可能なのは端子割り込みのみ。 */

	if (is_edge) {
		config = GICD_ICFGRn_EDGE;
	}
	else {
		config = GICD_ICFGRn_LEVEL;
	}

	if (is_1_n) {
		config |= GICD_ICFGRn_1_N;
	}
	else {
		config |= GICD_ICFGRn_N_N;
	}
	
	
	offset_addr = (id / 16) * 4;
	offset_bit  = (id % 16) * 2;

	cfgr_reg_val  = sil_rew_mem((void *)(ASP_GICD_ICDICFR + offset_addr));
	cfgr_reg_val &= ~(0x03U << offset_bit);
	cfgr_reg_val |= (0x03U & config) << offset_bit;
	sil_wrw_mem((void *)(ASP_GICD_ICDICFR + offset_addr), cfgr_reg_val);

#ifdef GIC_USE_FIQ
	offset_addr = (id / 32) * 4;
	offset_bit  = id % 32;

	sil_wrw_mem((void *)(ASP_GICD_ICDISR + offset_addr),
				sil_rew_mem((void *)(ASP_GICD_ICDISR + offset_addr)) & ~(1 << offset_bit));
#endif /* GIC_USE_FIQ */
}

/*
 *  割込み優先度のセット
 *  内部表現で渡す．
 */
void
gicd_set_priority(uint8_t id, int pri)
{
	uint16_t offset_addr;
	uint16_t offset_bit;
	uint32_t pr_reg_val;

	offset_addr = (id / 4) * 4;
	offset_bit  = ((id % 4) * 8) + GICC_PMR_OFFSET;

	pr_reg_val  = sil_rew_mem((void *)(ASP_GICD_ICDIPR + offset_addr));
	pr_reg_val &= ~(GICC_PMR_MASK << offset_bit);
	pr_reg_val |= (GICC_PMR_MASK & pri) << (offset_bit);
	sil_wrw_mem((void *)(ASP_GICD_ICDIPR + offset_addr), pr_reg_val);
}

/*
 *  DIC割込みターゲットの設定
 *  cpusはターゲットとするCPUのビットパターンで指定
 *   CPU0 : 0x01
 *   CPU1 : 0x02
 *   CPU2 : 0x04
 *   CPU3 : 0x08
 */
void
gicd_set_target(uint8_t id, uint8_t cpus)
{
	uint32_t offset_addr;
	uint32_t offset_bit;
	uint32_t itr_reg_val;

	offset_addr = (id / 4) * 4;
	offset_bit  = (id % 4) * 8;

	itr_reg_val  = sil_rew_mem((void *)(ASP_GICD_ICDIPTR + offset_addr));
	itr_reg_val &= ~(0xf << offset_bit);
	itr_reg_val |= (cpus << offset_bit);
	sil_wrw_mem((void *)(ASP_GICD_ICDIPTR + offset_addr), itr_reg_val);
}

/*
 *  Distributor 初期化
 */
void
gicd_init(void)
{
	int i;

	/* Distributor を無効に */
	sil_wrw_mem((void *)(ASP_GICD_ICDDCR), 0);

#ifdef GIC_USE_FIQ
	/* 割込みを全てグループ1(IRQ)に */
	for(i = 0; i < ASP_NUM_ICDISR_ENTRY; i++){
		sil_wrw_mem((void *)(ASP_GICD_ICDISR + (4 * i)), 0xffffffff);
	}
#endif /* GIC_USE_FIQ */

	/* モード初期化(1-N Level) */
	for(i = 0; i < ASP_NUM_ICDICFR_ENTRY; i++){
		sil_wrw_mem((void *)(ASP_GICD_ICDICFR + (4 * i)), intc_icdicfrn_table[i]);
	}

	/* 優先度最低に設定  */
	for(i = 0; i < ASP_NUM_ICDIPR_ENTRY; i++){
		sil_wrw_mem((void *)(ASP_GICD_ICDIPR + (4 * i)), 0xffffffff);
	}

	/* ターゲット初期化（全てCPU0へ） */
	for(i = ASP_NUM_ICDIPTR_ENTRY; i < GIC_TNUM_INT/4; i++){
		sil_wrw_mem((void *)(ASP_GICD_ICDIPTR + (4 * i)), 0x01010101);
	}

	/* 割込みを全て禁止 */
	for(i = 0; i < ASP_NUM_ICDICER_ENTRY; i++){
		sil_wrw_mem((void *)(ASP_GICD_ICDICER + (4 * i)), 0xffffffff);
	}

	/* ペンディングをクリア */
	for(i = 0; i < ASP_NUM_ICDICPR_ENTRY; i++){
		sil_wrw_mem((void *)(ASP_GICD_ICDICPR + (4 * i)), 0xffffffff);
	}

	/* Distibutor を有効に */
	sil_wrw_mem((void *)(ASP_GICD_ICDDCR), GICD_CTLR_ENABLE);
}

/*
 *  Distributor 終了
 */
void
gicd_stop(void)
{
	/* Distributor を無効に */
	sil_wrw_mem((void *)(ASP_GICD_ICDDCR), 0);
}
