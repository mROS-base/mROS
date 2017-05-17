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
 *  @(#) $Id: gic.c 2742 2016-01-09 04:25:18Z ertl-honda $
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
	sil_wrw_mem((void *)GICC_CTLR, 0);

	/* 最低優先度に設定 */
	gicc_set_priority(GIC_MIN_PRIORITY);

	/* 割込み優先度の全ビット有効に */
	gicc_set_bp(0);

	/* ペンディングしている可能性があるので，EOI によりクリア */
	sil_wrw_mem((void *)GICC_EOIR,
				sil_rew_mem((void *)GICC_IAR));

	/* CPUインタフェースを有効に */
#ifdef GIC_USE_FIQ
	sil_wrw_mem((void *)GICC_CTLR,
				(GICC_CTLR_FIQEn|GICC_CTLR_ENABLEGRP1|GICC_CTLR_ENABLEGRP0));
#else /* GIC_USE_FIQ */
	sil_wrw_mem((void *)GICC_CTLR, GICC_CTLR_EN);
#endif /* GIC_USE_FIQ */
}

/*
 *  CPU Interface の終了
 */
void
gicc_stop(void)
{
#ifndef GICC_NO_INIT
	sil_wrw_mem((void *)(GICC_CTLR), 0);
#endif /* GICC_NO_INIT */
}

/*
 *  Distoributor 関連
 */

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

	sil_wrw_mem((void *)(GICD_ICENABLERn + offset_addr), (1 << offset_bit));
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

	sil_wrw_mem((void *)(GICD_ISENABLERn + offset_addr), (1 << offset_bit));
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

	sil_wrw_mem((void *)(GICD_ICPENDRn + offset_addr), (1 << offset_bit));
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

	sil_wrw_mem((void *)(GICD_ISPENDRn + offset_addr), (1 << offset_bit));
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

	state = sil_rew_mem((void *)(GICD_ISPENDRn + offset_addr));

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

	cfgr_reg_val  = sil_rew_mem((void *)(GICD_ICFGRn + offset_addr));
	cfgr_reg_val &= ~(0x03U << offset_bit);
	cfgr_reg_val |= (0x03U & config) << offset_bit;
	sil_wrw_mem((void *)(GICD_ICFGRn + offset_addr), cfgr_reg_val);

#ifdef GIC_USE_FIQ
	offset_addr = (id / 32) * 4;
	offset_bit  = id % 32;

	sil_wrw_mem((void *)(GICD_IGROUPRn + offset_addr),
				sil_rew_mem((void *)(GICD_IGROUPRn+ offset_addr)) & ~(1 << offset_bit));
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

	pr_reg_val  = sil_rew_mem((void *)(GICD_IPRIORITYRn + offset_addr));
	pr_reg_val &= ~(GICC_PMR_MASK << offset_bit);
	pr_reg_val |= (GICC_PMR_MASK & pri) << (offset_bit);
	sil_wrw_mem((void *)(GICD_IPRIORITYRn + offset_addr), pr_reg_val);
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

	itr_reg_val  = sil_rew_mem((void *)(GICD_ITARGETSRn + offset_addr));
	itr_reg_val &= ~(0xf << offset_bit);
	itr_reg_val |= (cpus << offset_bit);
	sil_wrw_mem((void *)(GICD_ITARGETSRn + offset_addr), itr_reg_val);
}

/*
 *  Distributor 初期化
 */
void
gicd_init(void)
{
	int i;

	/* Distributor を無効に */
	sil_wrw_mem((void *)(GICD_CTLR), 0);

#ifdef GIC_USE_FIQ
	/* 割込みを全てグループ1(IRQ)に */
	for(i = 0; i < GIC_TNUM_INT/32; i++){
		sil_wrw_mem((void *)(GICD_IGROUPRn + (4 * i)), 0xffffffff);
	}
#endif /* GIC_USE_FIQ */

	/* 割込みを全て禁止 */
	for(i = 0; i < GIC_TNUM_INT/32; i++){
		sil_wrw_mem((void *)(GICD_ICENABLERn + (4 * i)), 0xffffffff);
	}

	/* ペンディングをクリア */
	for(i = 0; i < GIC_TNUM_INT/32; i++){
		sil_wrw_mem((void *)(GICD_ICPENDRn + (4 * i)), 0xffffffff);
	}

	/* 優先度最低に設定  */
	for(i = 0; i < GIC_TNUM_INT/4; i++){
		sil_wrw_mem((void *)(GICD_IPRIORITYRn + (4 * i)), 0xffffffff);
	}

	/* ターゲット初期化（全てCPU0へ） */
	for(i = TMIN_GLOBAL_INTNO/4; i < GIC_TNUM_INT/4; i++){
		sil_wrw_mem((void *)(GICD_ITARGETSRn + (4 * i)), 0x01010101);
	}

	/* モード初期化(1-N Level) */
	for(i = 0; i < GIC_TNUM_INT/16; i++){
		sil_wrw_mem((void *)(GICD_ICFGRn + (4 * i)), 0x55555555);
	}

	/* Distibutor を有効に */
	sil_wrw_mem((void *)(GICD_CTLR), GICD_CTLR_ENABLE);
}

/*
 *  Distributor 終了
 */
void
gicd_stop(void)
{
	/* Distributor を無効に */
	sil_wrw_mem((void *)(GICD_CTLR), 0);
}

#ifdef TOPPERS_SAFEG_SECURE
int spurious_cnt;

/*
 *  ID番号1022の割込み(Spurious)が入った場合，
 *  Activeな割込みがあるかチェックして，Activeな割込みがあれば
 *  そのIDをリターンする．
 *  Secure側の割込みが入ったにもかかわらず， ID番号1022の割込み(Spurious)として
 *  通知される現状(不具合?) に対応
 */
unsigned int
check_spurious_1022(void){
	unsigned int val, id;
	int i, e;

	spurious_cnt++;
	for(i = 0; i < GIC_TNUM_INT/32; i++) {
		val = sil_rew_mem((void *)GICD_ISACTIVERn + (4 * i));
		if (val != 0x00){
			for(e = 0; e < 32; e++) {
				if ((val & (1 << e)) != 0){
					id = (i * 32 + e);
					/* 割込み属性が設定されていれば(使用している割込みなら) */
					if (cfgint_tbl[id] == 1U) {
						return id;
					}
				}
			}
		}
	}
	return 1022;
}
#endif /* TOPPERS_SAFEG_SECURE */
