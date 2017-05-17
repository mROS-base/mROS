/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2013,2016 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: pl310.h 1060 2014-04-30 09:42:46Z ertl-honda $
 */

/*
 * L2キャッシュコントローラPL310向け定義
 */
#ifndef PL310_H
#define PL310_H

#include "kernel_impl.h"
#include <sil.h>
#include "target_config.h"

#define PL310_L2CACHE_BASE ASP_PL310_BASE

#define PL310_CACHE_ID			0x000
#define PL310_CACHE_ID_PART_MASK	(0xf << 6)
#define PL310_CACHE_ID_PART_L210	(1 << 6)
#define PL310_CACHE_ID_PART_L310	(3 << 6)
#define PL310_CACHE_TYPE			0x004
#define PL310_CTRL			0x100
#define PL310_AUX_CTRL			0x104
#define PL310_TAG_LATENCY_CTRL		0x108
#define PL310_DATA_LATENCY_CTRL		0x10C
#define PL310_EVENT_CNT_CTRL		0x200
#define PL310_EVENT_CNT1_CFG		0x204
#define PL310_EVENT_CNT0_CFG		0x208
#define PL310_EVENT_CNT1_VAL		0x20C
#define PL310_EVENT_CNT0_VAL		0x210
#define PL310_INTR_MASK			0x214
#define PL310_MASKED_INTR_STAT		0x218
#define PL310_RAW_INTR_STAT		0x21C
#define PL310_INTR_CLEAR			0x220
#define PL310_CACHE_SYNC			0x730
#define PL310_INV_LINE_PA		0x770
#define PL310_INV_WAY			0x77C
#define PL310_CLEAN_LINE_PA		0x7B0
#define PL310_CLEAN_LINE_IDX		0x7B8
#define PL310_CLEAN_WAY			0x7BC
#define PL310_CLEAN_INV_LINE_PA		0x7F0
#define PL310_CLEAN_INV_LINE_IDX		0x7F8
#define PL310_CLEAN_INV_WAY		0x7FC
#define PL310_LOCKDOWN_WAY_D		0x900
#define PL310_LOCKDOWN_WAY_I		0x904
#define PL310_TEST_OPERATION		0xF00
#define PL310_LINE_DATA			0xF10
#define PL310_LINE_TAG			0xF30
#define PL310_DEBUG_CTRL			0xF40
#define PL310_PREFETCH_CTRL		0xF60
#define PL310_POWER_CTRL			0xF80

#define L2X0_AUX_CTRL_MASK                      0xc0000fff
#define L2X0_AUX_CTRL_DATA_RD_LATENCY_SHIFT     0
#define L2X0_AUX_CTRL_DATA_RD_LATENCY_MASK      0x7
#define L2X0_AUX_CTRL_DATA_WR_LATENCY_SHIFT     3
#define L2X0_AUX_CTRL_DATA_WR_LATENCY_MASK      (0x7 << 3)
#define L2X0_AUX_CTRL_TAG_LATENCY_SHIFT         6
#define L2X0_AUX_CTRL_TAG_LATENCY_MASK          (0x7 << 6)
#define L2X0_AUX_CTRL_DIRTY_LATENCY_SHIFT       9
#define L2X0_AUX_CTRL_DIRTY_LATENCY_MASK        (0x7 << 9)
#define L2X0_AUX_CTRL_ASSOCIATIVITY_SHIFT       16
#define L2X0_AUX_CTRL_WAY_SIZE_SHIFT            17
#define L2X0_AUX_CTRL_WAY_SIZE_MASK             (0x7 << 17)
#define L2X0_AUX_CTRL_SHARE_OVERRIDE_SHIFT      22
#define L2X0_AUX_CTRL_NS_LOCKDOWN_SHIFT         26
#define L2X0_AUX_CTRL_NS_INT_CTRL_SHIFT         27
#define L2X0_AUX_CTRL_DATA_PREFETCH_SHIFT       28
#define L2X0_AUX_CTRL_INSTR_PREFETCH_SHIFT      29
#define L2X0_AUX_CTRL_EARLY_BRESP_SHIFT         30

#ifndef TOPPERS_MACRO_ONLY
extern void pl310_init(uint32_t aux_val, uint32_t aux_mask);
extern void pl310_debug_set(uint32_t val);
extern void pl310_flush_all(void);
extern void pl310_invalidate_all(void);
extern void pl310_disable(void);
#endif /* TOPPERS_MACRO_ONLY */

#endif /* PL310_H */
