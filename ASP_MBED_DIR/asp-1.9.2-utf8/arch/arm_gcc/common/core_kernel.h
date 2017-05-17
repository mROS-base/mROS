/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2007 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: core_kernel.h 2742 2016-01-09 04:25:18Z ertl-honda $
 */

/*
 *      kernel.hのコア依存部（ARM用）
 *
 *  このインクルードファイルは，target_kernel.h（または，そこからインク
 *  ルードされるファイル）のみからインクルードされる．他のファイルから
 *  直接インクルードしてはならない．
 */

#ifndef TOPPERS_CORE_KERNEL_H
#define TOPPERS_CORE_KERNEL_H

#ifndef TOPPERS_MACRO_ONLY

#include "arm.h"

#if (__TARGET_ARCH_ARM == 4) || (__TARGET_ARCH_ARM == 5)
typedef struct {
	uint32_t nest_count;
	uint32_t ipm;
	uint32_t cpsr;
	uint32_t r0;
	uint32_t r1;
	uint32_t r2;
	uint32_t r3;
	uint32_t r12;
	uint32_t lr;
	uint32_t pc;
} exc_frame_t;
#else /* (__TARGET_ARCH_ARM == 6) || (__TARGET_ARCH_ARM == 7) */
typedef struct {
	uint32_t nest_count;
	uint32_t ipm;
	uint32_t r0;
	uint32_t r1;
	uint32_t r2;
	uint32_t r3;
	uint32_t r12;
	uint32_t lr;
	uint32_t pc;
	uint32_t cpsr;
} exc_frame_t;
#endif /* (__TARGET_ARCH_ARM == 6) || (__TARGET_ARCH_ARM == 7) */


/*
 * CPU例外からの戻り先アドレスの取得
 */
Inline uint32_t
x_get_exc_raddr(void *p_excinf)
{
	return(((exc_frame_t *)(p_excinf))->pc);
}

/*
 * CPU例外からの戻り先アドレスの設定
 */
Inline void
x_set_exc_raddr(void *p_excinf, uint32_t pc)
{
	((exc_frame_t *)(p_excinf))->pc = pc;
}

#if __TARGET_ARCH_ARM == 7

/* 性能計測用のカウンタのデータ型 */
typedef uint32_t PERFCNT;

/*
 *  パフォーマンスカウンタの初期化
 */
Inline void
x_init_pcc(void)
{
	uint32_t tmp;
	/* 全カウンターの有効化 */
	CP15_PMCR_READ(tmp);

#ifdef TOPPERS_ARM_PCC_DIV64
	tmp |= CP15_PMCR_PMCCNTR_D;
#else /* !TOPPERS_ARM_PCC_DIV64 */
	tmp &= ~CP15_PMCR_PMCCNTR_D;
#endif /* TOPPERS_ARM_PCC_DIV64 */

	CP15_PMCR_WRITE(tmp|CP15_PMCR_ALLCNTR_ENABLE);

	/* パフォーマンスカウンタの有効化 */
	CP15_PMCNTENSET_READ(tmp);
	CP15_PMCNTENSET_WRITE(tmp|CP15_PMCNTENSET_CCNTR_ENABLE);
}

/*
 *  パフォーマンスカウンタの読み込み
 */
Inline void 
x_get_pcc(PERFCNT *p_count)
{
	CP15_PMCCNTR_READ(*p_count);
}

/*
 *  パフォーマンスカウンタのリセット
 */
Inline void
x_rst_pcc(void)
{
	uint32_t tmp;
	CP15_PMCR_READ(tmp);
	CP15_PMCR_WRITE(tmp|CP15_PMCR_PMCCNTR_CLEAR);
}

/*
 *  カウンタ値のnsecへの変換
 */
Inline ulong_t
x_cnv_nsec(PERFCNT count) {
#ifdef TOPPERS_ARM_PCC_DIV64
	return ((ulong_t)count*(1000/(CORE_CLK_MHZ/64)));
#else /* !TOPPERS_ARM_PCC_DIV64 */
	return ((ulong_t)count*(1000/CORE_CLK_MHZ));
#endif /* TOPPERS_ARM_PCC_DIV64 */
}

#elif __TARGET_ARCH_ARM == 6


#endif /* __TARGET_ARCH_ARM == 7 */

#endif /* TOPPERS_MACRO_ONLY */

#endif /* TOPPERS_CORE_KERNEL_H */
